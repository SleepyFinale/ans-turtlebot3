#!/usr/bin/env python3
"""Short-range stereo depth for the lidar blind band and floor drops.

Without config/stereo/<namespace>.yaml the node publishes a debug status and
nothing else, so Nav2 and the stop gate stay inactive until calibration exists.
"""

import json
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402
import stereo_visual_odometry  # noqa: E402


def _namespace_of(node: Node) -> str:
    return node.get_namespace().strip('/')


class StereoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__('stereo_depth_node')

        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('points_topic', 'stereo/points')
        self.declare_parameter('scan_topic', 'stereo_blob_scan')
        self.declare_parameter('debug_topic', 'stereo/debug')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('left_frame', '')
        self.declare_parameter('right_frame', '')
        self.declare_parameter('publish_hz', 4.0)
        self.declare_parameter('sync_slop_sec', 0.03)
        self.declare_parameter('min_range_m', 0.12)
        self.declare_parameter('max_range_m', 1.5)
        # Obstacles and clearing only within this bearing of straight ahead.
        # Near the edge of the ~120 deg lenses the calibration's lens model
        # misses by a few pixels of disparity: on inky a printer 1.7 m away at
        # 50 deg to the side was placed at 1.2 m as a low obstacle. The LiDAR
        # covers the sides; checked against it with stereo_fit_offset.py,
        # disparity is within 0.5 px out to about 41 deg.
        self.declare_parameter('max_bearing_deg', 45.0)
        self.declare_parameter('min_obstacle_height_m', 0.025)
        # base_scan is 0.172 m above base_link, and base_link is 0.010 m above
        # base_footprint, so the lidar plane is 0.182 m above the ground.
        self.declare_parameter('lidar_height_m', 0.182)
        self.declare_parameter('ground_x_min_m', 0.22)
        self.declare_parameter('ground_x_max_m', 1.20)
        self.declare_parameter('drop_z_m', -0.15)
        self.declare_parameter('min_valid_ratio', 0.05)
        self.declare_parameter('cloud_stride', 4)
        self.declare_parameter('num_disparities', 128)
        self.declare_parameter('block_size', 5)
        self.declare_parameter('close_obstacle_m', 0.45)
        # StereoBM's textureThreshold, applied to SGBM output. Blank floor
        # matches are not obstacles.
        self.declare_parameter('texture_threshold', 12.0)
        # Drops only trust pixels with clear texture. Glossy or blank surfaces
        # mirror the room and would otherwise read as a hole.
        self.declare_parameter('drop_texture_threshold', 20.0)
        # SGBM only searches along rows. When a camera is bumped, features land
        # rows apart and every depth is noise, so the node checks the row error
        # against real matches and goes quiet instead of reporting garbage.
        self.declare_parameter('alignment_check_sec', 5.0)
        self.declare_parameter('max_row_error_px', 2.0)
        # Drops are measured against the floor the mount fields predict. When
        # the fitted floor sits this far from z = 0 for floor_check_frames in
        # a row, camera_height_m / camera_pitch_rad no longer match the
        # calibration (a new chessboard calibration changes the rectification)
        # and the floor itself reads as a drop. Drops are then left out.
        self.declare_parameter('max_floor_offset_m', 0.03)
        self.declare_parameter('floor_check_frames', 5)
        # Stereo visual odometry on the same pairs, published on odom/visual
        # for the bringup EKF. It has its own row-error tolerance, so it keeps
        # running while misalignment turns obstacle marking off.
        self.declare_parameter('visual_odometry', False)
        self.declare_parameter('visual_odom_topic', 'odom/visual')
        self.declare_parameter('visual_odometry_hz', 5.0)
        self.declare_parameter('odom_frame', '')

        ns = _namespace_of(self)
        calib_param = str(self.get_parameter('calibration_file').value).strip()
        self._calib_path = stereo_common.calibration_file_path(ns, calib_param)
        prefix = f'{ns}/' if ns else ''
        self._parent_frame = str(self.get_parameter('parent_frame').value).strip() or (
            f'{prefix}base_footprint'
        )
        self._left_frame = str(self.get_parameter('left_frame').value).strip() or (
            f'{prefix}stereo_left_optical_frame'
        )
        self._right_frame = str(self.get_parameter('right_frame').value).strip() or (
            f'{prefix}stereo_right_optical_frame'
        )
        self._publish_period = 1.0 / max(0.5, float(self.get_parameter('publish_hz').value))
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._max_range = float(self.get_parameter('max_range_m').value)
        self._max_bearing = math.radians(
            min(90.0, max(5.0, float(self.get_parameter('max_bearing_deg').value))))
        self._min_height = float(self.get_parameter('min_obstacle_height_m').value)
        self._lidar_height = float(self.get_parameter('lidar_height_m').value)
        self._ground_x_min = float(self.get_parameter('ground_x_min_m').value)
        self._ground_x_max = float(self.get_parameter('ground_x_max_m').value)
        self._drop_z = float(self.get_parameter('drop_z_m').value)
        self._min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self._close_obstacle_m = float(self.get_parameter('close_obstacle_m').value)
        self._texture_threshold = float(self.get_parameter('texture_threshold').value)
        self._drop_texture_threshold = float(self.get_parameter('drop_texture_threshold').value)
        self._cloud_stride = max(1, int(self.get_parameter('cloud_stride').value))
        self._alignment_period = max(0.0, float(self.get_parameter('alignment_check_sec').value))
        self._max_row_error = float(self.get_parameter('max_row_error_px').value)
        self._last_alignment_s = 0.0
        self._row_error_px = None
        self._misaligned_checks = 0
        self._misaligned = False
        # Obstacles stay off until one alignment check passes. Starting out
        # trusted let a bumped pair publish garbage marks for the 5-10 s
        # before two failed checks turned it off, and low_obstacle_memory
        # kept those marks around the robot's start pose.
        self._alignment_verified = self._alignment_period <= 0.0

        self._bridge = CvBridge()
        self._last_status_s = 0.0
        self._obstacle_streak = 0
        self._drop_streak = 0
        self._floor_off_streak = 0
        self._last_floor_warn_s = 0.0
        self._shadow_obstacle_m = None
        self._shadow_until_s = 0.0
        self._warned_uncalibrated = False
        self._calibrated = False
        self._maps = None
        self._num_disp = 128

        # Two scans for one costmap layer: stereo_blob_scan marks confirmed
        # obstacles and drops, stereo_clear_scan clears only along beams where
        # the floor itself was matched. Marks persist (memory) until a clear
        # beam passes through them, so a moved object is forgotten only once
        # the cameras see floor where it stood.
        self._scan_pub = self.create_publisher(LaserScan, 'stereo_blob_scan', 10)
        self._clear_pub = self.create_publisher(LaserScan, 'stereo_clear_scan', 10)
        self._prev_obstacles = None
        self._points_pub = self.create_publisher(PointCloud2, 'stereo/points', 5)
        self._debug_pub = self.create_publisher(String, 'stereo/debug', 10)
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._load_calibration()

        self._vo = None
        if self._calibrated and bool(self.get_parameter('visual_odometry').value):
            self._vo = stereo_visual_odometry.StereoVO(self._calib_path)
            self._odom_frame = str(self.get_parameter('odom_frame').value).strip() or f'{prefix}odom'
            self._vo_pub = self.create_publisher(
                Odometry, str(self.get_parameter('visual_odom_topic').value), 10)
            self._vo_debug_pub = self.create_publisher(String, 'stereo/vo_debug', 10)
            self.create_timer(1.0, self._publish_vo_debug)
            # VO keeps its own frame queue and timer in its own callback group,
            # so it runs at its rate while a slow depth pair is in progress
            # (OpenCV releases the GIL; main() uses a multi-threaded executor).
            self._vo_left = deque(maxlen=3)
            self._vo_right = deque(maxlen=3)
            self.create_timer(
                1.0 / max(1.0, float(self.get_parameter('visual_odometry_hz').value)),
                self._on_vo_tick, callback_group=MutuallyExclusiveCallbackGroup())

        left_topic = str(self.get_parameter('left_image_topic').value)
        right_topic = str(self.get_parameter('right_image_topic').value)
        slop = max(0.005, float(self.get_parameter('sync_slop_sec').value))
        self._slop_ns = int(slop * 1e9)
        # The cameras send 30 fps each but only publish_hz pairs are matched.
        # Frames are kept serialized (raw=True) and decoded only when a pair
        # is picked; decoding all 60 frames/s in Python cost more CPU than
        # the stereo match.
        self._raw_left = deque(maxlen=3)
        self._raw_right = deque(maxlen=3)
        self.create_subscription(
            Image, left_topic, lambda raw: self._on_raw(raw, self._raw_left, 'left'),
            qos_profile_sensor_data, raw=True)
        self.create_subscription(
            Image, right_topic, lambda raw: self._on_raw(raw, self._raw_right, 'right'),
            qos_profile_sensor_data, raw=True)
        self.create_timer(
            self._publish_period, self._on_tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self._publish_idle_debug)

        self.get_logger().info(
            f'stereo depth: left={left_topic} right={right_topic} '
            f'calibration={self._calib_path} calibrated={self._calibrated}'
        )

    def _load_calibration(self) -> None:
        if not stereo_common.calibration_is_usable(self._calib_path):
            self.get_logger().error(
                f'Stereo calibration missing or incomplete: {self._calib_path}. '
                'Cameras may run, but no blob scan or stop will be published.'
            )
            self._warned_uncalibrated = True
            return
        try:
            import cv2
            with open(self._calib_path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
            size = (
                int(data['image_width']),
                int(data['image_height']),
            )
            k1 = stereo_common.as_matrix3(data['left_camera_matrix'])
            d1 = stereo_common.as_vector(data['left_distortion'])
            k2 = stereo_common.as_matrix3(data['right_camera_matrix'])
            d2 = stereo_common.as_vector(data['right_distortion'])
            rotation = stereo_common.as_matrix3(data['R'])
            translation = stereo_common.as_vector(data['T']).reshape(3)
            r1, _r2, p1, _p2, q, _roi1, _roi2 = cv2.stereoRectify(
                k1, d1, k2, d2, size, rotation, translation.reshape(3, 1),
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0,
            )
            map1 = cv2.initUndistortRectifyMap(k1, d1, r1, p1, size, cv2.CV_32FC1)
            map2 = cv2.initUndistortRectifyMap(k2, d2, _r2, _p2, size, cv2.CV_32FC1)
            num_disp = int(self.get_parameter('num_disparities').value)
            num_disp = max(16, num_disp - (num_disp % 16))
            self._num_disp = num_disp
            block = int(self.get_parameter('block_size').value)
            if block % 2 == 0:
                block += 1
            block = max(3, block)
            matcher = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=num_disp,
                blockSize=block,
                P1=8 * block * block,
                P2=32 * block * block,
                disp12MaxDiff=1,
                preFilterCap=31,
                uniquenessRatio=15,
                speckleWindowSize=50,
                speckleRange=2,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
            )
            baseline = float(np.linalg.norm(translation))
            if baseline < 1e-4:
                baseline = float(data.get('baseline_m', 0.0508))
            optical_r = stereo_common.left_optical_rotation(float(data.get('camera_pitch_rad', 0.354706)))
            optical_t = stereo_common.left_camera_translation(
                float(data.get('camera_x_m', 0.0254)),
                baseline,
                float(data.get('camera_height_m', 0.127)),
            )
            right_t, right_r = stereo_common.right_camera_pose_in_left(rotation, translation)
        except Exception as exc:
            self.get_logger().error(f'Failed to load stereo calibration {self._calib_path}: {exc}')
            return

        self._cv2 = cv2
        self._maps = {
            'size': size,
            'map1': map1,
            'map2': map2,
            'q': q,
            'floor': stereo_common.expected_floor_disparity(
                size, p1, r1, optical_r, optical_t, baseline,
            ),
            'r1': np.asarray(r1, dtype=float),
            'matcher': matcher,
            'optical_r': optical_r,
            'optical_t': optical_t,
            'k1': k1, 'd1': d1, 'k2': k2, 'd2': d2,
            'rotation': rotation,
            'translation': translation,
        }
        self._calibrated = True
        self._publish_static_tf(optical_r, optical_t, right_r, right_t)
        self.get_logger().info(f'Loaded stereo calibration {self._calib_path}')

    def _publish_static_tf(self, optical_r, optical_t, right_r, right_t) -> None:
        stamp = self.get_clock().now().to_msg()
        left = self._make_tf(stamp, self._parent_frame, self._left_frame, optical_r, optical_t)
        right = self._make_tf(stamp, self._left_frame, self._right_frame, right_r, right_t)
        self._tf_broadcaster.sendTransform([left, right])

    def _make_tf(self, stamp, parent, child, rotation, translation) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        t = np.asarray(translation, dtype=float).reshape(3)
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        qx, qy, qz, qw = stereo_common.rotation_matrix_to_quaternion(rotation)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _publish_idle_debug(self) -> None:
        if self._calibrated:
            return
        self._publish_debug({
            'status': 'uncalibrated',
            'calibrated': False,
            'calibration_file': self._calib_path,
            'obstacle_range_m': None,
            'drop': False,
            'drop_range_m': None,
            'obstacle_streak': 0,
            'drop_streak': 0,
        })

    def _publish_vo_debug(self) -> None:
        msg = String()
        msg.data = json.dumps(dict(self._vo.stats, calibrated=True))
        self._vo_debug_pub.publish(msg)

    def _publish_debug(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._debug_pub.publish(msg)

    def _on_raw(self, raw, queue, side: str) -> None:
        queue.append(raw)
        if self._vo is not None:
            (self._vo_left if side == 'left' else self._vo_right).append(raw)

    def _best_pair(self, raw_lefts, raw_rights):
        lefts = [deserialize_message(raw, Image) for raw in raw_lefts]
        rights = [deserialize_message(raw, Image) for raw in raw_rights]
        best = None
        for left in lefts:
            for right in rights:
                gap = abs(self._stamp_ns(left) - self._stamp_ns(right))
                if gap <= self._slop_ns and (best is None or gap < best[0]):
                    best = (gap, left, right)
        return best

    def _mono(self, msg: Image):
        import cv2
        width, height = self._vo.size
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if img.shape[1] != width or img.shape[0] != height:
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        return img

    def _on_vo_tick(self) -> None:
        best = self._best_pair(list(self._vo_left), list(self._vo_right))
        if best is None:
            return
        self._vo_left.clear()
        self._vo_right.clear()
        _gap, left_msg, right_msg = best
        result = self._vo.process(self._mono(left_msg), self._mono(right_msg), self._stamp_ns(left_msg))
        if result is not None:
            self._vo_pub.publish(stereo_visual_odometry.odometry_msg(
                left_msg.header.stamp, self._odom_frame, self._parent_frame, result))

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_tick(self) -> None:
        """Process the closest-in-time left/right pair, if one is within slop."""
        if not self._calibrated or self._maps is None:
            return
        # Snapshots: the camera callbacks append from another executor thread.
        best = self._best_pair(list(self._raw_left), list(self._raw_right))
        if best is None:
            return
        self._raw_left.clear()
        self._raw_right.clear()
        self._on_pair(best[1], best[2])

    def _on_pair(self, left_msg: Image, right_msg: Image) -> None:
        try:
            self._process_pair(left_msg, right_msg)
        except Exception as exc:
            self.get_logger().warning(f'stereo pair failed: {exc}')
            self._publish_debug({
                'status': 'stereo_failed',
                'calibrated': True,
                'obstacle_range_m': None,
                'drop': False,
                'drop_range_m': None,
                'obstacle_streak': 0,
                'drop_streak': 0,
                'error': str(exc),
            })

    def _process_pair(self, left_msg: Image, right_msg: Image) -> None:
        cv2 = self._cv2
        width, height = self._maps['size']
        left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
        if left.shape[1] != width or left.shape[0] != height:
            left = cv2.resize(left, (width, height), interpolation=cv2.INTER_AREA)
        if right.shape[1] != width or right.shape[0] != height:
            right = cv2.resize(right, (width, height), interpolation=cv2.INTER_AREA)
        stamp = left_msg.header.stamp
        self._check_alignment(left, right)
        if self._misaligned or not self._alignment_verified:
            self._obstacle_streak = 0
            self._drop_streak = 0
            self._publish_scan(stamp, None)
            self._publish_debug({
                'status': 'misaligned' if self._misaligned else 'verifying_alignment',
                'calibrated': True,
                'row_error_px': self._row_error_px,
                'obstacle_range_m': None,
                'drop': False,
                'drop_range_m': None,
                'obstacle_streak': 0,
                'drop_streak': 0,
            })
            return
        left_r = cv2.remap(left, self._maps['map1'][0], self._maps['map1'][1], cv2.INTER_LINEAR)
        right_r = cv2.remap(right, self._maps['map2'][0], self._maps['map2'][1], cv2.INTER_LINEAR)
        disparity = self._maps['matcher'].compute(left_r, right_r).astype(np.float32) / 16.0
        valid = disparity > 0.0
        valid_ratio = float(np.count_nonzero(valid)) / float(valid.size)

        if valid_ratio < self._min_valid_ratio:
            self._obstacle_streak = 0
            self._drop_streak = 0
            self._publish_scan(stamp, None)
            self._publish_debug({
                'status': 'stereo_failed',
                'calibrated': True,
                'valid_ratio': valid_ratio,
                'obstacle_range_m': None,
                'drop': False,
                'drop_range_m': None,
                'obstacle_streak': 0,
                'drop_streak': 0,
            })
            return

        points_rect = cv2.reprojectImageTo3D(disparity, self._maps['q'])
        mask = valid & np.isfinite(points_rect).all(axis=2)
        z = points_rect[:, :, 2]
        mask &= (z > self._min_range) & (z < self._max_range)
        kept = points_rect[mask]
        base = stereo_common.rectified_to_base(
            kept,
            self._maps['r1'],
            self._maps['optical_r'],
            self._maps['optical_t'],
        )
        grad_x = cv2.Sobel(left_r, cv2.CV_32F, 1, 0, ksize=3)
        texture = cv2.boxFilter(np.abs(grad_x), cv2.CV_32F, (5, 5))
        now_s = time.monotonic()
        remembered = self._shadow_obstacle_m if now_s <= self._shadow_until_s else None
        blob = stereo_common.build_blob_scan(
            base,
            angle_min=-self._max_bearing,
            angle_max=self._max_bearing,
            min_range=self._min_range,
            max_range=self._max_range,
            min_height=self._min_height,
            max_height=self._lidar_height,
            ground_x_min=self._ground_x_min,
            ground_x_max=self._ground_x_max,
            drop_z=self._drop_z,
            obstacle_textured=texture[mask] >= self._texture_threshold,
            shadow_obstacle_m=remembered,
            drop_result=stereo_common.ray_drop_check(
                disparity, texture, self._maps['floor'],
                x_min=max(0.15, self._min_range),
                x_max=min(0.72, self._ground_x_max),
                texture_min=self._drop_texture_threshold,
                min_column=self._num_disp,
            ),
        )
        floor_z = blob['floor_z_m']
        if floor_z is not None and abs(floor_z) > float(self.get_parameter('max_floor_offset_m').value):
            self._floor_off_streak += 1
        else:
            self._floor_off_streak = 0
        if self._floor_off_streak >= int(self.get_parameter('floor_check_frames').value):
            # Obstacles are measured against a locally fitted plane and stay.
            blob['drop'] = False
            blob['drop_range_m'] = None
            blob['ranges'] = list(blob['obstacle_ranges'])
            if time.monotonic() - self._last_floor_warn_s > 10.0:
                self._last_floor_warn_s = time.monotonic()
                self.get_logger().warning(
                    f'Stereo floor is {floor_z:+.3f} m from where the mount fields put it; '
                    'drops are OFF until it is back. A large object filling the view does '
                    'this and needs nothing. If it happens on open flat floor, refit with: '
                    'ros2 run turtlebot3_navigation2 stereo_fit_floor.py '
                    f'--ros-args -r __ns:={self.get_namespace()}')
        if blob['obstacle_range_m'] is not None:
            self._shadow_obstacle_m = blob['obstacle_range_m']
            self._shadow_until_s = now_s + 1.0
        close = (
            blob['obstacle_range_m'] is not None
            and blob['obstacle_range_m'] <= self._close_obstacle_m
        )
        self._obstacle_streak = self._obstacle_streak + 1 if close else 0
        self._drop_streak = self._drop_streak + 1 if blob['drop'] else 0
        self._publish_scan(stamp, blob)
        self._publish_cloud(stamp, base)
        self._publish_debug({
            'status': 'ok',
            'calibrated': True,
            'valid_ratio': valid_ratio,
            'row_error_px': self._row_error_px,
            'obstacle_range_m': blob['obstacle_range_m'],
            'floor_z_m': blob['floor_z_m'],
            'drop': blob['drop'],
            'drop_range_m': blob['drop_range_m'],
            'tall_beams_skipped': blob.get('tall_beams_skipped', 0),
            'obstacle_streak': self._obstacle_streak,
            'drop_streak': self._drop_streak,
        })
        now = time.monotonic()
        if (now - self._last_status_s) > 2.0:
            self._last_status_s = now
            obstacle = blob['obstacle_range_m']
            obstacle_text = 'none' if obstacle is None else f'{obstacle:.2f} m'
            drop_text = 'no' if not blob['drop'] else f'yes at {blob["drop_range_m"]:.2f} m'
            row_text = '?' if self._row_error_px is None else f'{self._row_error_px:.1f} px'
            self.get_logger().info(
                f'stereo floor_z={blob["floor_z_m"]:.3f} m obstacle={obstacle_text} '
                f'drop={drop_text} row_error={row_text} '
                f'lidar_height_beams={blob.get("tall_beams_skipped", 0)}'
            )

    def _check_alignment(self, left: np.ndarray, right: np.ndarray) -> None:
        """Measure how far apart matched features sit after rectification."""
        if self._alignment_period <= 0.0:
            return
        now = time.monotonic()
        if self._last_alignment_s and (now - self._last_alignment_s) < self._alignment_period:
            return
        self._last_alignment_s = now
        m = self._maps
        pts_l, pts_r = stereo_common.match_stereo_features(left, right, max_features=1000)
        if len(pts_l) < 30:
            return
        rows = stereo_common.rectified_row_error(
            pts_l, pts_r, m['k1'], m['d1'], m['k2'], m['d2'], m['size'],
            m['rotation'], m['translation'],
        )
        # Most matches are right; the median ignores the odd wrong one.
        error = float(np.median(np.abs(rows)))
        self._row_error_px = error
        if error > self._max_row_error:
            self._misaligned_checks += 1
        else:
            self._misaligned_checks = 0
            if not self._alignment_verified:
                self.get_logger().info(f'Stereo alignment verified ({error:.1f} px); obstacles on')
            self._alignment_verified = True
        was = self._misaligned
        self._misaligned = self._misaligned_checks >= 2
        if self._misaligned:
            self.get_logger().error(
                f'Stereo cameras are out of alignment: matched features are {error:.1f} px '
                f'apart vertically (limit {self._max_row_error:.1f} px). A camera moved since '
                'calibration. Stereo obstacles and drops are OFF. With the robot still, run: '
                'ros2 run turtlebot3_navigation2 stereo_refine_extrinsics.py '
                f'--ros-args -r __ns:={self.get_namespace()}'
            )
        elif was:
            self.get_logger().info(f'Stereo alignment recovered ({error:.1f} px)')

    def _confirmed_marks(self, blob) -> np.ndarray:
        """Hits seen in this frame and the previous one, so one bad frame is not remembered.

        A neighbour within 3 beams and 0.10 m counts, which allows for the
        robot turning between frames. Drops use the node's drop streak.
        """
        now = np.asarray(blob['obstacle_ranges'], dtype=float)
        prev = self._prev_obstacles
        self._prev_obstacles = now
        marks = np.full(now.shape, np.inf)
        if prev is not None and prev.shape == now.shape:
            for i in np.flatnonzero(np.isfinite(now)):
                lo, hi = max(0, i - 3), min(now.size, i + 4)
                if np.any(np.abs(prev[lo:hi] - now[i]) <= 0.10):
                    marks[i] = now[i]
        if blob['drop'] and self._drop_streak >= 2:
            marks = np.minimum(marks, np.asarray(blob['ranges'], dtype=float))
        return marks

    def _publish_scan(self, stamp, blob) -> None:
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self._parent_frame
        msg.range_min = float(self._min_range)
        msg.range_max = float(self._max_range)
        if blob is None:
            self._prev_obstacles = None
            msg.angle_min = -self._max_bearing
            msg.angle_max = self._max_bearing
            msg.angle_increment = 0.0174533
            count = int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
            msg.ranges = [float('inf')] * count
            self._scan_pub.publish(msg)
            self._clear_pub.publish(msg)
            return
        msg.angle_min = float(blob['angle_min'])
        msg.angle_max = float(blob['angle_max'])
        msg.angle_increment = float(blob['angle_increment'])
        marks = self._confirmed_marks(blob)
        msg.ranges = [float(v) for v in marks]
        self._scan_pub.publish(msg)
        # Clear up to a confirmed hit, or up to the farthest floor match on an
        # empty beam. inf (no evidence) leaves the remembered cells alone.
        free = np.asarray(blob['free_ranges'], dtype=float)
        clear = np.where(np.isfinite(marks), marks, free)
        msg.ranges = [float(v) for v in clear]
        self._clear_pub.publish(msg)

    def _publish_cloud(self, stamp, points_base: np.ndarray) -> None:
        pts = np.asarray(points_base, dtype=float)
        if pts.size == 0:
            return
        stride = self._cloud_stride
        sampled = pts[::stride]
        if len(sampled) > 8000:
            sampled = sampled[:8000]
        header = Header()
        header.stamp = stamp
        header.frame_id = self._parent_frame
        cloud = point_cloud2.create_cloud_xyz32(header, sampled.tolist())
        self._points_pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoDepthNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
