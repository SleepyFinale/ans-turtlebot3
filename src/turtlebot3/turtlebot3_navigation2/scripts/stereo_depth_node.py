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
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
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
        # 2.5 Hz: at 0.22 m/s the robot moves 9 cm between frames, well inside
        # the 0.28-1.10 m obstacle band. At 4 Hz this node took about a core of the Pi,
        # which left Nav2's controller missing its 10 Hz loop.
        self.declare_parameter('publish_hz', 2.5)
        self.declare_parameter('sync_slop_sec', 0.03)
        # About where the lowest image row meets the floor (0.20 m from Inky's
        # cameras). It sets the disparity levels, and with full_width the
        # padding: rectified focal length x baseline / min_range, so 80 levels
        # with Inky's 2026-10-22 calibration, where 0.12 m needed 128.
        self.declare_parameter('min_range_m', 0.20)
        self.declare_parameter('max_range_m', 1.5)
        # Obstacles and clearing only within this bearing of straight ahead.
        # Near the edge of the ~120 deg lenses the calibration's lens model
        # misses by a few pixels of disparity: on inky a printer 1.7 m away at
        # 50 deg to the side was placed at 1.2 m as a low obstacle. The LiDAR
        # covers the sides; checked against it with stereo_fit_offset.py,
        # disparity is within 0.5 px out to about 41 deg.
        self.declare_parameter('max_bearing_deg', 45.0)
        self.declare_parameter('min_obstacle_height_m', 0.025)
        # Low obstacles are reported between these distances ahead. Closer
        # than ~0.28 m the view no longer spans the robot's width; past
        # obstacle_far_x_m the needed height rises to min_obstacle_height_far_m
        # (see stereo_common.build_blob_scan). At 0.80 m the robot met a chair
        # base already inside the band's near edge before two frames confirmed it.
        self.declare_parameter('obstacle_near_x_m', 0.28)
        self.declare_parameter('obstacle_far_x_m', 1.10)
        self.declare_parameter('min_obstacle_height_far_m', 0.04)
        # Match the full width of the left image (stereo_common.padded_disparity).
        self.declare_parameter('full_width', True)
        # Image rows used for 3D points (1 = all).
        self.declare_parameter('point_row_stride', 2)
        # No pair processed for this long: publish empty scans and report
        # status no_images (see _check_images).
        self.declare_parameter('images_timeout_sec', 2.0)
        # base_scan is 0.172 m above base_link, and base_link is 0.010 m above
        # base_footprint, so the lidar plane is 0.182 m above the ground.
        self.declare_parameter('lidar_height_m', 0.182)
        # Stereo hits with a LiDAR return on the same bearing, from 0.10 m
        # nearer to lidar_match_m further (scan_topic_lidar, at most
        # lidar_max_age_sec old), are left to the LiDAR. lidar_x_m is the
        # LiDAR's offset ahead of base_footprint (the Burger's is behind).
        self.declare_parameter('scan_topic_lidar', 'scan')
        self.declare_parameter('lidar_x_m', -0.032)
        self.declare_parameter('lidar_match_m', 0.25)
        self.declare_parameter('lidar_max_age_sec', 1.0)
        # Obstacles and drops are measured against the floor where the mount
        # calibration puts it, which only holds while the robot is level.
        # Rolling over a floor cable tipped Inky 7 deg nose-up; the floor then
        # seemed to fall away and stereo reported drops at 0.40-0.45 m that
        # stopped it. Above max_tilt_deg (accelerometer, imu_topic) a frame
        # publishes nothing.
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('max_tilt_deg', 4.0)
        self.declare_parameter('ground_x_min_m', 0.22)
        self.declare_parameter('ground_x_max_m', 1.20)
        self.declare_parameter('drop_z_m', -0.15)
        # Travel between the two sightings that confirm a drop
        # (_drop_confirmed): ~2 frames at full speed. A drop is first seen
        # by 0.72 m, so it is confirmed with ~0.5 m still to go.
        self.declare_parameter('drop_confirm_move_m', 0.12)
        self.declare_parameter('min_valid_ratio', 0.05)
        self.declare_parameter('cloud_stride', 4)
        # 0 = just enough disparity levels to reach min_range_m with this
        # calibration (rectified focal length x baseline / min_range, rounded
        # up to a multiple of 16, at most 128). SGBM time grows with the
        # number of levels; a fixed 128 took 168 ms a pair on the Pi where 80
        # covers the same range.
        self.declare_parameter('num_disparities', 0)
        self.declare_parameter('block_size', 5)
        self.declare_parameter('close_obstacle_m', 0.45)
        # The stop gate (obstacle_streak / path_obstacle_m) only counts
        # confirmed marks within this distance of the robot's centre line.
        # The nearest hit anywhere in the +-45 deg view stopped Inky about
        # every 6 s while exploring, mostly at door jambs and furniture well
        # to the side of its path (the Burger is 0.18 m wide). The costmap
        # still gets every mark, so the planner steers around those.
        self.declare_parameter('path_half_width_m', 0.14)
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
        # Self-alignment, for a camera mount that is not rigid. The file stays
        # the baseline; drift is re-fitted in memory (and again after every
        # restart) with the same methods as stereo_refine_extrinsics.py,
        # stereo_fit_offset.py and stereo_fit_floor.py, only while the robot
        # is still (odom_topic; with no odometry at all it counts as still):
        #  - rows: re-fit pitch/roll between the cameras once the row error
        #    passes realign_row_trigger_px, before max_row_error_px turns
        #    obstacles off;
        #  - range: move the right principal point when the disparity of
        #    LiDAR returns 0.6-3 m away is off by more than offset_trigger_px;
        #  - floor: re-fit camera height and pitch when the floor check fails
        #    with the robot level (IMU), within floor_refit_max_* of the file
        #    so a ramp is never taken for the floor.
        self.declare_parameter('self_align', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('realign_row_trigger_px', 1.0)
        self.declare_parameter('realign_accept_px', 1.0)
        self.declare_parameter('realign_min_matches', 300)
        self.declare_parameter('offset_trigger_px', 0.5)
        self.declare_parameter('offset_min_samples', 150)
        self.declare_parameter('offset_max_total_px', 4.0)
        self.declare_parameter('floor_refit_frames', 6)
        self.declare_parameter('floor_refit_max_height_m', 0.03)
        self.declare_parameter('floor_refit_max_pitch_deg', 5.0)
        # Stereo visual odometry on the same pairs, published on odom/visual
        # for the bringup EKF. It has its own row-error tolerance, so it keeps
        # running while misalignment turns obstacle marking off.
        self.declare_parameter('visual_odometry', False)
        self.declare_parameter('visual_odom_topic', 'odom/visual')
        self.declare_parameter('visual_odometry_hz', 3.0)
        self.declare_parameter('odom_frame', '')
        # Depth and VO rate while the robot has stood still for a second
        # (odometry). This node took ~1.2 cores of Inky's Pi while exploring,
        # and the robot spends much of a run waiting for goals and plans;
        # pairs of an unchanged view add nothing. Full rate resumes on the
        # first tick after odometry shows motion. 0 keeps the full rate.
        self.declare_parameter('idle_hz', 1.0)

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
        self._point_row_stride = max(1, int(self.get_parameter('point_row_stride').value))
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
        self._path_half_width = float(self.get_parameter('path_half_width_m').value)
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
        self._self_align = bool(self.get_parameter('self_align').value)
        self._realign_pts = []  # (left Nx2, right Nx2) raw matches gathered while still
        self._offset_gaps = []  # measured - expected disparity at LiDAR returns
        self._offset_frames = 0
        # Gap from the previous batch; a correction needs two that agree.
        self._offset_pending = None
        self._floor_pts = []  # lower-half points in the rectified left frame
        self._last_moving_s = 0.0
        self._odom_seen = False
        self._odom_pose = None  # (x, y, yaw) in the odom frame
        self._drop_hits = []    # (time, robot x, y, edge x, y) of recent drop sightings
        self._drop_prev = False
        self._calib = None
        self._calib_file = None

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
        self._pad = 0
        # 40 s of grace while the cameras lock focus and start streaming (under
        # the full launch on Inky, pairs first matched in time after ~25 s).
        self._last_pair_s = time.monotonic() + 40.0
        self._last_image_s = {'left': time.monotonic() + 40.0, 'right': time.monotonic() + 40.0}
        self._images_lost = False
        self._last_images_warn_s = 0.0
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
        self._last_vo_s = 0.0
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
        self._lidar = None  # (monotonic time, Nx2 points in the base frame)
        # (monotonic time, ax, ay, az); ~30 s at the IMU's 20 Hz for the resting attitude.
        self._accel = deque(maxlen=600)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            lambda m: self._accel.append((time.monotonic(), m.linear_acceleration.x,
                                          m.linear_acceleration.y, m.linear_acceleration.z)),
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic_lidar').value), self._on_lidar,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10)
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
            _r1, _r2, _p1, _p2, _q, _roi1, _roi2 = cv2.stereoRectify(
                k1, d1, k2, d2, size, rotation, translation.reshape(3, 1),
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0,
            )
            num_disp = int(self.get_parameter('num_disparities').value)
            if num_disp <= 0:
                focal_baseline = abs(float(_p2[0, 3]))  # rectified f x baseline (px m)
                needed = focal_baseline / max(0.05, self._min_range)
                num_disp = min(128, int(math.ceil(needed / 16.0)) * 16)
            num_disp = max(16, num_disp - (num_disp % 16))
            self._num_disp = num_disp
            self._pad = num_disp if bool(self.get_parameter('full_width').value) else 0
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
            self._cv2 = cv2
            # The file is the baseline; self_align replaces these at runtime.
            self._calib = {
                'size': size, 'k1': k1, 'd1': d1, 'k2': k2, 'd2': d2,
                'rotation': rotation, 'translation': translation,
                'pitch': float(data.get('camera_pitch_rad', 0.354706)),
                'height': float(data.get('camera_height_m', 0.127)),
                'camera_x': float(data.get('camera_x_m', 0.0254)),
                'baseline_m': float(data.get('baseline_m', 0.0508)),
            }
            self._calib_file = dict(self._calib)
            self._matcher = matcher
            self._maps = self._build_maps(self._calib)
        except Exception as exc:
            self.get_logger().error(f'Failed to load stereo calibration {self._calib_path}: {exc}')
            return

        self._calibrated = True
        self.get_logger().info(f'Loaded stereo calibration {self._calib_path}')

    def _build_maps(self, c: dict) -> dict:
        """Rectification maps and floor model for one set of calibration values."""
        cv2 = self._cv2
        size = c['size']
        r1, r2, p1, p2, q, _roi1, _roi2 = cv2.stereoRectify(
            c['k1'], c['d1'], c['k2'], c['d2'], size, c['rotation'],
            np.asarray(c['translation'], dtype=float).reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
        )
        map1 = cv2.initUndistortRectifyMap(c['k1'], c['d1'], r1, p1, size, cv2.CV_32FC1)
        map2 = cv2.initUndistortRectifyMap(c['k2'], c['d2'], r2, p2, size, cv2.CV_32FC1)
        baseline = float(np.linalg.norm(c['translation']))
        if baseline < 1e-4:
            baseline = c['baseline_m']
        optical_r = stereo_common.left_optical_rotation(c['pitch'])
        optical_t = stereo_common.left_camera_translation(c['camera_x'], baseline, c['height'])
        right_t, right_r = stereo_common.right_camera_pose_in_left(c['rotation'], c['translation'])
        self._publish_static_tf(optical_r, optical_t, right_r, right_t)
        return {
            'size': size,
            'map1': map1,
            'map2': map2,
            'q': q,
            'p1': np.asarray(p1, dtype=float),
            'p2': np.asarray(p2, dtype=float),
            'baseline': baseline,
            'floor': stereo_common.expected_floor_disparity(
                size, p1, r1, optical_r, optical_t, baseline,
            ),
            'r1': np.asarray(r1, dtype=float),
            'matcher': self._matcher,
            'optical_r': optical_r,
            'optical_t': optical_t,
            'k1': c['k1'], 'd1': c['d1'], 'k2': c['k2'], 'd2': c['d2'],
            'rotation': c['rotation'],
            'translation': c['translation'],
        }

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
            self._check_images()
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

    def _check_images(self) -> None:
        """While no pair has been processed for images_timeout_sec, publish empty scans.

        Nav2 re-applies a source's last observation until a new one arrives.
        When a camera dropped off USB mid-run, the last stereo marks and
        clears were replayed at the spot where it died for the rest of the
        run. Empty scans with current stamps replace them: no marks, and no
        clearing of what stereo can no longer see.
        """
        now = time.monotonic()
        timeout = float(self.get_parameter('images_timeout_sec').value)
        if now - self._last_pair_s <= timeout:
            if self._images_lost:
                self._images_lost = False
                self.get_logger().info('Stereo images are back')
            return
        self._publish_scan(self.get_clock().now().to_msg(), None)
        missing = [side for side, t in self._last_image_s.items() if now - t > timeout]
        self._publish_debug({
            'status': 'no_images',
            'calibrated': True,
            'missing': missing,
            'obstacle_range_m': None,
            'drop': False,
            'drop_range_m': None,
            'obstacle_streak': 0,
            'drop_streak': 0,
        })
        if not self._images_lost or now - self._last_images_warn_s > 30.0:
            self._last_images_warn_s = now
            self.get_logger().error(
                f'No stereo pairs for {now - self._last_pair_s:.0f} s '
                f'(no frames from: {", ".join(missing) or "neither; pairs do not match in time"}). '
                'Stereo obstacles are OFF; the LiDAR still works. A camera that dropped off '
                'USB ("No such device" from v4l2_camera) needs its cable reseated and the stack '
                'restarted.')
        self._images_lost = True

    def _on_raw(self, raw, queue, side: str) -> None:
        self._last_image_s[side] = time.monotonic()
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

    def _idle_skip(self, last_s: float) -> bool:
        """True while still and the last frame was processed less than 1/idle_hz ago."""
        # Without odometry _still() is always true; never slow down then.
        idle_hz = float(self.get_parameter('idle_hz').value)
        return (
            idle_hz > 0.0 and self._odom_seen and self._still()
            and time.monotonic() - last_s < 1.0 / idle_hz)

    def _on_vo_tick(self) -> None:
        if self._idle_skip(self._last_vo_s):
            return
        best = self._best_pair(list(self._vo_left), list(self._vo_right))
        if best is None:
            return
        self._vo_left.clear()
        self._vo_right.clear()
        self._last_vo_s = time.monotonic()
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
        # _last_pair_s also feeds the 2 s images timeout; idle_hz stays above it.
        if self._idle_skip(self._last_pair_s):
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

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_seen = True
        twist = msg.twist.twist
        if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.02:
            self._last_moving_s = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_pose = (p.x, p.y, yaw)

    def _drop_confirmed(self, drop: bool, drop_range) -> bool:
        """A drop counts once two sightings taken drop_confirm_move_m apart in
        travel put the edge at the same odometry-frame spot.

        A real edge stays put while the robot approaches it. On Inky, drops
        "at 0.50-0.55 m" followed the robot across carpet and doorway
        thresholds (18 stops in one run) at a fixed place in the image.
        Without odometry, two frames in a row confirm, as before.
        """
        now = time.monotonic()
        self._drop_hits = [h for h in self._drop_hits if now - h[0] <= 4.0]
        prev, self._drop_prev = self._drop_prev, bool(drop and drop_range is not None)
        if not drop or drop_range is None:
            return False
        if self._odom_pose is None:
            return prev
        rx, ry, yaw = self._odom_pose
        ex, ey = rx + drop_range * math.cos(yaw), ry + drop_range * math.sin(yaw)
        move = float(self.get_parameter('drop_confirm_move_m').value)
        # An artefact moves with the robot, so its edge shifts as far as the
        # robot did; a real edge shifts only by the 5 cm range bins.
        ok = False
        for h in self._drop_hits:
            moved = math.hypot(rx - h[1], ry - h[2])
            if moved >= move and math.hypot(ex - h[3], ey - h[4]) <= min(0.12, 0.5 * moved):
                ok = True
                break
        self._drop_hits.append((now, rx, ry, ex, ey))
        return ok

    def _still(self) -> bool:
        """Stopped for at least a second. The cameras are not synchronized, so
        frames taken while moving are offset and would corrupt a re-fit."""
        if not self._odom_seen:
            return True
        return time.monotonic() - self._last_moving_s > 1.0

    def _on_lidar(self, msg: LaserScan) -> None:
        # Converted only when a stereo frame uses it (publish_hz, not 10 Hz).
        self._lidar = (time.monotonic(), msg)

    def _tilt_deg(self):
        """Tilt over the last 0.5 s against the robot's resting attitude, or None without an IMU.

        The resting attitude is the median accelerometer reading over the last
        30 s (the IMU on Inky sits ~1.5 deg off level). Measured from level,
        that offset plus the robot speeding up or braking (0.4-0.7 m/s^2,
        2-4 deg) read as tilted in 20% of frames while exploring, and those
        frames published no obstacles.
        """
        now = time.monotonic()
        # The IMU callback appends from another executor thread; list() copies
        # the deque in one step, a comprehension over it can see it change.
        samples = np.asarray(list(self._accel), dtype=float).reshape(-1, 4)
        recent = samples[now - samples[:, 0] <= 0.5]
        if len(recent) < 3:
            return None
        cur = recent[:, 1:].mean(axis=0)
        rest = np.median(samples[:, 1:], axis=0)
        if len(samples) < 50 or not np.all(np.isfinite(rest)):
            rest = np.array([0.0, 0.0, 1.0])
        cos_a = float(cur @ rest) / max(1e-9, float(np.linalg.norm(cur) * np.linalg.norm(rest)))
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))

    def _lidar_points(self):
        if self._lidar is None:
            return None
        if time.monotonic() - self._lidar[0] > float(self.get_parameter('lidar_max_age_sec').value):
            return None
        msg = self._lidar[1]
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(0.05, msg.range_min)) & (r < 2.0)
        x0 = float(self.get_parameter('lidar_x_m').value)
        return np.stack([r[ok] * np.cos(a[ok]) + x0, r[ok] * np.sin(a[ok])], axis=1)

    def _process_pair(self, left_msg: Image, right_msg: Image) -> None:
        self._last_pair_s = time.monotonic()
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
        tilt = self._tilt_deg()
        if tilt is not None and tilt > float(self.get_parameter('max_tilt_deg').value):
            self._obstacle_streak = 0
            self._drop_streak = 0
            self._publish_scan(stamp, None)
            self._publish_debug({
                'status': 'tilted',
                'calibrated': True,
                'tilt_deg': tilt,
                'obstacle_range_m': None,
                'drop': False,
                'drop_range_m': None,
                'obstacle_streak': 0,
                'drop_streak': 0,
            })
            return
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
        disparity = stereo_common.padded_disparity(
            self._maps['matcher'], left_r, right_r, self._pad)
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
        # Every point costs Python-side numpy time on the Pi (~50k a frame);
        # alternate rows keep ~20 hits per 1 deg beam on a caster 1 m away.
        stride = self._point_row_stride
        if stride > 1:
            mask[np.arange(mask.shape[0]) % stride != 0, :] = False
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
            lidar_xy=self._lidar_points(),
            lidar_match_m=float(self.get_parameter('lidar_match_m').value),
            min_range=self._min_range,
            max_range=self._max_range,
            min_height=self._min_height,
            max_height=self._lidar_height,
            obstacle_x_min=float(self.get_parameter('obstacle_near_x_m').value),
            obstacle_x_max=float(self.get_parameter('obstacle_far_x_m').value),
            min_height_far=float(self.get_parameter('min_obstacle_height_far_m').value),
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
                min_column=0 if self._pad else self._num_disp,
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
        if self._self_align:
            self._self_align_frame(disparity)
        if blob['obstacle_range_m'] is not None:
            self._shadow_obstacle_m = blob['obstacle_range_m']
            self._shadow_until_s = now_s + 1.0
        # A confirmed drop counts as two frames at once: the move between the
        # two sightings already took a frame.
        if self._drop_confirmed(blob['drop'], blob['drop_range_m']):
            self._drop_streak = max(2, self._drop_streak + 1)
        else:
            self._drop_streak = 0
        marks = self._confirmed_marks(blob)
        path_m, path_bearing = self._path_obstacle(blob, marks)
        close = path_m is not None and path_m <= self._close_obstacle_m
        self._obstacle_streak = self._obstacle_streak + 1 if close else 0
        self._publish_scan(stamp, blob, marks)
        self._publish_cloud(stamp, base)
        self._publish_debug({
            'status': 'ok',
            'calibrated': True,
            'valid_ratio': valid_ratio,
            'row_error_px': self._row_error_px,
            'obstacle_range_m': blob['obstacle_range_m'],
            'path_obstacle_m': path_m,
            'path_bearing_deg': path_bearing,
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
            if path_m is not None:
                obstacle_text += f' path={path_m:.2f} m@{path_bearing:+.0f}deg'
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
        # While gathering matches for a re-fit, check every second instead.
        period = 1.0 if self._realign_pts else self._alignment_period
        if self._last_alignment_s and (now - self._last_alignment_s) < period:
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
        if (
            self._self_align
            and error > float(self.get_parameter('realign_row_trigger_px').value)
            and self._still()
        ):
            if self._realign_rows(pts_l, pts_r):
                return
        else:
            # Moving, or aligned again: matches gathered so far no longer belong together.
            self._realign_pts = []
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

    def _realign_rows(self, pts_l: np.ndarray, pts_r: np.ndarray) -> bool:
        """Gather matches; once there are enough, re-fit the pose between the cameras.

        Returns True when a new alignment was applied.
        """
        cv2 = self._cv2
        c = self._calib
        norm_l = cv2.undistortPoints(pts_l.reshape(-1, 1, 2), c['k1'], c['d1']).reshape(-1, 2)
        norm_r = cv2.undistortPoints(pts_r.reshape(-1, 1, 2), c['k2'], c['d2']).reshape(-1, 2)
        _e, mask = cv2.findEssentialMat(
            norm_l, norm_r, np.eye(3), cv2.RANSAC, 0.999, 1.5 / float(c['k1'][0, 0]))
        if mask is None:
            return False
        keep = mask.ravel() > 0
        self._realign_pts.append((pts_l[keep], pts_r[keep]))
        total = sum(len(a) for a, _ in self._realign_pts)
        if total < int(self.get_parameter('realign_min_matches').value):
            return False
        all_l = np.vstack([a for a, _ in self._realign_pts])
        all_r = np.vstack([b for _, b in self._realign_pts])
        self._realign_pts = []
        rotation, translation, before, after, inliers = stereo_common.refine_extrinsics(
            all_l, all_r, c['k1'], c['d1'], c['k2'], c['d2'], c['size'],
            c['rotation'], c['translation'],
        )
        accept = float(self.get_parameter('realign_accept_px').value)
        if not np.isfinite(after) or after > accept or after >= before or inliers < 100:
            self.get_logger().warning(
                f'Stereo self-alignment: rows {before:.2f} px -> {after:.2f} px is not good '
                f'enough (need <= {accept:.2f} px); keeping the current alignment. If this '
                'repeats, the lens calibration needs redoing (stereo_calibrate.py).')
            return False
        candidate = dict(c, rotation=rotation, translation=np.asarray(translation).reshape(3))
        ok, reason = stereo_common.calibration_data_usable({
            **{k: v for k, v in self._file_payload().items()},
            'R': [float(v) for v in np.asarray(rotation).reshape(-1)],
            'T': [float(v) for v in candidate['translation']],
        })
        if not ok:
            self.get_logger().warning(f'Stereo self-alignment rejected ({reason})')
            return False
        self._apply_calib(candidate)
        self._misaligned_checks = 0
        self._misaligned = False
        self._alignment_verified = True
        self._row_error_px = after
        self.get_logger().info(
            f'Stereo self-aligned: rows {before:.2f} px -> {after:.2f} px over {inliers} matches '
            '(in memory; the calibration file is unchanged)')
        return True

    def _self_align_frame(self, disparity: np.ndarray) -> None:
        """Range-scale and floor checks on an aligned frame; re-fit when they drift."""
        if not self._still():
            self._offset_gaps = []
            self._offset_frames = 0
            self._floor_pts = []
            return
        self._check_range_offset(disparity)
        floor_frames = int(self.get_parameter('floor_check_frames').value)
        tilt = self._tilt_deg()
        if self._floor_off_streak >= floor_frames and tilt is not None and tilt < 1.5:
            self._refit_floor(disparity)
        else:
            self._floor_pts = []

    def _check_range_offset(self, disparity: np.ndarray) -> None:
        """Compare SGBM disparity at LiDAR returns with the disparity their range implies.

        A small sideways turn of one camera adds the same few pixels of
        disparity everywhere, which row alignment cannot see; it places
        everything too near or too far. As in stereo_fit_offset.py it is removed
        by moving the right camera's principal point.
        """
        lidar = self._lidar_points()
        if lidar is None or len(lidar) == 0:
            return
        m = self._maps
        rng = np.hypot(lidar[:, 0], lidar[:, 1])
        lidar = lidar[(rng > 0.6) & (rng < 3.0)]
        if len(lidar) == 0:
            return
        pts = np.column_stack([lidar, np.full(len(lidar), self._lidar_height)])
        rect = ((pts - m['optical_t']) @ m['optical_r']) @ m['r1'].T
        rect = rect[rect[:, 2] > 0.3]
        p1 = m['p1']
        f, cx, cy = float(p1[0, 0]), float(p1[0, 2]), float(p1[1, 2])
        baseline = abs(float(m['p2'][0, 3]) / float(m['p2'][0, 0]))
        u = np.rint(f * rect[:, 0] / rect[:, 2] + cx).astype(int)
        v = np.rint(f * rect[:, 1] / rect[:, 2] + cy).astype(int)
        expected = f * baseline / rect[:, 2]
        height, width = disparity.shape
        inside = (
            (u >= self._num_disp + 2) & (u < width - 2) & (v >= 8) & (v < height - 8)
            & (np.abs(rect[:, 0]) <= math.tan(self._max_bearing) * rect[:, 2])
        )
        for ui, vi, de in zip(u[inside], v[inside], expected[inside]):
            # Walls are vertical, so rows around the LiDAR plane see the same surface.
            w = disparity[vi - 8:vi + 9, ui - 1:ui + 2]
            w = w[w > 0.0]
            if w.size >= 8:
                self._offset_gaps.append(float(np.median(w)) - de)
        self._offset_frames += 1
        if len(self._offset_gaps) < int(self.get_parameter('offset_min_samples').value) \
                or self._offset_frames < 5:
            return
        gap = float(np.median(self._offset_gaps))
        samples = len(self._offset_gaps)
        self._offset_gaps = []
        self._offset_frames = 0
        trigger = float(self.get_parameter('offset_trigger_px').value)
        prev, self._offset_pending = self._offset_pending, gap
        if abs(gap) <= trigger:
            return
        # One batch is not enough: LiDAR returns off chair legs and table
        # edges sample the background in the disparity window. On Inky single
        # batches swung between -1.2 and +2.7 px (and +15 px) within a minute
        # and each was applied, moving every stereo depth by ~0.2 m per pixel
        # at 1 m: floor then read as obstacles or drops. Only two batches in a
        # row on the same side move cx, and only by the smaller of the two.
        if prev is None or abs(prev) <= trigger or (prev > 0) != (gap > 0):
            return
        gap = math.copysign(min(abs(gap), abs(prev)), gap)
        self._offset_pending = None
        # How far the right rectified image moves per pixel of right cx, at the centre.
        c = self._calib
        cv2 = self._cv2
        centre = np.array([[[c['size'][0] / 2.0, c['size'][1] / 2.0]]])
        r1, r2, p1n, p2n, _q, _a, _b = stereo_common._rectify(
            c['k1'], c['d1'], c['k2'], c['d2'], c['size'], c['rotation'], c['translation'])
        k2b = c['k2'].copy()
        k2b[0, 2] += 1.0
        x0 = cv2.undistortPoints(centre, c['k2'], c['d2'], R=r2, P=p2n)[0, 0, 0]
        x1 = cv2.undistortPoints(centre, k2b, c['d2'], R=r2, P=p2n)[0, 0, 0]
        if abs(x1 - x0) < 1e-3:
            return
        # disparity = x_left - x_right, so it changes by -(x1 - x0) per pixel of cx.
        correction = gap / float(x1 - x0)
        k2 = c['k2'].copy()
        k2[0, 2] += correction
        total = float(k2[0, 2] - self._calib_file['k2'][0, 2])
        limit = float(self.get_parameter('offset_max_total_px').value)
        if abs(total) > limit:
            self.get_logger().warning(
                f'Stereo range scale is off by {gap:+.2f} px against the LiDAR, but fixing it '
                f'needs the right cx moved {total:+.1f} px from the file (limit {limit:.1f}); '
                'recalibrate with stereo_calibrate.py.')
            return
        self._apply_calib(dict(c, k2=k2))
        self.get_logger().info(
            f'Stereo self-aligned: disparity was {gap:+.2f} px off the LiDAR over {samples} '
            f'samples; right cx moved {correction:+.2f} px ({total:+.2f} px from the file, in memory)')

    def _refit_floor(self, disparity: np.ndarray) -> None:
        """Re-fit camera height and pitch, as stereo_fit_floor.py does, when the floor check fails."""
        cv2 = self._cv2
        disp = disparity.copy()
        disp[: disp.shape[0] // 2, :] = 0.0  # floor is in the lower half
        xyz = cv2.reprojectImageTo3D(disp, self._maps['q'])
        pts = xyz[disp > 1.0]
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[(pts[:, 2] > self._min_range) & (pts[:, 2] < self._max_range * 1.2)]
        if len(pts) < 200:
            return
        self._floor_pts.append(pts[:: max(1, len(pts) // 2000)])
        if len(self._floor_pts) < int(self.get_parameter('floor_refit_frames').value):
            return
        pts = np.concatenate(self._floor_pts)
        self._floor_pts = []
        fit = stereo_common.plane_ransac(pts)
        if fit is None:
            return
        n, height, ratio = fit
        pitch, residual = stereo_common.pitch_for_floor_normal(self._maps['r1'], n)
        base = self._calib_file
        dh = height - base['height']
        dp = math.degrees(pitch - base['pitch'])
        ok = (
            ratio >= 0.6 and residual <= math.radians(3.0)
            and abs(dh) <= float(self.get_parameter('floor_refit_max_height_m').value)
            and abs(dp) <= float(self.get_parameter('floor_refit_max_pitch_deg').value)
        )
        if not ok:
            self.get_logger().warning(
                f'Stereo floor re-fit not applied: plane {ratio:.0%} inliers, height {dh:+.3f} m '
                f'and pitch {dp:+.1f} deg from the file, side tilt {math.degrees(residual):.1f} deg. '
                'Not open level floor, or the camera moved too far; if the floor is flat, run '
                'stereo_fit_floor.py.')
            return
        self._apply_calib(dict(self._calib, height=float(height), pitch=float(pitch)))
        self.get_logger().info(
            f'Stereo self-aligned: floor re-fitted, camera height {height:.4f} m '
            f'({dh:+.3f} m), pitch {pitch:.4f} rad ({dp:+.1f} deg) from the file, in memory')

    def _file_payload(self) -> dict:
        with open(self._calib_path, 'r', encoding='utf-8') as handle:
            return yaml.safe_load(handle) or {}

    def _apply_calib(self, calib: dict) -> None:
        """Swap in new calibration values; the tick callback is single-threaded."""
        self._calib = calib
        self._maps = self._build_maps(calib)
        self._prev_obstacles = None
        self._floor_off_streak = 0
        self._offset_gaps = []
        self._offset_frames = 0
        self._floor_pts = []

    def _confirmed_marks(self, blob) -> np.ndarray:
        """Hits seen in this frame and the previous one, so one bad frame is not remembered.

        A neighbour within 3 beams counts, which allows for the robot turning
        between frames, within 0.12 m plus a tenth of the range: at 0.22 m/s
        the robot closes 9 cm per frame, and at 1 m one pixel of disparity is
        already 0.2 m of range. Drops use the node's drop streak.
        """
        now = np.asarray(blob['obstacle_ranges'], dtype=float)
        prev = self._prev_obstacles
        self._prev_obstacles = now
        marks = np.full(now.shape, np.inf)
        if prev is not None and prev.shape == now.shape:
            for i in np.flatnonzero(np.isfinite(now)):
                lo, hi = max(0, i - 3), min(now.size, i + 4)
                if np.any(np.abs(prev[lo:hi] - now[i]) <= 0.12 + 0.10 * now[i]):
                    marks[i] = now[i]
        if blob['drop'] and self._drop_streak >= 2:
            marks = np.minimum(marks, np.asarray(blob['ranges'], dtype=float))
        return marks

    def _path_obstacle(self, blob, marks: np.ndarray):
        """Nearest confirmed mark (range m, bearing deg) inside the robot's path, or (None, None).

        Drops are left out: the depth node reports them through drop_streak.
        """
        obstacles = np.asarray(blob['obstacle_ranges'], dtype=float)
        hits = np.isfinite(marks) & np.isfinite(obstacles)
        if not np.any(hits):
            return None, None
        idx = np.flatnonzero(hits)
        ang = float(blob['angle_min']) + idx * float(blob['angle_increment'])
        rng = marks[idx]
        inside = np.abs(rng * np.sin(ang)) <= self._path_half_width
        if not np.any(inside):
            return None, None
        k = int(np.argmin(np.where(inside, rng * np.cos(ang), np.inf)))
        return float(rng[k] * math.cos(ang[k])), math.degrees(float(ang[k]))

    def _publish_scan(self, stamp, blob, marks=None) -> None:
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
        if marks is None:
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
        # Packing up to 8000 points one by one costs ~36 ms a frame on the
        # Pi; nothing on the robot reads stereo/points (it is for RViz).
        if self._points_pub.get_subscription_count() == 0:
            return
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
