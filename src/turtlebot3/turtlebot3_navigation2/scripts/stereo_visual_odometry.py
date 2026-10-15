#!/usr/bin/env python3
"""Frame-to-frame stereo visual odometry for the EKF.

Each matched pair is rectified and corners in the left image get a depth:
floor points from the floor plane the stereo calibration fitted, points above
the floor from a row-strip match into the right image. The next left image is
solved against those 3D points with PnP + RANSAC, which also throws out
points that were not really on the floor. The camera motion is moved into
base_footprint and published as a planar twist (vx, wz) on odom/visual.

The EKF in turtlebot3_bringup fuses that twist next to the wheels and the IMU,
so a wheel that slips or a gyro that drifts is outvoted. Frames the solver does
not trust (few inliers, sideways motion a diff drive cannot make, speeds the
Burger cannot reach) are dropped rather than published with a large variance.
"""

import json
import math
import os
import sys
from collections import deque

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402


def _stamp_ns(msg: Image) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _image_to_mono(msg: Image, width: int, height: int):
    """mono8 / yuv422 / rgb8 Image to a HxW uint8 array without cv_bridge."""
    import cv2
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('mono8', '8uc1'):
        img = data.reshape(msg.height, msg.step)[:, :msg.width]
    elif enc in ('rgb8', 'bgr8'):
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
    elif enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        img = cv2.cvtColor(img, cv2.COLOR_YUV2GRAY_YUYV)
    else:
        raise ValueError(f'unsupported encoding {msg.encoding}')
    if img.shape[1] != width or img.shape[0] != height:
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(img)


class StereoVisualOdometry(Node):
    def __init__(self) -> None:
        super().__init__('stereo_visual_odometry')

        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('odom_topic', 'odom/visual')
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('publish_hz', 6.0)
        self.declare_parameter('sync_slop_sec', 0.03)
        self.declare_parameter('max_features', 200)
        self.declare_parameter('min_feature_distance_px', 8.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 3.0)
        # Floor band trusted for plane depth, measured along the ground.
        self.declare_parameter('floor_min_m', 0.15)
        self.declare_parameter('floor_max_m', 1.5)
        # Rows searched above/below for a stereo match; the pair drifts a few
        # pixels out of row alignment between recalibrations.
        self.declare_parameter('max_row_error_px', 4.0)
        self.declare_parameter('min_inliers', 20)
        self.declare_parameter('ransac_reprojection_px', 1.5)
        # A frame gap longer than this is not one motion step (dropped frames,
        # a stall); the next pair starts a new baseline instead.
        self.declare_parameter('max_frame_gap_sec', 0.5)
        # Burger limits are 0.22 m/s and 2.84 rad/s; anything well above is a
        # bad solve, not motion.
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('max_angular_speed', 3.5)
        # A diff drive cannot slide sideways. Lateral speed above this means
        # the solve mixed in a moving object or a wrong match.
        self.declare_parameter('max_lateral_speed', 0.08)
        # 1-sigma at min_inliers; shrinks with sqrt(inliers / min_inliers).
        self.declare_parameter('linear_sigma', 0.04)
        self.declare_parameter('angular_sigma', 0.08)

        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        calib_param = str(self.get_parameter('calibration_file').value).strip()
        self._calib_path = stereo_common.calibration_file_path(ns, calib_param)
        self._odom_frame = str(self.get_parameter('odom_frame').value).strip() or f'{prefix}odom'
        self._base_frame = str(self.get_parameter('base_frame').value).strip() or f'{prefix}base_footprint'

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._max_features = int(p('max_features'))
        self._min_dist = float(p('min_feature_distance_px'))
        self._min_depth = float(p('min_depth_m'))
        self._max_depth = float(p('max_depth_m'))
        self._max_row_err = float(p('max_row_error_px'))
        self._floor_min_m = float(p('floor_min_m'))
        self._floor_max_m = float(p('floor_max_m'))
        self._min_inliers = max(6, int(p('min_inliers')))
        self._ransac_px = float(p('ransac_reprojection_px'))
        self._max_gap = float(p('max_frame_gap_sec'))
        self._max_v = float(p('max_linear_speed'))
        self._max_w = float(p('max_angular_speed'))
        self._max_vy = float(p('max_lateral_speed'))
        self._sigma_v = float(p('linear_sigma'))
        self._sigma_w = float(p('angular_sigma'))
        self._slop_ns = int(max(0.005, float(p('sync_slop_sec'))) * 1e9)

        self._odom_pub = self.create_publisher(Odometry, str(p('odom_topic')), 10)
        self._debug_pub = self.create_publisher(String, 'stereo/vo_debug', 10)

        self._calibrated = self._load_calibration()
        self._prev = None  # (stamp_ns, left_rect, points_3d Nx3, pixels Nx2)
        self._stats = {'published': 0, 'rejected': 0, 'reason': 'starting'}

        # Same scheme as stereo_depth_node: keep frames serialized and decode
        # only the pair that gets processed.
        self._raw_left = deque(maxlen=3)
        self._raw_right = deque(maxlen=3)
        self.create_subscription(
            Image, str(p('left_image_topic')), self._raw_left.append,
            qos_profile_sensor_data, raw=True)
        self.create_subscription(
            Image, str(p('right_image_topic')), self._raw_right.append,
            qos_profile_sensor_data, raw=True)
        self.create_timer(1.0 / max(1.0, float(p('publish_hz'))), self._on_tick)
        self.create_timer(1.0, self._publish_debug)

        self.get_logger().info(
            f'stereo VO: calibration={self._calib_path} calibrated={self._calibrated} '
            f'-> {p("odom_topic")} ({self._base_frame})'
        )

    def _load_calibration(self) -> bool:
        if not stereo_common.calibration_is_usable(self._calib_path):
            self.get_logger().error(
                f'Stereo calibration missing or unusable: {self._calib_path}. '
                'Visual odometry stays off.')
            self._stats['reason'] = 'uncalibrated'
            return False
        import cv2
        with open(self._calib_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        size = (int(data['image_width']), int(data['image_height']))
        k1 = stereo_common.as_matrix3(data['left_camera_matrix'])
        d1 = stereo_common.as_vector(data['left_distortion'])
        k2 = stereo_common.as_matrix3(data['right_camera_matrix'])
        d2 = stereo_common.as_vector(data['right_distortion'])
        rotation = stereo_common.as_matrix3(data['R'])
        translation = stereo_common.as_vector(data['T']).reshape(3)
        r1, r2, p1, p2, _q, _roi1, _roi2 = cv2.stereoRectify(
            k1, d1, k2, d2, size, rotation, translation.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self._size = size
        self._map_l = cv2.initUndistortRectifyMap(k1, d1, r1, p1, size, cv2.CV_16SC2)
        self._map_r = cv2.initUndistortRectifyMap(k2, d2, r2, p2, size, cv2.CV_16SC2)
        self._k = np.asarray(p1, dtype=float)[:, :3]
        self._fx = float(p1[0, 0])
        self._cx = float(p1[0, 2])
        self._cy = float(p1[1, 2])
        # Rectified baseline: P2[0,3] = -fx * B.
        self._baseline = abs(float(p2[0, 3]) / self._fx)

        baseline_m = float(np.linalg.norm(translation)) or float(data.get('baseline_m', 0.0508))
        optical_r = stereo_common.left_optical_rotation(float(data.get('camera_pitch_rad', 0.354706)))
        optical_t = stereo_common.left_camera_translation(
            float(data.get('camera_x_m', 0.0254)), baseline_m,
            float(data.get('camera_height_m', 0.127)))
        # X_base = optical_r @ R1.T @ X_rect + optical_t (see rectified_to_base).
        a = np.eye(4)
        a[:3, :3] = optical_r @ np.asarray(r1, dtype=float).T
        a[:3, 3] = optical_t
        self._base_from_cam = a
        self._cam_from_base = np.linalg.inv(a)
        self._cv2 = cv2
        self._lk = dict(
            winSize=(15, 15), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
        return True

    def _on_tick(self) -> None:
        if not self._calibrated:
            return
        lefts = [deserialize_message(raw, Image) for raw in self._raw_left]
        rights = [deserialize_message(raw, Image) for raw in self._raw_right]
        best = None
        for left in lefts:
            for right in rights:
                gap = abs(_stamp_ns(left) - _stamp_ns(right))
                if gap <= self._slop_ns and (best is None or gap < best[0]):
                    best = (gap, left, right)
        if best is None:
            return
        self._raw_left.clear()
        self._raw_right.clear()
        try:
            self._process(best[1], best[2])
        except Exception as exc:  # keep running; one bad frame resets the chain
            self._prev = None
            self._reject(f'error: {exc}')

    def _reject(self, reason: str) -> None:
        self._stats['rejected'] += 1
        self._stats['reason'] = reason

    def _floor_depth(self, pixels: np.ndarray) -> np.ndarray:
        """Depth (rect z) where each pixel ray meets the floor, 0 where it does not.

        The cameras look down at the floor, and floor texture (carpet, tile
        grain) repeats too much for a reliable left/right match while tracking
        well from frame to frame. The floor plane from the stereo calibration
        gives those points their depth instead.
        """
        rays = np.stack([
            (pixels[:, 0] - self._cx) / self._fx,
            (pixels[:, 1] - self._cy) / self._fx,
            np.ones(len(pixels)),
        ], axis=1)
        rot = self._base_from_cam[:3, :3]
        origin = self._base_from_cam[:3, 3]
        down = rays @ rot.T
        depth = np.zeros(len(pixels))
        hits = down[:, 2] < -1e-3
        scale = -origin[2] / down[hits, 2]
        ground = origin[:2] + down[hits, :2] * scale[:, None]
        reach = np.linalg.norm(ground, axis=1)
        good = (reach >= self._floor_min_m) & (reach <= self._floor_max_m)
        idx = np.flatnonzero(hits)[good]
        depth[idx] = scale[good]
        return depth

    def _stereo_depth(self, left, right, pixels: np.ndarray) -> np.ndarray:
        """Depth from a row-strip match into the right image, 0 when ambiguous.

        Used for points above the floor. The strip is a few rows tall so a
        small vertical misalignment of the pair still matches, and a match
        whose runner-up scores almost as well is dropped as repeated texture.
        """
        cv2 = self._cv2
        h, w = left.shape
        half = 5
        slack = int(math.ceil(self._max_row_err))
        max_disp = int(self._fx * self._baseline / self._min_depth) + 1
        depth = np.zeros(len(pixels))
        for i, (u, v) in enumerate(pixels):
            ui, vi = int(round(u)), int(round(v))
            if ui - half < 0 or ui + half >= w or vi - half - slack < 0 or vi + half + slack >= h:
                continue
            x0 = max(0, ui - half - max_disp)
            if ui + half + 1 - x0 < 2 * half + 3:
                continue
            patch = left[vi - half:vi + half + 1, ui - half:ui + half + 1]
            strip = right[vi - half - slack:vi + half + slack + 1, x0:ui + half + 1]
            score = cv2.matchTemplate(strip, patch, cv2.TM_CCOEFF_NORMED)
            col_best = score.max(axis=0)
            best = int(np.argmax(col_best))
            peak = float(col_best[best])
            if peak < 0.85:
                continue
            others = np.concatenate([col_best[:max(0, best - 2)], col_best[best + 3:]])
            if others.size and float(others.max()) > peak - 0.05:
                continue
            disparity = ui - (x0 + best + half)
            if 0 < best < len(col_best) - 1:
                a, b, c = col_best[best - 1], col_best[best], col_best[best + 1]
                denom = a - 2 * b + c
                if abs(denom) > 1e-6:
                    disparity -= 0.5 * (a - c) / denom
            if disparity > 1.0:
                depth[i] = self._fx * self._baseline / disparity
        return depth

    def _triangulate(self, left, right):
        """Corners in the rectified left image with their 3D point (rect frame)."""
        cv2 = self._cv2
        corners = cv2.goodFeaturesToTrack(
            left, maxCorners=self._max_features, qualityLevel=0.01,
            minDistance=self._min_dist, blockSize=5)
        if corners is None or len(corners) < self._min_inliers:
            return None, None
        pl = corners.reshape(-1, 2).astype(np.float64)
        depth = self._floor_depth(pl)
        off_floor = np.flatnonzero(depth == 0.0)
        if off_floor.size:
            depth[off_floor] = self._stereo_depth(left, right, pl[off_floor])
        ok = (depth >= self._min_depth) & (depth <= self._max_depth)
        self._stats['floor_pts'] = int(np.count_nonzero(ok) - np.count_nonzero(ok[off_floor]))
        self._stats['stereo_pts'] = int(np.count_nonzero(ok[off_floor]))
        if np.count_nonzero(ok) < self._min_inliers:
            return None, None
        pl = pl[ok]
        z = depth[ok]
        x = (pl[:, 0] - self._cx) * z / self._fx
        y = (pl[:, 1] - self._cy) * z / self._fx
        return np.stack([x, y, z], axis=1).astype(np.float32), pl.astype(np.float32)

    def _process(self, left_msg: Image, right_msg: Image) -> None:
        cv2 = self._cv2
        w, h = self._size
        left = cv2.remap(_image_to_mono(left_msg, w, h), *self._map_l, cv2.INTER_LINEAR)
        right = cv2.remap(_image_to_mono(right_msg, w, h), *self._map_r, cv2.INTER_LINEAR)
        stamp_ns = _stamp_ns(left_msg)

        prev = self._prev
        points, pixels = self._triangulate(left, right)
        self._prev = None if points is None else (stamp_ns, left, points, pixels)
        if prev is None:
            if points is None:
                self._reject('too few stereo features')
            return

        prev_ns, prev_left, prev_points, prev_pixels = prev
        dt = (stamp_ns - prev_ns) * 1e-9
        if dt <= 0.0 or dt > self._max_gap:
            self._reject(f'frame gap {dt:.3f}s')
            return

        # Track previous corners into this frame, keep forward-backward agreement.
        cur, st1, _ = cv2.calcOpticalFlowPyrLK(prev_left, left, prev_pixels.reshape(-1, 1, 2), None, **self._lk)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(left, prev_left, cur, None, **self._lk)
        fb = np.linalg.norm(back.reshape(-1, 2) - prev_pixels, axis=1)
        good = (st1.reshape(-1) == 1) & (st2.reshape(-1) == 1) & (fb < 1.0)
        if np.count_nonzero(good) < self._min_inliers:
            self._reject('too few tracked features')
            return

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            prev_points[good], cur.reshape(-1, 2)[good], self._k, None,
            iterationsCount=60, reprojectionError=self._ransac_px,
            confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE)
        n_in = 0 if inliers is None else len(inliers)
        if not ok or n_in < self._min_inliers:
            self._reject(f'pnp inliers {n_in}')
            return

        # [R|t] maps previous-camera points into the current camera, so the
        # current camera pose in the previous one is its inverse.
        rot, _ = cv2.Rodrigues(rvec)
        cur_from_prev = np.eye(4)
        cur_from_prev[:3, :3] = rot
        cur_from_prev[:3, 3] = tvec.reshape(3)
        motion = self._base_from_cam @ np.linalg.inv(cur_from_prev) @ self._cam_from_base

        vx = motion[0, 3] / dt
        vy = motion[1, 3] / dt
        wz = math.atan2(motion[1, 0], motion[0, 0]) / dt
        if abs(vx) > self._max_v or abs(wz) > self._max_w:
            self._reject(f'implausible speed vx={vx:.2f} wz={wz:.2f}')
            return
        if abs(vy) > self._max_vy:
            self._reject(f'lateral slip vy={vy:.2f}')
            return

        scale = math.sqrt(self._min_inliers / float(n_in))
        var_v = (self._sigma_v * scale) ** 2
        var_w = (self._sigma_w * scale) ** 2

        msg = Odometry()
        msg.header.stamp = left_msg.header.stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = 1e6
        msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e6
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.angular.z = float(wz)
        cov = [0.0] * 36
        cov[0] = var_v
        cov[7] = var_v  # vy is not fused; keep it finite for viewers
        cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = var_w
        msg.twist.covariance = cov
        self._odom_pub.publish(msg)

        self._stats.update({
            'published': self._stats['published'] + 1,
            'reason': 'ok',
            'inliers': int(n_in),
            'vx': round(float(vx), 4),
            'vy': round(float(vy), 4),
            'wz': round(float(wz), 4),
            'dt': round(dt, 3),
        })

    def _publish_debug(self) -> None:
        msg = String()
        msg.data = json.dumps(dict(self._stats, calibrated=self._calibrated))
        self._debug_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoVisualOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
