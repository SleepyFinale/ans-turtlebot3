#!/usr/bin/env python3
"""Fit camera_height_m and camera_pitch_rad in config/stereo/<namespace>.yaml to the floor.

Run after stereo_calibrate.py, with the robot on open, flat floor and nothing
in the lower half of the cameras' view, while stereo.launch.py (or the full
navigation launch) is publishing. A new chessboard calibration changes the
rectification, so mount values fitted with the old one no longer put the
floor at z = 0: the depth node then reports floor_z a few centimetres off and
reads the floor falling away as a drop across the corridor.

Floor points come from SGBM on the lower part of the rectified pair. A
RANSAC plane gives the floor normal and distance in the rectified left frame,
and the pitch is the one whose camera axes put that normal straight up. The
previous file is kept as <name>.yaml.bak.
"""

import math
import os
import shutil
import sys
import time

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402


def _plane_ransac(pts: np.ndarray, iters: int = 300, tol: float = 0.01):
    """Unit normal n and offset d with n.x = d for the dominant plane, plus inlier ratio."""
    rng = np.random.default_rng(0)
    best = None
    best_count = 0
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = float(n @ a)
        count = int(np.count_nonzero(np.abs(pts @ n - d) < tol))
        if count > best_count:
            best_count, best = count, (n, d)
    if best is None:
        return None
    n, d = best
    inl = pts[np.abs(pts @ n - d) < tol]
    centroid = inl.mean(axis=0)
    _u, _s, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    n = vt[2]
    d = float(n @ centroid)
    if d < 0:
        n, d = -n, -d
    return n, d, best_count / float(len(pts))


class StereoFitFloor(Node):
    def __init__(self) -> None:
        super().__init__('stereo_fit_floor')
        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('num_pairs', 10)
        # Floor band used for the fit, as a range along the ground (m).
        self.declare_parameter('min_range_m', 0.25)
        self.declare_parameter('max_range_m', 1.0)
        self.declare_parameter('max_roll_deg', 3.0)
        self.declare_parameter('min_inlier_ratio', 0.6)
        self.declare_parameter('dry_run', False)

        import cv2
        self._cv2 = cv2
        ns = self.get_namespace().strip('/')
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._path = stereo_common.calibration_file_path(ns, str(p('calibration_file')).strip())
        self._num_pairs = max(1, int(p('num_pairs')))
        self._min_range = float(p('min_range_m'))
        self._max_range = float(p('max_range_m'))
        self._max_roll = math.radians(float(p('max_roll_deg')))
        self._min_inliers = float(p('min_inlier_ratio'))
        self._dry_run = bool(p('dry_run'))

        with open(self._path, 'r', encoding='utf-8') as handle:
            self._data = yaml.safe_load(handle) or {}
        d = self._data
        size = (int(d['image_width']), int(d['image_height']))
        k1 = stereo_common.as_matrix3(d['left_camera_matrix'])
        d1 = stereo_common.as_vector(d['left_distortion'])
        k2 = stereo_common.as_matrix3(d['right_camera_matrix'])
        d2 = stereo_common.as_vector(d['right_distortion'])
        rot = stereo_common.as_matrix3(d['R'])
        trans = stereo_common.as_vector(d['T']).reshape(3)
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            k1, d1, k2, d2, size, rot, trans.reshape(3, 1), flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self._size = size
        self._map_l = cv2.initUndistortRectifyMap(k1, d1, r1, p1, size, cv2.CV_32FC1)
        self._map_r = cv2.initUndistortRectifyMap(k2, d2, r2, p2, size, cv2.CV_32FC1)
        self._q = q
        self._r1 = np.asarray(r1, dtype=float)
        block = 5
        self._matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            preFilterCap=31, uniquenessRatio=15, speckleWindowSize=50, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        self._bridge = CvBridge()
        self._left = None
        self._points = []
        self.create_subscription(Image, str(p('left_image_topic')), self._on_left, qos_profile_sensor_data)
        self.create_subscription(Image, str(p('right_image_topic')), self._on_right, qos_profile_sensor_data)
        self._last = 0.0
        self.done = False
        self.ok = False
        self.get_logger().info(
            f'Fitting the floor for {self._path} from {self._num_pairs} pairs. Keep the robot still '
            'on flat floor with nothing in the lower half of the view.')

    def _mono(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if (img.shape[1], img.shape[0]) != self._size:
            img = self._cv2.resize(img, self._size, interpolation=self._cv2.INTER_AREA)
        return img

    def _on_left(self, msg):
        self._left = msg

    def _on_right(self, msg):
        if self.done or self._left is None:
            return
        gap = abs((self._left.header.stamp.sec - msg.header.stamp.sec)
                  + (self._left.header.stamp.nanosec - msg.header.stamp.nanosec) * 1e-9)
        if gap > 0.03 or time.monotonic() - self._last < 0.4:
            return
        self._last = time.monotonic()
        cv2 = self._cv2
        left = cv2.remap(self._mono(self._left), *self._map_l, cv2.INTER_LINEAR)
        right = cv2.remap(self._mono(msg), *self._map_r, cv2.INTER_LINEAR)
        disp = self._matcher.compute(left, right).astype(np.float32) / 16.0
        h = disp.shape[0]
        disp[: h // 2, :] = 0.0  # floor is in the lower half
        xyz = cv2.reprojectImageTo3D(disp, self._q)
        pts = xyz[disp > 1.0]
        pts = pts[np.isfinite(pts).all(axis=1)]
        # Rough ground-range band in the rectified frame (z forward).
        pts = pts[(pts[:, 2] > self._min_range) & (pts[:, 2] < self._max_range * 1.2)]
        self._points.append(pts[:: max(1, len(pts) // 4000)])
        self.get_logger().info(f'Pair {len(self._points)}/{self._num_pairs}: {len(pts)} floor candidates')
        if len(self._points) >= self._num_pairs:
            self.done = True
            self._solve(np.concatenate(self._points))

    def _solve(self, pts: np.ndarray) -> None:
        if len(pts) < 500:
            self.get_logger().error(f'Only {len(pts)} floor points; is the floor textured and in view?')
            return
        fit = _plane_ransac(pts)
        if fit is None:
            self.get_logger().error('No floor plane found.')
            return
        n, height, ratio = fit
        if ratio < self._min_inliers:
            self.get_logger().error(
                f'Only {ratio:.0%} of points lie on one plane (need {self._min_inliers:.0%}); '
                'clear the lower half of the view and retry.')
            return
        # With n.x = d and d > 0, n points from the camera down to the floor, so
        # "up" in the rectified left frame is -n. The mount model gives
        # up_rect = R1 @ optical_r(pitch).T @ [0, 0, 1].
        best = None
        for pitch in np.linspace(-0.2, 1.0, 2401):
            up = self._r1 @ stereo_common.left_optical_rotation(pitch).T @ np.array([0.0, 0.0, 1.0])
            ang = math.acos(max(-1.0, min(1.0, float(up @ -n))))
            if best is None or ang < best[1]:
                best = (float(pitch), ang)
        pitch, residual = best
        old_h = float(self._data.get('camera_height_m', 0.0))
        old_p = float(self._data.get('camera_pitch_rad', 0.0))
        self.get_logger().info(
            f'Floor plane: {ratio:.0%} inliers. camera_height_m {old_h:.4f} -> {height:.4f}, '
            f'camera_pitch_rad {old_p:.4f} -> {pitch:.4f}, left-over tilt {math.degrees(residual):.1f} deg')
        if residual > self._max_roll:
            self.get_logger().error(
                f'The floor is tilted {math.degrees(residual):.1f} deg sideways in the camera '
                f'(limit {math.degrees(self._max_roll):.1f}); the mount model has no roll. '
                'Check the floor is flat and the cameras are level. The file was not updated.')
            return
        if self._dry_run:
            self.get_logger().info('dry_run: the file was not updated.')
            self.ok = True
            return
        self._data['camera_height_m'] = round(height, 4)
        self._data['camera_pitch_rad'] = round(pitch, 4)
        shutil.copyfile(self._path, self._path + '.bak')
        with open(self._path, 'r', encoding='utf-8') as handle:
            header = [line for line in handle.read().splitlines() if line.startswith('#')]
        with open(self._path, 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(header) + '\n')
            handle.write(
                f'# camera_height_m and camera_pitch_rad fitted to the floor by stereo_fit_floor.py '
                f'({time.strftime("%Y-%m-%d")}).\n')
            yaml.safe_dump(self._data, handle, sort_keys=False)
        self.get_logger().info(
            f'Wrote {self._path} (previous file kept as {os.path.basename(self._path)}.bak). '
            'Restart the stereo / navigation launch to use it.')
        self.ok = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoFitFloor()
    try:
        deadline = time.monotonic() + 90.0
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.get_logger().error('Timed out waiting for stereo pairs; is stereo.launch.py running?')
    finally:
        ok = node.ok
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
