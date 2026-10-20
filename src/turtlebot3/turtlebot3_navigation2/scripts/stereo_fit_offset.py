#!/usr/bin/env python3
"""Remove a constant disparity offset from config/stereo/<namespace>.yaml, measured against the LiDAR.

Run after stereo_calibrate.py / stereo_refine_extrinsics.py and before
stereo_fit_floor.py, with the robot still and walls or furniture 1-3 m away
in the cameras' view, while the bringup (for scan and TF) and stereo.launch.py
are running.

A small horizontal error between the two cameras (a slight toe-in, or a
principal point a pixel or two off) adds the same few pixels of disparity
everywhere. Row alignment cannot see it, so stereo_refine_extrinsics leaves it
alone, and stereo_fit_floor then absorbs it into a wrong camera height and
pitch: the floor still comes out flat, but everything standing up is placed
too close. On inky a +2.4 px offset put a printer 1.7 m away at 0.9 m, as a
"low obstacle" in front of the robot on empty floor.

LiDAR returns in the cameras' view are projected into the rectified left
image. Their depth gives the disparity they should have, and the median gap
to the measured SGBM disparity is the offset. It is removed by moving the
right camera's principal point (cx) sideways. Rotating the right camera about
its vertical axis would also remove it, but it tilts the rectified rows by
more than a pixel across the image; a cx shift leaves the rows as they were.
The previous file is kept as <name>.yaml.bak. Run stereo_fit_floor.py
afterwards; the old height and pitch were fitted around the error.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402


class StereoFitOffset(Node):
    def __init__(self) -> None:
        super().__init__('stereo_fit_offset')
        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('num_pairs', 6)
        self.declare_parameter('min_samples', 60)
        # Same limit as stereo_depth_node: only the part of the view that
        # stereo obstacles come from. Toward the edges of the wide lenses the
        # lens model is a few pixels off, and with those samples included the
        # result moved by more than a pixel from one scene to the next.
        self.declare_parameter('max_bearing_deg', 45.0)
        # Larger corrections mean a bad calibration or a moved camera; redo
        # the chessboard calibration instead.
        self.declare_parameter('max_correction_px', 8.0)
        self.declare_parameter('dry_run', False)

        import cv2
        self._cv2 = cv2
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._base_frame = str(p('base_frame')).strip() or f'{prefix}base_footprint'
        self._path = stereo_common.calibration_file_path(ns, str(p('calibration_file')).strip())
        self._num_pairs = max(1, int(p('num_pairs')))
        self._max_bearing = math.radians(float(p('max_bearing_deg')))
        self._min_samples = int(p('min_samples'))
        self._max_correction = float(p('max_correction_px'))
        self._dry_run = bool(p('dry_run'))

        with open(self._path, 'r', encoding='utf-8') as handle:
            self._data = yaml.safe_load(handle) or {}
        d = self._data
        self._size = (int(d['image_width']), int(d['image_height']))
        self._k1 = stereo_common.as_matrix3(d['left_camera_matrix'])
        self._d1 = stereo_common.as_vector(d['left_distortion'])
        self._k2 = stereo_common.as_matrix3(d['right_camera_matrix'])
        self._d2 = stereo_common.as_vector(d['right_distortion'])
        self._rot = stereo_common.as_matrix3(d['R'])
        self._trans = stereo_common.as_vector(d['T']).reshape(3)
        block = 5
        self._matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            preFilterCap=31, uniquenessRatio=15, speckleWindowSize=50, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        self._bridge = CvBridge()
        # The robots publish TF on <namespace>/tf and <namespace>/tf_static.
        # TransformListener only reads the absolute /tf topics, so the
        # relative ones are fed into the buffer here and no remap is needed.
        self._tf = Buffer()
        self.create_subscription(TFMessage, 'tf', lambda m: self._on_tf(m, False), 50)
        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, 'tf_static', lambda m: self._on_tf(m, True), latched)
        self._waiting_since = time.monotonic()
        self._last_wait_log = 0.0
        self._left = None
        self._pairs = []
        self._scans = []
        self._lidar_frame = None
        self._last = 0.0
        self.done = False
        self.ok = False
        self.create_subscription(Image, str(p('left_image_topic')), self._on_left, qos_profile_sensor_data)
        self.create_subscription(Image, str(p('right_image_topic')), self._on_right, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Fitting the stereo disparity offset for {self._path} against the LiDAR. Keep the robot still, '
            'with walls or furniture 1-3 m away in view.')

    def _mono(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if (img.shape[1], img.shape[0]) != self._size:
            img = self._cv2.resize(img, self._size, interpolation=self._cv2.INTER_AREA)
        return img

    def _on_scan(self, msg):
        self._lidar_frame = msg.header.frame_id
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > 0.3) & (r < 4.0)
        self._scans.append(np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1))
        self._scans = self._scans[-10:]

    def _on_tf(self, msg, static: bool) -> None:
        for transform in msg.transforms:
            if static:
                self._tf.set_transform_static(transform, 'stereo_fit_offset')
            else:
                self._tf.set_transform(transform, 'stereo_fit_offset')

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
        if len(self._pairs) < self._num_pairs:
            self._pairs.append((self._mono(self._left), self._mono(msg)))
            self.get_logger().info(f'Pair {len(self._pairs)}/{self._num_pairs}')
        if len(self._pairs) < self._num_pairs:
            return
        missing = []
        if not self._scans:
            missing.append(f'a LiDAR scan on {self.resolve_topic_name("scan")} (is the robot bringup running?)')
        elif not self._tf.can_transform(self._base_frame, self._lidar_frame, Time()):
            missing.append(f'the transform {self._base_frame} <- {self._lidar_frame} on '
                           f'{self.resolve_topic_name("tf_static")}')
        if missing:
            now = time.monotonic()
            if now - self._last_wait_log > 5.0:
                self._last_wait_log = now
                self.get_logger().warning('Stereo pairs ready; waiting for ' + ', '.join(missing))
            return
        self.done = True
        self._solve()

    def _lidar_points_base(self):
        """LiDAR returns (Nx3) in the base frame, from the scans received so far."""
        try:
            tf = self._tf.lookup_transform(self._base_frame, self._lidar_frame, Time())
        except TransformException as exc:
            self.get_logger().error(f'No transform {self._base_frame} <- {self._lidar_frame}: {exc}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        xy = np.concatenate(self._scans)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([
            t.x + c * xy[:, 0] - s * xy[:, 1],
            t.y + s * xy[:, 0] + c * xy[:, 1],
            np.full(len(xy), t.z),
        ], axis=1)

    def _right_matrix(self, correction: float) -> np.ndarray:
        k2 = self._k2.copy()
        k2[0, 2] += correction
        return k2

    def _offset(self, correction: float, lidar: np.ndarray):
        """Median measured-minus-expected disparity (px) and sample count with the right cx moved by correction."""
        cv2 = self._cv2
        k2 = self._right_matrix(correction)
        r1, r2, p1, p2, _q, _, _ = cv2.stereoRectify(
            self._k1, self._d1, k2, self._d2, self._size, self._rot, self._trans.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        map_l = cv2.initUndistortRectifyMap(self._k1, self._d1, r1, p1, self._size, cv2.CV_32FC1)
        map_r = cv2.initUndistortRectifyMap(k2, self._d2, r2, p2, self._size, cv2.CV_32FC1)
        f, cx, cy = float(p1[0, 0]), float(p1[0, 2]), float(p1[1, 2])
        baseline = abs(float(p2[0, 3]) / float(p2[0, 0]))
        d = self._data
        optical_r = stereo_common.left_optical_rotation(float(d.get('camera_pitch_rad', 0.0)))
        optical_t = stereo_common.left_camera_translation(
            float(d.get('camera_x_m', 0.0254)), baseline, float(d.get('camera_height_m', 0.127)))
        # base -> left optical -> rectified left
        rect = ((lidar - optical_t) @ optical_r) @ np.asarray(r1).T
        rect = rect[rect[:, 2] > 0.3]
        u = np.rint(f * rect[:, 0] / rect[:, 2] + cx).astype(int)
        v = np.rint(f * rect[:, 1] / rect[:, 2] + cy).astype(int)
        expected = f * baseline / rect[:, 2]
        width, height = self._size
        # SGBM has no result left of numDisparities; walls are vertical, so a
        # band of rows around the LiDAR plane still sees the same surface and
        # tolerates the mount height and pitch being off.
        inside = ((u >= 130) & (u < width - 2) & (v >= 8) & (v < height - 8)
                  & (np.abs(rect[:, 0]) <= math.tan(self._max_bearing) * rect[:, 2]))
        gaps = []
        for left, right in self._pairs:
            disp = self._matcher.compute(
                cv2.remap(left, *map_l, cv2.INTER_LINEAR),
                cv2.remap(right, *map_r, cv2.INTER_LINEAR)).astype(np.float32) / 16.0
            for ui, vi, de in zip(u[inside], v[inside], expected[inside]):
                w = disp[vi - 8:vi + 9, ui - 1:ui + 2]
                w = w[w > 0.0]
                if w.size >= 8:
                    gaps.append(float(np.median(w)) - de)
        if not gaps:
            return None, 0
        return float(np.median(gaps)), len(gaps)

    def _solve(self) -> None:
        lidar = self._lidar_points_base()
        if lidar is None:
            return
        c0, n = self._offset(0.0, lidar)
        if c0 is None or n < self._min_samples:
            self.get_logger().error(
                f'Only {n} LiDAR returns matched in the stereo view (need {self._min_samples}); '
                'face walls or furniture 1-3 m away and retry.')
            return
        # The offset is close to linear in the correction; two secant steps.
        step = -1.0 if c0 > 0 else 1.0
        a, ca = 0.0, c0
        b, cb = step, self._offset(step, lidar)[0]
        for _ in range(2):
            if cb is None or abs(cb - ca) < 1e-6:
                break
            nxt = b - cb * (b - a) / (cb - ca)
            a, ca = b, cb
            b, cb = nxt, self._offset(nxt, lidar)[0]
        correction, residual = b, cb
        self.get_logger().info(
            f'Disparity offset {c0:+.2f} px from {n} LiDAR samples; moving the right cx by '
            f'{correction:+.2f} px leaves {residual:+.2f} px')
        if residual is None or abs(residual) > 0.5:
            self.get_logger().error('The correction did not converge. The file was not updated.')
            return
        if abs(correction) > self._max_correction:
            self.get_logger().error(
                f'A {correction:+.1f} px correction is larger than '
                f'{self._max_correction:.1f} px; recalibrate with the chessboard. '
                'The file was not updated.')
            return
        if self._dry_run:
            self.get_logger().info('dry_run: the file was not updated.')
            self.ok = True
            return
        self._data['right_camera_matrix'] = [float(x) for x in self._right_matrix(correction).reshape(-1)]
        shutil.copyfile(self._path, self._path + '.bak')
        with open(self._path, 'r', encoding='utf-8') as handle:
            header = [line for line in handle.read().splitlines() if line.startswith('#')]
        with open(self._path, 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(header) + '\n')
            handle.write(
                f'# right_camera_matrix cx moved {correction:+.2f} px to remove a disparity offset '
                f'measured against the LiDAR by stereo_fit_offset.py ({time.strftime("%Y-%m-%d")}).\n')
            yaml.safe_dump(self._data, handle, sort_keys=False)
        self.get_logger().info(
            f'Wrote {self._path} (previous file kept as {os.path.basename(self._path)}.bak). '
            'Now run stereo_fit_floor.py, then restart the stereo / navigation launch.')
        self.ok = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoFitOffset()
    try:
        deadline = time.monotonic() + 120.0
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.done:
            node.get_logger().error(
                'Timed out waiting for stereo pairs and a scan; are the bringup and stereo.launch.py running?')
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.ok
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
