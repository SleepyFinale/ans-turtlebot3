#!/usr/bin/env python3
"""Collect chessboard pairs and write config/stereo/<namespace>.yaml.

Run this while stereo.launch.py is publishing both image topics. Avoidance
stays off until the written file has a real focal length.
"""

import os
import sys

import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402


class StereoCalibrate(Node):
    def __init__(self) -> None:
        super().__init__('stereo_calibrate')
        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('output_file', '')
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('num_pairs', 12)
        self.declare_parameter('min_corner_shift_px', 8.0)
        self.declare_parameter('camera_x_m', 0.05)
        self.declare_parameter('camera_height_m', 0.08)
        self.declare_parameter('camera_pitch_rad', 0.35)
        self.declare_parameter('baseline_m', 0.08)

        ns = self.get_namespace().strip('/')
        override = str(self.get_parameter('output_file').value).strip()
        self._output = stereo_common.calibration_file_path(ns, override)
        self._cols = int(self.get_parameter('board_cols').value)
        self._rows = int(self.get_parameter('board_rows').value)
        self._square = float(self.get_parameter('square_size_m').value)
        self._num_pairs = max(4, int(self.get_parameter('num_pairs').value))
        self._min_shift = float(self.get_parameter('min_corner_shift_px').value)
        self._mount = {
            'camera_x_m': float(self.get_parameter('camera_x_m').value),
            'camera_height_m': float(self.get_parameter('camera_height_m').value),
            'camera_pitch_rad': float(self.get_parameter('camera_pitch_rad').value),
            'baseline_m': float(self.get_parameter('baseline_m').value),
        }
        self._bridge = CvBridge()
        self._objpoints = []
        self._left_points = []
        self._right_points = []
        self._last_left = None
        self._image_size = None
        self._done = False

        import cv2
        import numpy as np
        self._cv2 = cv2
        self._np = np
        pattern = np.zeros((self._rows * self._cols, 3), np.float32)
        pattern[:, :2] = np.mgrid[0:self._cols, 0:self._rows].T.reshape(-1, 2)
        pattern *= self._square
        self._pattern = pattern

        left_topic = str(self.get_parameter('left_image_topic').value)
        right_topic = str(self.get_parameter('right_image_topic').value)
        left_sub = Subscriber(self, Image, left_topic, qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(self, Image, right_topic, qos_profile=qos_profile_sensor_data)
        sync = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.05)
        sync.registerCallback(self._on_pair)
        self._sync = sync
        self.get_logger().info(
            f'Collecting {self._num_pairs} chessboard pairs '
            f'({self._cols}x{self._rows} inner corners, square={self._square:.4f} m). '
            f'Output: {self._output}'
        )

    def _on_pair(self, left_msg: Image, right_msg: Image) -> None:
        if self._done:
            return
        cv2 = self._cv2
        np = self._np
        left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
        if left.shape != right.shape:
            self.get_logger().warning('Left and right image sizes differ; skipping pair')
            return
        pattern_size = (self._cols, self._rows)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok_l, corners_l = cv2.findChessboardCorners(left, pattern_size, flags=flags)
        ok_r, corners_r = cv2.findChessboardCorners(right, pattern_size, flags=flags)
        if not ok_l or not ok_r:
            return
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_l = cv2.cornerSubPix(left, corners_l, (11, 11), (-1, -1), criteria)
        corners_r = cv2.cornerSubPix(right, corners_r, (11, 11), (-1, -1), criteria)
        if self._last_left is not None:
            shift = float(np.mean(np.linalg.norm(corners_l - self._last_left, axis=2)))
            if shift < self._min_shift:
                return
        self._last_left = corners_l
        self._image_size = (left.shape[1], left.shape[0])
        self._objpoints.append(self._pattern.copy())
        self._left_points.append(corners_l)
        self._right_points.append(corners_r)
        self.get_logger().info(f'Accepted pair {len(self._objpoints)}/{self._num_pairs}')
        if len(self._objpoints) >= self._num_pairs:
            self._done = True
            self._solve_and_write()

    def _solve_and_write(self) -> None:
        cv2 = self._cv2
        np = self._np
        size = self._image_size
        k1 = cv2.initCameraMatrix2D(self._objpoints, self._left_points, size, 0)
        k2 = cv2.initCameraMatrix2D(self._objpoints, self._right_points, size, 0)
        d1 = np.zeros((5, 1))
        d2 = np.zeros((5, 1))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        rms, k1, d1, k2, d2, rotation, translation, _e, _f = cv2.stereoCalibrate(
            self._objpoints,
            self._left_points,
            self._right_points,
            k1,
            d1,
            k2,
            d2,
            size,
            criteria=criteria,
            flags=cv2.CALIB_FIX_ASPECT_RATIO,
        )
        t = np.asarray(translation, dtype=float).reshape(3)
        payload = {
            'image_width': int(size[0]),
            'image_height': int(size[1]),
            'left_camera_matrix': [float(v) for v in np.asarray(k1).reshape(-1)],
            'left_distortion': [float(v) for v in np.asarray(d1).reshape(-1)],
            'right_camera_matrix': [float(v) for v in np.asarray(k2).reshape(-1)],
            'right_distortion': [float(v) for v in np.asarray(d2).reshape(-1)],
            'R': [float(v) for v in np.asarray(rotation).reshape(-1)],
            'T': [float(v) for v in t],
            'baseline_calibrated_m': float(np.linalg.norm(t)),
            'rms': float(rms),
        }
        payload.update(self._mount)
        os.makedirs(os.path.dirname(self._output), exist_ok=True)
        with open(self._output, 'w', encoding='utf-8') as handle:
            handle.write(
                '# Stereo calibration. Mount fields are the values passed to stereo_calibrate.py.\n'
            )
            yaml.safe_dump(payload, handle, sort_keys=False)
        self.get_logger().info(
            f'Wrote {self._output} rms={rms:.4f} '
            f'calibrated_baseline={payload["baseline_calibrated_m"]:.4f} m. '
            'Restart SLAM/Nav2 so the local costmap and stop gate arm.'
        )
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoCalibrate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
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
