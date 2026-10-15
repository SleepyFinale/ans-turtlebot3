#!/usr/bin/env python3
"""Re-align config/stereo/<namespace>.yaml after a camera was bumped.

No chessboard needed. Point the robot at any textured scene (carpet and
furniture work), run this while stereo.launch.py is publishing, and it re-fits
the rotation between the cameras so matched features land on the same
rectified row. Intrinsics and the baseline length stay from stereo_calibrate.py.
The previous file is kept as <name>.yaml.bak.

It can only correct how the cameras sit relative to each other. When the lens
model from the chessboard is itself off, rows stay misaligned whatever the
pose, and the fix is a new chessboard calibration.
"""

import os
import shutil
import sys
import time

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402


class StereoRefineExtrinsics(Node):
    def __init__(self) -> None:
        super().__init__('stereo_refine_extrinsics')
        self.declare_parameter('left_image_topic', 'stereo_left/image_raw')
        self.declare_parameter('right_image_topic', 'stereo_right/image_raw')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('num_pairs', 8)
        self.declare_parameter('max_rms_px', 1.5)

        ns = self.get_namespace().strip('/')
        override = str(self.get_parameter('calibration_file').value).strip()
        self._path = stereo_common.calibration_file_path(ns, override)
        self._num_pairs = max(1, int(self.get_parameter('num_pairs').value))
        self._max_rms = float(self.get_parameter('max_rms_px').value)
        with open(self._path, 'r', encoding='utf-8') as handle:
            self._data = yaml.safe_load(handle) or {}
        d = self._data
        self._size = (int(d['image_width']), int(d['image_height']))
        self._k1 = stereo_common.as_matrix3(d['left_camera_matrix'])
        self._d1 = stereo_common.as_vector(d['left_distortion'])
        self._k2 = stereo_common.as_matrix3(d['right_camera_matrix'])
        self._d2 = stereo_common.as_vector(d['right_distortion'])
        self._rotation = stereo_common.as_matrix3(d['R'])
        self._translation = stereo_common.as_vector(d['T']).reshape(3)

        self._bridge = CvBridge()
        self._left_pts = []
        self._right_pts = []
        self._pairs = 0
        self._last_s = 0.0
        self._done = False
        left = Subscriber(self, Image, str(self.get_parameter('left_image_topic').value),
                          qos_profile=qos_profile_sensor_data)
        right = Subscriber(self, Image, str(self.get_parameter('right_image_topic').value),
                           qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer([left, right], queue_size=5, slop=0.03)
        self._sync.registerCallback(self._on_pair)
        self._got_images = False
        self.create_timer(5.0, self._warn_no_images)
        self.get_logger().info(
            f'Collecting {self._num_pairs} pairs to re-align {self._path}. '
            'No chessboard needed: aim at any textured scene and keep the robot still.'
        )

    def _warn_no_images(self) -> None:
        if self._got_images or self._done:
            return
        self.get_logger().warning(
            f'No synced images on {self._sync_topics()}. Start the cameras first '
            '(stereo.launch.py, or the SLAM/Nav2 launch) with the same ROS_DOMAIN_ID.'
        )

    def _sync_topics(self) -> str:
        ns = self.get_namespace().rstrip('/')
        return ' and '.join(
            f'{ns}/{self.get_parameter(name).value}'
            for name in ('left_image_topic', 'right_image_topic')
        )

    def _on_pair(self, left_msg: Image, right_msg: Image) -> None:
        self._got_images = True
        if self._done:
            return
        now = time.monotonic()
        if now - self._last_s < 0.5:
            return
        self._last_s = now
        import cv2
        left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
        if (left.shape[1], left.shape[0]) != self._size:
            left = cv2.resize(left, self._size, interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, self._size, interpolation=cv2.INTER_AREA)
        pts_l, pts_r = stereo_common.match_stereo_features(left, right)
        if len(pts_l) < 8:
            self.get_logger().warning(f'Only {len(pts_l)} matches in this pair; skipping')
            return
        # Drop matches that no rigid pose explains before fitting.
        norm_l = cv2.undistortPoints(pts_l.reshape(-1, 1, 2), self._k1, self._d1).reshape(-1, 2)
        norm_r = cv2.undistortPoints(pts_r.reshape(-1, 1, 2), self._k2, self._d2).reshape(-1, 2)
        _e, mask = cv2.findEssentialMat(
            norm_l, norm_r, np.eye(3), cv2.RANSAC, 0.999, 1.5 / float(self._k1[0, 0]),
        )
        if mask is None:
            return
        keep = mask.ravel() > 0
        self._left_pts.append(pts_l[keep])
        self._right_pts.append(pts_r[keep])
        self._pairs += 1
        self.get_logger().info(
            f'Pair {self._pairs}/{self._num_pairs}: {int(keep.sum())} matches'
        )
        if self._pairs >= self._num_pairs:
            self._done = True
            self._solve()

    def _solve(self) -> None:
        pts_l = np.vstack(self._left_pts)
        pts_r = np.vstack(self._right_pts)
        rotation, translation, before, after, inliers = stereo_common.refine_extrinsics(
            pts_l, pts_r, self._k1, self._d1, self._k2, self._d2, self._size,
            self._rotation, self._translation,
        )
        self.get_logger().info(
            f'Row error {before:.2f} px -> {after:.2f} px over {inliers}/{len(pts_l)} matches'
        )
        if not np.isfinite(after) or after > self._max_rms or inliers < 50:
            self.get_logger().error(
                f'Re-alignment left {after:.2f} px of row error (limit {self._max_rms:.2f} px); '
                'the file was not updated. If the scene was well lit and textured, the lens '
                'calibration itself is off and only a new chessboard calibration fixes it: '
                'ros2 run turtlebot3_navigation2 stereo_calibrate.py --ros-args -r __ns:=/inky'
            )
            raise SystemExit(1)
        payload = dict(self._data)
        payload['R'] = [float(v) for v in np.asarray(rotation).reshape(-1)]
        payload['T'] = [float(v) for v in np.asarray(translation).reshape(-1)]
        payload['extrinsics_refined_row_rms_px'] = float(after)
        ok, reason = stereo_common.calibration_data_usable(payload)
        if not ok:
            self.get_logger().error(f'Refined calibration rejected ({reason}); file not updated.')
            raise SystemExit(1)
        shutil.copyfile(self._path, self._path + '.bak')
        with open(self._path, 'w', encoding='utf-8') as handle:
            handle.write(
                '# Stereo calibration. Mount fields are the values passed to stereo_calibrate.py.\n'
                '# R and T re-aligned by stereo_refine_extrinsics.py.\n'
            )
            yaml.safe_dump(payload, handle, sort_keys=False)
        self.get_logger().info(
            f'Wrote {self._path} (previous file kept as {os.path.basename(self._path)}.bak). '
            'Restart SLAM/Nav2 to use it.'
        )
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoRefineExtrinsics()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    except KeyboardInterrupt:
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
    sys.exit(code)


if __name__ == '__main__':
    main()
