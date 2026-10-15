#!/usr/bin/env python3
"""Collect chessboard pairs and write config/stereo/<namespace>.yaml.

Run this while stereo.launch.py is publishing both image topics. Avoidance
stays off until the written file has a real focal length.
"""

import os
import sys
import time

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
        # 11x8 squares -> 10x7 inner corners. The sheet measures 8.5 x 6.5 in,
        # so the 11-square edge is 19.6 mm per square and the 8-square edge is 20.6 mm.
        self.declare_parameter('board_cols', 10)
        self.declare_parameter('board_rows', 7)
        self.declare_parameter('long_square_m', 0.2159 / 11.0)
        self.declare_parameter('short_square_m', 0.1651 / 8.0)
        self.declare_parameter('num_pairs', 12)
        # The lens model is only right where the board has been. Views only
        # near the top of the frame left Inky's corners and floor rows wrong,
        # so collection continues until every cell of a 3x3 grid has seen
        # corners in both cameras, or max_pairs is reached.
        self.declare_parameter('max_pairs', 30)
        self.declare_parameter('min_corner_shift_px', 30.0)
        self.declare_parameter('min_pair_interval_sec', 2.0)
        self.declare_parameter('max_corner_mismatch_px', 12.0)
        self.declare_parameter('camera_x_m', 0.0254)
        self.declare_parameter('camera_height_m', 0.127)
        self.declare_parameter('camera_pitch_rad', 0.354706)
        self.declare_parameter('baseline_m', 0.0508)
        self.declare_parameter('preview_dir', '~/turtlebot3/logs/stereo_preview')
        self.declare_parameter('preview_period_sec', 1.0)

        ns = self.get_namespace().strip('/')
        override = str(self.get_parameter('output_file').value).strip()
        self._output = stereo_common.calibration_file_path(ns, override)
        self._cols = int(self.get_parameter('board_cols').value)
        self._rows = int(self.get_parameter('board_rows').value)
        self._long_square = float(self.get_parameter('long_square_m').value)
        self._short_square = float(self.get_parameter('short_square_m').value)
        self._num_pairs = max(4, int(self.get_parameter('num_pairs').value))
        self._max_pairs = max(self._num_pairs, int(self.get_parameter('max_pairs').value))
        self._covered = (set(), set())
        self._last_coverage_log_s = 0.0
        self._min_shift = float(self.get_parameter('min_corner_shift_px').value)
        self._min_pair_interval = max(0.0, float(self.get_parameter('min_pair_interval_sec').value))
        self._max_corner_mismatch = float(self.get_parameter('max_corner_mismatch_px').value)
        self._last_accept_s = 0.0
        self._last_mismatch_log_s = 0.0
        self._last_done = 0.0
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
        self._preview_dir = os.path.expanduser(str(self.get_parameter('preview_dir').value).strip())
        self._preview_period = max(0.2, float(self.get_parameter('preview_period_sec').value))
        self._last_preview_s = 0.0

        import cv2
        import numpy as np
        self._cv2 = cv2
        self._np = np
        self._pattern = self._make_pattern(self._cols, self._rows)

        left_topic = str(self.get_parameter('left_image_topic').value)
        right_topic = str(self.get_parameter('right_image_topic').value)
        left_sub = Subscriber(self, Image, left_topic, qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(self, Image, right_topic, qos_profile=qos_profile_sensor_data)
        sync = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.05)
        sync.registerCallback(self._on_pair)
        self._sync = sync
        preview_path = os.path.join(self._preview_dir, 'preview.png')
        self.get_logger().info(
            f'Collecting {self._num_pairs} chessboard pairs '
            f'({self._cols}x{self._rows} inner corners, '
            f'squares={self._long_square:.4f} x {self._short_square:.4f} m). '
            f'Output: {self._output}. Live view: {preview_path}'
        )

    def _on_pair(self, left_msg: Image, right_msg: Image) -> None:
        if self._done:
            return
        now = time.monotonic()
        if now - self._last_done < 0.35:
            return
        try:
            self._handle_pair(left_msg, right_msg)
        finally:
            self._last_done = time.monotonic()

    def _handle_pair(self, left_msg: Image, right_msg: Image) -> None:
        cv2 = self._cv2
        left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')
        if left.shape != right.shape:
            self.get_logger().warning('Left and right image sizes differ; skipping pair')
            return
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok_l, corners_l, cols, rows = self._detect_board(left)
        ok_r, corners_r = (False, None)
        if ok_l:
            self._set_board_size(cols, rows)
            ok_r, corners_r = cv2.findChessboardCorners(right, (cols, rows), flags=flags)
        self._save_preview(left, right, ok_l, ok_r, corners_l, corners_r)
        if not ok_l or not ok_r:
            return
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_l = cv2.cornerSubPix(left, corners_l, (5, 5), (-1, -1), criteria)
        corners_r = cv2.cornerSubPix(right, corners_r, (5, 5), (-1, -1), criteria)
        corners_l, contrast_l = self._orient_dark_origin(left, corners_l)
        corners_r, contrast_r = self._orient_dark_origin(right, corners_r)
        self._save_preview(left, right, corners_l is not None, corners_r is not None, corners_l, corners_r)
        if corners_l is None or corners_r is None:
            now = time.monotonic()
            if (now - self._last_mismatch_log_s) > 2.0:
                self._last_mismatch_log_s = now
                self.get_logger().warning(
                    'Corner colors are too weak to number the board. '
                    'Keep the whole sheet flat, in view, and evenly lit.'
                )
            return
        match_px = self._homography_error(corners_l, corners_r)
        if match_px > self._max_corner_mismatch:
            now = time.monotonic()
            if (now - self._last_mismatch_log_s) > 2.0:
                self._last_mismatch_log_s = now
                self.get_logger().warning(
                    f'Corner order does not agree between cameras (fit error {match_px:.1f} px). '
                    'Keep the whole board visible in both images.'
                )
            return
        if self._last_left is not None:
            shift = self._point_set_shift(corners_l, self._last_left)
            elapsed = time.monotonic() - self._last_accept_s
            if shift < self._min_shift or elapsed < self._min_pair_interval:
                return
        size = (left.shape[1], left.shape[0])
        cells_l = self._cells(corners_l, size)
        cells_r = self._cells(corners_r, size)
        adds = bool((cells_l - self._covered[0]) | (cells_r - self._covered[1]))
        if len(self._objpoints) >= self._num_pairs and not adds:
            # Enough views already; only take ones that reach new parts of the image.
            self._log_missing_coverage()
            return
        self._last_left = corners_l
        self._last_accept_s = time.monotonic()
        self._image_size = size
        self._objpoints.append(self._pattern.copy())
        self._left_points.append(corners_l)
        self._right_points.append(corners_r)
        self._covered[0].update(cells_l)
        self._covered[1].update(cells_r)
        missing = self._missing_cells()
        self.get_logger().info(
            f'Accepted pair {len(self._objpoints)}/{self._num_pairs} '
            f'(corner match {match_px:.1f} px, contrast {min(contrast_l, contrast_r):.0f}); '
            + (f'still need the board near: {", ".join(missing)}' if missing else 'image fully covered')
        )
        enough = len(self._objpoints) >= self._num_pairs
        if (enough and not missing) or len(self._objpoints) >= self._max_pairs:
            if missing:
                self.get_logger().warning(
                    f'Stopping at {self._max_pairs} pairs without the board near: '
                    f'{", ".join(missing)}. Depth there will be less accurate.'
                )
            self._done = True
            self._solve_and_write()

    _CELL_NAMES = (
        ('top-left', 'top', 'top-right'),
        ('left', 'center', 'right'),
        ('bottom-left', 'bottom', 'bottom-right'),
    )

    def _cells(self, corners, size):
        pts = self._np.asarray(corners, dtype=float).reshape(-1, 2)
        cols = self._np.clip((pts[:, 0] * 3 / size[0]).astype(int), 0, 2)
        rows = self._np.clip((pts[:, 1] * 3 / size[1]).astype(int), 0, 2)
        return set(zip(rows.tolist(), cols.tolist()))

    def _missing_cells(self):
        both = self._covered[0] & self._covered[1]
        return [
            self._CELL_NAMES[r][c] for r in range(3) for c in range(3) if (r, c) not in both
        ]

    def _log_missing_coverage(self) -> None:
        now = time.monotonic()
        if now - self._last_coverage_log_s < 3.0:
            return
        self._last_coverage_log_s = now
        self.get_logger().info(
            'Move the board toward: ' + ', '.join(self._missing_cells())
            + ' (it must be fully visible in both cameras).'
        )

    def _detect_board(self, image):
        cv2 = self._cv2
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        sizes = [(self._cols, self._rows)]
        if not self._objpoints and (self._rows, self._cols) not in sizes:
            sizes.append((self._rows, self._cols))
        for cols, rows in sizes:
            ok, corners = cv2.findChessboardCorners(image, (cols, rows), flags=flags)
            if ok:
                return True, corners, cols, rows
        return False, None, self._cols, self._rows

    def _set_board_size(self, cols: int, rows: int) -> None:
        if cols == self._cols and rows == self._rows:
            return
        if self._objpoints:
            return
        self._cols = cols
        self._rows = rows
        self._pattern = self._make_pattern(cols, rows)

    def _make_pattern(self, cols: int, rows: int):
        np = self._np
        long_m = self._long_square
        short_m = self._short_square
        sx, sy = (long_m, short_m) if cols >= rows else (short_m, long_m)
        pattern = np.zeros((rows * cols, 3), np.float32)
        grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        pattern[:, 0] = grid[:, 0] * sx
        pattern[:, 1] = grid[:, 1] * sy
        return pattern

    def _save_preview(self, left, right, ok_l, ok_r, corners_l, corners_r) -> None:
        now = time.monotonic()
        if (now - self._last_preview_s) < self._preview_period:
            return
        self._last_preview_s = now
        cv2 = self._cv2
        if right.shape != left.shape:
            right = cv2.resize(right, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_AREA)
        left_bgr = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        right_bgr = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
        pattern_size = (self._cols, self._rows)
        if ok_l and corners_l is not None:
            cv2.drawChessboardCorners(left_bgr, pattern_size, corners_l, True)
        if ok_r and corners_r is not None:
            cv2.drawChessboardCorners(right_bgr, pattern_size, corners_r, True)
        self._label(left_bgr, 'left', ok_l)
        self._label(right_bgr, 'right', ok_r)
        side = cv2.hconcat([left_bgr, right_bgr])
        os.makedirs(self._preview_dir, exist_ok=True)
        path = os.path.join(self._preview_dir, 'preview.png')
        tmp = path + '.tmp.png'
        if not cv2.imwrite(tmp, side):
            self.get_logger().warning(f'Could not write {path}')
            return
        os.replace(tmp, path)

    def _label(self, image, name: str, found: bool) -> None:
        cv2 = self._cv2
        text = f'{name}: board found' if found else f'{name}: no board'
        color = (0, 180, 0) if found else (0, 0, 220)
        cv2.putText(image, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    def _orient_dark_origin(self, image, corners):
        """Number corners so the first square is black.

        One inner-corner count is even, so OpenCV may start at either of two
        opposite corners. A flat-board mapping fits both, because a half turn
        of a plane is still a flat mapping. On an 11x8 square board those two
        corners have opposite colors, so the darker square picks one numbering.
        """
        upright = self._retile(corners, False, 0)
        turned = self._retile(corners, False, 2)
        score_up = self._dark_origin_score(image, upright)
        score_turned = self._dark_origin_score(image, turned)
        best = max(score_up, score_turned)
        other = min(score_up, score_turned)
        if best < 30.0 or best < other + 8.0:
            return None, best
        if score_up >= score_turned:
            return upright, score_up
        return turned, score_turned

    def _dark_origin_score(self, image, corners) -> float:
        np = self._np
        pts = np.asarray(corners, dtype=np.float32).reshape(self._rows, self._cols, 2)
        total = 0.0
        weight = 0.0
        for row in range(self._rows - 1):
            for col in range(self._cols - 2):
                span = min(self._square_span(pts, row, col), self._square_span(pts, row, col + 1))
                if span < 3.0:
                    continue
                delta = self._sample_square(image, pts, row, col) - self._sample_square(image, pts, row, col + 1)
                total += span * (-delta if (row + col) % 2 == 0 else delta)
                weight += span
        for row in range(self._rows - 2):
            for col in range(self._cols - 1):
                span = min(self._square_span(pts, row, col), self._square_span(pts, row + 1, col))
                if span < 3.0:
                    continue
                delta = self._sample_square(image, pts, row, col) - self._sample_square(image, pts, row + 1, col)
                total += span * (-delta if (row + col) % 2 == 0 else delta)
                weight += span
        if weight <= 0.0:
            return 0.0
        return float(total / weight)

    def _square_span(self, pts, row: int, col: int) -> float:
        np = self._np
        width = float(np.linalg.norm(pts[row, col + 1] - pts[row, col]))
        height = float(np.linalg.norm(pts[row + 1, col] - pts[row, col]))
        return min(width, height)

    def _sample_square(self, image, pts, row: int, col: int) -> float:
        np = self._np
        center = 0.25 * (
            pts[row, col] + pts[row, col + 1] + pts[row + 1, col] + pts[row + 1, col + 1]
        )
        x = int(round(float(center[0])))
        y = int(round(float(center[1])))
        height, width = image.shape[:2]
        x = min(max(x, 1), width - 2)
        y = min(max(y, 1), height - 2)
        patch = image[y - 1:y + 2, x - 1:x + 2]
        return float(np.median(patch))

    def _retile(self, corners, mirror: bool, quarter_turns: int):
        np = self._np
        grid = np.asarray(corners, dtype=np.float32).reshape(self._rows, self._cols, 2)
        if mirror:
            grid = grid[:, ::-1, :]
        grid = np.rot90(grid, quarter_turns, axes=(0, 1))
        return np.ascontiguousarray(grid, dtype=np.float32).reshape(-1, 1, 2)

    def _homography_error(self, src, dst) -> float:
        cv2 = self._cv2
        np = self._np
        src2 = np.asarray(src, dtype=np.float32).reshape(-1, 2)
        dst2 = np.asarray(dst, dtype=np.float32).reshape(-1, 2)
        homography, _mask = cv2.findHomography(src2, dst2, 0)
        if homography is None:
            return 1.0e9
        projected = cv2.perspectiveTransform(src2.reshape(-1, 1, 2), homography).reshape(-1, 2)
        return float(np.mean(np.linalg.norm(projected - dst2, axis=1)))

    def _point_set_shift(self, current, previous) -> float:
        np = self._np
        cur = np.asarray(current, dtype=np.float32).reshape(-1, 2)
        prev = np.asarray(previous, dtype=np.float32).reshape(-1, 2)
        distances = np.linalg.norm(cur[:, None, :] - prev[None, :, :], axis=2)
        return float(np.mean(np.min(distances, axis=1)))

    def _view_errors(self, imgpoints, camera, distortion, rvecs, tvecs):
        cv2 = self._cv2
        np = self._np
        scores = []
        for obj, img, rvec, tvec in zip(self._objpoints, imgpoints, rvecs, tvecs):
            projected, _jac = cv2.projectPoints(obj, rvec, tvec, camera, distortion)
            diff = projected.reshape(-1, 2) - np.asarray(img, dtype=np.float32).reshape(-1, 2)
            scores.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
        return scores

    def _drop_outlier_views(self) -> None:
        cv2 = self._cv2
        np = self._np
        if len(self._objpoints) < 6:
            return
        size = self._image_size
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-5)
        k1 = cv2.initCameraMatrix2D(self._objpoints, self._left_points, size, 0)
        k2 = cv2.initCameraMatrix2D(self._objpoints, self._right_points, size, 0)
        distortion = np.zeros((5, 1))
        _rms_l, k1, d1, rv1, tv1 = cv2.calibrateCamera(
            self._objpoints, self._left_points, size, k1, distortion,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS, criteria=criteria,
        )
        _rms_r, k2, d2, rv2, tv2 = cv2.calibrateCamera(
            self._objpoints, self._right_points, size, k2, distortion.copy(),
            flags=cv2.CALIB_USE_INTRINSIC_GUESS, criteria=criteria,
        )
        worst = [
            max(left, right)
            for left, right in zip(
                self._view_errors(self._left_points, k1, d1, rv1, tv1),
                self._view_errors(self._right_points, k2, d2, rv2, tv2),
            )
        ]
        median = float(np.median(worst))
        limit = max(2.0, median * 1.8)
        keep = [index for index, err in enumerate(worst) if err <= limit]
        self.get_logger().info(
            'View errors px: ' + ', '.join(f'{err:.1f}' for err in worst)
        )
        if len(keep) < 6 or len(keep) == len(worst):
            return
        dropped = [index + 1 for index, err in enumerate(worst) if err > limit]
        self.get_logger().info(
            f'Dropping views {dropped} (above {limit:.1f} px). Keeping {len(keep)}.'
        )
        self._objpoints = [self._objpoints[index] for index in keep]
        self._left_points = [self._left_points[index] for index in keep]
        self._right_points = [self._right_points[index] for index in keep]

    def _solve_and_write(self) -> None:
        cv2 = self._cv2
        np = self._np
        self._drop_outlier_views()
        size = self._image_size
        k1 = cv2.initCameraMatrix2D(self._objpoints, self._left_points, size, 0)
        k2 = cv2.initCameraMatrix2D(self._objpoints, self._right_points, size, 0)
        d1 = np.zeros((5, 1))
        d2 = np.zeros((5, 1))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        mono_flags = cv2.CALIB_USE_INTRINSIC_GUESS
        rms_l, k1, d1, _rv, _tv = cv2.calibrateCamera(
            self._objpoints, self._left_points, size, k1, d1, flags=mono_flags, criteria=criteria,
        )
        rms_r, k2, d2, _rv, _tv = cv2.calibrateCamera(
            self._objpoints, self._right_points, size, k2, d2, flags=mono_flags, criteria=criteria,
        )
        self.get_logger().info(f'Single-camera fit left={rms_l:.2f} px right={rms_r:.2f} px')
        try:
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
                flags=cv2.CALIB_USE_INTRINSIC_GUESS,
            )
        except cv2.error as exc:
            self.get_logger().error(
                f'Stereo solve failed ({exc}). '
                f'Single-camera fit left={rms_l:.2f} px right={rms_r:.2f} px. '
                'The file was not updated.'
            )
            raise SystemExit(1) from exc
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
        # The intrinsics only hold at the focus the cameras were locked to.
        try:
            with open(self._output, 'r', encoding='utf-8') as handle:
                previous = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            previous = {}
        for key in ('left_focus_absolute', 'right_focus_absolute'):
            if key in previous:
                payload[key] = previous[key]
        # Pitch and height fitted to the floor beat the built-in defaults; keep
        # them unless a different value was passed on the command line.
        defaults = {'camera_x_m': 0.0254, 'camera_height_m': 0.127,
                    'camera_pitch_rad': 0.354706, 'baseline_m': 0.0508}
        for key, default in defaults.items():
            if key in previous and abs(float(self._mount[key]) - default) < 1e-9:
                payload[key] = previous[key]
        ok, reason = stereo_common.calibration_data_usable(payload)
        if not ok:
            self.get_logger().error(
                f'Rejecting calibration ({reason}). '
                f'mono_rms={rms_l:.2f}/{rms_r:.2f} stereo_rms={rms:.2f} '
                f'fitted_baseline={payload["baseline_calibrated_m"]:.3f} m '
                f'measured_baseline={self._mount["baseline_m"]:.3f} m. '
                'The file was not updated.'
            )
            raise SystemExit(1)
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
