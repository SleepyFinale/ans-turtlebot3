#!/usr/bin/env python3
"""Frame-to-frame stereo visual odometry for the EKF.

stereo_depth_node runs this on every pair it already decodes (visual_odometry
parameter) and publishes the result on odom/visual. Sharing the depth node's
frames matters on the Pi: a second process subscribing to both cameras
doubled the image traffic and cost more than a core.

Each pair is rectified and corners in the left image get a depth: floor
points from the floor plane the stereo calibration fitted, points above the
floor from a row-strip match into the right image. The next left image is
solved against those 3D points with PnP + RANSAC, which also throws out
points that were not really on the floor. The camera motion is moved into
base_footprint as a planar twist (vx, wz).

The EKF in turtlebot3_bringup fuses that twist next to the wheels and the IMU,
so a wheel that slips or a gyro that drifts is outvoted. Frames the solver does
not trust (few inliers, sideways motion a diff drive cannot make, speeds the
Burger cannot reach) are dropped rather than published with a large variance.
"""

import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stereo_common  # noqa: E402

DEFAULTS = {
    'max_features': 200,
    'min_feature_distance_px': 8.0,
    'min_depth_m': 0.15,
    'max_depth_m': 3.0,
    # Floor band trusted for plane depth, measured along the ground.
    'floor_min_m': 0.15,
    'floor_max_m': 1.5,
    # Rows searched above/below for a stereo match; the pair drifts a few
    # pixels out of row alignment between recalibrations.
    'max_row_error_px': 4.0,
    'min_inliers': 20,
    'ransac_reprojection_px': 1.5,
    # A frame gap longer than this is not one motion step (dropped frames,
    # a stall); the next pair starts a new baseline instead.
    'max_frame_gap_sec': 0.6,
    # Burger limits are 0.22 m/s and 2.84 rad/s; anything well above is a
    # bad solve, not motion.
    'max_linear_speed': 0.40,
    'max_angular_speed': 3.5,
    # A diff drive cannot slide sideways. Lateral speed above this means
    # the solve mixed in a moving object or a wrong match.
    'max_lateral_speed': 0.08,
    # 1-sigma at min_inliers; shrinks with sqrt(inliers / min_inliers).
    'linear_sigma': 0.04,
    'angular_sigma': 0.08,
}


class StereoVO:
    """Visual odometry engine. Feed raw (unrectified) mono pairs to process()."""

    def __init__(self, calibration_path: str, params: dict = None) -> None:
        import cv2
        cfg = dict(DEFAULTS, **(params or {}))
        self._max_features = int(cfg['max_features'])
        self._min_dist = float(cfg['min_feature_distance_px'])
        self._min_depth = float(cfg['min_depth_m'])
        self._max_depth = float(cfg['max_depth_m'])
        self._max_row_err = float(cfg['max_row_error_px'])
        self._floor_min_m = float(cfg['floor_min_m'])
        self._floor_max_m = float(cfg['floor_max_m'])
        self._min_inliers = max(6, int(cfg['min_inliers']))
        self._ransac_px = float(cfg['ransac_reprojection_px'])
        self._max_gap = float(cfg['max_frame_gap_sec'])
        self._max_v = float(cfg['max_linear_speed'])
        self._max_w = float(cfg['max_angular_speed'])
        self._max_vy = float(cfg['max_lateral_speed'])
        self._sigma_v = float(cfg['linear_sigma'])
        self._sigma_w = float(cfg['angular_sigma'])
        self._prev = None  # (stamp_ns, left_rect, points_3d Nx3, pixels Nx2)
        self.stats = {'published': 0, 'rejected': 0, 'reason': 'starting'}

        with open(calibration_path, 'r', encoding='utf-8') as handle:
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
        self.size = size
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

    def _reject(self, reason: str) -> None:
        self.stats['rejected'] += 1
        self.stats['reason'] = reason

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
        self.stats['floor_pts'] = int(np.count_nonzero(ok) - np.count_nonzero(ok[off_floor]))
        self.stats['stereo_pts'] = int(np.count_nonzero(ok[off_floor]))
        if np.count_nonzero(ok) < self._min_inliers:
            return None, None
        pl = pl[ok]
        z = depth[ok]
        x = (pl[:, 0] - self._cx) * z / self._fx
        y = (pl[:, 1] - self._cy) * z / self._fx
        return np.stack([x, y, z], axis=1).astype(np.float32), pl.astype(np.float32)

    def process(self, left_raw: np.ndarray, right_raw: np.ndarray, stamp_ns: int):
        """Twist since the previous pair as (vx, wz, var_v, var_w), or None."""
        try:
            return self._process(left_raw, right_raw, stamp_ns)
        except Exception as exc:  # keep running; one bad frame resets the chain
            self._prev = None
            self._reject(f'error: {exc}')
            return None

    def _process(self, left_raw, right_raw, stamp_ns):
        cv2 = self._cv2
        left = cv2.remap(left_raw, *self._map_l, cv2.INTER_LINEAR)
        right = cv2.remap(right_raw, *self._map_r, cv2.INTER_LINEAR)

        prev = self._prev
        points, pixels = self._triangulate(left, right)
        self._prev = None if points is None else (stamp_ns, left, points, pixels)
        if prev is None:
            if points is None:
                self._reject('too few stereo features')
            return None

        prev_ns, prev_left, prev_points, prev_pixels = prev
        dt = (stamp_ns - prev_ns) * 1e-9
        if dt <= 0.0 or dt > self._max_gap:
            self._reject(f'frame gap {dt:.3f}s')
            return None

        # Track previous corners into this frame, keep forward-backward agreement.
        cur, st1, _ = cv2.calcOpticalFlowPyrLK(prev_left, left, prev_pixels.reshape(-1, 1, 2), None, **self._lk)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(left, prev_left, cur, None, **self._lk)
        fb = np.linalg.norm(back.reshape(-1, 2) - prev_pixels, axis=1)
        good = (st1.reshape(-1) == 1) & (st2.reshape(-1) == 1) & (fb < 1.0)
        if np.count_nonzero(good) < self._min_inliers:
            self._reject('too few tracked features')
            return None

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            prev_points[good], cur.reshape(-1, 2)[good], self._k, None,
            iterationsCount=60, reprojectionError=self._ransac_px,
            confidence=0.99, flags=cv2.SOLVEPNP_ITERATIVE)
        n_in = 0 if inliers is None else len(inliers)
        if not ok or n_in < self._min_inliers:
            self._reject(f'pnp inliers {n_in}')
            return None

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
            return None
        if abs(vy) > self._max_vy:
            self._reject(f'lateral slip vy={vy:.2f}')
            return None

        scale = math.sqrt(self._min_inliers / float(n_in))
        self.stats.update({
            'published': self.stats['published'] + 1,
            'reason': 'ok',
            'inliers': int(n_in),
            'vx': round(float(vx), 4),
            'vy': round(float(vy), 4),
            'wz': round(float(wz), 4),
            'dt': round(dt, 3),
        })
        return float(vx), float(wz), (self._sigma_v * scale) ** 2, (self._sigma_w * scale) ** 2


def odometry_msg(stamp, odom_frame: str, base_frame: str, result):
    """nav_msgs/Odometry carrying a StereoVO twist; pose fields are marked unknown."""
    from nav_msgs.msg import Odometry
    vx, wz, var_v, var_w = result
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = odom_frame
    msg.child_frame_id = base_frame
    msg.pose.covariance = [1e6 if i in (0, 7, 14, 21, 28, 35) else 0.0 for i in range(36)]
    msg.twist.twist.linear.x = vx
    msg.twist.twist.angular.z = wz
    cov = [0.0] * 36
    cov[0] = var_v
    cov[7] = var_v  # vy is not fused; keep it finite for viewers
    cov[14] = cov[21] = cov[28] = 1e6
    cov[35] = var_w
    msg.twist.covariance = cov
    return msg
