#!/usr/bin/env python3
"""Shared stereo paths, capture-device detection, and blob geometry.

Kept free of rclpy so the launch file and the depth node use one calibration
path and the same obstacle projection.
"""

import glob
import math
import os
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
V4L2_CAP_META_CAPTURE = 0x00800000
V4L2_CAP_DEVICE_CAPS = 0x80000000
# _IOR('V', 0, struct v4l2_capability) with sizeof(v4l2_capability) == 104.
VIDIOC_QUERYCAP = 0x80685600


def turtlebot3_root() -> str:
    return os.path.expanduser(os.environ.get('TURTLEBOT3_WS', '~/turtlebot3'))


def calibration_file_path(namespace: str, override: str = '') -> str:
    """Path of config/stereo/<namespace>.yaml under the robot tree."""
    if override and str(override).strip():
        return os.path.expanduser(str(override).strip())
    ns = (namespace or '').strip().strip('/') or 'robot'
    return os.path.join(turtlebot3_root(), 'config', 'stereo', f'{ns}.yaml')


def calibration_is_usable(path: str) -> bool:
    """True when the file exists and contains a real left-camera focal length."""
    if not path or not os.path.isfile(path):
        return False
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except Exception:
        return False
    if not isinstance(data, dict):
        return False
    matrix = data.get('left_camera_matrix')
    if not isinstance(matrix, (list, tuple)) or len(matrix) < 1:
        return False
    try:
        return float(matrix[0]) > 1.0
    except (TypeError, ValueError):
        return False


def list_capture_devices() -> List[str]:
    """Return sorted V4L capture nodes, skipping metadata-only video nodes."""
    found: List[str] = []
    for path in sorted(glob.glob('/dev/video*')):
        if _is_capture_device(path):
            found.append(path)
    return found


def _is_capture_device(path: str) -> bool:
    try:
        import fcntl
        import ctypes
    except ImportError:
        return False

    class V4L2Capability(ctypes.Structure):
        _fields_ = [
            ('driver', ctypes.c_char * 16),
            ('card', ctypes.c_char * 32),
            ('bus_info', ctypes.c_char * 32),
            ('version', ctypes.c_uint32),
            ('capabilities', ctypes.c_uint32),
            ('device_caps', ctypes.c_uint32),
            ('reserved', ctypes.c_uint32 * 3),
        ]

    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        cap = V4L2Capability()
        try:
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
        except OSError:
            return False
    finally:
        os.close(fd)

    caps = int(cap.device_caps) if (int(cap.capabilities) & V4L2_CAP_DEVICE_CAPS) else int(cap.capabilities)
    if caps & V4L2_CAP_META_CAPTURE and not (caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)):
        return False
    return bool(caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
    r = np.asarray(rotation, dtype=float)
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def left_optical_rotation(pitch_down_rad: float) -> np.ndarray:
    """Rotation of the left optical frame in base_footprint.

    Optical axes are x right, y down, z forward. Positive pitch_down_rad
    aims the optical z axis downward (positive rotation about base +Y).
    """
    pitch = float(pitch_down_rad)
    c = math.cos(pitch)
    s = math.sin(pitch)
    # Unpitched optical axes as columns in base_footprint.
    unpitched = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=float)
    pitch_about_y = np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ], dtype=float)
    return pitch_about_y @ unpitched


def left_camera_translation(camera_x_m: float, baseline_m: float, camera_height_m: float) -> np.ndarray:
    """Left camera origin in base_footprint. The pair is centered on x."""
    half = 0.5 * abs(float(baseline_m))
    return np.array([float(camera_x_m), half, float(camera_height_m)], dtype=float)


def right_camera_pose_in_left(rotation: np.ndarray, translation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Pose of the right camera in the left camera frame.

    OpenCV stereoCalibrate maps a left-camera point to the right camera with
    X_right = R * X_left + T. The right camera origin in the left frame is
    -R.T @ T, and its orientation is R.T.
    """
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    t = np.asarray(translation, dtype=float).reshape(3)
    return -r.T @ t, r.T


def rectified_to_base(
    points_rect: np.ndarray,
    rectify_rotation: np.ndarray,
    optical_rotation: np.ndarray,
    camera_translation: np.ndarray,
) -> np.ndarray:
    """Map rectified-left points (Nx3) into base_footprint."""
    pts = np.asarray(points_rect, dtype=float)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=float)
    r1 = np.asarray(rectify_rotation, dtype=float).reshape(3, 3)
    # X_rect = R1 * X_left  =>  X_left = R1.T * X_rect
    left = pts @ r1
    base_rot = np.asarray(optical_rotation, dtype=float).reshape(3, 3)
    return pts_shape(left @ base_rot.T + np.asarray(camera_translation, dtype=float).reshape(3))


def pts_shape(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim == 1:
        return arr.reshape(1, 3)
    return arr.reshape(-1, 3)


def build_blob_scan(
    points_base: np.ndarray,
    *,
    angle_min: float = -1.5707963,
    angle_max: float = 1.5707963,
    angle_increment: float = 0.0174533,
    min_range: float = 0.12,
    max_range: float = 1.5,
    min_height: float = 0.03,
    max_height: float = 0.182,
    ground_x_min: float = 0.40,
    ground_x_max: float = 1.20,
    ground_y_abs: float = 0.40,
    ground_z_abs: float = 0.04,
    drop_z: float = -0.08,
    drop_min_points: int = 12,
    ground_min_points: int = 8,
    corridor_half_angle: float = 0.60,
) -> dict:
    """Project blind-band obstacles and floor drops into one forward LaserScan.

    A missing floor is marked only when neighboring or side bins still see the
    ground. A fully failed match, or one isolated textured patch, does not
    invent a wall.
    """
    n = max(2, int(round((angle_max - angle_min) / angle_increment)) + 1)
    ranges = np.full(n, np.inf, dtype=float)
    ground_counts = np.zeros(n, dtype=int)
    drop_counts = np.zeros(n, dtype=int)
    drop_ranges = np.full(n, np.inf, dtype=float)

    pts = np.asarray(points_base, dtype=float).reshape(-1, 3) if np.size(points_base) else np.zeros((0, 3))
    finite = np.isfinite(pts).all(axis=1) if len(pts) else np.zeros(0, dtype=bool)
    pts = pts[finite]

    def _bin_of(angle: float) -> Optional[int]:
        if angle < angle_min or angle > angle_max:
            return None
        idx = int(round((angle - angle_min) / angle_increment))
        if idx < 0 or idx >= n:
            return None
        return idx

    for x, y, z in pts:
        rng = math.hypot(float(x), float(y))
        if rng < min_range or rng > max_range or float(x) <= 0.0:
            continue
        ang = math.atan2(float(y), float(x))
        idx = _bin_of(ang)
        if idx is None:
            continue
        if min_height < float(z) < max_height:
            if rng < ranges[idx]:
                ranges[idx] = rng
        in_ground_band = (
            ground_x_min <= float(x) <= ground_x_max and abs(float(y)) <= ground_y_abs
        )
        if not in_ground_band:
            continue
        if abs(float(z)) <= ground_z_abs:
            ground_counts[idx] += 1
        elif float(z) < drop_z:
            drop_counts[idx] += 1
            if rng < drop_ranges[idx]:
                drop_ranges[idx] = rng

    positive = ranges[np.isfinite(ranges)]
    positive_range = float(np.min(positive)) if positive.size else None

    corridor = []
    for idx in range(n):
        ang = angle_min + idx * angle_increment
        if abs(ang) <= corridor_half_angle:
            corridor.append(idx)

    bins_with_ground = [idx for idx in corridor if ground_counts[idx] >= ground_min_points]
    ground_set = set(bins_with_ground)
    drop_marked = False
    drop_range = None
    for idx in corridor:
        if drop_counts[idx] >= drop_min_points and math.isfinite(drop_ranges[idx]):
            if drop_ranges[idx] < ranges[idx]:
                ranges[idx] = drop_ranges[idx]
            drop_marked = True
            if drop_range is None or drop_ranges[idx] < drop_range:
                drop_range = float(drop_ranges[idx])
            continue
        if ground_counts[idx] >= ground_min_points or not ground_set:
            continue
        # A hole counts only when floor is still visible on both sides of it.
        # One textured patch must not turn the neighboring empty bins into a cliff.
        if (idx - 1) not in ground_set or (idx + 1) not in ground_set:
            continue
        mark = max(min_range, ground_x_min)
        if mark < ranges[idx]:
            ranges[idx] = mark
        drop_marked = True
        if drop_range is None or mark < drop_range:
            drop_range = float(mark)

    # Center corridor empty while the sides still see the floor: the ground
    # ends in front of the robot (a stair edge wider than one angular bin).
    center = []
    sides = []
    for idx in corridor:
        ang = angle_min + idx * angle_increment
        if abs(ang) <= 0.25:
            center.append(idx)
        elif abs(ang) <= corridor_half_angle:
            sides.append(idx)
    if any(idx in ground_set for idx in sides):
        for idx in center:
            if idx in ground_set:
                continue
            mark = max(min_range, ground_x_min)
            if mark < ranges[idx]:
                ranges[idx] = mark
            drop_marked = True
            if drop_range is None or mark < drop_range:
                drop_range = float(mark)

    return {
        'ranges': ranges.tolist(),
        'angle_min': float(angle_min),
        'angle_max': float(angle_min + (n - 1) * angle_increment),
        'angle_increment': float(angle_increment),
        'obstacle_range_m': positive_range,
        'drop': bool(drop_marked),
        'drop_range_m': drop_range,
    }


def as_matrix3(values: Sequence[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float).reshape(3, 3)
    return arr


def as_vector(values: Iterable[float]) -> np.ndarray:
    return np.asarray(list(values), dtype=float).reshape(-1)
