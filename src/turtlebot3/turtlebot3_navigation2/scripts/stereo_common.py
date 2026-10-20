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
    """True when the yaml exists and the stereo solution is physically plausible."""
    if not path or not os.path.isfile(path):
        return False
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except Exception:
        return False
    ok, _reason = calibration_data_usable(data)
    return ok


def calibration_data_usable(data) -> Tuple[bool, str]:
    """Reject solutions whose focal point, error, or baseline cannot be a real rig."""
    if not isinstance(data, dict):
        return False, 'calibration file is empty'
    try:
        width = int(data.get('image_width') or 0)
        height = int(data.get('image_height') or 0)
        rms = float(data.get('rms'))
        measured = float(data.get('baseline_m') or 0.0)
        fitted = float(data.get('baseline_calibrated_m') or 0.0)
    except (TypeError, ValueError):
        return False, 'calibration is missing image size, rms, or baseline'
    if width < 2 or height < 2:
        return False, 'image size is invalid'
    if rms > 1.0:
        return False, f'rms {rms:.2f} px is above 1.0'
    for name in ('left_camera_matrix', 'right_camera_matrix'):
        matrix = data.get(name)
        if not isinstance(matrix, (list, tuple)) or len(matrix) < 9:
            return False, f'{name} is incomplete'
        try:
            fx = float(matrix[0])
            fy = float(matrix[4])
            cx = float(matrix[2])
            cy = float(matrix[5])
        except (TypeError, ValueError):
            return False, f'{name} is not numeric'
        if fx <= 1.0 or fy <= 1.0:
            return False, f'{name} focal length is not real'
        if not (0.0 < cx < width and 0.0 < cy < height):
            return False, f'{name} principal point ({cx:.1f}, {cy:.1f}) is outside the image'
    if measured <= 0.0 or fitted <= 0.0:
        return False, 'baseline is missing'
    translation = data.get('T')
    if isinstance(translation, (list, tuple)) and len(translation) >= 3:
        try:
            tx, ty, tz = (abs(float(v)) for v in translation[:3])
        except (TypeError, ValueError):
            return False, 'T is not numeric'
        # The pair sits side by side, so T is mostly along x. A large y or z
        # part means the chessboard views did not pin the geometry down.
        if max(ty, tz) > 0.35 * max(tx, 1e-6):
            return False, (
                f'T ({tx:.3f}, {ty:.3f}, {tz:.3f}) m is not side by side; '
                'recalibrate with the board across the whole image'
            )
    ratio = fitted / measured
    if ratio < 0.5 or ratio > 2.0:
        return False, (
            f'fitted baseline {fitted:.3f} m is not near the measured {measured:.3f} m'
        )
    return True, ''


# Platform image processors on the Pi, not lenses. /dev/video14 is often bcm2835-isp.
_SKIP_DRIVERS = frozenset({
    'bcm2835-isp',
    'bcm2835-codec',
    'pispbe',
    'rpivid',
})


def _video_index(path: str) -> int:
    digits = ''.join(ch for ch in os.path.basename(path) if ch.isdigit())
    try:
        return int(digits)
    except ValueError:
        return 0


def list_stereo_cameras() -> List[dict]:
    """USB capture nodes first, in numeric /dev/videoN order.

    Lexicographic order picks /dev/video14 before /dev/video2. The Pi ISP also
    advertises a capture node, so two Arducams must be chosen by driver.
    """
    found: List[dict] = []
    for path in sorted(glob.glob('/dev/video*'), key=_video_index):
        info = _query_capture_device(path)
        if info is not None:
            found.append(info)
    usb = [info for info in found if info['driver'] == 'uvcvideo']
    if len(usb) >= 2:
        return usb
    return [info for info in found if info['driver'] not in _SKIP_DRIVERS]


def list_capture_devices() -> List[str]:
    """Return capture-node paths suitable for a stereo pair."""
    return [info['path'] for info in list_stereo_cameras()]


def _query_capture_device(path: str) -> Optional[dict]:
    try:
        import fcntl
        import ctypes
    except ImportError:
        return None

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
        return None
    try:
        cap = V4L2Capability()
        try:
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
        except OSError:
            return None
    finally:
        os.close(fd)

    caps = int(cap.device_caps) if (int(cap.capabilities) & V4L2_CAP_DEVICE_CAPS) else int(cap.capabilities)
    if caps & V4L2_CAP_META_CAPTURE and not (caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)):
        return None
    if not (caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)):
        return None
    return {
        'path': path,
        'driver': cap.driver.split(b'\x00', 1)[0].decode('utf-8', errors='replace'),
        'card': cap.card.split(b'\x00', 1)[0].decode('utf-8', errors='replace'),
    }


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


def match_stereo_features(left: np.ndarray, right: np.ndarray, max_features: int = 2000):
    """Matched raw pixel positions (Nx2 left, Nx2 right) between two mono images.

    SGBM only searches along rows, so these matches are how a bumped camera is
    noticed. CLAHE lifts the carpet texture enough to match in dim rooms.
    """
    import cv2
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    left = clahe.apply(np.asarray(left, dtype=np.uint8))
    right = clahe.apply(np.asarray(right, dtype=np.uint8))
    if hasattr(cv2, 'SIFT_create'):
        detector = cv2.SIFT_create(int(max_features))
        norm = cv2.NORM_L2
    else:
        detector = cv2.ORB_create(int(max_features))
        norm = cv2.NORM_HAMMING
    kp_l, des_l = detector.detectAndCompute(left, None)
    kp_r, des_r = detector.detectAndCompute(right, None)
    empty = np.zeros((0, 2), dtype=float)
    if des_l is None or des_r is None or len(kp_l) < 2 or len(kp_r) < 2:
        return empty, empty
    pairs = cv2.BFMatcher(norm).knnMatch(des_l, des_r, k=2)
    left_pts = []
    right_pts = []
    for pair in pairs:
        if len(pair) < 2:
            continue
        best, second = pair
        if best.distance < 0.8 * second.distance:
            left_pts.append(kp_l[best.queryIdx].pt)
            right_pts.append(kp_r[best.trainIdx].pt)
    if not left_pts:
        return empty, empty
    return np.asarray(left_pts, dtype=float), np.asarray(right_pts, dtype=float)


def _rectify(k1, d1, k2, d2, size, rotation, translation):
    import cv2
    return cv2.stereoRectify(
        k1, d1, k2, d2, size, rotation, np.asarray(translation, dtype=float).reshape(3, 1),
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )


def rectified_row_error(left_pts, right_pts, k1, d1, k2, d2, size, rotation, translation) -> np.ndarray:
    """Rectified left row minus right row for each match. Zero for a good calibration."""
    import cv2
    left_pts = np.asarray(left_pts, dtype=float).reshape(-1, 2)
    right_pts = np.asarray(right_pts, dtype=float).reshape(-1, 2)
    if left_pts.size == 0:
        return np.zeros(0, dtype=float)
    r1, r2, p1, p2, _q, _roi1, _roi2 = _rectify(k1, d1, k2, d2, size, rotation, translation)
    a = cv2.undistortPoints(left_pts.reshape(-1, 1, 2), k1, d1, R=r1, P=p1).reshape(-1, 2)
    b = cv2.undistortPoints(right_pts.reshape(-1, 1, 2), k2, d2, R=r2, P=p2).reshape(-1, 2)
    return a[:, 1] - b[:, 1]


def refine_extrinsics(
    left_pts,
    right_pts,
    k1, d1, k2, d2,
    size,
    rotation,
    translation,
    iterations: int = 25,
):
    """Re-fit R and the off-axis part of T so matched features share a rectified row.

    Intrinsics and the baseline length stay from the chessboard. Pitch and roll
    between the cameras are what a bump changes, and they are what SGBM cannot
    tolerate. Yaw only shows up as a range bias, which row error cannot see,
    so it is kept from the chessboard too.

    Returns (rotation, translation, rms_before_px, rms_after_px, inliers).
    """
    import cv2
    left_pts = np.asarray(left_pts, dtype=float).reshape(-1, 2)
    right_pts = np.asarray(right_pts, dtype=float).reshape(-1, 2)
    t0 = np.asarray(translation, dtype=float).reshape(3)
    baseline = float(np.linalg.norm(t0))
    sign = -1.0 if t0[0] <= 0.0 else 1.0
    params = np.r_[cv2.Rodrigues(np.asarray(rotation, dtype=float).reshape(3, 3))[0].ravel(), 0.0, 0.0]
    # rx (pitch), rz (roll), ty, tz. ry (yaw) stays fixed.
    free = [0, 2, 3, 4]

    def _unpack(x):
        ty, tz = float(x[3]), float(x[4])
        tx = sign * math.sqrt(max(1e-8, baseline * baseline - ty * ty - tz * tz))
        return cv2.Rodrigues(np.asarray(x[:3], dtype=float))[0], np.array([tx, ty, tz])

    def _residual(x, keep):
        r, t = _unpack(x)
        return rectified_row_error(left_pts[keep], right_pts[keep], k1, d1, k2, d2, size, r, t)

    keep = np.ones(len(left_pts), dtype=bool)
    before = _residual(params, keep)
    rms_before = float(np.sqrt(np.mean(before ** 2))) if before.size else float('nan')
    for it in range(int(iterations)):
        if int(np.count_nonzero(keep)) < 20:
            break
        r = _residual(params, keep)
        jac = np.zeros((r.size, len(free)))
        for j, i in enumerate(free):
            step = params.copy()
            step[i] += 1e-5
            jac[:, j] = (_residual(step, keep) - r) / 1e-5
        delta = np.linalg.lstsq(jac, -r, rcond=None)[0]
        params[free] += delta
        if it in (4, 9, 14):
            err = np.abs(_residual(params, np.ones(len(left_pts), dtype=bool)))
            keep = err < max(1.5, 3.0 * float(np.median(err)))
    rot, trans = _unpack(params)
    after = _residual(params, keep)
    rms_after = float(np.sqrt(np.mean(after ** 2))) if after.size else float('nan')
    return rot, trans, rms_before, rms_after, int(np.count_nonzero(keep))


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


def expected_floor_disparity(
    size: Tuple[int, int],
    rect_projection: np.ndarray,
    rectify_rotation: np.ndarray,
    optical_rotation: np.ndarray,
    camera_translation: np.ndarray,
    baseline_m: float,
) -> dict:
    """Where each rectified-left pixel's ray meets the floor, and the disparity it would have.

    Pixels whose ray does not reach the floor ahead get NaN. The mount (height,
    pitch) fixes this map, so it is built once per calibration.
    """
    width, height = int(size[0]), int(size[1])
    p = np.asarray(rect_projection, dtype=float)
    f, cx, cy = float(p[0, 0]), float(p[0, 2]), float(p[1, 2])
    uu, vv = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
    rays = np.stack([(uu - cx) / f, (vv - cy) / f, np.ones_like(uu)], axis=-1)
    to_base = np.asarray(optical_rotation, dtype=float) @ np.asarray(rectify_rotation, dtype=float).T
    dirs = rays @ to_base.T
    origin = np.asarray(camera_translation, dtype=float).reshape(3)
    down = dirs[..., 2] < -1e-6
    # Rays have rectified depth 1, so the scale to the floor is the rectified depth.
    depth = np.where(down, -origin[2] / np.where(down, dirs[..., 2], -1.0), np.nan)
    x = origin[0] + depth * dirs[..., 0]
    y = origin[1] + depth * dirs[..., 1]
    return {
        'x': x,
        'y': y,
        'disparity': f * float(baseline_m) / depth,
    }


def ray_drop_check(
    disparity: np.ndarray,
    texture: np.ndarray,
    floor: dict,
    *,
    x_min: float = 0.15,
    x_max: float = 0.72,
    bin_m: float = 0.05,
    y_abs: float = 0.25,
    texture_min: float = 20.0,
    min_hole_px: int = 40,
    hole_fraction: float = 0.25,
    hole_ratio: float = 3.0,
    below_scale: float = 0.6,
    min_column: int = 0,
) -> Tuple[bool, Optional[float]]:
    """A drop is where pixels aimed at the floor see something much farther away.

    Each pixel aimed at the floor within x_min..x_max is compared with the
    disparity the floor would give there. Much smaller disparity means the
    surface is below the floor: over an edge the ray reaches the ground far
    below. The 3D cloud cannot show this because that ground is usually past
    the range cut. Only textured pixels count, since blank surfaces match
    nothing reliably. A glossy surface mirrors the room, so a few of its
    pixels also read as far away (about 3-12% of a band on Inky's table);
    over a real edge it is 40-60%. With no floor seen, a band is a hole only
    when hole_fraction of all its pixels say so. Once real floor has matched
    closer in, a band whose textured pixels are mostly far below counts on
    its own: past a stair edge the view is often a plain wall, so only 13-18%
    of the band has texture at all. min_column is the first column SGBM can
    match (numDisparities); pixels left of it never have a disparity.

    Returns (drop, range_m). A hole followed by floor farther out is a bad
    patch, not an edge.
    """
    disp = np.asarray(disparity, dtype=np.float32)
    tex = np.asarray(texture, dtype=np.float32)
    x = floor['x']
    expect = floor['disparity']
    in_band = (
        np.isfinite(x) & (np.abs(floor['y']) <= float(y_abs))
        & (x >= float(x_min)) & (x < float(x_max))
    )
    in_band[:, :max(0, int(min_column))] = False
    usable = in_band & (disp > 0.0) & (tex >= float(texture_min))
    if not np.any(usable):
        return False, None
    band_x = x[in_band]
    xs = x[usable]
    d = disp[usable]
    e = expect[usable]
    tol = np.maximum(3.0, 0.25 * e)
    on_floor = np.abs(d - e) <= tol
    below = d < float(below_scale) * e
    above = d > e + tol
    edges = np.arange(float(x_min), float(x_max) + 1e-9, float(bin_m))
    states = []
    seen_floor = False
    for i in range(edges.size - 1):
        sel = (xs >= edges[i]) & (xs < edges[i + 1])
        n_floor = int(np.count_nonzero(on_floor & sel))
        n_below = int(np.count_nonzero(below & sel))
        n_above = int(np.count_nonzero(above & sel))
        n_band = int(np.count_nonzero((band_x >= edges[i]) & (band_x < edges[i + 1])))
        dominant = (
            n_below >= int(min_hole_px)
            and n_below >= float(hole_ratio) * (n_floor + n_above)
        )
        if dominant and (seen_floor or n_below >= float(hole_fraction) * n_band):
            states.append('hole')
        elif n_floor >= int(min_hole_px) and n_floor >= n_below:
            states.append('floor')
            seen_floor = True
        else:
            states.append('unknown')
    for i, state in enumerate(states):
        if state != 'hole':
            continue
        later = states[i + 1:]
        if 'floor' in later:
            continue
        # One hole bin at the far end is enough; otherwise the next bin must agree.
        if later and later[0] != 'hole':
            continue
        # The band before the first hole straddles the edge unless it is
        # clearly floor, so the edge may be anywhere in it. Report its start.
        if i > 0 and states[i - 1] != 'floor':
            return True, float(edges[i - 1])
        return True, float(edges[i])
    return False, None


def filter_disparity_to_ground(
    disparity: np.ndarray,
    *,
    num_disparities: int,
    ground_band_px: float = 4.0,
    min_obstacle_rows: int = 8,
) -> np.ndarray:
    """Keep the floor line, real obstacles, and real drops. Discard the rest.

    In the v-disparity image the floor is a line: disparity grows toward the
    bottom of the frame. Pixels on that line are the floor. An obstacle is a
    vertical run of pixels clearly closer than the line. A drop is a patch
    much farther than the line. Pixels only a few disparities off the line are
    the usual floor mismatch, and those were being rebuilt as a near obstacle.
    """
    disp = np.asarray(disparity, dtype=np.float32)
    if disp.ndim != 2 or disp.size == 0:
        return disp
    valid = disp > 0.5
    line = _ground_disparity_line(disp, valid, int(num_disparities))
    if line is None:
        return disp
    slope, intercept = line
    rows = np.arange(disp.shape[0], dtype=np.float32)
    ground = slope * rows[:, None] + intercept
    ground = np.broadcast_to(ground, disp.shape).copy()
    usable = (ground > 1.0) & (ground < float(num_disparities) - 1.0)
    delta = disp - ground
    on_ground = valid & usable & (np.abs(delta) <= float(ground_band_px))
    above = valid & usable & (delta > float(ground_band_px))
    drop_gap = np.maximum(6.0, 0.25 * ground)
    below = valid & usable & (delta < -drop_gap)
    upright = _vertical_runs(above, int(min_obstacle_rows))
    out = np.zeros_like(disp)
    out[on_ground] = ground[on_ground]
    out[below] = disp[below]
    out[upright] = disp[upright]
    return out


def _ground_disparity_line(
    disp: np.ndarray,
    valid: np.ndarray,
    num_disparities: int,
) -> Optional[Tuple[float, float]]:
    """Weighted line through the dominant disparity of each lower image row."""
    height, width = disp.shape
    bins = np.arange(0, max(2, num_disparities) + 1, 1)
    rows = []
    peaks = []
    weights = []
    # The upper rows look above the horizon on this downward camera.
    for row in range(int(height * 0.20), height):
        samples = disp[row, valid[row]]
        if samples.size < max(12, width // 20):
            continue
        hist, _ = np.histogram(samples, bins=bins)
        peak = int(np.argmax(hist))
        if int(hist[peak]) < 8:
            continue
        rows.append(float(row))
        peaks.append(float(peak) + 0.5)
        weights.append(float(hist[peak]))
    if len(rows) < 10:
        return None
    row_v = np.asarray(rows, dtype=float)
    peak_v = np.asarray(peaks, dtype=float)
    weight_v = np.asarray(weights, dtype=float)

    def _fit(row_v, peak_v, weight_v):
        total = float(weight_v.sum())
        if total <= 0.0:
            return None
        mean_row = float((weight_v * row_v).sum() / total)
        mean_peak = float((weight_v * peak_v).sum() / total)
        variance = float((weight_v * (row_v - mean_row) ** 2).sum())
        if variance < 1e-3:
            return None
        slope = float((weight_v * (row_v - mean_row) * (peak_v - mean_peak)).sum() / variance)
        return slope, mean_peak - slope * mean_row

    first = _fit(row_v, peak_v, weight_v)
    if first is None:
        return None
    error = np.abs(peak_v - (first[0] * row_v + first[1]))
    keep = error <= 4.0
    if int(np.count_nonzero(keep)) < 8:
        return None
    second = _fit(row_v[keep], peak_v[keep], weight_v[keep])
    if second is None or second[0] <= 0.02:
        # A closer floor is lower in the image, so disparity must increase with row.
        return None
    return second


def _vertical_runs(mask: np.ndarray, min_rows: int) -> np.ndarray:
    """True where a vertical run of mask pixels is at least min_rows tall."""
    on = np.asarray(mask, dtype=bool)
    height, width = on.shape
    run = np.zeros(width, dtype=np.int32)
    length = np.zeros((height, width), dtype=np.int32)
    for row in range(height):
        run = np.where(on[row], run + 1, 0)
        length[row] = run
    span = np.zeros(width, dtype=np.int32)
    keep = np.zeros_like(on)
    for row in range(height - 1, -1, -1):
        span = np.where(on[row], np.maximum(span, length[row]), 0)
        keep[row] = span >= int(min_rows)
    return keep


def pts_shape(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim == 1:
        return arr.reshape(1, 3)
    return arr.reshape(-1, 3)


def _corridor_step(
    pts: np.ndarray,
    floor_z: float,
    *,
    obstacle_range_m: Optional[float],
    ground_x_min: float,
    ground_x_max: float,
    ground_y_abs: float,
    ground_z_abs: float,
    drop_z: float,
    corridor_half_angle: float,
    ground_min_points: int,
) -> Tuple[bool, Optional[float]]:
    """A drop is a step down from a solid nearer floor, across the corridor.

    Slices are pooled over the forward view. A single noisy beam does not
    count, and neither does the shadow just past an object. Past about 0.7 m
    a few disparity pixels are already taller than a step, so that region is
    left alone. A slice that returns to the floor is a bad match, not a hole.
    """
    if pts.size == 0:
        return False, None
    x = pts[:, 0]
    y = pts[:, 1]
    z_rel = pts[:, 2] - float(floor_z)
    ang = np.arctan2(y, x)
    near = float(ground_x_min)
    far = min(float(ground_x_max), 0.72)
    if far - near < 0.20:
        return False, None
    band = (
        (x >= near) & (x <= far)
        & (np.abs(y) <= float(ground_y_abs))
        & (np.abs(ang) <= float(corridor_half_angle))
        & np.isfinite(z_rel)
    )
    xb = x[band]
    zb = z_rel[band]
    edges = np.arange(near, far + 1e-9, 0.10)
    if edges.size < 3:
        return False, None
    slices = []
    for i in range(edges.size - 1):
        slices.append(zb[(xb >= edges[i]) & (xb < edges[i + 1])])
    min_count = max(20, int(ground_min_points))

    def _solid_floor(zz: np.ndarray) -> bool:
        if zz.size < min_count:
            return False
        med = float(np.median(zz))
        inliers = float(np.mean(np.abs(zz) <= float(ground_z_abs)))
        return inliers >= 0.50 and abs(med) <= float(ground_z_abs)

    def _solid_low(zz: np.ndarray) -> bool:
        if zz.size < min_count:
            return False
        med = float(np.median(zz))
        high = float(np.percentile(zz, 75))
        inliers = float(np.mean(np.abs(zz) <= float(ground_z_abs)))
        return inliers <= 0.20 and med < float(drop_z) and high < float(drop_z) + 0.05

    seen_floor = False
    empty_run = 0
    floor_end = None
    shadow_until = None
    if obstacle_range_m is not None and math.isfinite(float(obstacle_range_m)):
        shadow_until = float(obstacle_range_m) + 0.30
    for i, zz in enumerate(slices):
        if _solid_floor(zz):
            seen_floor = True
            empty_run = 0
            floor_end = float(edges[i + 1])
            continue
        if not seen_floor:
            continue
        start = float(edges[i])
        if shadow_until is not None and start < shadow_until:
            empty_run = 0
            continue
        later = slices[i + 1:]
        if _solid_low(zz):
            if any(_solid_floor(rest) for rest in later):
                continue
            return True, start
        if zz.size < 8:
            empty_run += 1
            if empty_run >= 2 and floor_end is not None and not any(
                _solid_floor(rest) for rest in later
            ):
                return True, floor_end
        else:
            empty_run = 0
    return False, None


def _fit_floor_plane(
    pts: np.ndarray,
    x_min: float = 0.22,
    x_max: float = 0.45,
) -> Optional[Tuple[float, float, float]]:
    """Fit z = a*x + b*y + c to the ground. None when the cloud will not support it.

    A single height treats a tilted floor as a near obstacle and hides the
    real object behind that reading. RANSAC keeps the large flat set and
    leaves an object standing above the plane.
    """
    # Stop before the object and the drop window so neither one can tilt the plane.
    band = (pts[:, 0] >= float(x_min)) & (pts[:, 0] <= float(x_max)) & (np.abs(pts[:, 1]) <= 0.35)
    sample = pts[band]
    if sample.shape[0] < 40:
        return None
    step = max(1, sample.shape[0] // 1500)
    sample = sample[::step]
    count = sample.shape[0]
    if count < 40:
        return None
    rng = np.random.default_rng(0)
    best = None
    best_inliers = 0
    best_mask = None
    xyz = sample
    for _ in range(60):
        picked = xyz[rng.choice(count, 3, replace=False)]
        design = np.column_stack([picked[:, 0], picked[:, 1], np.ones(3)])
        coeff, _, _, _ = np.linalg.lstsq(design, picked[:, 2], rcond=None)
        if abs(float(coeff[0])) > 0.6 or abs(float(coeff[1])) > 0.6:
            continue
        residual = xyz[:, 2] - (coeff[0] * xyz[:, 0] + coeff[1] * xyz[:, 1] + coeff[2])
        mask = np.abs(residual) <= 0.025
        inliers = int(np.count_nonzero(mask))
        if inliers > best_inliers:
            best_inliers = inliers
            best = coeff
            best_mask = mask
    if best is None or best_inliers < max(40, int(0.35 * count)) or best_mask is None:
        return None
    design = np.column_stack([
        xyz[best_mask, 0], xyz[best_mask, 1], np.ones(best_inliers),
    ])
    coeff, _, _, _ = np.linalg.lstsq(design, xyz[best_mask, 2], rcond=None)
    if abs(float(coeff[0])) > 0.6 or abs(float(coeff[1])) > 0.6:
        return None
    return float(coeff[0]), float(coeff[1]), float(coeff[2])


def _local_slice_floor(pts: np.ndarray, x_min: float = 0.36) -> np.ndarray:
    """Height most of the points share in each small patch of the view.

    Only the ground past the steep near field votes. A lower tail in the same
    patch must not become the floor, or the surface itself is marked as an obstacle.
    """
    floors = np.full(len(pts), np.nan)
    if len(pts) == 0:
        return floors
    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    ang = np.arctan2(y, x)
    x_edges = np.arange(float(x_min), 1.14, 0.06)
    a_edges = np.arange(-0.80, 0.80 + 1e-6, 0.16)
    hist_edges = np.arange(-0.40, 0.50, 0.02)
    for i in range(x_edges.size - 1):
        for j in range(a_edges.size - 1):
            sel = (
                (x >= x_edges[i]) & (x < x_edges[i + 1])
                & (ang >= a_edges[j]) & (ang < a_edges[j + 1])
            )
            if int(np.count_nonzero(sel)) < 12:
                continue
            counts, _ = np.histogram(z[sel], bins=hist_edges)
            peak = int(np.argmax(counts))
            if int(counts[peak]) < 5:
                continue
            floors[sel] = float(hist_edges[peak] + 0.01)
    return floors


def build_blob_scan(
    points_base: np.ndarray,
    *,
    angle_min: float = -1.5707963,
    angle_max: float = 1.5707963,
    angle_increment: float = 0.0174533,
    min_range: float = 0.12,
    max_range: float = 1.5,
    min_height: float = 0.025,
    max_height: float = 0.182,
    # Closer than this, the downward view rebuilds the floor as a near wall.
    obstacle_x_min: float = 0.40,
    # Past this, one disparity pixel is already taller than a low object.
    obstacle_x_max: float = 0.80,
    ground_x_min: float = 0.22,
    ground_x_max: float = 1.20,
    ground_y_abs: float = 0.40,
    ground_z_abs: float = 0.04,
    drop_z: float = -0.15,
    drop_min_points: int = 12,
    ground_min_points: int = 8,
    corridor_half_angle: float = 0.60,
    obstacle_textured: Optional[np.ndarray] = None,
    shadow_obstacle_m: Optional[float] = None,
    drop_result: Optional[Tuple[bool, Optional[float]]] = None,
    free_min_points: int = 8,
) -> dict:
    """Project blind-band obstacles and floor drops into one forward LaserScan.

    A drop is a step down from floor that is still visible closer in. A failed
    match, or a low patch that the next slice shows is still floor, does not
    invent a hole.
    """
    n = max(2, int(round((angle_max - angle_min) / angle_increment)) + 1)
    ranges = np.full(n, np.inf, dtype=float)
    floor_counts = np.zeros(n, dtype=int)
    floor_far = np.zeros(n, dtype=float)
    # Nearest above-floor return per beam, textured or not, weak or not.
    above_counts = np.zeros(n, dtype=int)
    above_near = np.full(n, np.inf, dtype=float)

    pts = np.asarray(points_base, dtype=float).reshape(-1, 3) if np.size(points_base) else np.zeros((0, 3))
    finite = np.isfinite(pts).all(axis=1) if len(pts) else np.zeros(0, dtype=bool)
    if obstacle_textured is None:
        textured = np.ones(len(pts), dtype=bool)
    else:
        textured = np.asarray(obstacle_textured, dtype=bool).reshape(-1)
        if textured.shape[0] != pts.shape[0]:
            textured = np.ones(pts.shape[0], dtype=bool)
    pts = pts[finite]
    textured = textured[finite] if len(finite) else textured
    # One floor for the whole view, so an object cannot set the floor of its
    # own patch and disappear. The height most points share wins.
    floor_z = 0.0
    if len(pts):
        support = (pts[:, 0] >= 0.40) & (pts[:, 0] <= 0.90) & (np.abs(pts[:, 1]) <= 0.35)
        zz = pts[support, 2]
        zz = zz[np.isfinite(zz) & (np.abs(zz) <= 0.25)]
        if zz.size >= 30:
            edges = np.arange(-0.20, 0.25, 0.02)
            counts, _ = np.histogram(zz, bins=edges)
            floor_z = float(edges[int(np.argmax(counts))] + 0.01)
    plane = _fit_floor_plane(pts) if len(pts) else None
    obstacle_plane = _fit_floor_plane(pts, 0.40, 0.80) if len(pts) else None
    drop_pts = pts
    if plane is not None:
        drop_pts = np.array(pts, dtype=float, copy=True)
        drop_pts[:, 2] = pts[:, 2] - (plane[0] * pts[:, 0] + plane[1] * pts[:, 1] + plane[2])
    elif len(pts):
        near = (pts[:, 0] >= 0.22) & (pts[:, 0] <= 0.50) & (np.abs(pts[:, 1]) < 0.30)
        if int(np.count_nonzero(near)) >= 30:
            zz = pts[near, 2]
            edges = np.arange(-0.30, 0.40, 0.02)
            counts, _ = np.histogram(zz, bins=edges)
            floor_z = float(np.clip(edges[int(np.argmax(counts))] + 0.01, -0.10, 0.10))

    # Bin every point into a beam at once. A per-point Python loop here took
    # about 0.9 s of each pair on the Pi (some 27k points) and held the
    # depth node near 1 Hz instead of publish_hz.
    px, py, pz = pts[:, 0], pts[:, 1], pts[:, 2]
    rng_all = np.hypot(px, py)
    ang_all = np.arctan2(py, px)
    with np.errstate(invalid='ignore'):
        # np.rint rounds half to even, like the built-in round().
        idx_all = np.rint((ang_all - angle_min) / angle_increment)
    keep = (
        (rng_all >= min_range) & (rng_all <= max_range) & (px > 0.0)
        & (ang_all >= angle_min) & (ang_all <= angle_max)
        & (idx_all >= 0) & (idx_all < n)
        & (px >= obstacle_x_min) & (px <= obstacle_x_max)
    )
    bins = idx_all[keep].astype(int)
    rngs = rng_all[keep]
    is_tex = textured[keep]
    if obstacle_plane is not None:
        z_ref = (
            float(obstacle_plane[0]) * px[keep]
            + float(obstacle_plane[1]) * py[keep]
            + float(obstacle_plane[2])
        )
    else:
        z_ref = floor_z
    z_rel = pz[keep] - z_ref
    above = (z_rel > min_height) & (z_rel < max_height)
    np.add.at(above_counts, bins[above], 1)
    np.minimum.at(above_near, bins[above], rngs[above])
    on_floor = is_tex & ~above & (np.abs(z_rel) <= float(ground_z_abs))
    np.add.at(floor_counts, bins[on_floor], 1)
    np.maximum.at(floor_far, bins[on_floor], rngs[on_floor])
    obstacle = is_tex & above
    ob_bins = bins[obstacle]
    ob_rngs = rngs[obstacle]
    order = np.lexsort((ob_rngs, ob_bins))
    ob_bins = ob_bins[order]
    ob_rngs = ob_rngs[order]
    starts = np.searchsorted(ob_bins, np.arange(n), side='left')
    ends = np.searchsorted(ob_bins, np.arange(n), side='right')

    for idx in np.flatnonzero(ends - starts >= drop_min_points):
        samples = ob_rngs[starts[idx]:ends[idx]]
        med = float(np.median(samples))
        # Inside 0.50 m a thin tail above the floor is the near-field smear.
        # A real object fills its beam, so it has to outnumber the floor there.
        if med < 0.50 and len(samples) <= int(floor_counts[idx]):
            continue
        ranges[idx] = med
    obstacle_ranges = ranges.copy()

    # Beams where the floor itself was matched and nothing stood on it. These
    # clear remembered marks; beams with no floor match say nothing.
    # The camera sits above a low object and can see floor past it, so a
    # beam stops short of anything standing up in it, even returns too weak
    # to count as an obstacle. Otherwise it would erase a remembered object.
    stop = np.where(above_counts >= 3, above_near - 0.05, np.inf)
    free_ranges = np.where(
        (floor_counts >= int(free_min_points)) & ~np.isfinite(ranges),
        np.minimum(floor_far, stop), np.inf,
    )

    positive = ranges[np.isfinite(ranges)]
    positive_range = float(np.min(positive)) if positive.size else None
    shadow_range = positive_range
    if shadow_range is None and shadow_obstacle_m is not None and math.isfinite(float(shadow_obstacle_m)):
        shadow_range = float(shadow_obstacle_m)

    corridor = []
    for idx in range(n):
        ang = angle_min + idx * angle_increment
        if abs(ang) <= corridor_half_angle:
            corridor.append(idx)

    # drop_result comes from ray_drop_check, which sees ground past the range cut.
    drop_marked, drop_range = drop_result if drop_result is not None else _corridor_step(
        drop_pts,
        0.0 if plane is not None else floor_z,
        obstacle_range_m=shadow_range,
        ground_x_min=ground_x_min,
        ground_x_max=min(float(ground_x_max), 0.72),
        ground_y_abs=ground_y_abs,
        ground_z_abs=ground_z_abs,
        drop_z=drop_z,
        corridor_half_angle=corridor_half_angle,
        ground_min_points=ground_min_points,
    )
    if drop_marked and drop_range is not None:
        for idx in corridor:
            if drop_range < ranges[idx]:
                ranges[idx] = drop_range

    return {
        'ranges': ranges.tolist(),
        'obstacle_ranges': obstacle_ranges.tolist(),
        'free_ranges': free_ranges.tolist(),
        'angle_min': float(angle_min),
        'angle_max': float(angle_min + (n - 1) * angle_increment),
        'angle_increment': float(angle_increment),
        'obstacle_range_m': positive_range,
        'floor_z_m': float(
            obstacle_plane[2] + 0.55 * obstacle_plane[0]
            if obstacle_plane is not None else floor_z
        ),
        'drop': bool(drop_marked),
        'drop_range_m': drop_range,
    }


def as_matrix3(values: Sequence[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float).reshape(3, 3)
    return arr


def as_vector(values: Iterable[float]) -> np.ndarray:
    return np.asarray(list(values), dtype=float).reshape(-1)
