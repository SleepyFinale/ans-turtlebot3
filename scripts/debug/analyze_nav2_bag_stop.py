#!/usr/bin/env python3
"""Analyze Nav2 bag windows for stop-without-progress episodes.

This script highlights intervals where:
- global plan exists,
- cmd_vel_nav is rotate-only or near-zero linear,
- odom linear speed is near zero,
- distance_remaining is flat or improving very slowly.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


@dataclass
class Tick:
    t: float
    cmd_nav_lin: Optional[float]
    cmd_nav_ang: Optional[float]
    cmd_lin: Optional[float]
    odom_lin: Optional[float]
    odom_ang: Optional[float]
    dist_remaining: Optional[float]
    plan_poses: Optional[int]


def _latest_before(series: List[Tuple[float, object]], t: float) -> Optional[object]:
    out = None
    for ts, value in series:
        if ts > t:
            break
        out = value
    return out


def _is_stop_tick(
    tick: Tick,
    min_plan_poses: int,
    lin_cmd_eps: float,
    lin_odom_eps: float,
    ang_cmd_min: float,
) -> bool:
    if tick.plan_poses is None or tick.plan_poses < min_plan_poses:
        return False
    if tick.cmd_nav_lin is None or tick.odom_lin is None:
        return False
    rotate_only = abs(tick.cmd_nav_lin) <= lin_cmd_eps and (
        tick.cmd_nav_ang is not None and abs(tick.cmd_nav_ang) >= ang_cmd_min
    )
    near_zero_cmd = tick.cmd_lin is not None and abs(tick.cmd_lin) <= lin_cmd_eps
    near_zero_odom = abs(tick.odom_lin) <= lin_odom_eps
    return (rotate_only or near_zero_cmd) and near_zero_odom


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _pick_latest(paths: List[Path]) -> Optional[Path]:
    if not paths:
        return None
    return max(paths, key=lambda p: p.stat().st_mtime)


def _resolve_bag_path(user_input: Optional[str]) -> Path:
    root = _workspace_root()
    logs_dir = root / "logs"

    def to_bag_dir(p: Path) -> Path:
        if p.is_dir():
            return p
        if p.name.endswith(".db3"):
            return p.parent
        return p

    if user_input:
        candidate = Path(user_input).expanduser()
        if candidate.exists():
            return to_bag_dir(candidate.resolve())
        if not candidate.is_absolute():
            local_candidate = (Path.cwd() / candidate).resolve()
            if local_candidate.exists():
                return to_bag_dir(local_candidate)
            by_name = _pick_latest(list(logs_dir.glob(f"**/{candidate.name}")))
            if by_name and by_name.exists():
                return to_bag_dir(by_name.resolve())
        raise SystemExit(f"Bag input not found: {user_input}")

    latest_db3 = _pick_latest(list(logs_dir.glob("*/bag-*/bag-*_0.db3")))
    if latest_db3 is None:
        raise SystemExit(f"No bag db3 files found under: {logs_dir}")
    return latest_db3.parent


def _infer_robot_from_bag_path(bag_path: Path) -> Optional[str]:
    # Expected layout: <workspace>/logs/<robot>/bag-<timestamp>
    if bag_path.parent.parent.name == "logs":
        return bag_path.parent.name
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path", nargs="?", help="Bag directory path or bag-*_0.db3 file")
    ap.add_argument(
        "--robot",
        default=None,
        help="Robot namespace name (defaults to robot inferred from bag path, or pinky)",
    )
    ap.add_argument("--start", type=float, default=None, help="Start unix time (s)")
    ap.add_argument("--end", type=float, default=None, help="End unix time (s)")
    ap.add_argument("--sample-step", type=float, default=0.2, help="Sampling step (s)")
    ap.add_argument("--min-interval", type=float, default=2.0, help="Minimum interval duration (s)")
    ap.add_argument("--min-plan-poses", type=int, default=10, help="Minimum poses in plan")
    ap.add_argument("--lin-cmd-eps", type=float, default=0.02, help="Linear command near-zero threshold")
    ap.add_argument("--lin-odom-eps", type=float, default=0.02, help="Linear odom near-zero threshold")
    ap.add_argument("--ang-cmd-min", type=float, default=0.03, help="Angular command rotate-only threshold")
    args = ap.parse_args()

    bag_path = _resolve_bag_path(args.bag_path)
    robot_name = args.robot or _infer_robot_from_bag_path(bag_path) or "pinky"
    ns = f"/{robot_name}"
    topic_types = {
        f"{ns}/cmd_vel_nav": "geometry_msgs/msg/Twist",
        f"{ns}/cmd_vel": "geometry_msgs/msg/Twist",
        f"{ns}/odom": "nav_msgs/msg/Odometry",
        f"{ns}/plan": "nav_msgs/msg/Path",
        f"{ns}/navigate_to_pose/_action/feedback": "nav2_msgs/action/NavigateToPose_FeedbackMessage",
    }
    msg_cls = {topic: get_message(tp) for topic, tp in topic_types.items()}
    series: Dict[str, List[Tuple[float, object]]] = {t: [] for t in topic_types}

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )

    first_t = None
    last_t = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns / 1e9
        if topic not in topic_types:
            continue
        if first_t is None:
            first_t = t
        last_t = t
        if args.start is not None and t < args.start:
            continue
        if args.end is not None and t > args.end:
            continue
        msg = deserialize_message(data, msg_cls[topic])
        series[topic].append((t, msg))

    if first_t is None or last_t is None:
        raise SystemExit("No matching topics found in bag.")

    t0 = args.start if args.start is not None else first_t
    t1 = args.end if args.end is not None else last_t
    if t1 <= t0:
        raise SystemExit("Invalid time window.")

    ticks: List[Tick] = []
    t = t0
    while t <= t1:
        cmd_nav = _latest_before(series[f"{ns}/cmd_vel_nav"], t)
        cmd = _latest_before(series[f"{ns}/cmd_vel"], t)
        odom = _latest_before(series[f"{ns}/odom"], t)
        plan = _latest_before(series[f"{ns}/plan"], t)
        fb = _latest_before(series[f"{ns}/navigate_to_pose/_action/feedback"], t)
        ticks.append(
            Tick(
                t=t,
                cmd_nav_lin=(float(cmd_nav.linear.x) if cmd_nav else None),
                cmd_nav_ang=(float(cmd_nav.angular.z) if cmd_nav else None),
                cmd_lin=(float(cmd.linear.x) if cmd else None),
                odom_lin=(float(odom.twist.twist.linear.x) if odom else None),
                odom_ang=(float(odom.twist.twist.angular.z) if odom else None),
                dist_remaining=(float(fb.feedback.distance_remaining) if fb else None),
                plan_poses=(len(plan.poses) if plan else None),
            )
        )
        t += args.sample_step

    flags = [
        _is_stop_tick(
            tick,
            min_plan_poses=args.min_plan_poses,
            lin_cmd_eps=args.lin_cmd_eps,
            lin_odom_eps=args.lin_odom_eps,
            ang_cmd_min=args.ang_cmd_min,
        )
        for tick in ticks
    ]

    intervals: List[Tuple[int, int]] = []
    start_idx = None
    min_len = max(1, int(args.min_interval / args.sample_step))
    for i, val in enumerate(flags):
        if val and start_idx is None:
            start_idx = i
        elif not val and start_idx is not None:
            if i - start_idx >= min_len:
                intervals.append((start_idx, i - 1))
            start_idx = None
    if start_idx is not None and len(flags) - start_idx >= min_len:
        intervals.append((start_idx, len(flags) - 1))

    print(f"bag: {bag_path}")
    print(f"robot: {robot_name}")
    print(f"window: {t0:.3f} -> {t1:.3f}")
    print(f"ticks: {len(ticks)}")
    print(f"stop_intervals: {len(intervals)}")
    for n, (a, b) in enumerate(intervals, 1):
        seg = ticks[a:b + 1]
        first = seg[0]
        last = seg[-1]
        dr0 = first.dist_remaining
        dr1 = last.dist_remaining
        dr_delta = (dr1 - dr0) if (dr0 is not None and dr1 is not None) else None
        print(
            f"[{n}] {first.t:.3f} -> {last.t:.3f} "
            f"dur={last.t - first.t:.2f}s "
            f"plan_poses~{first.plan_poses} "
            f"dist_remaining {dr0} -> {dr1} "
            f"(delta={dr_delta})"
        )


if __name__ == "__main__":
    main()
