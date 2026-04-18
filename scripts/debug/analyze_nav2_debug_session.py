#!/usr/bin/env python3
"""Summarize Nav2 debug JSONL sessions for stop/stall diagnostics."""

import argparse
import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass
class Tick:
    ts_iso: str
    line: int
    nav_status: Optional[int]
    goal_id: Optional[str]
    robot_cost: Optional[int]
    plan_min_cost: Optional[int]
    plan_max_cost: Optional[int]
    goal_changes_10s: Optional[int]
    cmd_lin: Optional[float]
    odom_lin: Optional[float]
    anomalies: List[str]


def _f(v: Optional[float], default: float = 0.0) -> float:
    if v is None:
        return default
    return float(v)


def _load_ticks(path: str) -> List[Tick]:
    ticks: List[Tick] = []
    with open(path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, 1):
            if not line.startswith("{"):
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get("event") != "tick":
                continue
            ticks.append(
                Tick(
                    ts_iso=row.get("ts_iso", ""),
                    line=i,
                    nav_status=row.get("nav2_latest_status"),
                    goal_id=row.get("nav2_latest_goal_id"),
                    robot_cost=row.get("robot_cost"),
                    plan_min_cost=row.get("plan_min_cost"),
                    plan_max_cost=row.get("plan_max_cost"),
                    goal_changes_10s=row.get("goal_changes_10s"),
                    cmd_lin=(row.get("cmd_vel") or {}).get("lin_x"),
                    odom_lin=(row.get("odom_twist") or {}).get("lin_x"),
                    anomalies=list(row.get("anomalies") or []),
                )
            )
    return ticks


def _is_executing_freeze(t: Tick) -> bool:
    tagged = ("robot_in_high_cost" in t.anomalies) or ("goal_reached_near_obstacle" in t.anomalies)
    return (
        t.nav_status == 2
        and abs(_f(t.cmd_lin)) <= 1e-6
        and abs(_f(t.odom_lin)) <= 0.01
        and tagged
    )


def _intervals(pred: Sequence[bool], min_len: int) -> List[Tuple[int, int]]:
    out: List[Tuple[int, int]] = []
    start = None
    for i, ok in enumerate(pred):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            if i - start >= min_len:
                out.append((start, i - 1))
            start = None
    if start is not None and len(pred) - start >= min_len:
        out.append((start, len(pred) - 1))
    return out


def _summarize_interval(ticks: List[Tick], a: int, b: int, hz: float) -> Dict:
    seg = ticks[a : b + 1]
    goal_ids: List[str] = []
    for t in seg:
        if t.goal_id and t.goal_id not in goal_ids:
            goal_ids.append(t.goal_id)
    rc = [t.robot_cost for t in seg if t.robot_cost is not None]
    pmin = [t.plan_min_cost for t in seg if t.plan_min_cost is not None]
    pmax = [t.plan_max_cost for t in seg if t.plan_max_cost is not None]
    gch = [t.goal_changes_10s for t in seg if t.goal_changes_10s is not None]
    return {
        "start_ts": seg[0].ts_iso,
        "end_ts": seg[-1].ts_iso,
        "start_line": seg[0].line,
        "end_line": seg[-1].line,
        "duration_s": round((b - a + 1) / hz, 2),
        "goal_ids": goal_ids,
        "robot_cost_min": min(rc) if rc else None,
        "robot_cost_max": max(rc) if rc else None,
        "plan_min_cost_min": min(pmin) if pmin else None,
        "plan_max_cost_max": max(pmax) if pmax else None,
        "goal_changes_10s_max": max(gch) if gch else None,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl_path")
    ap.add_argument("--log-rate-hz", type=float, default=5.0)
    ap.add_argument("--min-freeze-ticks", type=int, default=5)
    args = ap.parse_args()

    ticks = _load_ticks(args.jsonl_path)
    if not ticks:
        raise SystemExit("No tick rows found")

    freeze_mask = [_is_executing_freeze(t) for t in ticks]
    freeze_intervals = _intervals(freeze_mask, args.min_freeze_ticks)

    mismatch_count = sum(1 for t in ticks if "cmd_vel_mismatch_vs_cmd_vel_nav" in t.anomalies)
    high_cost_count = sum(1 for t in ticks if "robot_in_high_cost" in t.anomalies)
    executing_count = sum(1 for t in ticks if t.nav_status == 2)
    preemption_burst_count = sum(1 for t in ticks if "high_goal_preemption_rate" in t.anomalies)

    print(f"session: {args.jsonl_path}")
    print(f"ticks: {len(ticks)}")
    print(f"executing_ticks: {executing_count}")
    print(f"high_cost_ticks: {high_cost_count}")
    print(f"preemption_burst_ticks: {preemption_burst_count}")
    print(f"cmd_vel_mismatch_ticks: {mismatch_count}")
    print(f"executing_freeze_intervals: {len(freeze_intervals)}")

    mismatch_idx = {i for i, t in enumerate(ticks) if "cmd_vel_mismatch_vs_cmd_vel_nav" in t.anomalies}
    for n, (a, b) in enumerate(freeze_intervals, 1):
        s = _summarize_interval(ticks, a, b, args.log_rate_hz)
        overlap = sum(1 for i in range(a, b + 1) if i in mismatch_idx)
        total = b - a + 1
        print(
            f"[{n}] {s['start_ts']} -> {s['end_ts']} "
            f"dur={s['duration_s']}s lines={s['start_line']}-{s['end_line']} "
            f"goal_changes_10s_max={s['goal_changes_10s_max']} mismatch_overlap={overlap}/{total}"
        )
        print(
            f"    costs robot[{s['robot_cost_min']},{s['robot_cost_max']}] "
            f"plan_min>={s['plan_min_cost_min']} plan_max<={s['plan_max_cost_max']} "
            f"goals={','.join(s['goal_ids'][:3])}{'...' if len(s['goal_ids']) > 3 else ''}"
        )


if __name__ == "__main__":
    main()
