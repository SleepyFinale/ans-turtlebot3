#!/usr/bin/env python3
"""
Analyze one Nav2 exploration run and print normalized stability metrics.

Inputs can include:
- Robot JSONL debug log (nav2_motion_debug_logger)
- Central explorer log text
- Robot Nav2 log text

This script is intentionally lightweight so A/B/C runs can be compared quickly.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass
class JsonlSummary:
    ticks: int
    anomalies: Counter


def _iter_lines(path: Path) -> Iterable[str]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            yield line.rstrip("\n")


def summarize_jsonl(path: Path) -> JsonlSummary:
    ticks = 0
    anomalies: Counter[str] = Counter()
    for raw in _iter_lines(path):
        if not raw:
            continue
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if payload.get("event") != "tick":
            continue
        ticks += 1
        for item in payload.get("anomalies") or []:
            anomalies[str(item)] += 1
    return JsonlSummary(ticks=ticks, anomalies=anomalies)


def count_matches(path: Path, patterns: dict[str, str]) -> Counter:
    counts: Counter[str] = Counter()
    compiled = {name: re.compile(expr) for name, expr in patterns.items()}
    for raw in _iter_lines(path):
        for name, rx in compiled.items():
            if rx.search(raw):
                counts[name] += 1
    return counts


def rate_per_minute(count: int, duration_sec: float) -> float:
    if duration_sec <= 0.0:
        return math.nan
    return count * 60.0 / duration_sec


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize one Nav2 run.")
    parser.add_argument("--duration-sec", type=float, required=True, help="Run duration in seconds.")
    parser.add_argument("--jsonl", type=Path, help="Path to nav2 motion JSONL log.")
    parser.add_argument("--central-log", type=Path, help="Path to central start_central output log.")
    parser.add_argument("--robot-log", type=Path, help="Path to robot navigation2_slam output log.")
    parser.add_argument("--out-json", type=Path, help="Optional path to write machine-readable summary JSON.")
    args = parser.parse_args()

    summary_out: dict = {"duration_sec": args.duration_sec}

    print("=== Nav2 Run Summary ===")
    print(f"duration_sec: {args.duration_sec:.2f}")
    print("")

    if args.jsonl:
        js = summarize_jsonl(args.jsonl)
        print(f"[JSONL] {args.jsonl}")
        print(f"ticks: {js.ticks}")
        summary_out["jsonl"] = {"ticks": js.ticks, "anomalies": dict(js.anomalies)}
        for key in (
            "plan_stale_while_executing",
            "global_costmap_unavailable_or_stale",
            "cmd_vel_without_nav2_cmd_vel_nav",
            "robot_in_high_cost",
            "forward_cmd_in_high_cost",
            "missing_tf_pose_map_to_base",
        ):
            value = js.anomalies.get(key, 0)
            frac = (value / js.ticks) if js.ticks else 0.0
            print(f"  {key}: {value} ({frac:.3%})")
        print("")

    if args.central_log:
        central_patterns = {
            "precheck_timeout": r"Path precheck timed out",
            "precheck_aborted": r"Path precheck failed \(ABORTED\)",
            "goal_aborted": r"Goal aborted — blacklisting",
            "goal_reached": r"Goal reached \(total:",
            "arrival_probe_fail": r"Arrival probe failed",
            "no_frontier_assigned": r"No frontier assigned:",
        }
        cc = count_matches(args.central_log, central_patterns)
        summary_out["central"] = dict(cc)
        print(f"[CENTRAL] {args.central_log}")
        for key in central_patterns:
            value = cc.get(key, 0)
            print(f"  {key}: {value} ({rate_per_minute(value, args.duration_sec):.2f}/min)")
        combined = cc.get("precheck_timeout", 0) + cc.get("precheck_aborted", 0)
        print(f"  precheck_timeout_plus_aborted: {combined} ({rate_per_minute(combined, args.duration_sec):.2f}/min)")
        print("")

    if args.robot_log:
        robot_patterns = {
            "planner_aborted_action": r"\[compute_path_to_pose\] \[ActionServer\] Aborting handle\.",
            "planner_no_valid_path": r"failed to create plan, no valid path found",
            "controller_failed_progress": r"Failed to make progress",
            "controller_patience_exceeded": r"Controller patience exceeded",
            "controller_action_abort": r"\[follow_path\] \[ActionServer\] Aborting handle\.",
            "collision_ahead": r"detected collision ahead",
            "tf_message_filter_drop": r"Message Filter dropping message: frame .* earlier than all the data in the transform cache",
        }
        rc = count_matches(args.robot_log, robot_patterns)
        summary_out["robot"] = dict(rc)
        print(f"[ROBOT] {args.robot_log}")
        for key in robot_patterns:
            value = rc.get(key, 0)
            print(f"  {key}: {value} ({rate_per_minute(value, args.duration_sec):.2f}/min)")
        print("")

    if args.out_json:
        args.out_json.write_text(json.dumps(summary_out, indent=2), encoding="utf-8")
        print(f"[WROTE] {args.out_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

