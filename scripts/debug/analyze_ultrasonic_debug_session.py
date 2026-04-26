#!/usr/bin/env python3
"""Summarize ultrasonic debug JSONL sessions."""

import argparse
import json
from collections import Counter, defaultdict
from typing import Dict, List, Optional


def _load_rows(path: str) -> List[Dict]:
    rows: List[Dict] = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            row["_line"] = line_no
            rows.append(row)
    return rows


def _mean(vals: List[float]) -> Optional[float]:
    if not vals:
        return None
    return sum(vals) / float(len(vals))


def _fmt(v: Optional[float], digits: int = 3) -> str:
    if v is None:
        return "n/a"
    return f"{v:.{digits}f}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl_path", help="Path to ultrasonic-session-*.jsonl")
    ap.add_argument(
        "--show-last-anomalies",
        type=int,
        default=8,
        help="How many recent anomaly rows to print",
    )
    args = ap.parse_args()

    rows = _load_rows(args.jsonl_path)
    if not rows:
        raise SystemExit("No JSON rows found")

    by_event: Dict[str, List[Dict]] = defaultdict(list)
    for row in rows:
        by_event[str(row.get("event", "unknown"))].append(row)

    sensors = ("l", "f", "r")
    health_by_sensor = {s: [] for s in sensors}
    sample_by_sensor = {s: [] for s in sensors}
    fusion_by_sensor = {s: [] for s in sensors}

    for row in by_event.get("range_health", []):
        sensor = row.get("sensor")
        if sensor in health_by_sensor:
            health_by_sensor[sensor].append(row)
    for row in by_event.get("range_sample", []):
        sensor = row.get("sensor")
        if sensor in sample_by_sensor:
            sample_by_sensor[sensor].append(row)
    for row in by_event.get("fusion_effect", []):
        for item in row.get("rows", []) or []:
            sensor = item.get("sensor")
            if sensor in fusion_by_sensor:
                fusion_by_sensor[sensor].append(item)

    anomaly_counter: Counter = Counter()
    anomaly_rows = by_event.get("anomaly", [])
    for row in anomaly_rows:
        for flag in row.get("flags", []) or []:
            anomaly_counter[str(flag)] += 1

    range_warn_counts = [
        int(r.get("rosout_no_range_readings_count", 0))
        for r in by_event.get("nav_context", [])
    ]
    max_range_warn = max(range_warn_counts) if range_warn_counts else 0
    collision_warn_counts = [
        int(r.get("rosout_collision_ahead_count", 0))
        for r in by_event.get("nav_context", [])
    ]
    max_collision_warn = max(collision_warn_counts) if collision_warn_counts else 0

    nav_rows = by_event.get("nav_context", [])
    cmd_vs_odom_stop_push = 0
    collision_ahead_true_rows = 0
    for row in nav_rows:
        cmd = row.get("cmd_vel") or {}
        odom = row.get("odom_twist") or {}
        cmd_x = float(cmd.get("linear_x", 0.0))
        odom_x = float(odom.get("linear_x", 0.0))
        collision_ahead = bool(row.get("nav2_collision_ahead", False))
        if collision_ahead:
            collision_ahead_true_rows += 1
        # "Stop-then-push" hint: planner still commands forward while robot speed is near zero.
        if cmd_x > 0.05 and abs(odom_x) < 0.02:
            cmd_vs_odom_stop_push += 1

    tri_rows = by_event.get("triangulation_decision", []) + by_event.get("triangulation", [])
    tri_counter: Counter = Counter()
    tri_blob_dist: List[float] = []
    for row in tri_rows:
        tri_counter[str(row.get("cluster", "unknown"))] += 1
        if row.get("blob_dist_m") is not None:
            tri_blob_dist.append(float(row.get("blob_dist_m")))

    print(f"session: {args.jsonl_path}")
    print(f"rows_total: {len(rows)}")
    print(f"events: {', '.join(f'{k}={len(v)}' for k, v in sorted(by_event.items()))}")
    print("")

    print("per-sensor summary:")
    for sensor in sensors:
        h = health_by_sensor[sensor]
        s = sample_by_sensor[sensor]
        f = fusion_by_sensor[sensor]
        rates = [float(x.get("rate_hz", 0.0)) for x in h if x.get("rate_hz") is not None]
        gaps = [float(x.get("last_gap_s", 0.0)) for x in h if x.get("last_gap_s") is not None]
        ages = [float(x.get("age_s", 0.0)) for x in s if x.get("age_s") is not None]
        stale_count = sum(1 for x in s if bool(x.get("stale")))
        flatline_max = max([int(x.get("flatline_streak", 0)) for x in h], default=0)
        deltas = [
            float(x.get("delta_raw_minus_norm_m"))
            for x in f
            if x.get("delta_raw_minus_norm_m") is not None
        ]
        fusion_effect_count = sum(1 for d in deltas if d > 0.03)
        print(
            f"  {sensor}: samples={len(s)} health={len(h)} "
            f"rate_mean={_fmt(_mean(rates), 2)}Hz gap_mean={_fmt(_mean(gaps), 3)}s "
            f"age_mean={_fmt(_mean(ages), 3)}s stale_samples={stale_count} "
            f"flatline_max={flatline_max} fusion_effect_rows={fusion_effect_count}"
        )

    print("")
    print(f"range_layer_no_input_warnings_max: {max_range_warn}")
    print(f"collision_ahead_warnings_max: {max_collision_warn}")
    if nav_rows:
        print(
            "cmd_forward_while_odom_near_zero_rows: "
            f"{cmd_vs_odom_stop_push}/{len(nav_rows)}"
        )
        print(
            "nav2_collision_ahead_true_rows: "
            f"{collision_ahead_true_rows}/{len(nav_rows)}"
        )
    if tri_rows:
        print("")
        print("triangulation summary:")
        print(
            "  clusters: "
            + ", ".join(f"{k}={v}" for k, v in tri_counter.most_common())
        )
        print(
            "  blob_dist_mean_m="
            + _fmt(_mean(tri_blob_dist), 3)
            + f" samples={len(tri_blob_dist)}"
        )
    print("anomaly counts:")
    if anomaly_counter:
        for flag, count in anomaly_counter.most_common():
            print(f"  {flag}: {count}")
    else:
        print("  (none)")

    print("")
    print("recent anomalies:")
    if anomaly_rows:
        for row in anomaly_rows[-max(0, args.show_last_anomalies):]:
            print(f"  line={row['_line']} ts={row.get('ts_iso')} flags={row.get('flags')}")
    else:
        print("  (none)")


if __name__ == "__main__":
    main()
