#!/usr/bin/env python3
"""Summarize ultrasonic debug JSONL sessions."""

import argparse
import json
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple


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


def _ts_iso_to_s(ts_iso: Optional[str]) -> Optional[float]:
    if not ts_iso:
        return None
    try:
        return datetime.fromisoformat(str(ts_iso).replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _pick_latest(paths: List[Path]) -> Optional[Path]:
    if not paths:
        return None
    return max(paths, key=lambda p: p.stat().st_mtime)


def _filter_rows_last_minutes(rows: List[Dict], last_minutes: float) -> List[Dict]:
    """Keep rows whose ts_iso is within last_minutes of the latest timestamp in the file."""
    if last_minutes <= 0:
        return rows
    ts_vals = [
        _ts_iso_to_s(r.get("ts_iso"))
        for r in rows
        if _ts_iso_to_s(r.get("ts_iso")) is not None
    ]
    if not ts_vals:
        return rows
    cutoff = max(ts_vals) - last_minutes * 60.0

    def _keep(row: Dict) -> bool:
        t = _ts_iso_to_s(row.get("ts_iso"))
        if t is None:
            return True
        return t >= cutoff

    return [r for r in rows if _keep(r)]


def _nearest_nav_row(
    nav_index: List[Tuple[float, Dict]], ts: float, window_s: float
) -> Optional[Dict]:
    """Binary search could be used; linear is fine for ~1k–2k nav rows."""
    best: Optional[Dict] = None
    best_dt = window_s + 1.0
    for tnav, nrow in nav_index:
        dt = abs(ts - tnav)
        if dt <= window_s and dt < best_dt:
            best_dt = dt
            best = nrow
    return best


def _print_cluster_nav_correlation(by_event: Dict[str, List[Dict]], window_s: float) -> None:
    """Match triangulation_decision timestamps to nearest nav_context row."""
    nav_rows = by_event.get("nav_context", [])
    index: List[Tuple[float, Dict]] = []
    for row in nav_rows:
        ts = _ts_iso_to_s(row.get("ts_iso"))
        if ts is not None:
            index.append((ts, row))
    index.sort(key=lambda x: x[0])
    tri = by_event.get("triangulation_decision", [])
    if not tri or not index:
        print("cluster_nav_correlation: (need triangulation_decision and nav_context rows)")
        return

    stats: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {"n": 0, "collision": 0, "forward_cmd": 0, "both": 0}
    )
    for row in tri:
        ts = _ts_iso_to_s(row.get("ts_iso"))
        if ts is None:
            continue
        cluster = str(row.get("cluster", "unknown"))
        nav = _nearest_nav_row(index, ts, window_s)
        if nav is None:
            continue
        st = stats[cluster]
        st["n"] += 1
        cmd = nav.get("cmd_vel") or {}
        forward = float(cmd.get("linear_x", 0.0)) > 0.05
        coll = bool(nav.get("nav2_collision_ahead", False))
        if coll:
            st["collision"] += 1
        if forward:
            st["forward_cmd"] += 1
        if coll and forward:
            st["both"] += 1

    print("")
    print(
        f"cluster_nav_correlation (triangulation_decision vs nav_context, "
        f"|dt|<={window_s}s):"
    )
    for cluster in sorted(stats.keys(), key=lambda c: -stats[c]["n"]):
        st = stats[cluster]
        n = st["n"]
        if n == 0:
            continue
        print(
            f"  {cluster}: n={n} "
            f"collision_frac={st['collision'] / n:.2f} "
            f"forward_cmd_frac={st['forward_cmd'] / n:.2f} "
            f"collision_and_forward_frac={st['both'] / n:.2f}"
        )


def _resolve_jsonl_path(user_input: Optional[str]) -> Path:
    root = _workspace_root()
    logs_dir = root / "logs"
    pattern = "*/ultrasonic-session-*.jsonl"

    if user_input:
        candidate = Path(user_input).expanduser()
        if candidate.exists():
            return candidate.resolve()
        if not candidate.is_absolute():
            local_candidate = (Path.cwd() / candidate).resolve()
            if local_candidate.exists():
                return local_candidate
            by_name = _pick_latest(list(logs_dir.glob(f"**/{candidate.name}")))
            if by_name and by_name.exists():
                return by_name.resolve()
        raise SystemExit(f"Ultrasonic session JSONL not found: {user_input}")

    latest = _pick_latest(list(logs_dir.glob(pattern)))
    if latest is None:
        raise SystemExit(f"No ultrasonic session JSONL files found under: {logs_dir}")
    return latest.resolve()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl_path", nargs="?", help="Path to ultrasonic-session-*.jsonl")
    ap.add_argument(
        "--show-last-anomalies",
        type=int,
        default=8,
        help="How many recent anomaly rows to print",
    )
    ap.add_argument(
        "--last-minutes",
        type=float,
        default=0.0,
        help="Only include rows whose ts_iso falls in the last N minutes (0 = full file).",
    )
    ap.add_argument(
        "--cluster-nav",
        action="store_true",
        help=(
            "Print triangulation cluster vs nearest nav_context: collision_ahead and "
            "forward cmd_vel fractions (use with --last-minutes for final approach)."
        ),
    )
    ap.add_argument(
        "--cluster-nav-window-s",
        type=float,
        default=0.15,
        help="Max |timestamp| difference for triangulation_decision to nav_context match.",
    )
    args = ap.parse_args()

    jsonl_path = _resolve_jsonl_path(args.jsonl_path)
    rows = _load_rows(str(jsonl_path))
    if not rows:
        raise SystemExit("No JSON rows found")
    if args.last_minutes and args.last_minutes > 0:
        rows = _filter_rows_last_minutes(rows, args.last_minutes)
        if not rows:
            raise SystemExit("No rows left after --last-minutes filter")

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

    front_emergency_ts: List[float] = []
    for row in by_event.get("triangulation_decision", []):
        cluster = str(row.get("cluster", ""))
        if cluster.startswith("front_emergency"):
            ts_s = _ts_iso_to_s(row.get("ts_iso"))
            if ts_s is not None:
                front_emergency_ts.append(ts_s)
    front_emergency_ts = sorted(front_emergency_ts)

    collision_true_ts: List[float] = []
    forward_cmd_ts: List[float] = []
    for row in nav_rows:
        ts_s = _ts_iso_to_s(row.get("ts_iso"))
        if ts_s is None:
            continue
        if bool(row.get("nav2_collision_ahead", False)):
            collision_true_ts.append(ts_s)
        cmd = row.get("cmd_vel") or {}
        if float(cmd.get("linear_x", 0.0)) > 0.05:
            forward_cmd_ts.append(ts_s)

    latency_buckets = Counter({
        "lt_0.25s": 0,
        "0.25_to_0.75s": 0,
        "0.75_to_2.0s": 0,
        "gt_2.0s": 0,
        "missing_collision_ahead": 0,
    })
    front_emergency_followed_by_forward = 0
    for ts_s in front_emergency_ts:
        next_collision = next((c for c in collision_true_ts if c >= ts_s), None)
        if next_collision is None:
            latency_buckets["missing_collision_ahead"] += 1
        else:
            dt = next_collision - ts_s
            if dt < 0.25:
                latency_buckets["lt_0.25s"] += 1
            elif dt < 0.75:
                latency_buckets["0.25_to_0.75s"] += 1
            elif dt <= 2.0:
                latency_buckets["0.75_to_2.0s"] += 1
            else:
                latency_buckets["gt_2.0s"] += 1
        next_forward = next((f for f in forward_cmd_ts if f >= ts_s and f <= ts_s + 2.0), None)
        if next_forward is not None:
            front_emergency_followed_by_forward += 1

    session_starts = len(by_event.get("session_start", []))
    session_ends = len(by_event.get("session_end", []))
    if session_starts > 1:
        print(
            "warning: multiple sessions detected in one JSONL "
            f"(session_start={session_starts}, session_end={session_ends}); "
            "results may include older runs."
        )
    print(f"session: {jsonl_path}")
    if args.last_minutes and args.last_minutes > 0:
        print(f"filter: last {args.last_minutes} minutes by ts_iso")
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
        if front_emergency_ts:
            print("  front_emergency_to_collision_ahead_latency:")
            print(
                "    "
                + ", ".join(f"{k}={v}" for k, v in latency_buckets.items())
            )
            print(
                "  front_emergency_followed_by_forward_cmd_rows: "
                f"{front_emergency_followed_by_forward}/{len(front_emergency_ts)}"
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

    if args.cluster_nav:
        _print_cluster_nav_correlation(by_event, args.cluster_nav_window_s)


if __name__ == "__main__":
    main()
