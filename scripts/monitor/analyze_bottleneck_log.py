#!/usr/bin/env python3
"""Analyze Raspberry Pi bottleneck monitor CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class Thresholds:
    cpu_usage_warn: float = 85.0
    cpu_usage_crit: float = 95.0
    cpu_temp_warn: float = 75.0
    cpu_temp_crit: float = 82.0
    mem_used_warn: float = 85.0
    mem_used_crit: float = 95.0
    swap_used_warn: float = 25.0
    swap_used_crit: float = 60.0
    disk_used_warn: float = 90.0
    disk_used_crit: float = 97.0
    net_util_warn: float = 80.0
    net_util_crit: float = 95.0
    err_drop_warn: float = 1.0
    err_drop_crit: float = 5.0
    sustained_seconds: float = 10.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze Raspberry Pi bottleneck CSV log")
    parser.add_argument("log_csv", type=Path, help="Path to monitor CSV log")
    parser.add_argument("--ros-log", type=Path, help="Optional ROS launch console log for pipeline timing analysis")
    parser.add_argument("--json-output", type=Path, help="Optional JSON report output path")
    parser.add_argument("--sustained-seconds", type=float, default=10.0, help="Window length for sustained pressure")
    return parser.parse_args()


def _to_float(row: Dict[str, str], key: str) -> float:
    raw = row.get(key, "").strip()
    if not raw:
        return 0.0
    if raw.startswith("0x"):
        return float(int(raw, 16))
    try:
        return float(raw)
    except ValueError:
        return 0.0


def load_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists() and not path.is_absolute():
        alt = Path("scripts/monitor/logs") / path.name
        if alt.exists():
            path = alt

    if not path.exists():
        raise FileNotFoundError(
            f"Log file not found: {path}. "
            "Try using the full path, e.g. scripts/monitor/logs/<your_log>.csv"
        )

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError("Log has no data rows.")
    return rows


def resolve_optional_log(path: Path | None) -> Path | None:
    if path is None:
        return None
    if path.exists():
        return path
    if not path.is_absolute():
        alt = Path("scripts/monitor/logs") / path.name
        if alt.exists():
            return alt
    return None


def avg_sample_period(rows: List[Dict[str, str]]) -> float:
    if len(rows) < 2:
        return 1.0
    deltas = []
    for idx in range(1, len(rows)):
        now_ms = _to_float(rows[idx], "epoch_ms")
        prev_ms = _to_float(rows[idx - 1], "epoch_ms")
        delta = (now_ms - prev_ms) / 1000.0
        if delta > 0:
            deltas.append(delta)
    return sum(deltas) / len(deltas) if deltas else 1.0


def span_seconds(rows: List[Dict[str, str]]) -> float:
    start_ms = _to_float(rows[0], "epoch_ms")
    end_ms = _to_float(rows[-1], "epoch_ms")
    return max((end_ms - start_ms) / 1000.0, 0.0)


def _scan_max(rows: List[Dict[str, str]], key: str) -> float:
    return max(_to_float(r, key) for r in rows)


def _scan_mean(rows: List[Dict[str, str]], key: str) -> float:
    vals = [_to_float(r, key) for r in rows]
    return sum(vals) / len(vals)


def _find_sustained_intervals(
    rows: List[Dict[str, str]], key: str, threshold: float, min_samples: int
) -> List[Tuple[str, str, float]]:
    intervals: List[Tuple[str, str, float]] = []
    start = None
    count = 0
    for row in rows:
        val = _to_float(row, key)
        if val >= threshold:
            if start is None:
                start = row["timestamp_iso"]
                count = 1
            else:
                count += 1
        elif start is not None:
            if count >= min_samples:
                end = row["timestamp_iso"]
                intervals.append((start, end, float(count)))
            start = None
            count = 0

    if start is not None and count >= min_samples:
        intervals.append((start, rows[-1]["timestamp_iso"], float(count)))
    return intervals


def build_events(rows: List[Dict[str, str]], th: Thresholds, period: float) -> List[Dict[str, str]]:
    events: List[Dict[str, str]] = []
    min_samples = max(int(th.sustained_seconds / max(period, 0.1)), 1)

    def add_sustained(key: str, warn: float, crit: float, label: str, advice: str) -> None:
        crit_intervals = _find_sustained_intervals(rows, key, crit, min_samples)
        warn_intervals = _find_sustained_intervals(rows, key, warn, min_samples)
        for start, end, samples in crit_intervals:
            events.append(
                {
                    "severity": "critical",
                    "label": label,
                    "window": f"{start} -> {end}",
                    "detail": f"Sustained {key} >= {crit} for {int(samples)} samples.",
                    "advice": advice,
                }
            )
        for start, end, samples in warn_intervals:
            events.append(
                {
                    "severity": "warning",
                    "label": label,
                    "window": f"{start} -> {end}",
                    "detail": f"Sustained {key} >= {warn} for {int(samples)} samples.",
                    "advice": advice,
                }
            )

    add_sustained(
        "cpu_usage_pct",
        th.cpu_usage_warn,
        th.cpu_usage_crit,
        "CPU Saturation",
        "Reduce node frequency, profile heavy callbacks, or pin expensive tasks.",
    )
    add_sustained(
        "cpu_temp_c",
        th.cpu_temp_warn,
        th.cpu_temp_crit,
        "CPU Temperature",
        "Improve cooling/airflow and reduce sustained compute bursts.",
    )
    add_sustained(
        "mem_used_pct",
        th.mem_used_warn,
        th.mem_used_crit,
        "Memory Pressure",
        "Reduce memory-heavy nodes/bags and check for leaks.",
    )
    add_sustained(
        "swap_used_pct",
        th.swap_used_warn,
        th.swap_used_crit,
        "Swap Pressure",
        "Swap usage can stall control loops; lower memory footprint.",
    )
    add_sustained(
        "disk_used_pct",
        th.disk_used_warn,
        th.disk_used_crit,
        "Disk Capacity",
        "Clean old bags/logs and ensure enough free space for ROS logging.",
    )

    for row in rows:
        max_util = max(_to_float(row, "rx_util_pct"), _to_float(row, "tx_util_pct"))
        if max_util >= th.net_util_crit:
            events.append(
                {
                    "severity": "critical",
                    "label": "Network Utilization",
                    "window": row["timestamp_iso"],
                    "detail": f"Link utilization reached {max_util:.2f}%.",
                    "advice": "Lower topic bandwidth, compress images, or increase link capacity.",
                }
            )
            break
        if max_util >= th.net_util_warn:
            events.append(
                {
                    "severity": "warning",
                    "label": "Network Utilization",
                    "window": row["timestamp_iso"],
                    "detail": f"Link utilization reached {max_util:.2f}%.",
                    "advice": "Watch for queueing latency on high-rate topics.",
                }
            )
            break

    for row in rows:
        err_drop_sum = (
            _to_float(row, "rx_err_ps")
            + _to_float(row, "tx_err_ps")
            + _to_float(row, "rx_drop_ps")
            + _to_float(row, "tx_drop_ps")
        )
        if err_drop_sum >= th.err_drop_crit:
            events.append(
                {
                    "severity": "critical",
                    "label": "Packet Errors/Drops",
                    "window": row["timestamp_iso"],
                    "detail": f"Error/drop rate reached {err_drop_sum:.2f}/s.",
                    "advice": "Inspect Wi-Fi quality/interference and interface health.",
                }
            )
            break
        if err_drop_sum >= th.err_drop_warn:
            events.append(
                {
                    "severity": "warning",
                    "label": "Packet Errors/Drops",
                    "window": row["timestamp_iso"],
                    "detail": f"Error/drop rate reached {err_drop_sum:.2f}/s.",
                    "advice": "Check antenna placement and network congestion.",
                }
            )
            break

    for row in rows:
        if int(_to_float(row, "undervoltage_now")) == 1 or int(_to_float(row, "undervoltage_past")) == 1:
            events.append(
                {
                    "severity": "critical",
                    "label": "Undervoltage",
                    "window": row["timestamp_iso"],
                    "detail": "Raspberry Pi undervoltage flag detected.",
                    "advice": "Use stable PSU/cabling and isolate noisy power loads.",
                }
            )
            break

    for row in rows:
        if int(_to_float(row, "throttled_now")) == 1 or int(_to_float(row, "throttled_past")) == 1:
            events.append(
                {
                    "severity": "critical",
                    "label": "CPU Throttling",
                    "window": row["timestamp_iso"],
                    "detail": "vcgencmd throttling flag detected.",
                    "advice": "Address thermal or voltage causes to avoid control latency.",
                }
            )
            break

    return events


def analyze_ros_pipeline_log(path: Path) -> Dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()

    counts = {
        "message_filter_drops": 0,
        "queue_full_drops": 0,
        "tf_too_old_errors": 0,
        "tf_earlier_than_cache_drops": 0,
        "control_loop_missed_rate": 0,
        "unable_transform_pose": 0,
        "controller_patience_exceeded": 0,
    }
    max_tf_lag_sec = 0.0

    lag_re = re.compile(
        r"Data time:\s*(\d+)s\s+(\d+)ns,\s*Transform time:\s*(\d+)s\s+(\d+)ns"
    )

    for line in lines:
        if "Message Filter dropping message" in line:
            counts["message_filter_drops"] += 1
        if "queue is full" in line:
            counts["queue_full_drops"] += 1
        if "Transform data too old" in line:
            counts["tf_too_old_errors"] += 1
        if "timestamp on the message is earlier than all the data in the transform cache" in line:
            counts["tf_earlier_than_cache_drops"] += 1
        if "Control loop missed its desired rate" in line:
            counts["control_loop_missed_rate"] += 1
        if "Unable to transform robot pose into global plan's frame" in line:
            counts["unable_transform_pose"] += 1
        if "Controller patience exceeded" in line:
            counts["controller_patience_exceeded"] += 1

        m = lag_re.search(line)
        if m:
            data_s, data_ns, tf_s, tf_ns = map(int, m.groups())
            data_time = float(data_s) + (float(data_ns) / 1e9)
            tf_time = float(tf_s) + (float(tf_ns) / 1e9)
            lag = max(data_time - tf_time, 0.0)
            if lag > max_tf_lag_sec:
                max_tf_lag_sec = lag

    ros_events: List[Dict[str, str]] = []

    if counts["controller_patience_exceeded"] > 0:
        ros_events.append(
            {
                "severity": "critical",
                "label": "Controller Patience Exceeded",
                "window": "ros_log",
                "detail": f"Detected {counts['controller_patience_exceeded']} events.",
                "advice": "Planner/controller timing is unstable; reduce load and inspect TF pipeline latency.",
            }
        )
    if counts["tf_too_old_errors"] > 0 or max_tf_lag_sec > 0.5:
        sev = "critical" if max_tf_lag_sec > 1.0 or counts["tf_too_old_errors"] >= 3 else "warning"
        ros_events.append(
            {
                "severity": sev,
                "label": "TF Staleness",
                "window": "ros_log",
                "detail": f"TF too-old errors: {counts['tf_too_old_errors']}, max TF lag: {max_tf_lag_sec:.3f}s.",
                "advice": "Check odom->map publication timeliness, clock sync, and callback starvation.",
            }
        )
    if counts["queue_full_drops"] > 0:
        sev = "critical" if counts["queue_full_drops"] >= 5 else "warning"
        ros_events.append(
            {
                "severity": sev,
                "label": "Message Queue Saturation",
                "window": "ros_log",
                "detail": f"Queue-full drops detected: {counts['queue_full_drops']}.",
                "advice": "Lower sensor rate or processing load, and tune message filter queue depths.",
            }
        )
    if counts["control_loop_missed_rate"] > 0:
        sev = "critical" if counts["control_loop_missed_rate"] >= 3 else "warning"
        ros_events.append(
            {
                "severity": sev,
                "label": "Control Loop Missed Rate",
                "window": "ros_log",
                "detail": f"Controller missed desired rate {counts['control_loop_missed_rate']} times.",
                "advice": "Reduce controller/planner workload or lower controller frequency slightly.",
            }
        )

    return {
        "path": str(path),
        "counts": counts,
        "max_tf_lag_sec": round(max_tf_lag_sec, 6),
        "events": ros_events,
    }


def print_report(
    rows: List[Dict[str, str]],
    events: List[Dict[str, str]],
    period: float,
    ros_summary: Dict[str, object] | None = None,
) -> Dict[str, object]:
    summary = {
        "samples": len(rows),
        "duration_seconds": round(span_seconds(rows), 2),
        "avg_sample_period_seconds": round(period, 3),
        "max_cpu_usage_pct": round(_scan_max(rows, "cpu_usage_pct"), 2),
        "avg_cpu_usage_pct": round(_scan_mean(rows, "cpu_usage_pct"), 2),
        "max_cpu_temp_c": round(_scan_max(rows, "cpu_temp_c"), 2),
        "avg_cpu_temp_c": round(_scan_mean(rows, "cpu_temp_c"), 2),
        "max_mem_used_pct": round(_scan_max(rows, "mem_used_pct"), 2),
        "max_swap_used_pct": round(_scan_max(rows, "swap_used_pct"), 2),
        "max_disk_used_pct": round(_scan_max(rows, "disk_used_pct"), 2),
        "max_rx_util_pct": round(_scan_max(rows, "rx_util_pct"), 2),
        "max_tx_util_pct": round(_scan_max(rows, "tx_util_pct"), 2),
    }

    merged_events = list(events)
    if ros_summary:
        merged_events.extend(ros_summary.get("events", []))

    critical = [e for e in merged_events if e["severity"] == "critical"]
    warning = [e for e in merged_events if e["severity"] == "warning"]
    status = "healthy"
    if critical:
        status = "critical"
    elif warning:
        status = "warning"

    print("=== Raspberry Pi Bottleneck Analysis ===")
    print(f"Status: {status.upper()}")
    print(f"Samples: {summary['samples']}  Duration: {summary['duration_seconds']}s  Avg Period: {summary['avg_sample_period_seconds']}s")
    print(
        "CPU avg/max: {avg_cpu_usage_pct:.2f}%/{max_cpu_usage_pct:.2f}% | Temp avg/max: {avg_cpu_temp_c:.2f}C/{max_cpu_temp_c:.2f}C".format(
            **summary
        )
    )
    print(
        "Mem max: {max_mem_used_pct:.2f}% | Swap max: {max_swap_used_pct:.2f}% | Disk max: {max_disk_used_pct:.2f}%".format(
            **summary
        )
    )
    print(
        "Net util max (rx/tx): {max_rx_util_pct:.2f}%/{max_tx_util_pct:.2f}%".format(
            **summary
        )
    )

    if merged_events:
        print("\nPotential bottlenecks:")
        for idx, e in enumerate(merged_events, 1):
            print(f"{idx}. [{e['severity'].upper()}] {e['label']} at {e['window']}")
            print(f"   - {e['detail']}")
            print(f"   - Action: {e['advice']}")
    else:
        print("\nNo threshold violations detected.")

    if ros_summary:
        counts = ros_summary.get("counts", {})
        print("\nROS pipeline timing summary:")
        print(f"- ROS log: {ros_summary.get('path')}")
        print(
            "- Drops queue_full/filter/cache: {}/{}/{}".format(
                counts.get("queue_full_drops", 0),
                counts.get("message_filter_drops", 0),
                counts.get("tf_earlier_than_cache_drops", 0),
            )
        )
        print(
            "- TF too-old errors: {} | max TF lag: {:.3f}s".format(
                counts.get("tf_too_old_errors", 0),
                float(ros_summary.get("max_tf_lag_sec", 0.0)),
            )
        )
        print(
            "- Control loop missed rate: {} | controller patience exceeded: {}".format(
                counts.get("control_loop_missed_rate", 0),
                counts.get("controller_patience_exceeded", 0),
            )
        )

    return {"status": status, "summary": summary, "events": merged_events, "ros_pipeline": ros_summary}


def main() -> None:
    args = parse_args()
    rows = load_rows(args.log_csv)
    th = Thresholds(sustained_seconds=args.sustained_seconds)
    period = avg_sample_period(rows)
    events = build_events(rows, th, period)
    ros_summary = None
    resolved_ros_log = resolve_optional_log(args.ros_log)
    if args.ros_log and not resolved_ros_log:
        raise FileNotFoundError(
            f"ROS log not found: {args.ros_log}. "
            "Try full path or scripts/monitor/logs/<your_ros_log>.log"
        )
    if resolved_ros_log:
        ros_summary = analyze_ros_pipeline_log(resolved_ros_log)

    report = print_report(rows, events, period, ros_summary)

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nJSON report written to: {args.json_output}")


if __name__ == "__main__":
    main()
