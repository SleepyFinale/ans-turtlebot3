#!/usr/bin/env python3
"""Unified scorecard for cooperative sensing + ORCA rollout gates."""

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional


def _load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if not line.startswith("{"):
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _safe_ratio(num: int, den: int) -> float:
    if den <= 0:
        return 0.0
    return float(num) / float(den)


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _dict_float(d: Any, keys: tuple[str, ...], default: float = 0.0) -> float:
    if not isinstance(d, dict):
        return default
    for key in keys:
        if key in d:
            return _as_float(d.get(key), default)
    return default


def _compute_metrics(
    ultrasonic_rows: list[dict], nav_rows: list[dict], orca_rows: list[dict]
) -> dict[str, Any]:
    nav_ctx = [r for r in ultrasonic_rows if r.get("event") == "nav_context"]
    tri = [
        r
        for r in ultrasonic_rows
        if r.get("event") in ("triangulation_decision", "triangulation")
    ]
    anomalies = [r for r in ultrasonic_rows if r.get("event") == "anomaly"]
    ticks = [r for r in nav_rows if r.get("event") == "tick"]
    orca_events = [r for r in orca_rows if r.get("event") == "orca_advisory"]

    fusion_no_effect = 0
    fusion_total = 0
    for row in ultrasonic_rows:
        if row.get("event") != "fusion_effect":
            continue
        for sub in row.get("rows", []) or []:
            fusion_total += 1
            flags = set(sub.get("flags") or [])
            if (
                "fusion_no_effect" in flags
                or "fusion_no_effect_l" in flags
                or "fusion_no_effect_f" in flags
                or "fusion_no_effect_r" in flags
            ):
                fusion_no_effect += 1

    front_emergency = 0
    for row in tri:
        cluster = str(row.get("cluster", ""))
        if cluster.startswith("front_emergency"):
            front_emergency += 1

    forward_when_emergency = 0
    for row in nav_ctx:
        if not bool(row.get("nav2_collision_ahead", False)):
            continue
        cmd = row.get("cmd_vel") or {}
        if _dict_float(cmd, ("linear_x", "lin_x"), 0.0) > 0.05:
            forward_when_emergency += 1

    ghost_following = 0
    for row in anomalies:
        flags = set(row.get("flags") or [])
        if "front_emergency_followed_by_forward_cmd" in flags:
            ghost_following += 1

    high_cost_ticks = 0
    for row in ticks:
        for flag in row.get("anomalies") or []:
            if flag == "robot_in_high_cost":
                high_cost_ticks += 1
                break

    max_stuck_interval_s = 0.0
    current = 0.0
    for row in ticks:
        cmd = row.get("cmd_vel") or {}
        odom = row.get("odom_twist") or {}
        cmd_x = _dict_float(cmd, ("lin_x", "linear_x"), 0.0)
        odom_x = _dict_float(odom, ("lin_x", "linear_x"), 0.0)
        if cmd_x > 0.05 and abs(odom_x) < 0.02:
            current += 0.2
        else:
            max_stuck_interval_s = max(max_stuck_interval_s, current)
            current = 0.0
    max_stuck_interval_s = max(max_stuck_interval_s, current)

    orca_stop_count = 0
    orca_slowdown_count = 0
    orca_min_sep = None
    for row in orca_events:
        reason = str(row.get("reason", "none"))
        if reason == "stop":
            orca_stop_count += 1
        if reason == "slowdown":
            orca_slowdown_count += 1
        sep = row.get("min_predicted_separation_m")
        if isinstance(sep, (int, float)):
            sep = float(sep)
            orca_min_sep = sep if orca_min_sep is None else min(orca_min_sep, sep)

    return {
        "counts": {
            "nav_context_rows": len(nav_ctx),
            "triangulation_rows": len(tri),
            "nav_ticks": len(ticks),
            "orca_events": len(orca_events),
        },
        "metrics": {
            "fusion_no_effect_rate": _safe_ratio(fusion_no_effect, fusion_total),
            "forward_when_collision_ahead_rate": _safe_ratio(
                forward_when_emergency, max(1, len(nav_ctx))
            ),
            "ghost_following_events": ghost_following,
            "high_cost_tick_rate": _safe_ratio(high_cost_ticks, max(1, len(ticks))),
            "max_stuck_interval_s": max_stuck_interval_s,
            "front_emergency_events": front_emergency,
            "orca_stop_count": orca_stop_count,
            "orca_slowdown_count": orca_slowdown_count,
            "orca_min_predicted_separation_m": orca_min_sep,
        },
    }


def _evaluate_gates(metrics: dict[str, Any]) -> dict[str, Any]:
    m = metrics["metrics"]
    gates = {
        "no_ghost_following": m["ghost_following_events"] == 0,
        "stuck_interval_under_20s": float(m["max_stuck_interval_s"]) <= 20.0,
        "forward_collision_rate_under_5pct": float(
            m["forward_when_collision_ahead_rate"]
        )
        < 0.05,
        "fusion_no_effect_under_35pct": float(m["fusion_no_effect_rate"]) < 0.35,
    }
    gates["all_pass"] = all(gates.values())
    return gates


def _default_path(path: Optional[str], fallback_glob: str) -> Optional[Path]:
    if path:
        p = Path(path).expanduser()
        if p.exists():
            return p.resolve()
    root = Path(__file__).resolve().parents[2]
    matches = sorted(root.glob(fallback_glob), key=lambda p: p.stat().st_mtime)
    if not matches:
        return None
    return matches[-1].resolve()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ultrasonic-jsonl", default=None)
    ap.add_argument("--nav2-jsonl", default=None)
    ap.add_argument("--orca-jsonl", default=None)
    ap.add_argument("--output-json", default=None)
    args = ap.parse_args()

    ultrasonic_path = _default_path(
        args.ultrasonic_jsonl, "logs/*/ultrasonic-session-*.jsonl"
    )
    nav2_path = _default_path(args.nav2_jsonl, "logs/*/session-*.jsonl")
    orca_path = _default_path(args.orca_jsonl, "logs/*/orca-shadow-*.jsonl")
    if ultrasonic_path is None or nav2_path is None:
        raise SystemExit("Need ultrasonic and nav2 jsonl logs to compute scorecard")

    ultrasonic_rows = _load_jsonl(ultrasonic_path)
    nav_rows = _load_jsonl(nav2_path)
    orca_rows = _load_jsonl(orca_path) if orca_path else []

    metrics = _compute_metrics(ultrasonic_rows, nav_rows, orca_rows)
    gates = _evaluate_gates(metrics)
    payload = {
        "inputs": {
            "ultrasonic_jsonl": str(ultrasonic_path),
            "nav2_jsonl": str(nav2_path),
            "orca_jsonl": str(orca_path) if orca_path else None,
        },
        **metrics,
        "gates": gates,
    }

    if args.output_json:
        out = Path(args.output_json).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"wrote: {out}")

    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
