#!/usr/bin/env python3
"""Decide hybrid advisory promotion vs full ORCA plugin from scorecard."""

import argparse
import json
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scorecard_json")
    args = ap.parse_args()

    path = Path(args.scorecard_json).expanduser().resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    gates = data.get("gates") or {}
    m = (data.get("metrics") or {})

    recommendation = "stay_hybrid_advisory"
    rationale = []
    if not gates.get("all_pass", False):
        recommendation = "hold_and_retune_advisory"
        rationale.append("One or more baseline safety gates failed.")
    elif (m.get("orca_stop_count") or 0) == 0 and (m.get("orca_slowdown_count") or 0) == 0:
        recommendation = "no_orca_benefit_detected_keep_baseline"
        rationale.append("ORCA did not materially intervene in this run set.")
    elif float(m.get("forward_when_collision_ahead_rate") or 0.0) > 0.02:
        recommendation = "consider_full_orca_plugin"
        rationale.append("Residual collision-ahead forward commands remain above target.")
    else:
        rationale.append("Hybrid advisory meets gates with measurable interventions.")

    output = {
        "recommendation": recommendation,
        "rationale": rationale,
        "scorecard": str(path),
    }
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
