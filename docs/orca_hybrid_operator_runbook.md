# ORCA Hybrid Operator Runbook

## Startup order

1. On robot: source ROS env and rebuild if code changed.
2. Start navigation stack with one of the profiles in `scripts/debug/orca_hybrid_profiles.md`.
3. On central computer: start mapping/fleet tools after robot TF is healthy.
4. Confirm topics:
   - `/scan_normalized`
   - `/ultrasonic_triangulation_debug`
   - `/nav2_motion_debug_logger` outputs JSONL
   - `/orca_shadow_debug` (when `orca_mode!=off`)

## Test checklist

1. Copy `scripts/debug/run_manifest_template.yaml` and fill the run metadata.
2. Execute scenario matrix run (doorway, corner, hallway, table legs, bag, angled pipe).
3. For each run record:
   - collisions
   - deadlock episodes
   - stuck interval
   - whether low obstacles were detected before contact
4. Generate scorecard:
   - `python3 scripts/debug/orca_hybrid_scorecard.py --output-json logs/<robot>/scorecard.json`
5. Generate recommendation:
   - `python3 scripts/debug/orca_promotion_decision.py logs/<robot>/scorecard.json`

## Emergency rollback

1. Stop launch.
2. Relaunch with rollback profile from `scripts/debug/orca_hybrid_profiles.md`.
3. Verify `orca_mode:=off`.
4. Verify safety overlays are restored (`enable_retrace_escape:=true`, `enable_ultrasonic_cmd_vel_enforcer:=true`).
5. Re-run one short smoke scenario before resuming full matrix.
