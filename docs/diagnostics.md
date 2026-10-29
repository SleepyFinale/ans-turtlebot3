# Robot-side diagnostics

Captures and analyzers for chasing a misbehaviour on one robot. Run them on the robot unless noted.

## Motion bag: jitter, idle time, recovery events

A small bag (a few MB per 10 minutes) is enough to compare two runs:

```bash
cd ~/turtlebot3
ros2 bag record --include-hidden-topics -o logs/<name> /<robot>/cmd_vel /<robot>/odom /<robot>/stuck/event \
  /<robot>/collision/diagnosis /<robot>/navigate_to_pose/_action/status /<robot>/cmd_vel_mux/active
python3 scripts/diagnostics/analyze_motion_bag.py logs/<name>
```

It reports the `cmd_vel` rate and how often the drive or turn direction reverses within half a second (two nodes fighting over the base, or a controller oscillating), the time spent standing still with a goal active (longest spans listed), the time with no goal at all, and counts of `stuck_detector` events, `collision_resolver` verdicts and `cmd_vel_mux` hand-overs. `--include-hidden-topics` is needed for the action status topic. Copy the bag off the robot before analysing it on the central PC: SQLite cannot be read over sshfs while it is being written.

## Nav2 motion debug capture (robot side)

Use this when a robot appears to drive into obstacles even when the assigned goal and global plan look safe.

1. Start robot bringup in terminal 1:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_bringup robot.launch.py
   ```

2. Start SLAM + Nav2 with structured debug logging in terminal 2:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
     use_sim_time:=false \
     use_rviz:=false \
     enable_debug_logging:=true \
     debug_log_dir:=~/turtlebot3/logs \
     debug_log_rate_hz:=5.0
   ```

3. Start rosbag capture in terminal 3:

   ```bash
   cd ~/turtlebot3
   ./scripts/diagnostics/start_nav2_debug_capture.sh
   # Optional explicit robot name:
   # ./scripts/diagnostics/start_nav2_debug_capture.sh pinky
   ```

### Files produced

- JSONL timeline: `~/turtlebot3/logs/<robot>/session-YYYYmmdd-HHMMSS.jsonl`
- rosbag2 capture: `~/turtlebot3/logs/<robot>/bag-YYYYmmdd-HHMMSS/`

### What gets recorded

- Raw topics (bag): map, map updates, local/global costmap (+ updates), plan, cmd_vel, cmd_vel_nav, odom, tf, action status/feedback/result, and /rosout.
- Derived JSONL metrics (at `debug_log_rate_hz`):
  - robot pose in map frame
  - robot costmap cell value at current pose
  - goal and robot-to-goal distance
  - plan length, plan endpoint, and plan points outside global costmap bounds
  - cmd_vel vs cmd_vel_nav vs odom twist
  - latest NavigateToPose action status
  - anomaly flags

### Quick triage checklist

- `robot_in_lethal_cost` or `robot_in_high_cost`: robot position entered high-cost/lethal cells.
- `plan_has_out_of_global_costmap_points`: global plan includes points outside current global costmap bounds.
- `robot_in_high_cost_while_plan_low_cost`: local execution diverged from what the global plan/costmap suggested.
- `forward_cmd_in_high_cost`: motion command remained forward while the robot was already in high-cost area.
- Compare `cmd_vel_nav` vs `cmd_vel` vs `odom_twist` to separate planner/controller intent from robot motion response.

### Stop-while-active analyzer (bag only)

Use this when a robot appears to "freeze" while Nav2 still has an active goal:

```bash
cd ~/turtlebot3
python3 scripts/diagnostics/analyze_nav2_bag_stop.py \
  logs/<robot>/bag-YYYYmmdd-HHMMSS \
  --robot <robot> \
  --start <unix_start_s> \
  --end <unix_end_s>
```

What this flags:

- plan still exists (`plan_poses` above threshold),
- controller commands rotate-only / near-zero linear velocity,
- odom linear speed remains near zero,
- `distance_remaining` trend across the interval.

### Rosout clue extractor (same bag window)

Use this on the stop interval to pull likely controller/planner reasons from `/rosout`:

```bash
cd ~/turtlebot3
python3 scripts/diagnostics/analyze_nav2_bag_rosout.py \
  logs/<robot>/bag-YYYYmmdd-HHMMSS \
  --start <interval_start_unix_s> \
  --end <interval_end_unix_s>
```

Tip: pair this with the interval printed by `analyze_nav2_bag_stop.py`.

## Ultrasonic diagnostics capture (robot side)

Use this when ultrasonic topics are publishing but obstacle behavior still looks wrong (flat values, no fusion effect, or Nav2 range-layer starvation).

1. Start bringup and `navigation2_slam.launch.py` as usual.

2. In another terminal, run:

    ```bash
    cd ~/turtlebot3
    ./scripts/diagnostics/start_ultrasonic_debug_capture.sh
    # Optional explicit robot name:
    # ./scripts/diagnostics/start_ultrasonic_debug_capture.sh blinky
    ```

3. Run your obstacle test, then stop capture with Ctrl+C.

### File produced

- JSONL diagnostics: `~/turtlebot3/logs/<robot>/ultrasonic-session-YYYYmmdd-HHMMSS.jsonl`

### Key event types in JSONL

- `range_sample`: raw ultrasonic values (`l/f/r`), message age, frame_id, stale/flatline hints.
- `range_health`: per-sensor rate, inter-arrival gap, message counts, flatline streak.
- `tf_snapshot`: `base_scan -> ultrasonic_frame` lookup status and failure counters.
- `scan_compare`: raw `/scan` vs `/scan_normalized` cone-window mins per sensor.
- `fusion_effect`: estimated count of cones where normalized scan moved closer than raw.
- `nav_context`: costmap freshness, `/rosout` warning counters, latest `cmd_vel`, latest `odom` twist, and `nav2_collision_ahead` state/age.
- `anomaly`: explicit flags such as `range_stale_*`, `range_flatlined_*`, `tf_lookup_failed_*`, `fusion_no_effect_*`, `range_layer_no_input_warn`.

### Quick interpretation

- Frequent `range_flatlined_*` with healthy rate suggests sensor or transport values are stuck.
- Repeated `tf_lookup_failed_*` means fusion cannot project ultrasonic cones reliably.
- Persistent `fusion_no_effect_*` indicates ultrasonics are not changing `scan_normalized`.
- `range_layer_no_input_warn` ties directly to Nav2 logs that range layer is not receiving usable readings.

### Post-run analyzer

Use this to get an immediate summary from one ultrasonic diagnostics JSONL:

```bash
cd ~/turtlebot3
python3 scripts/diagnostics/analyze_ultrasonic_debug_session.py \
  logs/<robot>/ultrasonic-session-YYYYmmdd-HHMMSS.jsonl
```

This reports per-sensor rates/ages/stale counts, flatline streak maxima, fusion-effect counts, aggregate anomaly counts, and recent anomaly rows.

### Add map + costmap rosbag for hard cases

For "stopped then drove into bar" episodes, collect a rosbag in parallel with ultrasonic JSONL:

```bash
cd ~/turtlebot3
./scripts/diagnostics/start_nav2_debug_capture.sh
# Optional explicit robot name:
# ./scripts/diagnostics/start_nav2_debug_capture.sh blinky
```

This captures `/map`, local/global costmaps, plan, `cmd_vel`, `odom`, TF, and Nav2 action status so you can align controller intent with obstacle marking over time.
