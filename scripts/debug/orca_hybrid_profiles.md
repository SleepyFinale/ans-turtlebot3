# ORCA Hybrid Launch Profiles

## Baseline-safe profile

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_rviz:=false enable_debug_logging:=true \
  ultrasonic_cooperative_mode:=true ultrasonic_profile:=safe \
  nav2_enable_ultrasonic_blob_layer:=true nav2_ultrasonic_blob_on_global:=false \
  ultrasonic_triangulation_blob_hold_sec:=0.20 ultrasonic_hard_block_duration_sec:=0.0 \
  ultrasonic_memory_decay_duration_sec:=0.0 ultrasonic_memory_replay_in_decay:=false \
  enable_retrace_escape:=false enable_controller_collision_watch:=false \
  enable_ultrasonic_cmd_vel_enforcer:=false use_custom_bt_recovery_tree:=false \
  orca_mode:=off
```

## Rollback profile (last-known-good style)

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_rviz:=false enable_debug_logging:=true \
  ultrasonic_cooperative_mode:=false ultrasonic_profile:=safe \
  nav2_enable_ultrasonic_blob_layer:=true nav2_ultrasonic_blob_on_global:=true \
  enable_retrace_escape:=true enable_controller_collision_watch:=true \
  enable_ultrasonic_cmd_vel_enforcer:=true use_custom_bt_recovery_tree:=true \
  orca_mode:=off
```

## ORCA shadow profile

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  ultrasonic_cooperative_mode:=true orca_mode:=shadow
```

## ORCA advisory profile

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  ultrasonic_cooperative_mode:=true orca_mode:=advisory \
  orca_max_linear_scale:=0.55 orca_stop_time_max_sec:=0.80
```
