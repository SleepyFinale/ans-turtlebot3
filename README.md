# TurtleBot3 fleet workspace (robot source)

This tree is the **canonical** copy of namespaced SLAM + Nav2 launch and parameters for the ANS TurtleBot3 fleet. Deploy and **`colcon build`** on each Raspberry Pi when you change packages; sshfs from a dev machine is fine for editing but local builds on the Pi are the reliable path.

## Key entry points

- **SLAM + Nav2:** `src/turtlebot3/turtlebot3_navigation2/launch/navigation2_slam.launch.py`
- **SLAM tuning:** `src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml` (and siblings)
- **Optional Pi load relief:** launch arg `scan_costmap_max_hz` (e.g. `6.0`) — costmaps subscribe to throttled `scan_costmap` while SLAM stays on full-rate `scan_normalized`.

Central PC coordination (`map_merge`, `multi_robot_explorer`, domain bridges) lives in the **`ans-central-computer`** repo.
