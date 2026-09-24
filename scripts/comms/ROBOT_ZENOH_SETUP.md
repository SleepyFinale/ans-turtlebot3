# Robot: Tailscale + Zenoh setup

Run these **on each robot Pi** (blinky / pinky / inky / clyde). Scripts and the client template live in this turtlebot3 workspace (`scripts/comms/`, `config/zenoh/`). Edit them here (e.g. via sshfs); there is no sync-from-central step.

The same commands work on every robot: identity and `ROS_DOMAIN_ID` are auto-detected from `ROBOT_NAME` → `TURTLEBOT3_ROBOT_NAME` → `USER` → hostname (prefer logging in as the robot user — stock hostnames like `ubuntu` are not enough).

| Robot  | Tailscale name | ROS_DOMAIN_ID |
| ------ | -------------- | ------------- |
| Blinky | `blinky`       | `5`           |
| Pinky  | `pinky`        | `22`          |
| Inky   | `inky`         | `19`          |
| Clyde  | `clyde`        | `80`          |

## 0. Prerequisites (once per robot)

- Tailscale up with MagicDNS hostname matching the robot (`sudo tailscale set --hostname=<robot>`)
- From central: `ssh <robot>@<robot>` works (e.g. `ssh clyde@clyde`)

## 1. Install Zenoh bridge + Cyclone RMW (once per robot)

Ubuntu 22.04: do **not** use `apt install zenoh-bridge-ros2dds` (needs glibc ≥ 2.38). Use:

```bash
cd ~/turtlebot3
./scripts/comms/install_zenoh_bridge.sh
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

Keep `ZENOH_BRIDGE_VERSION` the same on central and every robot (default `1.7.2`).

## 2. Runtime (each session)

`source scripts/env/ros_robot_env.bash` sets CycloneDDS and clears `ROS_LOCALHOST_ONLY` (required so Nav2/SLAM do not exhaust Cyclone participant indices). Do not set `ROS_LOCALHOST_ONLY=1` on the robot afterward.

Zenoh still carries cross-host traffic over Tailscale; local DDS on the Pi uses normal Cyclone discovery.

Order:

1. Central: `./scripts/comms/start_zenoh_central.sh` (all robots) or `… -c` / `… -bpic` for a subset (same letters as `start_central.sh`)
2. Robot Terminal 1 — bringup:

```bash
cd ~/turtlebot3
source scripts/env/ros_robot_env.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_bringup robot.launch.py
```

3. Robot Terminal 2 — SLAM + Nav2:

```bash
cd ~/turtlebot3
source scripts/env/ros_robot_env.bash

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_sim_time:=false use_rviz:=false fleet_mode:=true nav2_use_local_slam_map:=true
```

4. Robot Terminal 3 — Zenoh client (auto-detects robot from USER / `ROBOT_NAME`):

```bash
cd ~/turtlebot3
export PATH="$HOME/.local/bin:$PATH"
./scripts/comms/start_zenoh_robot.sh          # or: … reverie
# override: ROBOT_NAME=pinky ./scripts/comms/start_zenoh_robot.sh
```

5. Central: `./scripts/core/start_central.sh --comms-mode bridged_domains`  
   (optional filter: `-c` / `-b` / `-p` / `-i`)

## 3. Sanity checks

On central (pick the robot’s domain from the table above):

```bash
source /opt/ros/humble/setup.bash
# Example for Clyde (domain 80); use 5/22/19 for blinky/pinky/inky
ROS_DOMAIN_ID=80 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 \
  ros2 topic list | grep clyde
```

(`ROS_LOCALHOST_ONLY=1` is OK on **central** for checking Zenoh-injected topics; do not use it on the robot’s Nav2 stack.)

You should see `/<robot>/map` and/or `/<robot>/map_wire_z` and TF topics while SLAM is running.

## Notes

- Leave Tailscale `--accept-routes` off.
- Do **not** expect raw DDS across TAMU without Zenoh.
- Azure hotspot debug: skip Zenoh; use normal same-LAN DDS.
- Sensor topics (map/tf) are outbound-only on the robot allowlist — do not add them under Zenoh `subscribers` on the robot (causes TF reinjection).
