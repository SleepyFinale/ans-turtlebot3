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

`source scripts/env/ros_robot_env.bash` sets CycloneDDS, clears `ROS_LOCALHOST_ONLY`, and points `CYCLONEDDS_URI` at `config/cyclonedds/robot_localhost.xml`. That file binds DDS to loopback and raises `MaxAutoParticipantIndex` so Nav2/SLAM fit. Do not set `ROS_LOCALHOST_ONLY=1` on the robot afterward, and do not unset `CYCLONEDDS_URI`.

Zenoh still carries cross-host traffic over Tailscale. Local DDS does not use Wi-Fi, so a roam does not flood the terminal with `ddsi_udp_conn_write` errors.

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

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py
```

4. Robot Terminal 3 — Zenoh client (auto-detects robot from USER / `ROBOT_NAME`):

```bash
cd ~/turtlebot3
export PATH="$HOME/.local/bin:$PATH"
./scripts/comms/start_zenoh_robot.sh          # or: … reverie
# override: ROBOT_NAME=pinky ./scripts/comms/start_zenoh_robot.sh
```

5. Central: `./scripts/core/start_central.sh`  
   (optional filter: `-c` / `-b` / `-p` / `-i`). Or skip steps 2–5 and run
   `./scripts/core/start_fleet.sh -<letters>` on the central PC.

## 3. Sanity checks

On central, while the robot's SLAM and Zenoh client are running:

```bash
cd ~/central-computer
source scripts/env/central_env.bash          # central domain 50, Cyclone, localhost
ros2 topic echo --once /clyde/map_wire_z --field layout   # data is arriving
ros2 topic list | grep clyde
```

You should see `/<robot>/map`, `/<robot>/map_wire_z`, `/<robot>/tf_zenoh` and the `navigate_to_pose` action. All robots land in the one central domain; the robot domains in the table only matter on each Pi.

## Notes

- Leave Tailscale `--accept-routes` off.
- Do **not** expect raw DDS across TAMU without Zenoh.
- Zenoh is needed on every network, Azure included: robot DDS is loopback-only.
- Sensor topics (map/tf) are outbound-only on the robot allowlist — do not add them under Zenoh `subscribers` on the robot (causes TF reinjection).
