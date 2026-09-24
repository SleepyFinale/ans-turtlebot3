# Clyde: Tailscale + Zenoh robot-side steps

Run these **on Clyde** (not only via the sshfs mount). Central-computer has matching scripts under `scripts/comms/` and `config/zenoh/`.

## 0. Prerequisites (done if SSH works)

- Tailscale up, hostname `clyde`
- From central: `ssh clyde@clyde` works

## 1. Copy helpers from central (once)

On **central** (easiest):

```bash
cd ~/central-computer
source scripts/env/set_robot_env.sh clyde
./scripts/comms/sync_zenoh_to_robot.sh
```

Or manually:

```bash
scp -r scripts/comms config/zenoh "$ROBOT_SSH:~/turtlebot3/"
```

## 2. Install Zenoh bridge + Cyclone RMW (once on Clyde)

Ubuntu 22.04: do **not** use `apt install zenoh-bridge-ros2dds` (needs glibc ≥ 2.38). Use:

```bash
cd ~/turtlebot3
./scripts/comms/install_zenoh_bridge.sh
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

Keep `ZENOH_BRIDGE_VERSION` the same on central and Clyde (default `1.7.2`).

## 3. Runtime (each session)

**Do not set `ROS_LOCALHOST_ONLY=1` on Clyde for bringup/SLAM.**  
With CycloneDDS that caps participant indices (~10) and Nav2+SLAM+helpers will fail with:

`Failed to find a free participant index for domain 80`

Zenoh still carries cross-host traffic over Tailscale; local DDS on the Pi can use normal Cyclone discovery.

Order:

1. Central: `./scripts/comms/start_zenoh_central.sh clyde`
2. Clyde Terminal 1 — bringup:

```bash
cd ~/turtlebot3
source scripts/env/ros_domain_profile.bash clyde   # ROS_DOMAIN_ID=80
source scripts/env/ros_robot_env.bash
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

ros2 launch turtlebot3_bringup robot.launch.py
```

3. Clyde Terminal 2 — SLAM + Nav2:

```bash
cd ~/turtlebot3
source scripts/env/ros_domain_profile.bash clyde
source scripts/env/ros_robot_env.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_sim_time:=false use_rviz:=false fleet_mode:=true nav2_use_local_slam_map:=true
```

4. Clyde Terminal 3 — Zenoh client:

```bash
cd ~/turtlebot3
export PATH="$HOME/.local/bin:$PATH"
./scripts/comms/start_zenoh_robot.sh clyde reverie
```

5. Central: `./scripts/core/start_central.sh -c --comms-mode bridged_domains`

## 4. Sanity checks

On central:

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=80 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 \
  ros2 topic list | grep clyde
```

(`ROS_LOCALHOST_ONLY=1` is OK on **central** for checking Zenoh-injected topics; do not use it on Clyde’s Nav2 stack.)

You should see `/clyde/map` and/or `/clyde/map_wire_z` and TF topics while SLAM is running.

## Notes

- Leave Tailscale `--accept-routes` off.
- Do **not** expect raw DDS across TAMU without Zenoh.
- Azure hotspot debug: skip Zenoh; use normal same-LAN DDS.
