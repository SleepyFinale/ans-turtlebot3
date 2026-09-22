# Clyde: Tailscale + Zenoh robot-side steps

Run these **on Clyde** (not only via the sshfs mount). Central-computer has matching scripts under `scripts/comms/` and `config/zenoh/`.

## 0. Prerequisites (done if SSH works)

- Tailscale up, hostname `clyde`
- From central: `ssh clyde@clyde` works

## 1. Copy helpers from central (once)

On **central**:

```bash
cd ~/central-computer
source scripts/env/set_robot_env.sh clyde
scp -r scripts/comms config/zenoh "$ROBOT_SSH:~/turtlebot3/"
```

## 2. Install Zenoh bridge + Cyclone RMW (once on Clyde)

**Important:** On Ubuntu 22.04, `apt install zenoh-bridge-ros2dds` fails (`libc6 >= 2.38`). Use the standalone musl installer instead (same version as central):

```bash
cd ~/turtlebot3
./scripts/comms/install_zenoh_bridge.sh
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

Keep `ZENOH_BRIDGE_VERSION` the same on central and Clyde (default `1.7.2` in the install script).

## 3. Runtime (each session)

Order:

1. Central: `./scripts/comms/start_zenoh_central.sh clyde`
2. Clyde: bringup + SLAM/Nav2 with `ROS_DOMAIN_ID=80` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. Clyde Zenoh client:

```bash
cd ~/turtlebot3
source /opt/ros/humble/setup.bash
# source your robot workspace setup.bash if needed
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=80
export ROS_LOCALHOST_ONLY=1
export PATH="$HOME/.local/bin:$PATH"
./scripts/comms/start_zenoh_robot.sh clyde reverie
# If MagicDNS fails: ./scripts/comms/start_zenoh_robot.sh clyde 100.75.98.110
```

4. Central: `./scripts/core/start_central.sh -c --comms-mode bridged_domains`

## 4. Sanity checks

On central:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
ROS_DOMAIN_ID=80 ros2 topic list | grep clyde
```

You should see `/clyde/map` and/or `/clyde/map_wire_z` and TF topics while SLAM is running.

## Notes

- Leave `--accept-routes` off on Tailscale.
- Do **not** expect raw DDS/`ros2 topic list` across TAMU without Zenoh.
- Azure hotspot debug: skip Zenoh; use normal same-LAN DDS.
