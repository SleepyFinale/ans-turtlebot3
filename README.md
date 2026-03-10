# TurtleBot3 Burger Setup Notes

This repo is used to track configuration and code changes across multiple TurtleBot3 Burger robots. The same setup steps apply to each robot to connect to them.

## References (authoritative)

- **TurtleBot3 SBC setup (Robotis e-Manual)**: `https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/`
- **ROS 2 Humble install (Ubuntu debs)**: `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`

## Goal of this document

Document the steps to prepare a TurtleBot3 Raspberry Pi SBC **up through**:

- Robotis SBC setup (Ubuntu Server, Wi‑Fi/SSH, stability tweaks)
- ROS 2 Humble install and **Install and Build ROS Packages** (SBC setup Step 3.2.5, Step 2)

**Workspace = cloned repo:** Clone [ans-turtlebot3](https://github.com/SleepyFinale/ans-turtlebot3) into `~/turtlebot3_ws` on the Pi; that repo contains the TurtleBot3/LDS/Coin D4 packages and other files needed for the workspace. After building, any changes you make in `src/` can be committed and pushed. Build/install/log are in `.gitignore`.

---

## Robot fleet reference

Use this table when configuring a given robot. SSH using the hostname or IP for that robot. All robots and any Remote PCs or central computers talking to them should share **ROS_DOMAIN_ID=50**.

| Robot  | Lab (SSID: SNS)       | RaspAP (rpi)        | Azure (SSID: Azure) |
| ------ | --------------------- | ------------------- | --------------------|
| Blinky | blinky@192.168.0.158  | blinky@10.3.141.220 | blinky@172.20.10.13 |
| Pinky  | pinky@192.168.0.194   | pinky@10.3.141.194  | pinky@172.20.10.14  |
| Inky   | inky@192.168.0.139    | inky@10.3.141.139   | inky@172.20.10.15   |
| Clyde  | `clyde@<IP>`          | `clyde@<IP>`        | `clyde@<IP>`        |

- **Platform**: TurtleBot3 Burger  
- **SBC**: Raspberry Pi (Ubuntu Server)  
- **ROS distro**: Humble Hawksbill (Ubuntu 22.04 / Jammy)

When following this README, substitute your robot’s hostname/IP where indicated.

For **multi-robot SLAM** (e.g. Blinky + Pinky), the central PC and all robots share are distinguished by per-robot namespaces (e.g. `/blinky`, `/pinky`). See [Multi-robot and central computer](#multi-robot-and-central-computer) and the [central repo](https://github.com/SleepyFinale/ans-central-computer/tree/multi-robot-slam) for the full workflow.

---

## SBC Setup

### Install Ubuntu Server 22.04 onto microSD (Remote PC)

Robotis uses Raspberry Pi Imager to install Ubuntu Server 22.04 for Raspberry Pi.

- Install Raspberry Pi Imager: follow `https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/`
- In Raspberry Pi Imager:
  - Choose OS: **Ubuntu Server 22.04 LTS (64-bit)** (Server OS, not Desktop)
  - Choose storage: your microSD
  - Use the “Edit Setting” flow to set:
    - **username/password**
    - **Wi‑Fi SSID/password**
    - **Enable SSH** (password authentication)

### First Boot + Basic Configuration (TurtleBot3 SBC)

- Boot the Raspberry Pi with HDMI + keyboard (first boot is easiest locally).
- Log in (Ubuntu will prompt you to change password on first login if using the default user).

If Wi‑Fi was not configured via Imager, Robotis configures Wi‑Fi via netplan:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Edit Wi‑Fi settings (replace with your SSID/password), then save and reboot.

### Disable Unattended Upgrades

Robotis disables auto-upgrades to avoid surprise background package operations:

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

Set:

```text
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

### Prevent Boot Delays if Network is Slow/Unavailable

```bash
sudo systemctl mask systemd-networkd-wait-online.service
```

### Disable Sleep/Hibernate Targets

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

### Reboot

```bash
sudo reboot
```

### SSH in from a Remote PC

After reboot, SSH to the SBC. Use the **username@IP** (or hostname) for the robot you’re setting up—see the [Robot fleet reference](#robot-fleet-reference) table. For example, for Pinky on the SNS lab Wi‑Fi: `pinky@192.168.0.194`.

```bash
ssh <USERNAME>@<ROBOT_IP>
# e.g. ssh pinky@192.168.0.194
```

If you need to find the IP on the SBC, Robotis suggests installing net-tools:

```bash
sudo apt update
sudo apt install -y net-tools
ifconfig
```

### Swap File for Low-Memory Pi Models

If the Raspberry Pi has only **2GB RAM**, Robotis recommends adding swap before building packages later:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h
```

---

## ROS 2 Humble Install

Robotis points to the official ROS 2 Humble install guide. At this point we have completed:

### Step 1: Set locale (UTF‑8)

From `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#set-locale`:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

## Install and Build ROS Packages

After ROS 2 Humble is fully installed on the Pi (including "Setup Sources" and "Install ROS 2 packages"), run **Step 2** of "Install and Build ROS Packages". Clone the preconfigured workspace repo and install dependencies, then build.

**Reference:** [Robotis e-Manual — SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)

### On the Raspberry Pi

1. **Clone the TurtleBot3 workspace repo** into `turtlebot3_ws` (this repo contains the packages needed for TurtleBot3 Burger):

   ```bash
   cd ~
   git clone https://github.com/SleepyFinale/ans-turtlebot3.git turtlebot3_ws
   cd turtlebot3_ws
   ```

2. **Install dependencies and build** (use wall power; build can take over an hour):

   ```bash
   sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
   sudo apt install ros-humble-hls-lfcd-lds-driver
   sudo apt install ros-humble-turtlebot3-msgs
   sudo apt install ros-humble-nav2-bringup
   sudo apt install ros-humble-dynamixel-sdk
   sudo apt install ros-humble-xacro
   sudo apt install libudev-dev
   cd ~/turtlebot3_ws/
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   colcon build --symlink-install --parallel-workers 1
   echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Source the workspace** (or open a new shell):

   ```bash
   source ~/turtlebot3_ws/install/setup.bash
   ```

4. **Rebuild scripts:** After the initial build:
   - **Minimal rebuild (fast):** `scripts/minimal_rebuild.sh` builds only the packages needed for `ros2 launch turtlebot3_bringup robot.launch.py` (turtlebot3_description, turtlebot3_node, ld08_driver, turtlebot3_bringup). No clean step; use for quick iterations.
   - **Full clean rebuild:** `scripts/clean_rebuild.sh` removes `build/`, `install/`, and `log/`, runs a full colcon build, and sources the workspace. Use `--no-clean` to build without cleaning, or `--source` to only source the existing install.

   ```bash
   cd ~/turtlebot3_ws
   ./scripts/minimal_rebuild.sh    # fast: only what robot.launch.py needs
   ./scripts/clean_rebuild.sh      # full clean + build
   ```

   Both scripts use all CPU cores by default; on a 2GB Raspberry Pi, set `COLCON_PARALLEL_JOBS=1` before running. If scripts aren’t executable: `chmod +x ~/turtlebot3_ws/scripts/minimal_rebuild.sh ~/turtlebot3_ws/scripts/clean_rebuild.sh`

---

## Wi‑Fi switching and static IPs (`scripts/switch_wifi.sh`)

This repo includes a helper script to switch the robot’s Wi‑Fi network and apply consistent IP settings using **netplan**.

- Script path: `scripts/switch_wifi.sh`
- Netplan override file used: `/etc/netplan/99-wifi-switch.yaml`

### One‑time prerequisite

On each robot, ensure that `wlan0` is **only** configured via this script (to avoid duplicate netplan definitions):

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Comment out or remove the `wifis:` / `wlan0:` block from `50-cloud-init.yaml`, leaving any `ethernets:` config intact. Then:

```bash
sudo netplan apply
```

From this point on, Wi‑Fi is managed by `scripts/switch_wifi.sh`.

### Static IPs on the SNS lab Wi‑Fi

When connected to the **SNS** lab Wi‑Fi, each robot uses a fixed IP (based on the Linux user account running the script):

- **Blinky** (user `blinky`) → `192.168.0.158`
- **Pinky** (user `pinky`) → `192.168.0.194`

Gateway for SNS is assumed to be `192.168.0.1` with prefix `/24`.

### Static IPs on the Azure hotspot

When connected to the **Azure** mobile hotspot, each robot also uses a fixed IP:

- **Blinky** (user `blinky`) → `172.20.10.13/28`
- **Pinky** (user `pinky`) → `172.20.10.14/28`

Gateway for Azure is assumed to be `172.20.10.1` with prefix `/28` (typical iOS hotspot range).

### Usage

From `~/turtlebot3_ws` on the robot:

```bash
cd ~/turtlebot3_ws

# Connect to SNS lab Wi‑Fi with static IP (per robot/user)
sudo ./scripts/switch_wifi.sh lab

# Connect to Azure mobile hotspot with static IP (per robot/user)
sudo ./scripts/switch_wifi.sh azure

# Show current Wi‑Fi SSID and wlan0 IP
./scripts/switch_wifi.sh status
```

The script uses the invoking user (`SUDO_USER`/`$USER`) to choose the static IP. To override explicitly (e.g. when logged in as a different account):

```bash
sudo ./scripts/switch_wifi.sh lab pinky
sudo ./scripts/switch_wifi.sh lab blinky

# or
ROBOT_NAME=pinky sudo ./scripts/switch_wifi.sh lab
ROBOT_NAME=blinky sudo ./scripts/switch_wifi.sh lab
```

You can use the same override pattern for Azure:

```bash
sudo ./scripts/switch_wifi.sh azure pinky
sudo ./scripts/switch_wifi.sh azure blinky

# or
ROBOT_NAME=pinky sudo ./scripts/switch_wifi.sh azure
ROBOT_NAME=blinky sudo ./scripts/switch_wifi.sh azure
```

If you change the SNS or Azure networks (SSID, password, gateway, or IP scheme), update the constants at the top of `scripts/switch_wifi.sh` accordingly.

### Automatic WiFi connection on boot (`scripts/boot_wifi.sh`)

To prevent the robot from being stuck without WiFi when it boots (e.g., if it was connected to the Azure hotspot before shutdown and the hotspot is not nearby), a boot-time WiFi connection script automatically attempts to connect to WiFi networks in priority order.

**Behavior:**

- On boot, the robot **first attempts to connect to SNS (lab WiFi)**
- If SNS is unavailable or connection fails, it **automatically falls back to Azure (hotspot)**
- This ensures the robot can connect to WiFi even if the hotspot is not nearby when it boots

**Installation (one-time setup per robot):**

```bash
cd ~/turtlebot3_ws
sudo ./scripts/install_boot_wifi.sh
```

This installs a systemd service (`boot-wifi.service`) that runs on every boot. The service:

- Detects the robot name from the hostname
- Attempts to connect to SNS first (waits up to 30 seconds)
- If SNS fails, switches to Azure and waits for connection
- Logs all connection attempts to the systemd journal

**Checking boot WiFi status:**

```bash
# Check if the service is running/enabled
sudo systemctl status boot-wifi.service

# View boot WiFi connection logs
sudo journalctl -u boot-wifi.service

# View recent logs (last 50 lines)
sudo journalctl -u boot-wifi.service -n 50

# Follow logs in real-time
sudo journalctl -u boot-wifi.service -f
```

**Manual testing:**

You can test the boot WiFi script manually (without rebooting):

```bash
sudo ./scripts/boot_wifi.sh
# or specify robot name explicitly:
sudo ./scripts/boot_wifi.sh pinky
sudo ./scripts/boot_wifi.sh blinky
```

**Disabling boot WiFi (if needed):**

If you need to disable automatic WiFi connection on boot:

```bash
sudo systemctl disable boot-wifi.service
sudo systemctl stop boot-wifi.service
```

To re-enable:

```bash
sudo systemctl enable boot-wifi.service
sudo systemctl start boot-wifi.service
```

### USB port settings for OpenCR

After the workspace is built, set udev rules so the OpenCR is accessible:

```bash
sudo cp $(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### ROS_DOMAIN_ID

In this fleet, **all robots and all Remote PCs/central computers share a single ROS domain: `ROS_DOMAIN_ID=50`**. Every machine that needs to talk to the robots should use this same value.

On the robot (SBC):

```bash
echo 'export ROS_DOMAIN_ID=50' >> ~/.bashrc
source ~/.bashrc
```

**Warning (e-Manual):** Do not use the same ROS_DOMAIN_ID as another unrelated robot or PC on the same network, or ROS 2 traffic will conflict. Within this fleet we intentionally standardize on **50** across all machines.

### Multi-robot and central computer

For **multi-robot SLAM** (e.g. Blinky + Pinky), the **central PC** and all robots are distinguished by their namespaces (e.g. `/blinky`, `/pinky`). Domain bridges are no longer required in the standard setup; see the central repo for the multi-robot SLAM workflow and diagnostic commands: [ans-central-computer (multi-robot-slam branch)](https://github.com/SleepyFinale/ans-central-computer/tree/multi-robot-slam).

To check TF and connectivity from the central PC, run (from the central workspace):  
`ROS_DOMAIN_ID=50 python3 scripts/diagnose_multirobot_tf.py`  
(The script lives in the central repo.)

#### High-level changes

#### 1. Per-robot bringup with namespaces

Use the namespaced bringup wrapper in `turtlebot3_bringup`:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=50

# Example: Blinky
ros2 launch turtlebot3_bringup robot_namespaced.launch.py robot_name:=blinky

# Example: Pinky
ros2 launch turtlebot3_bringup robot_namespaced.launch.py robot_name:=pinky
```

`robot_namespaced.launch.py` computes an **effective namespace** from:

- `namespace` (if provided explicitly), otherwise
- `robot_name` (e.g. `blinky`, `pinky`, `inky`, `clyde`).

It then forwards that namespace into the existing `robot.launch.py`, so all bringup nodes (state publisher, lidar driver, `turtlebot3_node`) run under `/blinky`, `/pinky`, etc.

#### 2. Namespaced SLAM + Nav2 on each robot

Instead of running `navigation2_slam.launch.py` directly, use the namespaced variant in `turtlebot3_navigation2`:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=50

# Example: Blinky
ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py \
  robot_name:=blinky \
  use_sim_time:=false \
  use_rviz:=false

# Example: Pinky
ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py \
  robot_name:=pinky \
  use_sim_time:=false \
  use_rviz:=false
```

`navigation2_slam_namespaced.launch.py` does the following for each robot:

- Computes an effective namespace from `namespace` / `robot_name` (same rule as bringup).
- Starts **laser scan normalizer** inside that namespace:
  - Subscribes to `scan` (resolved as `/<robot>/scan`).
  - Publishes `scan_normalized` (resolved as `/<robot>/scan_normalized`).
  - Sets `frame_id_prefix:=<robot>` so scan TF frame is `<robot>/base_scan`.
- Starts **SLAM Toolbox** inside the namespace, consuming `scan_normalized`.
- Starts **Nav2** (via `nav2_bringup`) inside the same namespace so all navigation topics/actions are under `/<robot>/...`.

> **Note:** The older helper script `scripts/start_slam_with_normalizer.sh` and the global `/scan` + `/scan_normalized` topics are intended for **single-robot** setups only. For multi-robot namespaced operation, use `navigation2_slam_namespaced.launch.py` per robot instead of `start_slam_with_normalizer.sh`.

#### 3. Central computer behavior with namespaces

With this single-domain + namespace setup:

- The central computer also uses **ROS_DOMAIN_ID=50**.
- Multi-robot tools (RViZ, explorers, custom coordination nodes) can subscribe directly to:
  - `/blinky/scan_normalized`, `/blinky/map`, `/blinky/tf`, `/blinky/cmd_vel`
  - `/pinky/…`, `/inky/…`, `/clyde/…`
- You no longer *need* per-robot domains or domain bridges; robots are distinguished by their namespaces instead of `ROS_DOMAIN_ID`.

The existing domain-bridge–based workflow remains supported; this namespaced single-domain mode is an **alternative** configuration for experiments and future multi-robot setups.

### LDS configuration

We use **LDS-02**. On the robot (SBC):

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```

### OpenCR setup

Connect the OpenCR to the Raspberry Pi via micro USB, then on the robot (SBC) run:

```bash
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

If the upload fails, use recovery mode: hold PUSH SW2, press Reset, then release in order. After a successful upload, use the OpenCR test (PUSH SW 1 = move forward, PUSH SW 2 = rotate 180°) to verify assembly.

### Troubleshooting (Nav2 + SLAM on robot)

- **"No valid path found" (GridBased planner):** Goals may be in unknown space or outside the current map while SLAM is still building. The planner is configured with `allow_unknown: true` (in `burger.yaml`) so it can plan through unknown cells; if planning still fails, wait for the map to grow (move the robot slightly) or send goals closer to the current map.
- **"Sensor origin is out of map bounds":** The costmap may not yet include the robot. Wait for SLAM to publish a map that covers the robot, or move the robot slightly so the map extends; the warning often clears once the map has grown.
