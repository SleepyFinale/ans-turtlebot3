# TurtleBot3 fleet workspace (robot source)

This tree is the **canonical** copy of namespaced SLAM + Nav2 launch and parameters for the ANS TurtleBot3 fleet. Deploy and **`colcon build`** on each Raspberry Pi when you change packages; sshfs from a dev machine is fine for editing but local builds on the Pi are the reliable path.

## Key entry points

- **SLAM + Nav2:** `src/turtlebot3/turtlebot3_navigation2/launch/navigation2_slam.launch.py`
- **SLAM tuning:** `src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml` (and siblings)
- **Optional Pi load relief:** launch arg `scan_costmap_max_hz` (e.g. `6.0`) — costmaps subscribe to throttled `scan_costmap` while SLAM stays on full-rate `scan_normalized`.
- **Startup map seeding automation:** launch arg `enable_startup_map_seeding:=true` in `navigation2_slam.launch.py` (run before central start)

## Goal of this document

Document the steps to prepare a TurtleBot3 Raspberry Pi SBC **up through**:

- Robotis SBC setup (Ubuntu Server, Wi‑Fi/SSH, stability tweaks)
- ROS 2 Humble install and **Install and Build ROS Packages** (SBC setup Step 3.2.5, Step 2)

**Workspace = cloned repo:** Clone [ans-turtlebot3](https://github.com/SleepyFinale/ans-turtlebot3) into `~/turtlebot3` on the Pi; that repo contains the TurtleBot3/LDS/Coin D4 packages and other files needed for the workspace. After building, any changes you make in `src/` can be committed and pushed. Build/install/log are in `.gitignore`.

---

## Robot fleet reference

Use this table when configuring a given robot. SSH using the hostname or IP for that robot. All robots and any Remote PCs or central computers talking to them should share **ROS_DOMAIN_ID=50**.

| Robot  | SNS (lab)             | GCRI_LAB (gcri)      | ANS_starlink (star)  | RaspAP (rpi)        | Azure (azure)       |
| ------ | --------------------- | -------------------- | -------------------- | ------------------- | --------------------|
| Blinky | blinky@192.168.0.158  | blinky@192.168.50.158| blinky@192.168.1.158 | blinky@10.3.141.158 | blinky@172.20.10.13 |
| Pinky  | pinky@192.168.0.194   | pinky@192.168.50.194 | pinky@192.168.1.194  | pinky@10.3.141.194  | pinky@172.20.10.14  |
| Inky   | inky@192.168.0.139    | inky@192.168.50.139  | inky@192.168.1.139   | inky@10.3.141.139   | inky@172.20.10.15   |
| Clyde  | clyde@192.168.0.236   | clyde@192.168.50.236 | clyde@192.168.1.236  | clyde@10.3.141.236  | clyde@172.20.10.16  |

- **Platform**: TurtleBot3 Burger  
- **SBC**: Raspberry Pi (Ubuntu Server)  
- **ROS distro**: Humble Hawksbill (Ubuntu 22.04 / Jammy)

When following this README, substitute your robot’s hostname/IP where indicated.

For **multi-robot SLAM** (full fleet: Blinky, Pinky, Inky, Clyde), the central PC and all robots are distinguished by per-robot namespaces (e.g. `/blinky`, `/pinky`, `/inky`, `/clyde`). See [Multi-robot and central computer](#multi-robot-and-central-computer) and the [central repo](https://github.com/SleepyFinale/ans-central-computer/tree/multi-robot-slam) for the full workflow.

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

1. **Clone the TurtleBot3 workspace repo** into `turtlebot3` (this repo contains the packages needed for TurtleBot3 Burger):

   ```bash
   cd ~
   git clone https://github.com/SleepyFinale/ans-turtlebot3.git turtlebot3
   cd turtlebot3
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
   cd ~/turtlebot3/
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   colcon build --symlink-install --parallel-workers 1
   echo 'source ~/turtlebot3/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Source the workspace** (or open a new shell):

   ```bash
   source ~/turtlebot3/install/setup.bash
   ```

4. **Rebuild scripts:** After the initial build:
   - **Minimal rebuild (fast):** `scripts/minimal_rebuild.sh` builds only the packages needed for `ros2 launch turtlebot3_bringup robot.launch.py` (turtlebot3_description, turtlebot3_node, ld08_driver, turtlebot3_bringup). No clean step; use for quick iterations.
   - **Full clean rebuild:** `scripts/clean_rebuild.sh` removes `build/`, `install/`, and `log/`, runs a full colcon build, and sources the workspace. Use `--no-clean` to build without cleaning, or `--source` to only source the existing install.

   ```bash
   cd ~/turtlebot3
   ./scripts/minimal_rebuild.sh    # fast: only what robot.launch.py needs
   ./scripts/clean_rebuild.sh      # full clean + build
   ```

   Both scripts use all CPU cores by default; on a 2GB Raspberry Pi, set `COLCON_PARALLEL_JOBS=1` before running. If scripts aren’t executable: `chmod +x ~/turtlebot3/scripts/minimal_rebuild.sh ~/turtlebot3/scripts/clean_rebuild.sh`

---

## Wi‑Fi switching and static IPs (`scripts/wifi/switch_wifi.sh`)

This repo includes a helper script to switch the robot’s Wi‑Fi network and apply consistent IP settings using **netplan**.

- Script path: `scripts/wifi/switch_wifi.sh`
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

From this point on, Wi‑Fi is managed by `scripts/wifi/switch_wifi.sh`.

### Static IPs on the SNS lab Wi‑Fi

When connected to the **SNS** lab Wi‑Fi, each robot uses a fixed IP (based on the Linux user account running the script):

- **Blinky** (user `blinky`) → `192.168.0.158`
- **Pinky** (user `pinky`) → `192.168.0.194`
- **Inky** (user `inky`) → `192.168.0.139`
- **Clyde** (user `clyde`) → `192.168.0.236`

### Static IPs on the GCRI_LAB gcri Wi‑Fi

When connected to **GCRI_LAB**, each robot uses the same **last octet** as on the SNS lab network, on subnet `192.168.50.0/24`:

- **Blinky** (user `blinky`) → `192.168.50.158`
- **Pinky** (user `pinky`) → `192.168.50.194`
- **Inky** (user `inky`) → `192.168.50.139`
- **Clyde** (user `clyde`) → `192.168.50.236`

### Static IPs on the ANS_starlink star Wi‑Fi

When connected to **ANS_starlink**, each robot uses the same **last octet** as on the SNS lab network, on subnet `192.168.1.0/24`:

- **Blinky** (user `blinky`) → `192.168.1.158`
- **Pinky** (user `pinky`) → `192.168.1.194`
- **Inky** (user `inky`) → `192.168.1.139`
- **Clyde** (user `clyde`) → `192.168.1.236`

### Static IPs on the Azure azure hotspot

When connected to the **Azure** mobile hotspot, each robot also uses a fixed IP:

- **Blinky** (user `blinky`) → `172.20.10.13`
- **Pinky** (user `pinky`) → `172.20.10.14`
- **Inky** (user `inky`) → `172.20.10.15`
- **Clyde** (user `clyde`) → `172.20.10.16`

### Static IPs on RaspAP rpi Wi-Fi

When connected to **RaspAP** (e.g. Raspberry Pi hotspot), each robot uses a fixed IP (see `scripts/wifi/switch_wifi.sh`):

- **Blinky** → `10.3.141.158`
- **Pinky** → `10.3.141.194`
- **Inky** → `10.3.141.139`
- **Clyde** → `10.3.141.236`

### Usage

From `~/turtlebot3` on the robot:

```bash
cd ~/turtlebot3

# Connect to SNS lab Wi‑Fi with static IP (per robot/user)
sudo ./scripts/wifi/switch_wifi.sh lab

# Connect to GCRI_LAB with static IP (per robot/user)
sudo ./scripts/wifi/switch_wifi.sh gcri

# Connect to Starlink (ANS_starlink) with static IP (per robot/user)
sudo ./scripts/wifi/switch_wifi.sh star

# Connect to Azure mobile hotspot with static IP (per robot/user)
sudo ./scripts/wifi/switch_wifi.sh azure

# Connect to RaspAP with static IP (per robot/user)
sudo ./scripts/wifi/switch_wifi.sh rpi

# Show current Wi‑Fi SSID and wlan0 IP
./scripts/wifi/switch_wifi.sh status
```

The script uses the invoking user (`SUDO_USER`/`$USER`) to choose the static IP. To override explicitly (e.g. when logged in as a different account):

```bash
sudo ./scripts/wifi/switch_wifi.sh lab pinky
sudo ./scripts/wifi/switch_wifi.sh lab blinky
sudo ./scripts/wifi/switch_wifi.sh lab inky
sudo ./scripts/wifi/switch_wifi.sh lab clyde

# or
ROBOT_NAME=pinky sudo ./scripts/wifi/switch_wifi.sh lab
ROBOT_NAME=blinky sudo ./scripts/wifi/switch_wifi.sh lab
ROBOT_NAME=inky sudo ./scripts/wifi/switch_wifi.sh lab
ROBOT_NAME=clyde sudo ./scripts/wifi/switch_wifi.sh lab
```

If you change the SNS, GCRI_LAB, ANS_starlink, RaspAP, or Azure networks (SSID, password, gateway, or IP scheme), update the constants at the top of `scripts/wifi/switch_wifi.sh` accordingly.

### Automatic WiFi connection on boot (`scripts/wifi/boot_wifi.sh`)

To prevent the robot from being stuck without WiFi when it boots (e.g., if it was connected to the Azure hotspot before shutdown and the hotspot is not nearby), a boot-time WiFi connection script automatically attempts to connect to WiFi networks in priority order.

**Behavior:**

- On boot, the robot **first attempts to connect to SNS (lab)**.
- If that fails, it tries **GCRI_LAB (gcri)**.
- If that fails, it tries **ANS_starlink (star)**.
- If that fails, it tries **Azure (azure)**.
- This order helps the robot come up on lab or field WiFi before relying on the phone hotspot.

**Installation (one-time setup per robot):**

```bash
cd ~/turtlebot3
sudo ./scripts/wifi/install_boot_wifi.sh
```

This installs a systemd service (`boot-wifi.service`) that runs on every boot. The service:

- Detects the robot name from the hostname
- Attempts SNS first (waits up to 30 seconds per network)
- Then GCRI_LAB, then ANS_starlink, then Azure, each with the same timeout behavior
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
sudo ./scripts/wifi/boot_wifi.sh
# or specify robot name explicitly:
sudo ./scripts/wifi/boot_wifi.sh pinky
sudo ./scripts/wifi/boot_wifi.sh blinky
sudo ./scripts/wifi/boot_wifi.sh inky
sudo ./scripts/wifi/boot_wifi.sh clyde
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

The OpenCR enumerates as a USB serial device (`ttyACM*`). udev can recognize it by USB vendor and product ID (STM32 Virtual ComPort: `0483` / `5740`) and create a stable symlink **`/dev/opencr`**, so bringup and scripts do not depend on `ttyACM0` vs `ttyACM1` ordering. Bringup in this workspace defaults to `usb_port:=/dev/opencr`.

#### Step 1 — Create the rules file

```bash
sudo nano /etc/udev/rules.d/99-opencr.rules
```

#### Step 2 — Add this content

The comment line in the file reminds you which rules file you are editing.

```text
# /etc/udev/rules.d/99-opencr.rules
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE:="0666"
```

#### Step 3 — Reload and verify

Run `udevadm` to reload rules and re-trigger `tty` devices, then list `/dev/opencr`. Replug the OpenCR USB cable if the symlink is not created until the device is seen again.

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/opencr
```

`ls -l` should show `opencr` pointing at the underlying `ttyACM*` node.

**Note:** Robotis ships `99-turtlebot3-cdc.rules` in `turtlebot3_bringup` (symlink `tb3_lidar`, `ID_MM_DEVICE_IGNORE` for several USB IDs, permissions). Keeping **both** that file (copied to `/etc/udev/rules.d/`) and `99-opencr.rules` is normal: matching rules stack, and `99-opencr.rules` adds the **`opencr`** name for `0483:5740`. If you install **only** the snippet above and ModemManager ever claims the port, add `ENV{ID_MM_DEVICE_IGNORE}="1",` on the same rule line (Robotis does this for `5740` in their file).

### Stable named ports for sensors (OpenCR + LiDAR + dual GPS)

This workspace standardizes robot serial devices to fixed names:

- OpenCR: `/dev/opencr`
- LiDAR: `/dev/tb3_lidar`
- GPS #1: `/dev/gps1`
- GPS #2: `/dev/gps2`

`robot.launch.py` now defaults to these names, so bringup does not scan `ttyUSB*` order.

#### Apply udev rules from this repository

```bash
cd ~/turtlebot3
source /opt/ros/humble/setup.bash

# Install turtlebot3_bringup udev rules into /etc/udev/rules.d/
sudo src/turtlebot3/turtlebot3_bringup/script/create_udev_rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

#### Verify the named ports

```bash
ls -l /dev/opencr /dev/tb3_lidar /dev/gps1 /dev/gps2
```

Each symlink should point to the expected underlying `ttyACM*`/`ttyUSB*` node.

#### How GPS naming works (serial first, path fallback)

The `99-turtlebot3-cdc.rules` file supports a serial-first strategy and a physical-path fallback:

- If each USB-UART adapter has a unique serial, use `ENV{ID_SERIAL_SHORT}` rules.
- If serial values are duplicated/missing (common with CP2102 clones), map by `ENV{ID_PATH}`.

Inspect identifiers for each connected serial device:

```bash
udevadm info -q property -n /dev/ttyUSB0 | rg "ID_SERIAL_SHORT|ID_PATH"
udevadm info -q property -n /dev/ttyUSB1 | rg "ID_SERIAL_SHORT|ID_PATH"
udevadm info -q property -n /dev/ttyUSB2 | rg "ID_SERIAL_SHORT|ID_PATH"
```

Then update the GPS/LiDAR entries in:

- `src/turtlebot3/turtlebot3_bringup/script/99-turtlebot3-cdc.rules`

to match your hardware wiring.

#### Diagnose GPS checksum corruption quickly

If `robot.launch.py` shows repeated `nmea_serial_driver` invalid-checksum warnings, run the robot-side serial diagnostic before changing Nav2/SLAM settings:

```bash
cd ~/turtlebot3
./scripts/debug/diagnose_robot_serial_gps.sh --seconds 8
```

This checks:

- `/dev/gps1` and `/dev/gps2` symlink targets
- duplicate symlink targets (both aliases pointing at one `ttyUSB*`)
- udev identity fields (`ID_PATH`, `ID_SERIAL_SHORT`)
- short raw NMEA samples for both receivers

To isolate one receiver at a time during bringup:

```bash
ros2 launch turtlebot3_bringup robot.launch.py gps_enable_2:=false
# or
ros2 launch turtlebot3_bringup robot.launch.py gps_enable_1:=false
```

To probe likely baud rates and pick the best candidate:

```bash
python3 scripts/debug/probe_gps_baud.py /dev/gps1 --seconds 5
python3 scripts/debug/probe_gps_baud.py /dev/gps2 --seconds 5
```

Then relaunch bringup with explicit baud overrides:

```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  gps_baud_1:=9600 gps_baud_2:=9600
```

If serial data becomes clean but EKF logs still show `Failed to meet update rate`, temporarily lower rates from launch args:

```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  ekf_frequency:=4.0 navsat_frequency:=4.0
```

### ROS_DOMAIN_ID

This fleet supports two operating modes:

- `shared_domain` (legacy): all robots + central share one domain (typically `50`)
- `bridged_domains` (recommended): each robot uses its own domain and central bridges the contract topics/actions

For bridged mode, load a robot domain profile before launching bringup/Nav2:

```bash
cd ~/turtlebot3
source scripts/ros_domain_profile.bash
echo $ROS_DOMAIN_ID
```

You can also pass a robot name explicitly if hostnames differ:

```bash
source scripts/ros_domain_profile.bash pinky
```

Domain assignments are defined on central in `ans-central-computer/config/fleet_domain_map.yaml` and must stay in sync with robot hostnames.

### Multi-robot and central computer

For **multi-robot SLAM** (full fleet: Blinky, Pinky, Inky, Clyde), robots remain namespaced (`/blinky`, `/pinky`, `/inky`, `/clyde`). In `bridged_domains` mode, central bridges only the explicit fleet contract topics/actions; this reduces DDS noise and keeps Nav2 local on each robot.

To check TF and connectivity from the central PC, run (from the central workspace):  
`ROS_DOMAIN_ID=50 python3 scripts/diagnose_multirobot_tf.py`  
(The script lives in the central repo.) After pulling TF-frame changes, rebuild `turtlebot3_navigation2` on each Pi and re-run the diagnostic on the central PC with all robots up.

#### High-level changes

#### 1. Per-robot bringup with namespaces

Use the standard bringup launch file in `turtlebot3_bringup`, which now **automatically determines the robot name and applies a matching namespace** (e.g. `/blinky`, `/pinky`, `/inky`, `/clyde`):

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

# Bring up the robot (runs under /<robot_name>, e.g. /blinky)
ros2 launch turtlebot3_bringup robot.launch.py
```

#### 2. Namespaced SLAM + Nav2 on each robot

Use the standard SLAM + Nav2 launch file in `turtlebot3_navigation2`. It now **runs SLAM Toolbox, the scan normalizer, and Nav2 under the per-robot namespace**, automatically using the robot’s name.

**With the central PC** (`./scripts/start_central.sh` on the remote machine): you **must** enable fleet mode so Nav2 uses global `/tf`, `/tf_static`, and `/map` (merged map from `map_merge`, or a relay of `/<robot>/map` when only one robot runs). Use exactly this pattern:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_sim_time:=false \
  use_rviz:=false \
  fleet_mode:=true \
  nav2_use_local_slam_map:=true
```

**Path 2 (recommended with the current `ans-central-computer` stack):** `nav2_use_local_slam_map:=true` keeps Nav2’s global costmap on each robot’s **`/<robot>/map`** (SLAM) while still using **global** `/tf` from the central relay / `map_merge`. The central **`multi_robot_explorer`** transforms `NavigateToPose` / `compute_path_to_pose` goals into **`<robot>/map`**; omit `nav2_use_local_slam_map` only if you intentionally want Nav2 to consume the merged **`/map`** on the fleet graph instead.

By default, `robot.launch.py` and `navigation2_slam.launch.py` use `HOSTNAME` as the robot namespace (for example host `pinky` -> namespace `pinky`), so `robot_name:=...` is optional unless you want to override it manually. This uses `navigation_launch_multirobot.py`, remapping `tf` → `/tf`, `tf_static` → `/tf_static`, and `map` → `/map` so the robot shares the same TF graph and merged (or relayed) `/map` as the central stack. The deprecated alias `use_central_tf_map:=true` still enables the same behavior as `fleet_mode:=true`.

**Robot only** (bench test, no central stack): omit `fleet_mode` or set `fleet_mode:=false` (default). Nav2 then uses `/<robot>/tf` and the global costmap subscribes to `/<robot>/map` (see `burger.yaml`). `navigation2_slam.launch.py` starts `standalone_world_map_tf.py` so frame `map` matches `/<robot>/map` for goals.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=false use_rviz:=false
```

**Map topics:** Each robot still publishes its own SLAM map (e.g. `/pinky/map`) for `map_merge` and tools. With `fleet_mode:=true`, Nav2’s costmaps use **`/map` on the network**: multi-robot **merged** map from the central PC, or **single-robot** relay from `/<robot>/map` started by `start_central.sh`. With `fleet_mode:=false`, the global costmap static layer uses `/<robot>/map` only. On the robot, `navigation2_slam.launch.py` starts a namespaced static TF `map` → `<robot>/map` (identity) in standalone mode so Nav2 and default goals that use frame `map` match SLAM’s `<robot>/map` frame.

**Optional fleet tuning (Wi‑Fi, CPU):** Keep **Chrony** in sync on every Pi and the central PC (`scripts/chrony/setup_chrony_fleet_client.sh`; check with `scripts/chrony/verify_chrony_fleet.sh`). With **`fleet_mode:=true`**, `navigation2_slam.launch.py` supports **`nav2_use_composition:=true`** (default) to run Nav2 in one component container; **`fleet_map_relay_hz:=1.5`** (example) to throttle merged `/map` to `/map_relay` for Nav2; **`nav2_use_local_slam_map:=true`** so Nav2 costmaps use local `/<robot>/map` from SLAM instead of the network `/map` (then send goals in `/<robot>/map` or ensure your central stack transforms goals into that frame); and **`slam_toolbox_mode:=sync`** to try the synchronous SLAM Toolbox node. If both `fleet_map_relay_hz` and `nav2_use_local_slam_map` are set, the relay takes precedence.

> **Note:** The older helper script `scripts/start_slam_with_normalizer.sh` and the global `/scan` + `/scan_normalized` topics are intended for **single-robot** setups only. For typical single-robot and multi-robot operation, prefer `navigation2_slam.launch.py` instead of `start_slam_with_normalizer.sh`.

#### 3. Central computer behavior with namespaces

With this single-domain + namespace setup (standard `robot.launch.py` and `navigation2_slam.launch.py` creating per-robot namespaces automatically):

- The central computer also uses **ROS_DOMAIN_ID=50**.
- Multi-robot tools (RViZ, explorers, custom coordination nodes) can subscribe directly to:
  - `/blinky/scan_normalized`, `/blinky/map`, `/blinky/tf`, `/blinky/cmd_vel`
  - `/pinky/…`, `/inky/…`, `/clyde/…`
- You no longer *need* per-robot domains or domain bridges; robots are distinguished by their namespaces instead of `ROS_DOMAIN_ID`.

### LDS configuration

We use **LDS-02**. On the robot (SBC):

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```

`LDS-02` provides **360-degree coverage**, but that does not mean ROS always receives exactly 360 rays per scan in this driver path. The sensor uses a fixed sampling stream and the driver assembles/bins points into `LaserScan`, so beam count can vary by RPM and packet timing. In this repo, `normalize_laser_scan.py` now auto-detects a stable beam count during startup and locks `scan_normalized` to that value for SLAM/Nav2 consistency (with `target_readings` as fallback/override).

### OpenCR setup

Connect the OpenCR to the Raspberry Pi via micro USB. Use **`/dev/opencr`** for the serial port; that path requires the udev rules in [USB port settings for OpenCR](#usb-port-settings-for-opencr) (replug USB after applying them if the symlink is missing).

On the robot (SBC) run:

```bash
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
export OPENCR_PORT=/dev/opencr
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

If the upload fails, use recovery mode: hold PUSH SW2, press Reset, then release in order. After a successful upload, use the OpenCR test (PUSH SW 1 = move forward, PUSH SW 2 = rotate 180°) to verify assembly.

### Troubleshooting (Nav2 + SLAM on robot)

- **Planner: `source_frame` / frame `map` does not exist / "Could not transform the start or goal pose in the costmap frame":**
  - **Robot only** (no central PC): launch with **`fleet_mode:=false`** (default) and rebuild/install so `standalone_world_map_tf.py` runs. Check logs for `standalone_world_map_tf: Publishing static TF map -> <robot>/map`.
  - **If you use `fleet_mode:=true`:** Nav2 listens on **global** `/tf` and **`/map` must be available on the DDS graph** — start **`start_central.sh`** on the PC (multi-robot: `map_merge`; single-robot: `single_robot_map_relay.py` republishes `/<robot>/map` → `/map`). Running `fleet_mode:=true` on the robot **without** the central stack (no relay, no merge) will fail with missing `map` / costmap data.
- **Robot drives toward the goal through walls / ignores the global plan in RViz** while using the central explorer: Ensure SLAM + Nav2 was started with **`fleet_mode:=true`** (or `use_central_tf_map:=true`) when `start_central.sh` is running. On the central PC you can verify the chain with `ROS_DOMAIN_ID=50 python3 scripts/diagnose_multirobot_tf.py` (from the central workspace).
- **"No valid path found" (GridBased planner):** Goals may be in unknown space or outside the current map while SLAM is still building. The planner is configured with `allow_unknown: true` (in `burger.yaml`) so it can plan through unknown cells; if planning still fails, wait for the map to grow (move the robot slightly) or send goals closer to the current map.
- **"Sensor origin is out of map bounds":** The costmap may not yet include the robot. Wait for SLAM to publish a map that covers the robot, or move the robot slightly so the map extends; the warning often clears once the map has grown.
- **`Starting point in lethal space` / robot gets stuck near inflated obstacles:** The robot Nav2 behavior tree now runs an explicit lethal-escape sequence on planner failure (short backup -> local/global costmap clear -> replan) before falling back to wider recoveries. This does not change costmap area definitions; it changes recovery priority so the robot actively exits lethal cells and retries planning.
  - Verify this path by watching for `GridBased: failed to create plan, invalid use: Starting point in lethal space!` followed by visible reverse motion and a new path publication on `/<robot>/plan`.
  - Keep `fleet_mode:=true` when using `start_central.sh` so recovery and replanning use the same global `/tf` and `/map` graph as the central explorer.

#### Stability tuning profile (Mar 2026)

These values are tuned to reduce false `Failed to make progress` aborts when using the central explorer:

- `controller_server.progress_checker.required_movement_radius: 0.10`
- `controller_server.progress_checker.movement_time_allowance: 45.0`
- `controller_server.FollowPath.min_speed_theta: 0.10`
- `local_costmap.local_costmap.transform_timeout: 0.5`
- `global_costmap.global_costmap.transform_timeout: 0.5`
- `local_costmap.local_costmap.width/height: 4.0`
- `local_costmap.local_costmap.inflation_layer.inflation_radius: 0.25`

Use this with `fleet_mode:=true` when `start_central.sh` is running.

#### A/B/C validation sequence

Use short 8-12 minute runs with comparable space/starting pose:

1. **Run A (topology only)**: keep old parameters, but launch with `fleet_mode:=true` and explicit `robot_name:=...`.
2. **Run B (A + robot tuning)**: apply only robot `burger.yaml` tuning from this section.
3. **Run C (B + central tuning)**: apply central `multi_robot_explorer.yaml` anti-thrashing tuning.

For each run, record:

- count of `Failed to make progress` in robot Nav2 logs
- count of `Goal canceled` in central explorer logs
- average time from goal completion/cancel to next goal assignment
- rough map growth (coverage) over the same duration

#### Ultrasonic fusion baseline + A/B/C extension (Apr 2026)

Current baseline in this repo:

- OpenCR publishes `ultrasonic_l`, `ultrasonic_f`, `ultrasonic_r` (`sensor_msgs/Range`) from `turtlebot3_node`.
- `normalize_laser_scan.py` fuses those ranges into `scan_normalized`, and Nav2 consumes `scan_normalized` in costmaps.
- `normalize_laser_scan.py` auto-locks a stable target beam count from startup scans (`auto_target_readings:=true` by default), then keeps `scan_normalized` fixed-size for SLAM/Nav2.
- During auto-lock warmup, `scan_normalized` publish is briefly delayed until target count is locked, preventing SLAM from latching one beam count and later seeing a different one.
- Default launch profile is now `ultrasonic_profile:=safe` (conservative overwrite of lidar).

Recommended experiment runs on one robot (e.g. Pinky), then repeat on fleet:

1. **Run A (baseline)**: `ultrasonic_profile:=off nav2_enable_range_layer:=false`
2. **Run B (new default fusion)**: `ultrasonic_profile:=safe nav2_enable_range_layer:=false`
3. **Run C (optional direct range layer)**: `ultrasonic_profile:=safe nav2_enable_range_layer:=true`

Use the same route/start pose and keep run durations equal (8-12 min).

Robot launch examples:

```bash
cd ~/turtlebot3
./scripts/minimal_rebuild.sh

source /opt/ros/humble/setup.bash
source ~/turtlebot3/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_bringup robot.launch.py

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_sim_time:=false \
  use_rviz:=false \
  fleet_mode:=true \
  ultrasonic_profile:=safe \
  nav2_enable_range_layer:=false
```

When you pull or edit params/launch in this repo, rebuild on the Pi before relaunching Nav2.

Quick scan-normalizer verification (robot side):

```bash
# Compare raw scan length variability vs normalized fixed length
ros2 topic echo /<robot>/scan --once --field ranges | wc -w
ros2 topic echo /<robot>/scan_normalized --once --field ranges | wc -w

# Check rates (raw and normalized should both be alive)
ros2 topic hz /<robot>/scan
ros2 topic hz /<robot>/scan_normalized

# Confirm auto-lock/fallback logs from the normalizer
ros2 topic echo /rosout | rg "laser_scan_normalizer|Auto target lock|fallback"
```

### Nav2 motion debug capture (robot side)

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
   ./scripts/start_nav2_debug_capture.sh
   # Optional explicit robot name:
   # ./scripts/start_nav2_debug_capture.sh pinky
   ```

#### Files produced

- JSONL timeline: `~/turtlebot3/logs/<robot>/session-YYYYmmdd-HHMMSS.jsonl`
- rosbag2 capture: `~/turtlebot3/logs/<robot>/bag-YYYYmmdd-HHMMSS/`

#### What gets recorded

- Raw topics (bag): map, map updates, local/global costmap (+ updates), plan, cmd_vel, cmd_vel_nav, odom, tf, action status/feedback/result, and /rosout.
- Derived JSONL metrics (at `debug_log_rate_hz`):
  - robot pose in map frame
  - robot costmap cell value at current pose
  - goal and robot-to-goal distance
  - plan length, plan endpoint, and plan points outside global costmap bounds
  - cmd_vel vs cmd_vel_nav vs odom twist
  - latest NavigateToPose action status
  - anomaly flags

#### Quick triage checklist

- `robot_in_lethal_cost` or `robot_in_high_cost`: robot position entered high-cost/lethal cells.
- `plan_has_out_of_global_costmap_points`: global plan includes points outside current global costmap bounds.
- `robot_in_high_cost_while_plan_low_cost`: local execution diverged from what the global plan/costmap suggested.
- `forward_cmd_in_high_cost`: motion command remained forward while the robot was already in high-cost area.
- Compare `cmd_vel_nav` vs `cmd_vel` vs `odom_twist` to separate planner/controller intent from robot motion response.

#### Stop-while-active analyzer (bag only)

Use this when a robot appears to "freeze" while Nav2 still has an active goal:

```bash
cd ~/turtlebot3
python3 scripts/analyze_nav2_bag_stop.py \
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

#### Rosout clue extractor (same bag window)

Use this on the stop interval to pull likely controller/planner reasons from `/rosout`:

```bash
cd ~/turtlebot3
python3 scripts/analyze_nav2_bag_rosout.py \
  logs/<robot>/bag-YYYYmmdd-HHMMSS \
  --start <interval_start_unix_s> \
  --end <interval_end_unix_s>
```

Tip: pair this with the interval printed by `analyze_nav2_bag_stop.py`.

### Ultrasonic diagnostics capture (robot side)

Use this when ultrasonic topics are publishing but obstacle behavior still looks wrong (flat values, no fusion effect, or Nav2 range-layer starvation).

1. Start bringup and `navigation2_slam.launch.py` as usual.
1. In another terminal, run:

```bash
cd ~/turtlebot3
./scripts/debug/start_ultrasonic_debug_capture.sh
# Optional explicit robot name:
# ./scripts/debug/start_ultrasonic_debug_capture.sh blinky
```

1. Run your obstacle test, then stop capture with Ctrl+C.

#### File produced

- JSONL diagnostics: `~/turtlebot3/logs/<robot>/ultrasonic-session-YYYYmmdd-HHMMSS.jsonl`

#### Key event types in JSONL

- `range_sample`: raw ultrasonic values (`l/f/r`), message age, frame_id, stale/flatline hints.
- `range_health`: per-sensor rate, inter-arrival gap, message counts, flatline streak.
- `tf_snapshot`: `base_scan -> ultrasonic_frame` lookup status and failure counters.
- `scan_compare`: raw `/scan` vs `/scan_normalized` cone-window mins per sensor.
- `fusion_effect`: estimated count of cones where normalized scan moved closer than raw.
- `nav_context`: costmap freshness, `/rosout` warning counters, latest `cmd_vel`, latest `odom` twist, and `nav2_collision_ahead` state/age.
- `anomaly`: explicit flags such as `range_stale_*`, `range_flatlined_*`, `tf_lookup_failed_*`, `fusion_no_effect_*`, `range_layer_no_input_warn`.

#### Quick interpretation

- Frequent `range_flatlined_*` with healthy rate suggests sensor or transport values are stuck.
- Repeated `tf_lookup_failed_*` means fusion cannot project ultrasonic cones reliably.
- Persistent `fusion_no_effect_*` indicates ultrasonics are not changing `scan_normalized`.
- `range_layer_no_input_warn` ties directly to Nav2 logs that range layer is not receiving usable readings.

#### Post-run analyzer

Use this to get an immediate summary from one ultrasonic diagnostics JSONL:

```bash
cd ~/turtlebot3
python3 scripts/debug/analyze_ultrasonic_debug_session.py \
  logs/<robot>/ultrasonic-session-YYYYmmdd-HHMMSS.jsonl
```

This reports per-sensor rates/ages/stale counts, flatline streak maxima, fusion-effect counts, aggregate anomaly counts, and recent anomaly rows.

#### Add map + costmap rosbag for hard cases

For "stopped then drove into bar" episodes, collect a rosbag in parallel with ultrasonic JSONL:

```bash
cd ~/turtlebot3
./scripts/debug/start_nav2_debug_capture.sh
# Optional explicit robot name:
# ./scripts/debug/start_nav2_debug_capture.sh blinky
```

This captures `/map`, local/global costmaps, plan, `cmd_vel`, `odom`, TF, and Nav2 action status so you can align controller intent with obstacle marking over time.

### Ultrasonic triangulated costmap blob (Apr 2026)

For overlapping ultrasonic cones, enable triangulation-to-blob and keep a hard short-range cap:

```bash
cd ~/turtlebot3
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  nav2_enable_range_layer:=false \
  ultrasonic_profile:=safe \
  ultrasonic_hard_max_range_m:=0.50 \
  ultrasonic_triangulation_enabled:=true \
  nav2_enable_ultrasonic_blob_layer:=true \
  ultrasonic_triangulation_similarity_m:=0.06 \
  ultrasonic_triangulation_similarity_scale_per_m:=0.18 \
  ultrasonic_triangulation_similarity_max_m:=0.14 \
  ultrasonic_triangulation_blob_radius_m:=0.08 \
  ultrasonic_triangulation_max_age_sec:=0.15 \
  ultrasonic_triangulation_require_pair_agreement:=true \
  ultrasonic_disable_scan_fusion_when_blob:=true \
  ultrasonic_front_emergency_range_m:=0.38 \
  ultrasonic_front_emergency_required_streak:=1 \
  ultrasonic_front_emergency_blob_radius_m:=0.16 \
  ultrasonic_triangulation_blob_hold_sec:=0.90
```

Notes:

- `ultrasonic_hard_max_range_m` is enforced for scan fusion and triangulation input.
- Triangulation publishes `ultrasonic_blob_scan` and `ultrasonic_triangulation_debug`.
- Nav2 obstacle layers can consume this dedicated blob source via `nav2_enable_ultrasonic_blob_layer:=true`.
- `ultrasonic_front_emergency_*` adds a close-range front-only fail-safe if pair matching drops.
- `ultrasonic_triangulation_blob_hold_sec` prevents brief dropouts from immediately unmarking close obstacles.
- `ultrasonic_triangulation_similarity_*` supports distance-scaled pair/triple matching (more tolerant far, tighter near).

#### A/B validation matrix

Use the same route/object placement for all runs:

1. Baseline fusion only:
   - `ultrasonic_triangulation_enabled:=false`
   - `nav2_enable_ultrasonic_blob_layer:=false`
2. Triangulated blob only:
   - `ultrasonic_triangulation_enabled:=true`
   - `nav2_enable_ultrasonic_blob_layer:=true`
   - `ultrasonic_profile:=off`
3. Triangulated blob + front-only fusion:
   - `ultrasonic_use_left:=false ultrasonic_use_front:=true ultrasonic_use_right:=false`
4. Triangulated blob + all fusion (stress):
   - `ultrasonic_use_left:=true ultrasonic_use_front:=true ultrasonic_use_right:=true`

Compare:

- planner warnings (`Starting point in lethal space`)
- recovery loop frequency (spin/backup/wait)
- goal completion rate
- analyzer triangulation summary (`triangulation_decision` clusters and blob distance stats)
