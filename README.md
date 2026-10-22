# TurtleBot3 fleet workspace (robot side)

This is the robot half of the fleet. Each Raspberry Pi runs bringup, SLAM Toolbox and Nav2 under its own namespace (`/blinky`, `/pinky`, `/inky`, `/clyde`) and talks to the central PC only through Zenoh over Tailscale. The central PC (`~/central-computer`) picks frontiers and sends `NavigateToPose` goals; everything that keeps the robot safe and moving runs here, so a dropped link stops new goals but never drives the robot blind.

## Key entry points

| What | Where |
| --- | --- |
| Bringup (base, lidar, URDF/TF) | `src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py` |
| SLAM + Nav2 + helpers | `src/turtlebot3/turtlebot3_navigation2/launch/navigation2_slam.launch.py` |
| Nav2 servers (namespaced TF) | `src/turtlebot3/turtlebot3_navigation2/launch/navigation_launch_multirobot.py` |
| Nav2 / SLAM parameters | `src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml`, `mapper_params_online_async_fast.yaml` |
| Zenoh link to the central PC | `scripts/comms/start_zenoh_robot.sh`, `config/zenoh/robot_client.json5.tmpl` |
| Odometry fusion (EKF) | `src/turtlebot3/turtlebot3_bringup/param/humble/ekf.yaml`, `turtlebot3_navigation2/scripts/stereo_visual_odometry.py` ([Sensor fusion](#sensor-fusion-odometry-ekf)) |
| Stuck detection, low-obstacle memory | `turtlebot3_navigation2/scripts/stuck_detector.py`, `low_obstacle_memory.py` ([Stuck detection and low-obstacle memory](#stuck-detection-and-low-obstacle-memory)) |
| Shell environment | `scripts/env/ros_robot_env.bash` (sources ROS, overlay, Cyclone loopback config, domain) |

## Quick start

Normally you do not log in to the robot at all. From the central PC:

```bash
cd ~/central-computer
./scripts/core/start_fleet.sh -i        # inky; letters b/p/i/c pick robots
```

That SSHes in and runs the three robot commands below, then starts the central stack. To run them by hand, use three SSH terminals on the robot:

```bash
# 1. bringup
ros2 launch turtlebot3_bringup robot.launch.py
# 2. SLAM + Nav2 (waits for odom TF, then starts Nav2; a short forward move seeds the map)
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py
# 3. Zenoh link to the central PC (Tailscale name of the central PC)
./scripts/comms/start_zenoh_robot.sh reverie
```

`~/.bashrc` (see [Install and Build ROS Packages](#install-and-build-ros-packages)) sources `scripts/env/ros_robot_env.bash` and sets `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-02`.

## Robot fleet reference

| Robot  | Tailscale (preferred) | Azure hotspot         | Robot ROS domain |
| ------ | --------------------- | --------------------- | ---------------- |
| Blinky | `blinky@blinky`       | `blinky@172.20.10.13` | 5  |
| Pinky  | `pinky@pinky`         | `pinky@172.20.10.14`  | 22 |
| Inky   | `inky@inky`           | `inky@172.20.10.15`   | 19 |
| Clyde  | `clyde@clyde`         | `clyde@172.20.10.16`  | 80 |

On boot the Pi tries **TAMU_WiFi** (WPA Enterprise, DHCP) first, then the **Azure** hotspot. Tailscale gives the same name on either network. Hardware: TurtleBot3 Burger, Raspberry Pi 4, Ubuntu Server 22.04, ROS 2 Humble.

The namespace is the hostname unless it is a stock name such as `ubuntu`; then it is the login user. Pass `robot_name:=<name>` to both launch files to override it.

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

### Disable Wi-Fi Power Saving (`wlan0`)

Disable Wi-Fi power saving early in SBC setup to reduce latency spikes and connection dropouts during robot bringup and multi-robot operation.

1. Install `iw`:

   ```bash
   sudo apt install iw
   ```

2. Create a one-shot systemd service:

   ```bash
   sudo nano /etc/systemd/system/wifi-powersave-off.service
   ```

   Paste:

   ```ini
   [Unit]
   Description=Disable Wi-Fi power save on wlan0
   After=network-pre.target
   Wants=network-pre.target

   [Service]
   Type=oneshot
   ExecStart=/usr/sbin/iw dev wlan0 set power_save off
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```

3. Reload systemd and enable the service:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now wifi-powersave-off.service
   ```

4. Verify:

   ```bash
   systemctl status wifi-powersave-off.service --no-pager
   iw dev wlan0 get power_save
   ```

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

After reboot, SSH to the SBC. Use the **username@IP** (or hostname) for the robot you’re setting up—see the [Robot fleet reference](#robot-fleet-reference) table. For example, for Pinky on Azure: `ssh pinky@172.20.10.14`; on TAMU_WiFi: `ssh pinky@<dhcp-ip>`.

```bash
ssh <USERNAME>@<ROBOT_IP>
# e.g. ssh pinky@172.20.10.14
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

2. **Install dependencies** (required for `robot.launch.py` + `navigation2_slam.launch.py`):

   ```bash
   sudo apt update
   sudo apt install -y \
     python3-argcomplete python3-colcon-common-extensions \
     build-essential libboost-system-dev libudev-dev \
     ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization \
     ros-humble-dynamixel-sdk ros-humble-turtlebot3-msgs \
     ros-humble-hls-lfcd-lds-driver ros-humble-xacro
   ```

   Optional (only if you run outdoor GPS mode with `outdoor:=true` in `robot.launch.py`):

   ```bash
   sudo apt install -y ros-humble-nmea-navsat-driver
   ```

   Quick verification (recommended):

   ```bash
   ros2 pkg list | rg "slam_toolbox|robot_localization|nav2_bringup|turtlebot3_msgs|dynamixel_sdk"
   ```

3. **Build and source the workspace** (use wall power; build can take over an hour):

   ```bash
   cd ~/turtlebot3/
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   colcon build --symlink-install --parallel-workers 1
   echo 'source ~/turtlebot3/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Recommended `~/.bashrc` block (robot):**

   Add this block to `~/.bashrc` so interactive shells source the ROS/TurtleBot3 environment automatically and start in `~/turtlebot3`:

   ```bash
   # --- ROS 2 / TurtleBot3 environment ---
   # Sources /opt/ros/humble + ~/turtlebot3/install/setup.bash with error checking.
   if [ -f "$HOME/turtlebot3/scripts/env/ros_robot_env.bash" ]; then
       # shellcheck disable=SC1091
       source "$HOME/turtlebot3/scripts/env/ros_robot_env.bash"
   fi

   # Per-robot ROS_DOMAIN_ID (USER -> blinky=05, pinky=22, inky=19, clyde=80).
   if [ -f "$HOME/turtlebot3/scripts/env/ros_domain_profile.bash" ]; then
       # shellcheck disable=SC1091
       source "$HOME/turtlebot3/scripts/env/ros_domain_profile.bash"
   fi

   # TurtleBot3 hardware model + LiDAR variant.
   export TURTLEBOT3_MODEL=burger
   export LDS_MODEL=LDS-02

   # Default cwd for interactive shells (includes console auto-login via .profile).
   cd "$HOME/turtlebot3" 2>/dev/null || true
   ```

   Then open a new shell or run:

   ```bash
   source ~/.bashrc
   ```

5. **Source the workspace** (or open a new shell):

   ```bash
   source ~/turtlebot3/install/setup.bash
   ```

6. **Rebuild script:** After the initial build, use `scripts/build/rebuild_common.sh`:
   - **Minimal rebuild (fast):** `minimal` mode builds only the packages needed for `ros2 launch turtlebot3_bringup robot.launch.py`.
   - **Full rebuild:** `clean` mode removes `build/`, `install/`, and `log/`, then runs a full colcon build and sources the workspace.
   - Optional flags for `clean`: `--no-clean` (skip deletion) and `--source` (source only).

   ```bash
   cd ~/turtlebot3
   ./scripts/build/rebuild_common.sh minimal
   ./scripts/build/rebuild_common.sh clean
   ./scripts/build/rebuild_common.sh clean --no-clean
   ./scripts/build/rebuild_common.sh clean --source
   ```

   The script defaults to `COLCON_PARALLEL_JOBS=1` (safe for low-memory SBCs). Override when needed, e.g. `COLCON_PARALLEL_JOBS=2 ./scripts/build/rebuild_common.sh minimal`.

---

## Wi‑Fi switching (`scripts/network/switch_wifi.sh`)

This repo includes a helper script to switch the robot’s Wi‑Fi between the **Azure** hotspot and **TAMU_WiFi** using **netplan**.

- Script path: `scripts/network/switch_wifi.sh`
- Netplan override file used: `/etc/netplan/99-wifi-switch.yaml`
- Default TAMU NetID identity is built into the script (`schen08`); override with `TAMU_IDENTITY` / `TAMU_PASSWORD` if needed.

### Recommended right after cloning this repo

After cloning `~/turtlebot3` on a robot, set up Wi-Fi management in this order:

1. **One-time prerequisite:** ensure only `switch_wifi.sh` manages `wlan0` to avoid duplicate netplan definitions. Comment out (or remove) the `wifis:` / `wlan0:` block in:

   ```bash
   sudo nano /etc/netplan/50-cloud-init.yaml
   sudo netplan apply
   ```

2. Install the boot-time Wi-Fi service from this repo:

   ```bash
   cd ~/turtlebot3
   sudo ./scripts/network/install_boot_wifi.sh
   ```

3. Use the Wi-Fi switch helper for normal network changes:

   ```bash
   cd ~/turtlebot3
   sudo ./scripts/network/switch_wifi.sh azure
   sudo ./scripts/network/switch_wifi.sh tamu
   ./scripts/network/switch_wifi.sh status
   ```

### Static IPs on the Azure hotspot

When connected to **Azure**, each robot uses a fixed IP (based on the Linux user account running the script):

- **Blinky** (user `blinky`) → `172.20.10.13`
- **Pinky** (user `pinky`) → `172.20.10.14`
- **Inky** (user `inky`) → `172.20.10.15`
- **Clyde** (user `clyde`) → `172.20.10.16`

Gateway: `172.20.10.1`, prefix `/28`.

### Usage

From `~/turtlebot3` on the robot:

```bash
cd ~/turtlebot3

# Connect to Azure hotspot with static IP (per robot/user)
sudo ./scripts/network/switch_wifi.sh azure

# Connect to TAMU_WiFi with DHCP (WPA enterprise; credentials default in script)
sudo ./scripts/network/switch_wifi.sh tamu

# Show current Wi‑Fi SSID and wlan0 IP
./scripts/network/switch_wifi.sh status
```

The Azure mode uses the invoking user (`SUDO_USER`/`$USER`) to choose the static IP. To override explicitly:

```bash
sudo ./scripts/network/switch_wifi.sh azure pinky
# or
ROBOT_NAME=pinky sudo ./scripts/network/switch_wifi.sh azure
```

Optional TAMU overrides:

```bash
sudo env TAMU_IDENTITY=<netid> TAMU_PASSWORD='<password>' ./scripts/network/switch_wifi.sh tamu
```

If you change the Azure or TAMU networks (SSID, password, gateway, or IP scheme), update the constants at the top of `scripts/network/switch_wifi.sh` accordingly.

### Automatic WiFi connection on boot (`scripts/network/boot_wifi.sh`)

To prevent the robot from being stuck without WiFi when it boots, a boot-time WiFi connection script automatically attempts networks in priority order.

**Behavior:**

- On boot, the robot **first attempts to connect to TAMU_WiFi**.
- If that fails, it tries **Azure** (hotspot).
- Each attempt waits up to 30 seconds (TAMU: DHCP IP plus reachable gateway or `8.8.8.8`; Azure: ping `172.20.10.1`).

**Installation (one-time setup per robot):**

```bash
cd ~/turtlebot3
sudo ./scripts/network/install_boot_wifi.sh
```

This installs a systemd service (`boot-wifi.service`) that runs on every boot. The service:

- Detects the robot name from the hostname (needed for Azure static IP)
- Attempts TAMU_WiFi first, then Azure
- Logs connection attempts to the systemd journal

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
sudo ./scripts/network/boot_wifi.sh
# or specify robot name explicitly:
sudo ./scripts/network/boot_wifi.sh pinky
sudo ./scripts/network/boot_wifi.sh blinky
sudo ./scripts/network/boot_wifi.sh inky
sudo ./scripts/network/boot_wifi.sh clyde
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
./scripts/diagnostics/diagnose_robot_serial_gps.sh --seconds 8
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
python3 scripts/diagnostics/probe_gps_baud.py /dev/gps1 --seconds 5
python3 scripts/diagnostics/probe_gps_baud.py /dev/gps2 --seconds 5
```

Then relaunch bringup with explicit baud overrides:

```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  gps_baud_1:=9600 gps_baud_2:=9600
```

If serial data becomes clean but EKF logs still show `Failed to meet update rate`, lower the rates from launch args. The EKF also publishes `odom -> base_footprint`, so keep `ekf_frequency` at 10 Hz or more for Nav2:

```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  ekf_frequency:=10.0 navsat_frequency:=4.0
```

### Sensor fusion (odometry EKF)

`robot.launch.py` runs a `robot_localization` EKF (`ekf_filter_node`) on every robot. It publishes the fused estimate on `/<robot>/odom` and owns the `odom -> base_footprint` transform, so SLAM, Nav2 and the helper nodes use it with no changes. Every robot has the standard sensors; cameras and GPS are fused only where they exist:

| Sensor | Topic | Fused | Present on |
| --- | --- | --- | --- |
| Wheel encoders | `odom/wheel` | forward speed | all robots |
| 9-axis IMU (OpenCR) | `imu` | heading, yaw rate | all robots (the magnetometer reads zero, so heading is gyro + accel) |
| Stereo visual odometry | `odom/visual` | forward speed, yaw rate | robots with two cameras and a usable `config/stereo/<robot>.yaml` (Inky) |
| GPS via `navsat_transform` | `odometry/gps` | x, y | `outdoor:=true` only |
| 360° LiDAR | — | through `slam_toolbox` scan matching (`map -> odom`) | all robots |

Wheel yaw rate is deliberately not fused: wheels over-count turns when they slip on carpet. In a 90° spin on Inky (LiDAR scan-match reference 84.5°), wheels read 88.3°, IMU 83.7°, VO 81.2°, and the EKF 83.8°. Ultrasonics stay obstacle sensors (`laser_scan_normalizer`) and are not odometry inputs.

The magnetometer cannot be added. The `/<robot>/magnetic_field` topic is all zeros because Inky's OpenCR has the 6-axis ICM-20648 (later OpenCR boards replaced the 9-axis MPU9250), and the OpenCR firmware's magnetometer read for that chip is an empty stub. Even on MPU9250 boards the firmware's orientation filter ignores the magnetometer. Indoors a magnetometer near the motors and building steel would also hurt heading more than help; heading drift is corrected by SLAM's scan matching instead. For an absolute heading outdoors, add a separate IMU with a calibrated magnetometer and fuse it as `imu1`.

Visual odometry (`stereo_visual_odometry.py`) runs inside `stereo_depth_node` on its own thread at 5 Hz, on the same matched pairs. It tracks corners in the left camera. Floor points get their depth from the floor plane in the stereo calibration (carpet texture repeats too much to match left/right reliably), points above the floor from a row-strip stereo match, and PnP + RANSAC gives the motion. Frames with fewer than 20 inliers, sideways motion, or impossible speeds are dropped, and the EKF rejects readings far from its estimate. Its status is on `/<robot>/stereo/vo_debug`.

- `use_ekf:=false` (bringup) returns to the stock driver: IMU heading inside wheel odometry, driver-owned TF.
- `stereo_visual_odometry:=false` (`navigation2_slam.launch.py`) leaves the EKF on wheels + IMU.
- Parameters: `turtlebot3_bringup/param/humble/ekf.yaml`, plus `ekf_gps.yaml` outdoors.
- Check inputs and rates: `./scripts/diagnostics/diagnose_ekf_inputs.sh <robot>`.

### ROS domain and DDS on the robot

Robot DDS never leaves the Pi. `scripts/env/ros_robot_env.bash` selects Cyclone DDS with `config/cyclonedds/robot_localhost.xml` (loopback only, raised participant cap) and sets the robot's domain from `scripts/env/ros_domain_profile.bash` (table above). Do not set `ROS_LOCALHOST_ONLY=1` on the robot; it brings back Cyclone's small participant cap. Because each robot is isolated, the domain number only has to match between the processes on that one Pi.


**UDP receive buffers (do once per robot).** `robot_localhost.xml` asks Cyclone for 8 MB socket receive buffers, but the kernel caps them at `net.core.rmem_max`, which is 208 KB on Ubuntu. With the stereo streams on loopback and the CPU busy, that is too small. Packets are dropped (`netstat -su | grep "receive buffer errors"` keeps climbing), and the Zenoh bridge can miss discovering `tf_zenoh_publish`. The central explorer then shows `pos=na` and never sends goals. Raise the cap on each robot:

```bash
echo 'net.core.rmem_max=8388608' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```

### Multi-robot and central computer

Each robot keeps a complete local stack: `/<robot>/tf`, `/<robot>/map` from SLAM, and Nav2 planning in frame `<robot>/map`. The central PC never sends TF or maps back. It merges maps, picks frontiers, transforms each goal into `<robot>/map`, and calls `/<robot>/navigate_to_pose` (plus `compute_path_to_pose` as a reachability check).

What crosses the network (`config/zenoh/robot_client.json5.tmpl`):

| Topic | Rate cap | Why |
| --- | --- | --- |
| `/<robot>/map_wire_z` | 1 Hz | zlib-compressed `/<robot>/map`; central rebuilds the map from it |
| `/<robot>/tf_zenoh`, `/<robot>/tf_static_zenoh` | 10 Hz / 1 Hz | best-effort copy of `/<robot>/tf` made by `scripts/comms/tf_zenoh_publish.py`, so the bridge never holds a reliable reader on SLAM's TF |
| `/<robot>/navigate_to_pose`, `/<robot>/compute_path_to_pose` | — | Nav2 actions, both directions |
| `/<robot>/nav2_collision_ahead`, `nav2_retrace_active` | — | optional helper signals for the explorer |
| `/<robot>/plan`, `scan_normalized`, `local_costmap/costmap` | 2/2/1 Hz | RViz on the central PC; sent only while something there subscribes |

If you add a topic, add it to both this template and `config/zenoh/central_router.json5.tmpl` in the central repo.

#### `navigation2_slam.launch.py`

| Node | Role |
| --- | --- |
| `laser_scan_normalizer` | fixed-width `scan_normalized` for SLAM (+ ultrasonic fusion); also publishes a 6 Hz `scan_costmap` copy for the costmaps (`scan_costmap_max_hz`) |
| `ultrasonic_triangulation_blob` | low-obstacle blob scans for the local costmap |
| `slam_toolbox` | `/<robot>/map`, `map -> odom` at 20 Hz |
| `wait_for_tf.py` | holds Nav2 until `odom -> base_footprint` exists |
| Nav2 (`navigation_launch_multirobot.py`) | one process per server, namespaced TF, lifecycle bonds off |
| `startup_map_seeder` | one 3 s forward move once the controller is active, then exits |
| `map_wire_compressed_republisher` | `map_wire_z` for Zenoh (`fleet_mode:=true`) |
| `standalone_world_map_tf` | static `map -> <robot>/map` for RViz goals (`fleet_mode:=false` only) |
| stereo cameras + `stereo_depth_node` + `stereo_cmd_vel_enforcer` | only when two cameras exist; costmap layer and stop gate only with a usable `config/stereo/<robot>.yaml` |
| `stereo_pair_throttle` (C++) | takes both 30 fps camera streams and republishes time-matched pairs at 5 Hz on `stereo_left/image_pair`, `stereo_right/image_pair` for `stereo_depth_node` |
| visual odometry (inside `stereo_depth_node`) | `odom/visual` for the bringup EKF; only with two cameras and a usable stereo calibration (`stereo_visual_odometry:=false` to turn off) |
| `stuck_detector` | `stuck`, `stuck/event`, `bump_points`, `nav2_lethal_inflation` (`enable_stuck_detector:=false` to turn off) |
| `low_obstacle_memory` | `low_obstacle_points` for the `memory_layer` obstacle layer in both costmaps, and `low_obstacle_points_local` for `stuck_detector` (`low_obstacle_memory:=false` to turn off) |

Optional hardware is listed per robot in `config/sensors/<robot>.yaml`. A robot without the file keeps the defaults. `ultrasonic: false` (Inky) leaves out ultrasonic fusion, `ultrasonic_triangulation_blob` and its costmap layer. The OpenCR publishes a constant 3.0 m on the ultrasonic topics even with no sensors fitted, and the triangulation node alone used about a third of a core. The launch log prints `sensors: ultrasonic=...`.

Off by default (turn on with the matching launch argument): `enable_debug_logging`, `enable_lethal_watch`, `enable_retrace_escape`, `enable_controller_collision_watch`, `enable_ultrasonic_cmd_vel_enforcer`, `orca_mode:=shadow|advisory` (ORCA output is not wired into the drive path).

Main arguments:

- `fleet_mode` (default `true`): driven by the central PC. `false` is a bench test without it.
- `robot_name` / `namespace`: override the namespace.
- `scan_costmap_max_hz` (default `6.0`): costmap scan rate; `0` feeds costmaps the full-rate scan.
- `slam_toolbox_mode` (`async` default, or `sync`).
- `enable_startup_map_seeding` (default `true`).
- the `ultrasonic_*`, `retrace_*` and `stereo_*` tuning arguments listed in the launch file.

Arguments removed in the October 2026 cleanup, because the central PC never sends TF or `/map` to the robot: `nav2_use_local_slam_map` (always on now), `fleet_map_relay_hz`, `fleet_require_global_tf_before_nav2`, `fleet_tf_map_wait_timeout_sec`, `auto_fleet_*`, `use_central_tf_map`, `nav2_use_composition`, `nav2_container_name`, `nav2_use_isolated_container`, `container_sig*_timeout`. `fleet_mode:=auto` is still accepted and means `true`.

### Stuck detection and low-obstacle memory

**Stuck detection** (`stuck_detector.py`) compares what the robot is told to do with what it actually does, over a 2 s window:

| Check | Fires when |
| --- | --- |
| stalled | `cmd_vel` steadily asks for at least 7 cm of travel and the wheel encoders turn less than 30% of it. Windows where the stereo or ultrasonic stop gate also publishes zero `cmd_vel` do not count: the base is being held on purpose. |
| slipping | the wheels report at least 7 cm while the heading changed less than 0.3 rad, but the robot did not translate. The LiDAR scan is tested against "moved as far as the wheels say" and "did not move", and stereo VO is used where it exists. If the scan cannot tell the two apart (for example a long featureless corridor), it abstains. |
| not turning | `cmd_vel` steadily asks for a turn in place and the heading barely changes (mixed turn/stop commands do not count) |

A check must fire on two scans in a row. The result is published on `stuck` (Bool), with one JSON line per detection on `stuck/event` and live numbers on `stuck/debug`. On a stall or slip during a steady push forward or back, the contact side goes out on `bump_points`. Stop-and-go commands, as in a "collision ahead" loop, are still reported but place no bump: in one fleet run, four of five bumps came from such windows and walled Inky in where it had stopped.

**Bumper reflex.** After a contact that places a bump, `stuck_detector` backs the robot straight away 0.20 m at 0.06 m/s. It publishes `cmd_vel` directly at 40 Hz, faster than the velocity smoother, and reports on `stuck/reflex_active`. It moves only when the strip it will drive through is empty (body width, 0.20 m plus a 0.10 m margin) in the LiDAR scan and in the low-obstacle memory (`low_obstacle_points_local`, which the LiDAR passes over), and stops early if that changes or after 6 s. At most one reflex runs every 4 s (`reflex_cooldown_sec`; it was 10 s, and a robot caught on a chair leg spent most of each attempt waiting). Nav2's own backup cannot do this: it refuses to start while any lethal cell touches the footprint outline, and the object just touched is exactly that. A low object behind the robot is invisible to the LiDAR; the robot came from that side moments before, which is why it backs straight up.

**Turn stalls.** One stalled turn in place (`not_turning`, or a slip while turning) runs the same reflex for 0.15 m: straight back, or forward if the back is blocked. It used to wait for two stalls within 8 s; on 2026-10-22 Inky, a wheel against a chair caster, repeated stall, back off, retry the same turn for five minutes. A clean `not_turning` reading also leaves a small bump (`turn_bump_x_m` × `turn_bump_y_m`, 2 × 3 cells) ahead of the wheel that was driving forward: a clockwise turn drives the left wheel forward, so the mark goes ahead of the left wheel, where a caster beside a front wheel stops it. Nav2 then plans the turn elsewhere. A wrong guess costs a 10 × 5 cm mark that stereo clears on its next look. The 0.15 m back-off takes the mark outside the global costmap's 0.18 m robot radius, so Nav2 can plan from where the robot stops.

If a stuck state lasts `stuck_reflex_after_sec` (4 s) without a reflex, one runs anyway (0.15 m): toward the lower end if tilted, away from the push if it was driving, otherwise back if the back is clear. A wheel caught on a chair leg mid-turn was declared stuck once and then left for 32 s.

**Clearance escape.** Nav2 Humble's planner refuses to start ("Starting point in lethal space") while anything is within the global costmap's `robot_radius` (0.18 m) of the axle. The Burger's front edge is only 0.04 m ahead of the axle, and the controller lets it stop closer than that (local radius 0.15 m). On Inky a stop facing a large object left Nav2 unable to plan from where it stood for 40 s, and the explorer blacklisted three goals. With no motion command for 3 s and a LiDAR return or a remembered low-obstacle cell within `clearance_radius_m` (0.21 m), `stuck_detector` drives straight forward or back the shortest distance, at most 0.15 m, that clears it, through a strip that is empty in both. (Before the memory counted, a bump in front left Nav2 failing "Starting point in lethal space" for over a minute with nothing near the robot in the LiDAR.) `clearance_radius_m` is 0.26 m, not the 0.18 m robot radius: the planner compares 5 cm cells, and on Inky a return 0.225 m away sat in a cell 0.19 m from the robot's own. At 0.21 m the robot stayed in "Starting point in lethal space" for four minutes. When the obstacle runs alongside and no straight move clears it, the robot first turns in place to face away from it (only with nothing within 0.16 m, the body's turning radius plus a margin), then makes the straight move.

While the robot is idle in that state, `stuck_detector` publishes `nav2_lethal_inflation` = true (forwarded to the central PC by Zenoh). The central explorer then holds the current goal and does not blacklist goals that fail to plan, and resends the goal once the flag clears. Without it, those four minutes blacklisted every frontier, and exploration ended with six left. (`enable_lethal_watch:=true` starts the older `nav2_lethal_watch`, which publishes the same topic from the costmap; leave it off with the stuck detector on.)

**Stale costmap marks.** Nav2's obstacle layer keeps a LiDAR mark until a beam passes through it, so a chair moved, or a person's legs, seen where the LiDAR cannot look again stays. On Inky such marks closed every way out of a spot it had driven into (719 cells blocked in the costmap that were free in the SLAM map), and every plan failed with "no valid path found". Nav2's own navigation clears the costmap when planning fails, but the central explorer's path checks call the planner directly. `stuck_detector` watches `compute_path_to_pose/_action/status`: after `plan_fail_clear_count` (4) aborted plans within 30 s with the robot standing still, it calls `global_costmap/clear_entirely_global_costmap`, at most once per 30 s. The LiDAR, stereo and the low-obstacle memory re-mark what is really there within a second. Tested live: the robot left that spot within seconds of the clear.

**Squeeze.** The global planner keeps 0.18 m from obstacles, the controller only 0.15 m, so the robot can drive through a gap that the planner will not route back through. On Inky it entered a room through a doorway with exactly 0.18 m clearance in the SLAM map and stayed: every plan out failed, and clearing the costmap changed nothing. When plans keep failing within 90 s of a clear (`squeeze_after_clear_sec`), `stuck_detector` sets the global costmap's `robot_radius` to 0.15 m (`squeeze_radius_m`, the controller's) through its parameter service, and restores 0.18 m (`squeeze_normal_radius_m`, keep it equal to `burger.yaml`) once the robot has moved 0.8 m or after 60 s. If the node is killed mid-squeeze, the radius stays at 0.15 m until the stack restarts.

**Tilt escape.** A Burger that drives onto something low ends up tilted. On Inky, the floor runner of a sled-base (cantilever) chair, a chrome tube about 2-3 cm tall, got under the rear caster and left it 11° nose-down. The LiDAR only sees the angled upright part of that tube, well away from the runner. Three turn-stall reflexes backed up and moved 2-5 cm each, because backing drove the caster further onto the tube. `stuck_detector` now reads the accelerometer. When the robot is tilted more than `tilt_threshold_deg` (5°) for a second, it drives toward its lower end (forward when nose-down, back when nose-up) by `tilt_escape_m` (0.15 m) and leaves a bump at the raised end. Tested live on Inky stuck on that chair: 10° nose-down, forward 0.15 m, level again (pitch −1.2°). Any reflex that covers less than 3 cm makes the next one, within a minute, try the other direction. While tilted, the LiDAR also hits the floor about 1 m away and stereo's floor reads 10+ cm off ("Stereo floor is +0.12 m…"). Those marks clear once the robot is level.

Two guards against tilt ghosts, after Inky rolled over a floor cable (7° nose-up):
- The tilt escape fires only when the wheels moved less than `tilt_max_travel_m` (3 cm) while tilted, i.e. the robot is held. Rolling over a cable used to trigger a back-off and leave a bump wall across the cable.
- `stereo_depth_node` reads the IMU (`imu_topic`). While the robot is tilted more than `max_tilt_deg` (4°), it publishes no obstacles or drops (status `tilted`). With the nose up, the floor seemed to fall away, and stereo reported drops at 0.40–0.45 m that stopped the robot. Tilt is measured against the robot's resting attitude, the median accelerometer reading of the last 30 s. Measured from level, Inky's ~1.5° IMU mount offset plus ordinary speeding up and braking (2–4°) read as tilted in 20% of frames while exploring, and those frames published no obstacles.

**Large low objects.** A large dark box, too short for the LiDAR, had three weaknesses:
- A bump marked only an 18 cm line. Nav2 planned around its end and touched the box again 12 cm further along. Bumps are now ±0.18 m wide and two cells deep (`bump_half_width_m`, `bump_depth_m`).
- `low_obstacle_memory` joins a bump to any earlier one within 0.6 m in the last 2 minutes (`bump_link_m`, `bump_link_sec`), so repeated touches along one face build a wall. The log says `joined to N earlier bump(s)`.
- When the object fills the 0.4–0.8 m floor-fit window of the stereo band, the band's floor plane was fitted to the object itself, which was then never reported (`floor_z` 0.1–0.24 m). A fitted plane more than `max_plane_offset_m` (4 cm) from the calibrated floor is now rejected, and heights are measured from the calibrated floor.

Up close, a plain dark surface filling the cameras gives stereo nothing to match, so bumps remain the only sensor for it at that point.

Tested on Inky against a low black box that is below the LiDAR plane and too close for stereo (nothing closer than 0.40 m): pushing into it at 0.05 m/s was detected as slipping with a steady command. A bump was remembered at the box edge, and the reflex backed off 0.21 m. The memory then held the edge 0.31–0.36 m ahead, and Nav2's spin succeeded. `reflex_enabled:=false` on the node turns it off.

Tests on Inky:
- Fake wheel data claiming 8 cm of motion while the robot stood still: slipping detected about 1.3 s after the wheels "started".
- `cmd_vel` with no wheel motion: stall detected.
- Real driving (15 cm forward, a 90° spin): no detection. During the forward move the LiDAR supported the full wheel distance and VO measured 7–10 cm.

**Low-obstacle memory** (`low_obstacle_memory.py`) keeps its own record, in the SLAM map frame, of:
- stereo marks (`stereo_blob_scan`) seen in at least two separate scans within 10 s, the second from at least 0.10 m or 0.2 rad away from the first (`confirm_move_m`, `confirm_turn_rad`), so a one-frame flicker is not remembered and neither is an artefact of one viewpoint: floor noise at the far edge of the stereo band, seen frame after frame while the robot waited, was remembered as an arc on open floor. The local costmap's stereo layer still marks at once. Each mark is extended 15 cm along the beam, because stereo only sees the near face of a low object. When the robot turns around a box, its unseen back half is still treated as occupied. A scan is placed only with the robot pose at the scan's own timestamp; stereo frames are often a second old, and the current pose would smear marks sideways during a turn.
- bumps from `stuck_detector`: a 0.36 m wide, two-cell-deep line at the body edge that touched (0.13 m ahead or 0.14 m behind). An earlier 0.25 m line landed on floor beyond a thin object, where stereo correctly cleared it, and the object was forgotten. The line starts inside Nav2's footprint, which stops Nav2 pushing, and the bumper reflex then moves the robot clear of it. This works on robots with no cameras.

Cells are forgotten when stereo matches floor through them (`stereo_clear_scan`), or after 2 min (marks) or 3 min (bumps). The memory covers what the robot just passed, not the whole map. That keeps a false mark from walling the robot in; it happened once in an early fleet run with 5/10 min lifetimes, single-frame marks and a false stall.

The node republishes its cells every second as `low_obstacle_points` (map frame, cells within 6 m, none within 0.12 m of the robot, so a costmap clear re-marks even an object right in front, which stereo and LiDAR may both miss). It also publishes the cells within 0.8 m in the robot's base frame as `low_obstacle_points_local`, stamped with the pose they are relative to; `stuck_detector` moves them along with wheel odometry between updates, so its reflexes need no TF listener. A `memory_layer` obstacle layer in both costmaps marks them and clears from `stereo_clear_scan`. Nav2's stock recovery tree calls `ClearEntireCostmap`, which wipes every layer. The memory is back within a second of a clear: tested on Inky, local lethal cells went 334, then 0 right after the clear, then 219 within 2.8 s. Because it is an ordinary layer before inflation, the remembered cells are lethal and the ring within the robot radius is inscribed (253), as for any obstacle.

With the memory on, the global costmap has no `stereo_layer` of its own (the local costmap keeps one for immediate marks): the memory layer carries the same confirmed marks there, with clearing and a time-to-live, and it is what `stuck_detector` sees. On 2026-10-22 a camera dropped off USB mid-run. Nav2 re-applies a source's last observation until a new one arrives, so the global stereo layer's marks around the robot froze with nothing to clear or expire them, the planner refused to start, and the stuck detector could not see why. Separately, `stereo_depth_node` now notices when no pair has arrived for `images_timeout_sec` (2 s, after 40 s of grace at startup). It then logs which camera is silent, reports status `no_images` on `stereo/debug`, and publishes empty mark and clear scans every 0.5 s, so the local layer drops the stale frame too. The LiDAR keeps working; reseat the camera's cable and restart the stack to get stereo back.

An earlier version used a keepout filter mask instead. A filter can only say lethal or at most 251, so the ring around each cell was lethal. When Inky stopped about 0.3 m from a low object, its own footprint touched lethal cells, and Nav2 refused to spin or back up until the memory expired. With the layer, the same approach-then-escape test on Inky (creep toward the object, then spin 0.5 rad and back up 0.10 m) succeeds.

Stereo marks are only trusted from a verified pair. `stereo_depth_node` publishes no obstacles until its first alignment check passes (`Stereo alignment verified` in the log). The memory drops every stereo mark, keeping bumps, as soon as the depth node reports the pair out of alignment. Without that, a pair bumped out of alignment, for example during a battery swap, published garbage obstacles for the 5–10 s before it was caught. The memory kept them around the start pose, where nothing could clear them, and Nav2 could not move, spin or back up.

**CPU on a Pi 4** (full SLAM + Nav2 + stereo stack, robot idle):
- `stuck_detector` and `low_obstacle_memory` use about 0.2 core each, mostly rclpy overhead per message.
- Visual odometry adds about 0.3 core.
- `build_blob_scan` was vectorized and gives the same output on live frames: 700 ms per pair down to 255 ms.
- The camera streams are paired in C++ (`stereo_pair_throttle`).

With all of this, stereo obstacle scans run at about 1.5 Hz, against 1.35 Hz before these changes without VO. The Pi is still close to saturated; `metricbeat`/`packetbeat` take a noticeable share of what remains.

### LDS configuration

We use **LDS-02**. On the robot (SBC):

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```

`LDS-02` provides **360-degree coverage**, but that does not mean ROS always receives exactly 360 rays per scan in this driver path. The sensor uses a fixed sampling stream and the driver assembles/bins points into `LaserScan`, so beam count can vary by RPM and packet timing. In this repo, `normalize_laser_scan.py` now auto-detects a stable beam count during startup and locks `scan_normalized` to that value for SLAM/Nav2 consistency (with `target_readings` as fallback/override).

**Tilt gate.** The LiDAR is 0.18 m above the floor: with the robot pitched 2° it sees the floor about 5 m away, at 4° about 2.6 m. SLAM uses returns to 8 m and marks a cell occupied at an occupancy threshold of 0.1, so such hits stay in the map as walls on open floor. On Inky (2026-10-22) walls appeared in places that were physically empty and closed off a doorway the planner then would not use; floor hits during tilt are the likeliest source (people's legs are another). `normalize_laser_scan.py` withholds scans taken while the IMU's fused orientation is more than `max_tilt_deg` (1.5°) from the robot's resting attitude (a 20 s average, `tilt_rest_sec`), plus 0.2 s after. While exploring, the fused tilt stayed under 1.5° 99% of the time (accelerometer alone reads 2-4° from braking, which is why the fused orientation is used). Its log says how many scans it withheld; `max_tilt_deg:=0` turns it off.

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

- **Nav2 never starts after `wait_for_tf`:** the launch exits if `odom -> base_footprint` never appears on `/<robot>/tf`. Check bringup is running in the same namespace and domain (`echo $ROS_DOMAIN_ID`, `ros2 topic echo /<robot>/odom --once`).
- **"No valid path found" (GridBased planner):** the goal is behind unknown or blocked space. The central explorer blacklists such frontiers after its precheck fails; repeated failures on one spot are normal while the map grows.
- **"Sensor origin is out of map bounds":** the map does not cover the robot yet; it clears once SLAM grows the map.
- **`Starting point in lethal space` / stuck near walls:** inflation reaches the robot center. Recoveries (backup, spin, wait) usually move it out; if it happens often, reduce `inflation_radius` in `burger.yaml` slightly.
- **"Control loop missed its desired rate":** the Pi is short on CPU. Check with `top`; the stereo depth node is the largest consumer (about one core at `publish_hz` 4). Lower its rate or disable stereo on robots that do not need it.
- **"Stereo cameras are out of alignment":** the stereo node's row check exceeded `max_row_error_px` (2.0), so stereo obstacles, drops and the stop gate are off (the robot still drives on LiDAR, ultrasonics where present, and bump memory). It can appear now and then depending on the scene. If it is constant, recalibrate with the robot still: `ros2 run turtlebot3_navigation2 stereo_refine_extrinsics.py --ros-args -r __ns:=/<robot>`. On Inky the error has moved between 0.8 and 6.5 px across battery swaps, which points to a loose camera mount.
- **"Stereo floor is ... from where the mount fields put it":** normal when a large object fills the cameras' view (drops are off until the floor is visible again). In open space it means `camera_height_m` and `camera_pitch_rad` in `config/stereo/<robot>.yaml` no longer match the calibration, so the floor reads a few centimetres low and looks like a drop across the corridor. The robot then stops, and Nav2 logs "collision ahead" with nothing there. The depth node turns drops off after 5 frames more than 3 cm off. To fix, put the robot on open flat floor with nothing in the lower half of the view, run the cameras (`ros2 launch turtlebot3_navigation2 stereo.launch.py namespace:=<robot>`), and run `ros2 run turtlebot3_navigation2 stereo_fit_floor.py --ros-args -r __ns:=/<robot>` (`-p dry_run:=true` to preview). It keeps the old file as `.bak`. On Inky, after the Oct 2026 chessboard calibration, the fit moved the pitch from 0.269 to 0.162 rad and the height from 0.139 to 0.131 m; the floor went from −0.049 m to 0.000 m.
- **Which camera is left:** the two USB cameras are ordered by USB port (`bus_info`, e.g. `usb-0000:01:00.0-1.1` before `-1.2`), not by `/dev/videoN`. The device numbers follow enumeration order: after Inky's left camera dropped off USB and was reseated, it came back as `/dev/video2` and the right one was `/dev/video0`, which would have loaded the pair swapped. Keep each camera's cable in the same USB port it was calibrated in; moving them swaps left and right. `python3 -c "import stereo_common as s; print(s.list_stereo_cameras())"` (from the package's `scripts` directory) shows the order used.
- **Stereo recalibration order:** `stereo_calibrate.py` (chessboard: lens and pair), then `stereo_refine_extrinsics.py` if the rows drift later, then `stereo_fit_offset.py` (disparity offset against the LiDAR; needs both `robot.launch.py` (for `scan` and TF) and `stereo.launch.py` running, and walls or furniture 1-3 m away in view: `ros2 run turtlebot3_navigation2 stereo_fit_offset.py --ros-args -r __ns:=/<robot>`, `-p dry_run:=true` to only measure; it says what it is waiting for), then `stereo_fit_floor.py` (mount height and pitch). The chessboard step keeps the old height and pitch, which no longer fit the new rectification. Run the offset fit before the floor fit: the floor fit otherwise absorbs the offset into a wrong height and pitch.
- **Stereo sees an obstacle on empty floor ("collision ahead" with nothing there, `obstacle=` in the stereo status line while the floor reads about 0):** the disparity has a constant offset. Neither the row check nor the floor fit can see it, and everything standing up is placed too close. On Inky, after the Oct 2026 chessboard calibration, +2.4 px put a printer 1.72 m away at 0.88 m, in the low-obstacle band. `stereo_fit_offset.py` measured the offset against the LiDAR and moved the right camera's principal point by −2.1 px. The floor fit that followed reached 93% inliers and 0.4° tilt, up from 76% and 2.8°. Stereo obstacles are also limited to `max_bearing_deg` (45°) of straight ahead: beyond about 45° the 120° lenses are off by another 2-3 px. `stereo_fit_offset.py` uses the same ±45°: with the edges included, its result moved by more than 1 px from one scene to the next. Without them, two runs agreed within 0.01 px.
- **Robot hesitates at doors or walls it can pass ("collision ahead" near something the LiDAR already maps):** stereo used to mark anything 2.5–18 cm above the floor, including the bottom of a door or wall. It placed them by its own calibration, slightly off from the LiDAR, and the memory kept those copies for 2 minutes. The trip back through a door the robot had just passed then looked blocked. Stereo now drops a hit when the same object also stands 18–33 cm high at the same range: that is the LiDAR's height, so the LiDAR has it. Low objects, including one under a table top or in front of a wall, are still marked. The status line's `lidar_height_beams=` counts the beams it dropped. Stereo also drops a hit when the LiDAR has a return on the same bearing, from 0.10 m nearer to `lidar_match_m` (0.25 m) further. A plain white wall gives stereo nothing to match above its textured baseboard. At about 1 m, one pixel of disparity is roughly 0.2 m of range, and the band's 0.8 m edge keeps only the near end of the baseboard's spread. On Inky a wall the LiDAR put at 0.98 m read as a 3 cm obstacle at 0.75 m in 12 of 40 frames, and in none with this check. Beyond 0.10 m past the hit, only a surface at least `lidar_wide_m` (0.10 m) across within ±6° counts, like a wall. A chair column is about 5 cm wide, and the caster at the tip of an arm pointing at the robot stands 0.25–0.30 m in front of it; with the plain window that caster was left to the column. Remaining trade-off: a low object within 0.10 m of something the LiDAR sees, or within 0.25 m in front of a wall, is left to it and to the bumper. Without a LiDAR (`stereo.launch.py` alone), the check is skipped.
- **Robot runs into chair bases (casters, star-base arms) it should have avoided:** these stand 5–10 cm high, below the LiDAR plane (18 cm), which only sees the chair's column. Stereo is the sensor for them. Before 2026-10-22, stereo missed nearly all of them. In 13 "collision ahead" snapshots from one day of runs, the blocking cells were LiDAR or bumps and almost never stereo. Causes, each measured on Inky's own frames:
  - SGBM leaves the first `numDisparities` columns of the left image unmatched: with 80 levels on 320 px, everything beyond about 20° to the left was blind, while the right side reached the image edge. `stereo_common.padded_disparity` pads both images with that many blank columns and drops matches that land in the padding, so the full ±27–37° (depending on the calibration's focal length) is matched (`full_width`). With `min_range_m` 0.20 m (where the lowest row meets the floor), 80 levels on a 400 px pair cost no more than the old 128 levels on 320 px.
  - Obstacles were only reported 0.40–0.80 m ahead. A chair base enters view at 1–1.2 m and at 0.22 m/s it was already inside 0.8 m before two frames confirmed it. The band is now 0.28–1.10 m (`obstacle_near_x_m`, `obstacle_far_x_m`). Floor points on Inky's carpet stay within 1.5 cm of the floor to 0.8 m, so past 0.80 m the height needed rises linearly to 4 cm at 1.10 m (`min_obstacle_height_far_m`). Chair arms (6–10 cm) and casters (5 cm) still clear it.
  - A beam's range was the median of every low hit on its bearing, which put a caster at 0.45 m with the chair base behind it at 0.8 m in the free space between. It is now the nearest run of hits within 8 cm (`cluster_m`).
  - Thin hits closer than 0.50 m had to outnumber every floor point on the whole beam, which a caster never does. Now the floor within 8 cm of the hit has to outnumber it two to one.
  - Two-frame confirmation allowed 0.10 m of range change; the robot closes 9 cm per frame, and far ranges are noisier. It now allows 0.12 m plus a tenth of the range.
  The costmap's stereo layer marks out to 1.4 m (`stereo_obstacle_max_range`), and clearing starts at 0.35 m (`stereo_clear_min_range` and `low_obstacle_memory`'s `clear_min_range_m`), the range at which every bearing in view is inside the band, so a beam never erases a cell stereo did not check. To see what stereo makes of a scene: `ros2 topic echo /<robot>/stereo/debug`, or the `stereo/points` cloud in RViz.
- **Robot drives up to a gap, backs off, spins, and tries the same gap again (between chairs, a narrow passage):** the planner (global costmap) and the controller (local costmap) used to check the same 0.15 m circle on maps that can be a cell or so apart. A gap that only just fits on the planner's map was then a "collision ahead" on the controller's. Nav2's recovery (wait, back up, spin) replanned the same gap each time. The local costmap uses 0.15 m and the global 0.18 m: the planner only routes through gaps the controller clears with about 3 cm to spare, and goes around marginal ones. (A local 0.13 m, just over the 0.125 m body, left the wheels clipping chair legs, whose feet reach a few cm beyond where the LiDAR sees them.)
- **Controller "missed its desired rate" often, sluggish or confused driving:** the Pi is overloaded. On Inky (Oct 2026) the load average reached 9.6 on 4 cores. Top consumers: `stereo_depth_node` about 0.9 core, `stuck_detector` 0.4, `stuck_snapshot` 0.2, `low_obstacle_memory` 0.2, `tf_zenoh_publish` 0.2 and `normalize_laser_scan` 0.2. Every Python subscription and timer wake-up costs CPU in rclpy's executor. So `stuck_snapshot` subscribes to the scan, stereo and memory topics only during a "collision ahead" streak, `stuck_detector` checks every other scan and runs its 40 Hz reflex timer only while a reflex drives, and `stereo_depth_node` converts the latest LiDAR scan only when a stereo frame uses it. Those trims saved only about 0.1 core, so the stereo rates were also lowered: the depth node publishes at 2.5 Hz (was 4), visual odometry runs at 3 Hz (was 5), and `stereo_pair_throttle` delivers pairs at 3 Hz (was 5). At 0.22 m/s the robot moves 9 cm between stereo frames, well inside the 0.28–1.10 m obstacle band. `stuck_snapshot` is off by default; turn it on with `enable_stuck_snapshot:=true` when chasing a "collision ahead" problem. The depth node itself was saturated: it delivered 1.7 Hz while using a full core. Three changes:
  - The RANSAC floor fit is vectorized (it was 150 ms per call under load, twice per frame).
  - SGBM uses only as many disparity levels as `min_range_m` needs for the calibration (`num_disparities: 0` = auto, 80 for Inky's current calibration instead of 128).
  - `stereo/points` is only built when something subscribes (it cost 36 ms per frame for no reader).

  Stereo alone, it now runs at its full 2.5 Hz on about 0.7 core. Check with `top -o %CPU` on the robot.
- **"collision ahead" that will not clear:** with `enable_stuck_snapshot:=true` on `navigation2_slam.launch.py`, after 5 s of repeated warnings `stuck_snapshot` logs one line: `collision ahead for Ns: lethal cells within 0.6 m from LiDAR=… stereo=… memory=… unexplained=…; nearest (x ahead, y left): …`. It also saves the local costmap and each layer's latest input to `~/.ros/stuck_snapshots/` on the robot, at most once a minute. "unexplained" usually means older stereo marks that the latest stereo frame no longer shows. In NavigateToPose terms, the controller (RPP) stops when a lethal cell touches the 0.15 m footprint at any pose along the next second of commanded motion, or at the current pose.

#### Stability tuning profile (Mar 2026)

These values reduce false `Failed to make progress` aborts with the central explorer:

- `controller_server.progress_checker.required_movement_radius: 0.10`
- `controller_server.progress_checker.movement_time_allowance: 45.0`
- `controller_server.FollowPath.min_speed_theta: 0.10`
- `local_costmap.local_costmap.transform_timeout: 0.5`
- `global_costmap.global_costmap.transform_timeout: 0.5`
- `local_costmap.local_costmap.width/height: 4.0`
- `local_costmap.local_costmap.inflation_layer.inflation_radius: 0.25`

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
./scripts/build/rebuild_common.sh minimal

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
   ./scripts/diagnostics/start_nav2_debug_capture.sh
   # Optional explicit robot name:
   # ./scripts/diagnostics/start_nav2_debug_capture.sh pinky
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

#### Rosout clue extractor (same bag window)

Use this on the stop interval to pull likely controller/planner reasons from `/rosout`:

```bash
cd ~/turtlebot3
python3 scripts/diagnostics/analyze_nav2_bag_rosout.py \
  logs/<robot>/bag-YYYYmmdd-HHMMSS \
  --start <interval_start_unix_s> \
  --end <interval_end_unix_s>
```

Tip: pair this with the interval printed by `analyze_nav2_bag_stop.py`.

### Ultrasonic diagnostics capture (robot side)

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
python3 scripts/diagnostics/analyze_ultrasonic_debug_session.py \
  logs/<robot>/ultrasonic-session-YYYYmmdd-HHMMSS.jsonl
```

This reports per-sensor rates/ages/stale counts, flatline streak maxima, fusion-effect counts, aggregate anomaly counts, and recent anomaly rows.

#### Add map + costmap rosbag for hard cases

For "stopped then drove into bar" episodes, collect a rosbag in parallel with ultrasonic JSONL:

```bash
cd ~/turtlebot3
./scripts/diagnostics/start_nav2_debug_capture.sh
# Optional explicit robot name:
# ./scripts/diagnostics/start_nav2_debug_capture.sh blinky
```

This captures `/map`, local/global costmaps, plan, `cmd_vel`, `odom`, TF, and Nav2 action status so you can align controller intent with obstacle marking over time.
