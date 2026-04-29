# TurtleBot3 fleet workspace (robot source)

This tree is the **canonical** copy of namespaced SLAM + Nav2 launch and parameters for the ANS TurtleBot3 fleet. Deploy and **`colcon build`** on each Raspberry Pi when you change packages; sshfs from a dev machine is fine for editing but local builds on the Pi are the reliable path.

## Key entry points

- **SLAM + Nav2:** `src/turtlebot3/turtlebot3_navigation2/launch/navigation2_slam.launch.py`
- **SLAM tuning:** `src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml` (and siblings)
- **Optional Pi load relief:** launch arg `scan_costmap_max_hz` (e.g. `6.0`) — costmaps subscribe to throttled `scan_costmap` while SLAM stays on full-rate `scan_normalized`.
- **Startup map seeding automation:** launch arg `enable_startup_map_seeding:=true` in `navigation2_slam.launch.py` (run before central start)

## Quick start for new collaborators

Use this for a minimal end-to-end fleet bring-up:

1. **[ROBOT-SBC]** On each robot (`~/turtlebot3` from the `ans-turtlebot3` checkout):
   - `ros2 launch turtlebot3_bringup robot.launch.py`
   - `ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py`
2. **[CENTRAL-PC]** On central (`~/central-computer` from the `ans-central-computer` checkout):
   - `./scripts/core/start_central.sh`
   - `./scripts/core/start_rviz_central.sh`
3. Verify from central:
   - `ros2 topic echo /map --once`
   - `ros2 action list | rg navigate_to_pose`

### SSHFS editing reliability note

Editing robot files over SSHFS is supported, but for reliable deployment:

- run build and launch commands directly on the Pi shell (not through a flaky mount),
- avoid changing netplan/systemd files during unstable Wi‑Fi sessions,
- keep a fallback access path (HDMI/keyboard or direct SSH on known-good Wi‑Fi).

## Goal of this document

Document the steps to prepare a TurtleBot3 Raspberry Pi SBC **up through**:

- Robotis SBC setup (Ubuntu Server, Wi‑Fi/SSH, stability tweaks)
- ROS 2 Humble install and **Install and Build ROS Packages** (SBC setup Step 3.2.5, Step 2)

**Workspace = cloned repo:** Clone [ans-turtlebot3](https://github.com/SleepyFinale/ans-turtlebot3) into `~/turtlebot3` on the Pi; that repo contains the TurtleBot3/LDS/Coin D4 packages and other files needed for the workspace. After building, any changes you make in `src/` can be committed and pushed. Build/install/log are in `.gitignore`.

---

## Robot fleet reference

Use this table when configuring a given robot. SSH using the hostname or IP for that robot. For fleet runs, follow the `bridged_domains` policy in [ROS_DOMAIN_ID](#ros_domain_id): robots use per-robot domain assignments and central uses the configured central domain (default `50`).

| Robot  | SNS (lab)             | GCRI_LAB (gcri)      | RaspAP (rpi)        |
| ------ | --------------------- | -------------------- | ------------------- |
| Blinky | blinky@192.168.0.158  | blinky@192.168.50.158| blinky@10.3.141.158 |
| Pinky  | pinky@192.168.0.194   | pinky@192.168.50.194 | pinky@10.3.141.194  |
| Inky   | inky@192.168.0.139    | inky@192.168.50.139  | inky@10.3.141.139   |
| Clyde  | clyde@192.168.0.236   | clyde@192.168.50.236 | clyde@10.3.141.236  |

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

## Wi‑Fi switching and static IPs (`scripts/network/switch_wifi.sh`)

This repo includes a helper script to switch the robot’s Wi‑Fi network and apply consistent IP settings using **netplan**.

- Script path: `scripts/network/switch_wifi.sh`
- Netplan override file used: `/etc/netplan/99-wifi-switch.yaml`

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
   sudo ./scripts/network/switch_wifi.sh lab
   sudo ./scripts/network/switch_wifi.sh gcri
   sudo ./scripts/network/switch_wifi.sh rpi
   sudo env TAMU_IDENTITY=<netid> TAMU_PASSWORD='<password>' ./scripts/network/switch_wifi.sh tamu
   ./scripts/network/switch_wifi.sh status
   ```

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

### Static IPs on RaspAP rpi Wi-Fi

When connected to **RaspAP** (e.g. Raspberry Pi hotspot), each robot uses a fixed IP (see `scripts/network/switch_wifi.sh`):

- **Blinky** → `10.3.141.158`
- **Pinky** → `10.3.141.194`
- **Inky** → `10.3.141.139`
- **Clyde** → `10.3.141.236`

### Usage

From `~/turtlebot3` on the robot:

```bash
cd ~/turtlebot3

# Connect to SNS lab Wi‑Fi with static IP (per robot/user)
sudo ./scripts/network/switch_wifi.sh lab

# Connect to GCRI_LAB with static IP (per robot/user)
sudo ./scripts/network/switch_wifi.sh gcri

# Connect to RaspAP with static IP (per robot/user)
sudo ./scripts/network/switch_wifi.sh rpi

# Connect to TAMU_WiFi with DHCP (WPA enterprise)
sudo env TAMU_IDENTITY=<netid> TAMU_PASSWORD='<password>' ./scripts/network/switch_wifi.sh tamu

# Show current Wi‑Fi SSID and wlan0 IP
./scripts/network/switch_wifi.sh status
```

The script uses the invoking user (`SUDO_USER`/`$USER`) to choose the static IP. To override explicitly (e.g. when logged in as a different account):

```bash
sudo ./scripts/network/switch_wifi.sh lab pinky
sudo ./scripts/network/switch_wifi.sh lab blinky
sudo ./scripts/network/switch_wifi.sh lab inky
sudo ./scripts/network/switch_wifi.sh lab clyde

# or
ROBOT_NAME=pinky sudo ./scripts/network/switch_wifi.sh lab
ROBOT_NAME=blinky sudo ./scripts/network/switch_wifi.sh lab
ROBOT_NAME=inky sudo ./scripts/network/switch_wifi.sh lab
ROBOT_NAME=clyde sudo ./scripts/network/switch_wifi.sh lab
```

If you change the SNS, GCRI_LAB, RaspAP, or TAMU networks (SSID, password, gateway, or IP scheme), update the constants at the top of `scripts/network/switch_wifi.sh` accordingly.

### Automatic WiFi connection on boot (`scripts/network/boot_wifi.sh`)

To prevent the robot from being stuck without WiFi when it boots, a boot-time WiFi connection script automatically attempts to connect to WiFi networks in priority order.

**Behavior:**

- On boot, the robot **first attempts to connect to SNS (lab)**.
- If that fails, it tries **GCRI_LAB (gcri)**.
- If that fails, it tries **RaspAP (rpi)**.
- This order helps the robot come up on lab WiFi before falling back to local hotspot WiFi.

**Installation (one-time setup per robot):**

```bash
cd ~/turtlebot3
sudo ./scripts/network/install_boot_wifi.sh
```

This installs a systemd service (`boot-wifi.service`) that runs on every boot. The service:

- Detects the robot name from the hostname
- Attempts SNS first (waits up to 30 seconds per network)
- Then GCRI_LAB, then RaspAP, each with the same timeout behavior
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

If serial data becomes clean but EKF logs still show `Failed to meet update rate`, temporarily lower rates from launch args:

```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  ekf_frequency:=4.0 navsat_frequency:=4.0
```

### ROS_DOMAIN_ID

This stack supports two comms modes, but **fleet runs must use bridges**.

1) `bridged_domains` (**use this**)
   - Central uses `fleet_domain_map.central_domain_id` (default `50` in-repo; override in YAML if needed).
   - Each robot uses a deterministic per-robot domain from `scripts/env/ros_domain_profile.bash` (robot-side helper).
   - Central starts per-robot domain bridges and keeps explorer/action behavior unchanged.
   - With the default Fast DDS port layout, keep **every** `ROS_DOMAIN_ID` you use (central and each robot) **<= 232** so multicast ports stay valid.
2) `shared_domain` (legacy compatibility only)
   - All robots and central share one domain (lab default `50`).
   - **Not acceptable** for this project's multi-robot SLAM workload (timing, discovery, and load artifacts); do not A/B fleet debugging against shared domain.

For bridged mode, load a robot domain profile before launching bringup/Nav2 on each robot:

```bash
cd ~/turtlebot3
source scripts/env/ros_domain_profile.bash
echo $ROS_DOMAIN_ID
```

You can also pass a robot name explicitly if hostnames differ:

```bash
source scripts/env/ros_domain_profile.bash pinky
```

Domain assignments are defined in `~/central-computer/config/fleet_domain_map.yaml` (`ans-central-computer`) and must stay in sync with robot hostnames.

### Multi-robot and central computer

This repository is the **robot side** of a multi-robot SLAM system. Each TurtleBot3 robot runs **bringup + SLAM + Nav2 on the robot SBC**, and the central PC handles coordination (`./scripts/core/start_central.sh`) and visualization (`./scripts/core/start_rviz_central.sh`) from the **`ans-central-computer`** repository.

For **multi-robot SLAM** (full fleet: Blinky, Pinky, Inky, Clyde), robots remain namespaced (`/blinky`, `/pinky`, `/inky`, `/clyde`). In `bridged_domains` mode, central bridges only the explicit fleet contract topics/actions; this reduces DDS noise and keeps Nav2 local on each robot.

**Canonical startup sequence (multi-robot default):**

1. **[ROBOT-SBC]** On each robot: run bringup (`robot.launch.py`) in terminal 1.
2. **[ROBOT-SBC]** On each robot: run SLAM + Nav2 (`navigation2_slam.launch.py`) in terminal 2.
3. **[CENTRAL-PC]** In `~/central-computer`, run `./scripts/core/start_central.sh`.
4. **[CENTRAL-PC]** In `~/central-computer`, run `./scripts/core/start_rviz_central.sh`.
5. In this default flow, central runs `map_merge` and RViz opens in **GLOBAL** mode with `/map`.

If you use `fleet_mode:=auto`, robots may delay Nav2 activation until the central stack publishes required global TF/map links. That delay is expected in auto mode.

To check TF and connectivity from the central PC, run (from `~/central-computer`):  
`ROS_DOMAIN_ID=<central_domain_id> python3 scripts/diagnostics/diagnose_multirobot_tf.py`  
After pulling TF-frame changes, rebuild `turtlebot3_navigation2` on each Pi and re-run the diagnostic on the central PC with all robots up.

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

**With the central PC** (`~/central-computer/scripts/core/start_central.sh`): you **must** enable fleet mode so Nav2 uses global `/tf`, `/tf_static`, and `/map` (merged map from `map_merge`, or a relay of `/<robot>/map` when only one robot runs). Use exactly this pattern:

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

**Map topics:** Each robot still publishes its own SLAM map (e.g. `/pinky/map`) for `map_merge` and tools. With `fleet_mode:=true`, Nav2’s costmaps use **`/map` on the network**: multi-robot **merged** map from the central PC, or **single-robot** relay from `/<robot>/map` started by `~/central-computer/scripts/core/start_central.sh`. With `fleet_mode:=false`, the global costmap static layer uses `/<robot>/map` only. On the robot, `navigation2_slam.launch.py` starts a namespaced static TF `map` → `<robot>/map` (identity) in standalone mode so Nav2 and default goals that use frame `map` match SLAM’s `<robot>/map` frame.

**Optional fleet tuning (Wi‑Fi, CPU):** Keep host clocks in sync on every Pi and the central PC (Chrony or systemd-timesyncd) and verify with `timedatectl`/`chronyc`. With **`fleet_mode:=true`**, `navigation2_slam.launch.py` supports **`nav2_use_composition:=false`** by default (set `true` to run Nav2 in one component container on capable hardware), **`fleet_map_relay_hz:=1.5`** (example) to throttle merged `/map` to `/map_relay` for Nav2, **`nav2_use_local_slam_map:=true`** so Nav2 costmaps use local `/<robot>/map` from SLAM instead of network `/map`, and **`slam_toolbox_mode:=sync`** to try the synchronous SLAM Toolbox node. If both `fleet_map_relay_hz` and `nav2_use_local_slam_map` are set, `nav2_use_local_slam_map` takes precedence and disables the merged-map relay path for Nav2.

> **Note:** The old helper path `scripts/start_slam_with_normalizer.sh` is retired in this workspace. Use `navigation2_slam.launch.py` for SLAM + Nav2; `robot.launch.py` keeps `start_slam_with_normalizer` only as a deprecated no-op compatibility flag.

#### 3. Central computer behavior with namespaces

With namespaced robots and `bridged_domains` as the fleet policy:

- The central computer uses the configured central domain (default `50` in `config/fleet_domain_map.yaml` in the central repo).
- Multi-robot tools (RViz, explorers, custom coordination nodes) can subscribe to bridged fleet topics such as:
  - `/blinky/scan_normalized`, `/blinky/map`, `/blinky/tf`, `/blinky/cmd_vel`
  - `/pinky/…`, `/inky/…`, `/clyde/…`
- Fleet behavior remains defined by central bridge contract/domain map configuration rather than a shared single-domain policy.

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
- **Robot drives toward the goal through walls / ignores the global plan in RViz** while using the central explorer: Ensure SLAM + Nav2 was started with **`fleet_mode:=true`** (or `use_central_tf_map:=true`) when `start_central.sh` is running. On the central PC you can verify the chain with `ROS_DOMAIN_ID=<central_domain_id> python3 scripts/diagnostics/diagnose_multirobot_tf.py` (from `~/central-computer`).
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
