# Robot setup (one time per Pi)

How a TurtleBot3 Burger Pi is installed for this fleet: OS, ROS 2 Humble, this workspace, Wi-Fi, udev names, OpenCR firmware and LiDAR model. Day-to-day use is in the [README](../README.md).

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

After reboot, SSH to the SBC. Use the **username@IP** (or hostname) for the robot you’re setting up—see the [Robot fleet reference](../README.md#robot-fleet-reference) table. For example, for Pinky on Azure: `ssh pinky@172.20.10.14`; on TAMU_WiFi: `ssh pinky@<dhcp-ip>`.

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

## LDS configuration

We use **LDS-02**. On the robot (SBC):

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```

`LDS-02` provides **360-degree coverage**, but that does not mean ROS always receives exactly 360 rays per scan in this driver path. The sensor uses a fixed sampling stream and the driver assembles/bins points into `LaserScan`, so beam count can vary by RPM and packet timing. In this repo, `normalize_laser_scan.py` now auto-detects a stable beam count during startup and locks `scan_normalized` to that value for SLAM/Nav2 consistency (with `target_readings` as fallback/override).


## OpenCR setup

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
