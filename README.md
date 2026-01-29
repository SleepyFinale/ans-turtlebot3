# TurtleBot3 Burger (Blinky) Setup Notes

This repo is being used to track configuration + code changes on TurtleBot3 robots. This specific robot is **Blinky** (one of four robots).

### References (authoritative)

- **TurtleBot3 SBC setup (Robotis e-Manual)**: `https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/`
- **ROS 2 Humble install (Ubuntu debs)**: `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`

### Goal of this document

Document the steps to prepare the TurtleBot3 Raspberry Pi SBC **up through**:

- Robotis SBC setup (Ubuntu Server, Wi‑Fi/SSH, stability tweaks)
- ROS 2 Humble install and **Install and Build ROS Packages** (SBC setup Step 3.2.5, Step 2)

**Workspace = this repo:** We use this repo as the `turtlebot3_ws` workspace on the Pi. The script installs into `src/` here so any changes you make to the cloned TurtleBot3/LDS/Coin D4 packages can be committed and pushed to this GitHub repo. Build/install/log are in `.gitignore`.

---

## Robot identity

- **Robot name**: Blinky
- **Platform**: TurtleBot3 Burger
- **SBC**: Raspberry Pi (Ubuntu Server)
- **ROS distro target**: Humble Hawksbill (Ubuntu 22.04 / Jammy)

---

## SBC setup (Robotis e-Manual) — up to ROS 2 install

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

### First boot + basic configuration (on the TurtleBot3 SBC)

- Boot the Raspberry Pi with HDMI + keyboard (first boot is easiest locally).
- Log in (Ubuntu will prompt you to change password on first login if using the default user).

If Wi‑Fi was not configured via Imager, Robotis configures Wi‑Fi via netplan:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Edit Wi‑Fi settings (replace with your SSID/password), then save and reboot.

### Disable unattended upgrades (Robotis)

Robotis disables auto-upgrades to avoid surprise background package operations:

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

Set:

```text
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

### Prevent boot delays if network is slow/unavailable (Robotis)

```bash
sudo systemctl mask systemd-networkd-wait-online.service
```

### Disable sleep/hibernate targets (Robotis)

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

### Reboot (Robotis)

```bash
sudo reboot
```

### SSH in from a remote PC (Robotis)

After reboot, SSH to the SBC:

```bash
ssh ubuntu@<RASPBERRY_PI_IP>
```

If you need to find the IP on the SBC, Robotis suggests installing net-tools:

```bash
sudo apt update
sudo apt install -y net-tools
ifconfig
```

### (Optional) Swap file for low-memory Pi models (Robotis note)

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

## ROS 2 Humble install (Ubuntu debs) — Step 1 complete

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

**Stop point**: Next ROS guide section is **“Setup Sources”** (not done yet in this README).

---

## SBC setup Step 3.2.5 — Install and Build ROS Packages (Step 2)

After ROS 2 Humble is fully installed on the Pi (including "Setup Sources" and "Install ROS 2 packages"), run **Step 2** of "Install and Build ROS Packages" so everything lives in this repo and changes can be committed.

### On the Raspberry Pi (SBC)

1. **Clone this repo** on the Pi into the workspace path you want (e.g. `turtlebot3_ws`):

   ```bash
   cd ~
   git clone <your-repo-url> turtlebot3_ws
   cd turtlebot3_ws
   ```

2. **Optional (2GB Pi):** Create swap before building (Robotis recommendation):

   ```bash
   sudo fallocate -l 2G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   free -h
   ```

3. **Run the install script** (use wall power; build can take over an hour):

   ```bash
   chmod +x scripts/install_ros_packages_step2.sh
   ./scripts/install_ros_packages_step2.sh
   ```

   The script will:
   - Install apt and ROS packages (hls-lfcd-lds-driver, turtlebot3-msgs, dynamixel-sdk, xacro, etc.)
   - Create `src/` and clone `turtlebot3`, `ld08_driver`, `coin_d4_driver` (Humble)
   - Remove `turtlebot3_cartographer` and `turtlebot3_navigation2`
   - Run `colcon build --symlink-install --parallel-workers 1`
   - **Step 3:** Install USB udev rules for OpenCR (copy `99-turtlebot3-cdc.rules`, reload udev)
   - Append `source <workspace>/install/setup.bash` and ROS Humble setup to `~/.bashrc`

4. **Source the workspace** (or open a new shell):

   ```bash
   source ~/turtlebot3_ws/install/setup.bash
   ```

After this, `src/` contains the cloned repos; you can edit them and commit to this repo.

**If you already ran the script before Step 3 was added**, run these on the Pi (after sourcing the workspace) to apply USB port settings for OpenCR:

```bash
source ~/turtlebot3_ws/install/setup.bash
sudo cp $(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Next step on the e-manual: **3.2.5 Step 4** (ROS_DOMAIN_ID).