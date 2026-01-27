# TurtleBot3 Burger (Blinky) Setup Notes

This repo is being used to track configuration + code changes on TurtleBot3 robots. This specific robot is **Blinky** (one of four robots).

### References (authoritative)

- **TurtleBot3 SBC setup (Robotis e-Manual)**: `https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/`
- **ROS 2 Humble install (Ubuntu debs)**: `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`

### Goal of this document

Document the steps to prepare the TurtleBot3 Raspberry Pi SBC **up through**:

- Robotis SBC setup (Ubuntu Server, Wi‑Fi/SSH, stability tweaks)
- ROS 2 Humble install **Step 1: Set locale (UTF‑8)** (and nothing beyond that yet)

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