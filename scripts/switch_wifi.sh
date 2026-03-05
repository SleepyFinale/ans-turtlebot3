#!/bin/bash
#
# Switch Raspberry Pi WiFi between SNS (lab), RaspAP (rpi), and Azure (mobile hotspot).
#
# Usage:
#   sudo ./scripts/switch_wifi.sh lab       # SNS WiFi with static IP (per robot/user)
#   sudo ./scripts/switch_wifi.sh rpi       # RaspAP WiFi with static IP (per robot/user)
#   sudo ./scripts/switch_wifi.sh azure     # Azure hotspot with static IP (per robot/user)
#   ./scripts/switch_wifi.sh status         # show current WiFi (no sudo)
#
# Prereq: Remove or comment out the wifis/wlan0 block from
#   /etc/netplan/50-cloud-init.yaml so this script's 99-wifi-switch.yaml
#   is the only WiFi config (avoids "Duplicate access point SSID").
#
# Static IPs are chosen by current user (blinky vs pinky):
#   SNS (lab):   blinky -> 192.168.0.158, pinky -> 192.168.0.194
#   RaspAP:      blinky -> 10.3.141.220, pinky -> 10.3.141.194
#   Azure:       blinky -> 172.20.10.13,  pinky -> 172.20.10.14
# When run with sudo we use SUDO_USER so \"blinky\" user gets the blinky IPs.
# Override with: $0 lab blinky / $0 azure blinky / $0 rpi blinky or ROBOT_NAME=blinky.
#

set -e
NETPLAN_OVERRIDE="/etc/netplan/99-wifi-switch.yaml"

# Lab WiFi (SNS) static IP config
LAB_GATEWAY="192.168.0.1"
LAB_PREFIX="24"
LAB_IP_BLINKY="192.168.0.158"
LAB_IP_PINKY="192.168.0.194"

# RPi (RaspAP) static IP config
RPI_GATEWAY="10.3.141.1"
RPI_PREFIX="24"
RPI_IP_BLINKY="10.3.141.220"
RPI_IP_PINKY="10.3.141.194"

# Azure hotspot static IP config
AZURE_GATEWAY="172.20.10.1"
AZURE_PREFIX="28"
AZURE_IP_BLINKY="172.20.10.13"
AZURE_IP_PINKY="172.20.10.14"

SNS_SSID="SNS"
SNS_PASSWORD="sn5_rox!"

RASPAP_SSID="RaspAP"
RASPAP_PASSWORD="sn5_rox!"

AZURE_SSID="Azure"
AZURE_PASSWORD="howdoyouwanttodothis"

# Resolve robot name: current user (when using sudo we use SUDO_USER), or ROBOT_NAME env or second argument
get_robot_name() {
  local name
  name="${SUDO_USER:-$USER}"
  [ -n "$name" ] && name=$(echo "$name" | tr '[:upper:]' '[:lower:]')
  echo "${name:-}"
}

# Set per-robot static IPs for SNS, RaspAP, and Azure. Exits if unknown robot.
set_robot_static_ips() {
  local robot
  robot="${1:-$(get_robot_name)}"
  robot="${robot,,}"
  case "$robot" in
    blinky)
      LAB_STATIC_IP="$LAB_IP_BLINKY"
      RPI_STATIC_IP="$RPI_IP_BLINKY"
      AZURE_STATIC_IP="$AZURE_IP_BLINKY"
      ;;
    pinky)
      LAB_STATIC_IP="$LAB_IP_PINKY"
      RPI_STATIC_IP="$RPI_IP_PINKY"
      AZURE_STATIC_IP="$AZURE_IP_PINKY"
      ;;
      echo "Unknown robot: '$robot'. Current user is: $(get_robot_name)."
      echo "Use: $0 lab blinky   or   $0 lab pinky   (or set ROBOT_NAME=blinky/pinky)"
      exit 1
      ;;
  esac
}

usage() {
  echo "Usage: $0 { lab | azure | rpi | status } [robot]"
  echo "  lab [blinky|pinky]   - connect to SNS (static IP by robot)"
  echo "  azure [blinky|pinky] - connect to Azure hotspot (static IP by robot)"
  echo "  rpi [blinky|pinky]   - connect to RaspAP WiFi (static IP by robot)"
  echo "  status               - show current WiFi (no sudo)"
  exit 1
}

# Run netplan apply but hide the harmless Open vSwitch warning
netplan_apply_quiet() {
  netplan apply 2> >(grep -v -E 'Open vSwitch|ovsdb-server' >&2)
}

write_netplan_lab() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${LAB_STATIC_IP}/${LAB_PREFIX}
      routes:
        - to: default
          via: ${LAB_GATEWAY}
      nameservers:
        addresses:
          - ${LAB_GATEWAY}
          - 8.8.8.8
      access-points:
        "${SNS_SSID}":
          password: "${SNS_PASSWORD}"
EOF
}

write_netplan_rpi() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${RPI_STATIC_IP}/${RPI_PREFIX}
      routes:
        - to: default
          via: ${RPI_GATEWAY}
      nameservers:
        addresses:
          - ${RPI_GATEWAY}
          - 8.8.8.8
      access-points:
        "${RASPAP_SSID}":
          password: "${RASPAP_PASSWORD}"
EOF
}

write_netplan_azure() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - ${AZURE_STATIC_IP}/${AZURE_PREFIX}
      routes:
        - to: default
          via: ${AZURE_GATEWAY}
      nameservers:
        addresses:
          - ${AZURE_GATEWAY}
          - 8.8.8.8
      access-points:
        "${AZURE_SSID}":
          password: "${AZURE_PASSWORD}"
EOF
}

case "${1:-}" in
  lab)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for SNS: sudo $0 lab"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_lab > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to SNS (static IP $LAB_STATIC_IP)."
    ;;
  rpi)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for RaspAP: sudo $0 rpi"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_rpi > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to RaspAP WiFi (SSID ${RASPAP_SSID}, static IP $RPI_STATIC_IP)."
    ;;
  azure)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for Azure: sudo $0 azure"
      exit 1
    fi
    set_robot_static_ips "${2:-$ROBOT_NAME}"
    write_netplan_azure > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to Azure (static IP $AZURE_STATIC_IP)."
    ;;
  status)
    ssid=""
    iwgetid -r &>/dev/null && ssid=$(iwgetid -r)
    [ -z "$ssid" ] && ssid=$(wpa_cli -i wlan0 status 2>/dev/null | sed -n 's/^ssid=//p')
    if [ -n "$ssid" ]; then
      echo "SSID: $ssid"
    else
      echo "SSID: (unknown or no WiFi)"
    fi
    ip4=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    if [ -n "$ip4" ]; then
      echo "wlan0 IP: $ip4"
    fi
    ;;
  *)
    usage
    ;;
esac
