#!/bin/bash
#
# Switch Raspberry Pi WiFi between SNS (lab), legacy GCRI_LAB (lab_old), and Azure (mobile hotspot).
# Uses netplan only; no NetworkManager required.
#
# Usage:
#   sudo ./scripts/switch_wifi.sh lab       # SNS WiFi (DHCP)
#   sudo ./scripts/switch_wifi.sh lab_old   # GCRI_LAB with static IP (per robot/user)
#   sudo ./scripts/switch_wifi.sh azure     # Azure hotspot (DHCP)
#   ./scripts/switch_wifi.sh status         # show current WiFi (no sudo)
#
# Prereq: Remove or comment out the wifis/wlan0 block from
#   /etc/netplan/50-cloud-init.yaml so this script's 99-wifi-switch.yaml
#   is the only WiFi config (avoids "Duplicate access point SSID").
#
# Static IP on GCRI_LAB (lab_old) is chosen by current user (pinky -> .219, blinky -> .193).
# When run with sudo we use SUDO_USER so "pinky" user gets .219. Override: $0 lab_old blinky or ROBOT_NAME=blinky.
#

set -e
NETPLAN_OVERRIDE="/etc/netplan/99-wifi-switch.yaml"

# Legacy lab WiFi (old GCRI_LAB)
LAB_OLD_SSID="GCRI_LAB"
LAB_OLD_PASSWORD="j1-gcri!"
LAB_GATEWAY="192.168.50.1"
LAB_PREFIX="24"

# Static IP per robot on GCRI_LAB (username -> IP)
LAB_IP_PINKY="192.168.50.219"
LAB_IP_BLINKY="192.168.50.193"

# New lab WiFi (SNS, DHCP)
SNS_SSID="SNS"
SNS_PASSWORD="sn5_rox!"

AZURE_SSID="Azure"
AZURE_PASSWORD="howdoyouwanttodothis"

# Resolve robot name: current user (when using sudo we use SUDO_USER), or ROBOT_NAME env or second argument for "lab"
get_robot_name() {
  local name
  name="${SUDO_USER:-$USER}"
  [ -n "$name" ] && name=$(echo "$name" | tr '[:upper:]' '[:lower:]')
  echo "${name:-}"
}

# Set LAB_STATIC_IP for GCRI_LAB from robot name (pinky -> LAB_IP_PINKY, blinky -> LAB_IP_BLINKY). Exits if unknown.
set_lab_static_ip() {
  local robot
  robot="${1:-$(get_robot_name)}"
  robot="${robot,,}"
  case "$robot" in
    pinky)  LAB_STATIC_IP="$LAB_IP_PINKY"  ;;
    blinky) LAB_STATIC_IP="$LAB_IP_BLINKY" ;;
    *)
      echo "Unknown robot: '$robot'. Current user is: $(get_robot_name)."
      echo "Use: $0 lab_old pinky   or   $0 lab_old blinky   (or set ROBOT_NAME=pinky/blinky)"
      exit 1
      ;;
  esac
}

usage() {
  echo "Usage: $0 { lab | lab_old | azure | status } [robot]"
  echo "  lab                 - connect to SNS (DHCP)"
  echo "  lab_old [pinky|blinky] - connect to legacy GCRI_LAB (static IP by robot)"
  echo "  azure               - connect to Azure hotspot (DHCP)"
  echo "  status              - show current WiFi (no sudo)"
  exit 1
}

# Run netplan apply but hide the harmless Open vSwitch warning
netplan_apply_quiet() {
  netplan apply 2> >(grep -v -E 'Open vSwitch|ovsdb-server' >&2)
}

write_netplan_lab_old() {
  if [ -n "$LAB_STATIC_IP" ]; then
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
        "${LAB_OLD_SSID}":
          password: "${LAB_OLD_PASSWORD}"
EOF
  else
    cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "${LAB_OLD_SSID}":
          password: "${LAB_OLD_PASSWORD}"
EOF
  fi
}

# New lab WiFi (SNS, always DHCP)
write_netplan_lab() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "${SNS_SSID}":
          password: "${SNS_PASSWORD}"
EOF
}

write_netplan_azure() {
  cat << EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "${AZURE_SSID}":
          password: "${AZURE_PASSWORD}"
EOF
}

case "${1:-}" in
  lab)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for lab/azure: sudo $0 lab"
      exit 1
    fi
    write_netplan_lab > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to SNS (DHCP)."
    ;;
  lab_old)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for lab_old/azure: sudo $0 lab_old"
      exit 1
    fi
    set_lab_static_ip "${2:-$ROBOT_NAME}"
    write_netplan_lab_old > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to legacy GCRI_LAB (static IP $LAB_STATIC_IP)."
    ;;
  azure)
    if [ "$(id -u)" -ne 0 ]; then
      echo "Run with sudo for lab/azure: sudo $0 azure"
      exit 1
    fi
    write_netplan_azure > "$NETPLAN_OVERRIDE"
    chmod 600 "$NETPLAN_OVERRIDE"
    netplan_apply_quiet
    echo "Switched to Azure (DHCP)."
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
