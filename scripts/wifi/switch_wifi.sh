#!/bin/bash
#
# Switch Raspberry Pi WiFi between SNS (lab), RaspAP (rpi), and Azure.
#
# Usage:
#   sudo ./scripts/wifi/switch_wifi.sh lab       # SNS WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh rpi       # RaspAP WiFi with static IP (per robot/user)
#   sudo ./scripts/wifi/switch_wifi.sh azure     # Azure hotspot with static IP (per robot/user)
#   ./scripts/wifi/switch_wifi.sh status         # show current WiFi (no sudo)
#
# Prereq: Remove or comment out the wifis/wlan0 block from
#   /etc/netplan/50-cloud-init.yaml so this script's 99-wifi-switch.yaml
#   is the only WiFi config (avoids "Duplicate access point SSID").
#
# Static IPs are chosen by current user (blinky / pinky / inky / clyde):
#   SNS:      blinky -> 192.168.0.158,  pinky -> 192.168.0.194,  inky -> 192.168.0.139,  clyde -> 192.168.0.236
#   RaspAP:   blinky -> 10.3.141.220,   pinky -> 10.3.141.194,   inky -> 10.3.141.139,  clyde -> 10.3.141.236
#   Azure:    blinky -> 172.20.10.13,   pinky -> 172.20.10.14,   inky -> 172.20.10.15,   clyde -> 172.20.10.16
# When run with sudo we use SUDO_USER so \"blinky\" user gets the blinky IPs.
# Override with: $0 lab blinky / $0 rpi pinky / $0 azure inky or ROBOT_NAME=clyde.
#

set -e
NETPLAN_OVERRIDE="/etc/netplan/99-wifi-switch.yaml"

# SNS WiFi (lab) static IP config
LAB_GATEWAY="192.168.0.1"
LAB_PREFIX="24"
LAB_IP_BLINKY="192.168.0.158"
LAB_IP_PINKY="192.168.0.194"
LAB_IP_INKY="192.168.0.139"
LAB_IP_CLYDE="192.168.0.236"

# RaspAP WiFi (rpi) static IP config
RPI_GATEWAY="10.3.141.1"
RPI_PREFIX="24"
RPI_IP_BLINKY="10.3.141.220"
RPI_IP_PINKY="10.3.141.194"
RPI_IP_INKY="10.3.141.139"
RPI_IP_CLYDE="10.3.141.236"

# Azure Wifi static IP config
AZURE_GATEWAY="172.20.10.1"
AZURE_PREFIX="28"
AZURE_IP_BLINKY="172.20.10.13"
AZURE_IP_PINKY="172.20.10.14"
AZURE_IP_INKY="172.20.10.15"
AZURE_IP_CLYDE="172.20.10.16"

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
    inky)
      LAB_STATIC_IP="$LAB_IP_INKY"
      RPI_STATIC_IP="$RPI_IP_INKY"
      AZURE_STATIC_IP="$AZURE_IP_INKY"
      ;;
    clyde)
      LAB_STATIC_IP="$LAB_IP_CLYDE"
      RPI_STATIC_IP="$RPI_IP_CLYDE"
      AZURE_STATIC_IP="$AZURE_IP_CLYDE"
      ;;
    *)
      echo "Unknown robot: '$robot'. Current user is: $(get_robot_name)"
      echo "Use: $0 {lab|rpi|azure} {blinky|pinky|inky|clyde}   (or set ROBOT_NAME=…)"
      exit 1
      ;;
  esac
}

usage() {
  echo "Usage: $0 { lab | azure | rpi | status } [robot]"
  echo "  lab [blinky|pinky|inky|clyde]   - connect to SNS (static IP by robot)"
  echo "  rpi [blinky|pinky|inky|clyde]   - connect to RaspAP (static IP by robot)"
  echo "  azure [blinky|pinky|inky|clyde] - connect to Azure (static IP by robot)"
  echo "  status                    - show current WiFi (no sudo)"
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
    echo "Switched to SNS (SSID ${SNS_SSID}, static IP ${LAB_STATIC_IP})."
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
    echo "Switched to RaspAP (SSID ${RASPAP_SSID}, static IP ${RPI_STATIC_IP})."
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
    echo "Switched to Azure (SSID ${AZURE_SSID}, static IP ${AZURE_STATIC_IP})."
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
