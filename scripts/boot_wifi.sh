#!/bin/bash
#
# Boot-time WiFi connection script: tries SNS (lab) first, falls back to Azure (hotspot).
#
# This script is called by systemd on boot to ensure the robot connects to WiFi.
# It attempts to connect to SNS first, and only switches to Azure if SNS is unavailable.
#
# Usage: sudo ./scripts/boot_wifi.sh [robot]
#   robot: optional robot name (pinky/blinky). If not provided, detected from hostname.
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SWITCH_WIFI_SCRIPT="${SCRIPT_DIR}/switch_wifi.sh"

# Lab WiFi (SNS) gateway for connectivity check
LAB_GATEWAY="192.168.0.1"
AZURE_GATEWAY="172.20.10.1"

# Timeout for WiFi connection attempts (seconds)
CONNECTION_TIMEOUT=30
PING_TIMEOUT=3

# Detect robot name from hostname or use provided argument
get_robot_name() {
  local robot="${1:-}"
  
  if [ -n "$robot" ]; then
    echo "${robot,,}"
    return
  fi
  
  # Try to detect from hostname
  local hostname=$(hostname 2>/dev/null || echo "")
  hostname="${hostname,,}"
  
  case "$hostname" in
    pinky*)
      echo "pinky"
      ;;
    blinky*)
      echo "blinky"
      ;;
    inky*)
      echo "inky"
      ;;
    clyde*)
      echo "clyde"
      ;;
    *)
      # Fallback: try to get from /etc/hostname or use first non-root user
      if [ -f /etc/hostname ]; then
        local hname=$(cat /etc/hostname | tr '[:upper:]' '[:lower:]')
        case "$hname" in
          pinky*|blinky*|inky*|clyde*)
            echo "${hname%%[^a-z]*}"
            return
            ;;
        esac
      fi
      # Last resort: use first non-root user (if running as root)
      if [ "$(id -u)" -eq 0 ]; then
        local first_user=$(getent passwd | awk -F: '$3 >= 1000 && $1 != "nobody" {print $1; exit}')
        [ -n "$first_user" ] && echo "${first_user,,}" || echo "pinky"
      else
        echo "$(whoami | tr '[:upper:]' '[:lower:]')"
      fi
      ;;
  esac
}

# Check if WiFi interface has an IP address
has_ip() {
  ip -4 addr show wlan0 2>/dev/null | grep -q "inet "
}

# Check if we can ping the gateway
can_ping_gateway() {
  local gateway="$1"
  ping -c 1 -W "$PING_TIMEOUT" "$gateway" >/dev/null 2>&1
}

# Wait for WiFi connection with timeout
wait_for_connection() {
  local gateway="$1"
  local timeout="$2"
  local elapsed=0
  local interval=2
  
  while [ $elapsed -lt $timeout ]; do
    if has_ip && can_ping_gateway "$gateway"; then
      return 0
    fi
    sleep $interval
    elapsed=$((elapsed + interval))
  done
  
  return 1
}

# Main function
main() {
  local robot_name=$(get_robot_name "$1")
  
  if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
  fi
  
  echo "[boot_wifi] Starting WiFi connection for robot: $robot_name"
  
  # Step 1: Try to connect to SNS (lab WiFi)
  echo "[boot_wifi] Attempting to connect to SNS (lab WiFi)..."
  ROBOT_NAME="$robot_name" "$SWITCH_WIFI_SCRIPT" lab "$robot_name"
  
  # Wait for connection to establish
  if wait_for_connection "$LAB_GATEWAY" "$CONNECTION_TIMEOUT"; then
    local current_ssid=$(iwgetid -r 2>/dev/null || echo "unknown")
    local current_ip=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    echo "[boot_wifi] Successfully connected to SNS (SSID: $current_ssid, IP: $current_ip)"
    exit 0
  fi
  
  # Step 2: SNS failed, try Azure (hotspot)
  echo "[boot_wifi] SNS connection failed, attempting Azure (hotspot)..."
  ROBOT_NAME="$robot_name" "$SWITCH_WIFI_SCRIPT" azure "$robot_name"
  
  # Wait for connection to establish
  if wait_for_connection "$AZURE_GATEWAY" "$CONNECTION_TIMEOUT"; then
    local current_ssid=$(iwgetid -r 2>/dev/null || echo "unknown")
    local current_ip=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    echo "[boot_wifi] Successfully connected to Azure (SSID: $current_ssid, IP: $current_ip)"
    exit 0
  fi
  
  # Both failed
  echo "[boot_wifi] ERROR: Failed to connect to both SNS and Azure WiFi networks"
  exit 1
}

main "$@"
