#!/bin/bash
#
# Boot-time WiFi connection script: tries Azure (hotspot), then TAMU_WiFi.
#
# This script is called by systemd on boot to ensure the robot connects to WiFi.
# Order: azure -> tamu (each step runs only if the previous did not get connectivity).
#
# Usage: sudo ./scripts/network/boot_wifi.sh [robot]
#   robot: optional robot name (blinky/pinky/inky/clyde). If not provided, detected from hostname.
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SWITCH_WIFI_SCRIPT="${SCRIPT_DIR}/switch_wifi.sh"

# Gateway for Azure hotspot connectivity checks
AZURE_GATEWAY="172.20.10.1"

# Timeout for WiFi connection attempts (seconds)
CONNECTION_TIMEOUT=30
PING_TIMEOUT=3
PING_FALLBACK="8.8.8.8"

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
    blinky*)
      echo "blinky"
      ;;
    pinky*)
      echo "pinky"
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
          blinky*|pinky*|inky*|clyde*)
            echo "${hname%%[^a-z]*}"
            return
            ;;
        esac
      fi
      # Last resort: use first non-root user (if running as root)
      if [ "$(id -u)" -eq 0 ]; then
        local first_user=$(getent passwd | awk -F: '$3 >= 1000 && $1 != "nobody" {print $1; exit}')
        [ -n "$first_user" ] && echo "${first_user,,}" || echo "blinky"
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

# Default gateway from the routing table (DHCP), if any
get_default_gateway() {
  ip -4 route show default 2>/dev/null | awk '/default/ {print $3; exit}'
}

# Check if we can ping a host
can_ping() {
  local host="$1"
  [ -n "$host" ] && ping -c 1 -W "$PING_TIMEOUT" "$host" >/dev/null 2>&1
}

# Wait for WiFi connection with timeout.
# If $1 is a gateway, ping that; otherwise ping DHCP default route or fallback.
wait_for_connection() {
  local gateway_or_empty="$1"
  local timeout="$2"
  local elapsed=0
  local interval=2
  
  while [ $elapsed -lt $timeout ]; do
    if has_ip; then
      if [ -n "$gateway_or_empty" ]; then
        if can_ping "$gateway_or_empty"; then
          return 0
        fi
      else
        local gw
        gw=$(get_default_gateway)
        if can_ping "$gw" || can_ping "$PING_FALLBACK"; then
          return 0
        fi
      fi
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
  
  # Step 1: Try Azure hotspot
  echo "[boot_wifi] Attempting to connect to Azure (hotspot)..."
  ROBOT_NAME="$robot_name" "$SWITCH_WIFI_SCRIPT" azure "$robot_name"
  
  if wait_for_connection "$AZURE_GATEWAY" "$CONNECTION_TIMEOUT"; then
    local current_ssid=$(iwgetid -r 2>/dev/null || echo "unknown")
    local current_ip=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    echo "[boot_wifi] Successfully connected to Azure (SSID: $current_ssid, IP: $current_ip)"
    exit 0
  fi
  
  # Step 2: Azure failed, try TAMU_WiFi
  echo "[boot_wifi] Azure connection failed, attempting TAMU_WiFi..."
  "$SWITCH_WIFI_SCRIPT" tamu
  
  if wait_for_connection "" "$CONNECTION_TIMEOUT"; then
    local current_ssid=$(iwgetid -r 2>/dev/null || echo "unknown")
    local current_ip=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | head -1)
    echo "[boot_wifi] Successfully connected to TAMU_WiFi (SSID: $current_ssid, IP: $current_ip)"
    exit 0
  fi
  
  echo "[boot_wifi] ERROR: Failed to connect to Azure (hotspot) or TAMU_WiFi"
  echo "[boot_wifi] ERROR: Please check your WiFi connections and try again."
  exit 1
}

main "$@"
