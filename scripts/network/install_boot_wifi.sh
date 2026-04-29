#!/bin/bash
#
# Install boot WiFi service for automatic WiFi connection on boot.
# This script sets up the systemd service to run boot_wifi.sh on startup.
#
# Usage: sudo ./scripts/network/install_boot_wifi.sh
#

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "Error: This script must be run as root (use sudo)"
  exit 1
fi

# Get the workspace directory (two levels up from this script: workspace/scripts/network)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
SERVICE_NAME="boot-wifi.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"
TARGET_SERVICE="$SYSTEMD_DIR/$SERVICE_NAME"

echo "Installing boot WiFi service..."
echo "Workspace directory: $WORKSPACE_DIR"

# Basic sanity check: ensure the target script exists at the expected (post-move) path.
BOOT_WIFI_SCRIPT="$WORKSPACE_DIR/scripts/network/boot_wifi.sh"
if [ ! -f "$BOOT_WIFI_SCRIPT" ]; then
  echo "Error: Expected boot WiFi script not found at: $BOOT_WIFI_SCRIPT"
  echo "Please ensure you are running this from a valid turtlebot3 checkout."
  exit 1
fi

# Create the service file with the correct workspace path
cat > "$TARGET_SERVICE" << EOF
[Unit]
Description=Boot WiFi Connection (lab, then gcri, then rpi)
After=network-pre.target
Wants=network-pre.target
Before=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$BOOT_WIFI_SCRIPT
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "Service file installed to $TARGET_SERVICE"

# Reload systemd and enable the service
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"

echo ""
echo "Boot WiFi service installed and enabled."
echo "The service will start automatically on boot."
echo ""
echo "To check status: sudo systemctl status $SERVICE_NAME"
echo "To view logs: sudo journalctl -u $SERVICE_NAME"
echo "To test now: sudo systemctl start $SERVICE_NAME"
echo ""
echo "Prereq: only one netplan file should configure wlan0. If \`switch_wifi.sh\` applies lab/gcri/rpi,"
echo "  remove the wifis/wlan0 block from /etc/netplan/50-cloud-init.yaml (see scripts/network/switch_wifi.sh"
echo "  header), then: sudo netplan apply"

netplan_ok=1
netplan_out=$(netplan generate 2>&1) || netplan_ok=0
if [ "$netplan_ok" -eq 0 ]; then
  echo ""
  echo "WARNING: netplan is currently invalid — boot WiFi will fail until this is fixed:"
  echo "$netplan_out"
  echo ""
  echo "If you see 'Duplicate access point SSID', delete or comment out wlan0 under wifis in"
  echo "  /etc/netplan/50-cloud-init.yaml so /etc/netplan/99-wifi-switch.yaml is the only WiFi config."
fi

# If the service is enabled, restart only when netplan is valid.
if systemctl is-enabled --quiet "$SERVICE_NAME" && [ "$netplan_ok" -eq 1 ]; then
  systemctl restart "$SERVICE_NAME" || true
elif systemctl is-enabled --quiet "$SERVICE_NAME"; then
  echo ""
  echo "Skipping: systemctl restart $SERVICE_NAME (fix netplan first, then: sudo systemctl start $SERVICE_NAME)"
fi