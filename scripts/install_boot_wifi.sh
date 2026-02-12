#!/bin/bash
#
# Install boot WiFi service for automatic WiFi connection on boot.
# This script sets up the systemd service to run boot_wifi.sh on startup.
#
# Usage: sudo ./scripts/install_boot_wifi.sh
#

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "Error: This script must be run as root (use sudo)"
  exit 1
fi

# Get the workspace directory (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="boot-wifi.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"
TARGET_SERVICE="$SYSTEMD_DIR/$SERVICE_NAME"

echo "Installing boot WiFi service..."
echo "Workspace directory: $WORKSPACE_DIR"

# Create the service file with the correct workspace path
cat > "$TARGET_SERVICE" << EOF
[Unit]
Description=Boot WiFi Connection (SNS first, Azure fallback)
After=network-pre.target
Wants=network-pre.target
Before=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$WORKSPACE_DIR/scripts/boot_wifi.sh
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
