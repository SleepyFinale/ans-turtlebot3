#!/usr/bin/env bash
# Install Chrony on Ubuntu and point it at a fleet time source (NTP pool or central PC).
# Wi‑Fi-only robot ↔ central: use the same stratum-1/2 source on Pis and the central PC,
# or set FLEET_TIME_SERVER to the central PC LAN IP so all robots track the same clock.
#
# Usage:
#   sudo FLEET_TIME_SERVER=192.168.0.10 ./scripts/chrony/setup_chrony_fleet_client.sh
#   sudo ./scripts/chrony/setup_chrony_fleet_client.sh   # uses pool.ntp.org
set -euo pipefail

FLEET_TIME_SERVER="${FLEET_TIME_SERVER:-pool.ntp.org}"
CHRONY_CONF="/etc/chrony/chrony.conf"

if [[ "${EUID:-0}" -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y chrony

# Backup once
if [[ ! -f "${CHRONY_CONF}.bak" ]]; then
  cp -a "$CHRONY_CONF" "${CHRONY_CONF}.bak"
fi

cat >"$CHRONY_CONF" <<EOF
# Managed by turtlebot3 scripts/chrony/setup_chrony_fleet_client.sh
# FLEET_TIME_SERVER=${FLEET_TIME_SERVER}

# Use fleet server or public NTP pool
server ${FLEET_TIME_SERVER} iburst

# Fallback pools (comment if you use a single LAN server only)
pool 2.ubuntu.pool.ntp.org iburst maxsources 2

driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
EOF

systemctl enable chrony.service
systemctl restart chrony.service

echo "Chrony configured. Check: chronyc tracking && timedatectl status"
