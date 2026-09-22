#!/usr/bin/env bash
# Start Zenoh router + ros2dds bridge on the central PC for one robot domain.
# Usage:
#   ./scripts/comms/start_zenoh_central.sh [robot]
# Default robot: clyde
#
# Listens on tcp/0.0.0.0:7447 (reachable over Tailscale as tcp/<central-name>:7447).
# Injects allowed /<robot>/* interfaces into the robot ROS_DOMAIN_ID on localhost
# so existing domain_bridge in start_central.sh keeps working.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROBOT="$(echo "${1:-clyde}" | tr '[:upper:]' '[:lower:]')"

case "$ROBOT" in
  clyde)
    CONFIG="${WORKSPACE_DIR}/config/zenoh/central_clyde.json5"
    DOMAIN_ID=80
    ;;
  *)
    echo "ERROR: Zenoh central config currently shipped for 'clyde' only (got: ${ROBOT})."
    echo "Add config/zenoh/central_${ROBOT}.json5 and extend this script when onboarding more robots."
    exit 1
    ;;
esac

if [[ ! -f "$CONFIG" ]]; then
  echo "ERROR: missing config: $CONFIG"
  exit 1
fi

# shellcheck source=zenoh_bridge_path.bash
source "${SCRIPT_DIR}/zenoh_bridge_path.bash"
if ! ZENOH_BIN="$(resolve_zenoh_bridge_ros2dds)"; then
  echo "ERROR: zenoh-bridge-ros2dds not found."
  echo "  On Ubuntu 22.04 the apt package needs glibc 2.38+ and will fail."
  echo "  Install the musl standalone binary instead:"
  echo "    ${WORKSPACE_DIR}/scripts/comms/install_zenoh_bridge.sh"
  exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# Avoid zenoh assuming ROS Iron when Humble env wasn't sourced in this shell.
export ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  # shellcheck disable=SC1090
  set +u
  # Prefer Cyclone when setup.bash would reset RMW.
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi

echo "Starting Zenoh central bridge"
echo "  robot:    ${ROBOT}"
echo "  binary:   ${ZENOH_BIN}"
echo "  config:   ${CONFIG}"
echo "  domain:   ${ROS_DOMAIN_ID}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  distro:   ${ROS_DISTRO}"
echo "  listen:   tcp/0.0.0.0:7447"
echo "  localhost DDS only: ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo ""
echo "On Clyde, run: ./scripts/comms/start_zenoh_robot.sh clyde <central-tailscale-name>"
echo "  e.g. ./scripts/comms/start_zenoh_robot.sh clyde reverie"
echo ""

exec "${ZENOH_BIN}" -c "$CONFIG"
