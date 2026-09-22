#!/usr/bin/env bash
# Copy Zenoh comms helpers + configs from central-computer to a robot over Tailscale.
# Usage:
#   source scripts/env/set_robot_env.sh clyde
#   ./scripts/comms/sync_zenoh_to_robot.sh
# Or:
#   ./scripts/comms/sync_zenoh_to_robot.sh clyde@clyde
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TARGET="${1:-${ROBOT_SSH:-}}"

if [[ -z "$TARGET" ]]; then
  echo "Usage: $0 [user@host]"
  echo "  or:  source scripts/env/set_robot_env.sh clyde && $0"
  exit 1
fi

REMOTE_ROOT="${REMOTE_ROOT:-~/turtlebot3}"

echo "Syncing Zenoh helpers to ${TARGET}:${REMOTE_ROOT}"
ssh "$TARGET" "mkdir -p ${REMOTE_ROOT}/scripts ${REMOTE_ROOT}/config ${REMOTE_ROOT}/third_party"
scp -r \
  "${WORKSPACE_DIR}/scripts/comms" \
  "${TARGET}:${REMOTE_ROOT}/scripts/"
scp -r \
  "${WORKSPACE_DIR}/config/zenoh" \
  "${TARGET}:${REMOTE_ROOT}/config/"

echo ""
echo "On the robot, run:"
echo "  cd ${REMOTE_ROOT}"
echo "  ./scripts/comms/install_zenoh_bridge.sh"
echo "  sudo apt install -y ros-humble-rmw-cyclonedds-cpp   # if not already"
echo "  ./scripts/comms/start_zenoh_robot.sh clyde reverie"
echo "Done."
