#!/usr/bin/env bash
# Start Zenoh client + ros2dds bridge on any fleet robot.
# Usage:
#   ./scripts/comms/start_zenoh_robot.sh              # auto-detect robot + domain
#   ./scripts/comms/start_zenoh_robot.sh reverie       # optional central Tailscale host
#   ROBOT_NAME=pinky ./scripts/comms/start_zenoh_robot.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CENTRAL_HOST="${1:-${ZENOH_CENTRAL_HOST:-reverie}}"

detect_robot() {
  local name=""
  if [[ -n "${ROBOT_NAME:-}" ]]; then
    name="${ROBOT_NAME}"
  elif [[ -n "${TURTLEBOT3_ROBOT_NAME:-}" ]]; then
    name="${TURTLEBOT3_ROBOT_NAME}"
  elif [[ -n "${USER:-}" ]]; then
    name="${USER}"
  else
    name="${HOSTNAME:-}"
  fi
  echo "${name}" | tr '[:upper:]' '[:lower:]'
}

domain_for_robot() {
  case "$1" in
    blinky) echo 5 ;;
    pinky) echo 22 ;;
    inky) echo 19 ;;
    clyde) echo 80 ;;
    *) return 1 ;;
  esac
}

ROBOT="$(detect_robot)"
if ! DOMAIN_ID="$(domain_for_robot "$ROBOT")"; then
  echo "ERROR: unknown robot identity '${ROBOT}'."
  echo "Valid robots: blinky, pinky, inky, clyde"
  echo "Set ROBOT_NAME=<robot> or log in as that user (hostname alone is often 'ubuntu')."
  exit 1
fi

TEMPLATE="${WORKSPACE_DIR}/config/zenoh/robot_client.json5.tmpl"
if [[ ! -f "$TEMPLATE" ]]; then
  echo "ERROR: missing template: $TEMPLATE"
  exit 1
fi

# shellcheck source=zenoh_bridge_path.bash
source "${SCRIPT_DIR}/zenoh_bridge_path.bash"
if ! ZENOH_BIN="$(resolve_zenoh_bridge_ros2dds)"; then
  echo "ERROR: zenoh-bridge-ros2dds not found."
  echo "  On Ubuntu 22.04 use the standalone installer (apt needs glibc 2.38+):"
  echo "    ${WORKSPACE_DIR}/scripts/comms/install_zenoh_bridge.sh"
  exit 1
fi

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/zenoh_bridge_${ROBOT}"
mkdir -p "$RUNTIME_DIR"
RUNTIME_CONFIG="${RUNTIME_DIR}/client.json5"

python3 - "$TEMPLATE" "$RUNTIME_CONFIG" "$ROBOT" "$DOMAIN_ID" "$CENTRAL_HOST" <<'PY'
import pathlib, sys
src, dst, robot, domain, host = (
    pathlib.Path(sys.argv[1]),
    pathlib.Path(sys.argv[2]),
    sys.argv[3],
    sys.argv[4],
    sys.argv[5],
)
text = src.read_text(encoding="utf-8")
for old, new in (
    ("__ROBOT__", robot),
    ("__DOMAIN__", domain),
    ("__CENTRAL_HOST__", host),
):
    text = text.replace(old, new)
dst.write_text(text, encoding="utf-8")
print(f"Wrote {dst} (robot={robot}, domain={domain}, central=tcp/{host}:7447)")
PY

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DOMAIN_ID}"
unset ROS_LOCALHOST_ONLY
export ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  unset ROS_LOCALHOST_ONLY
fi

echo "Starting Zenoh robot bridge"
echo "  robot:    ${ROBOT} (auto-detected)"
echo "  central:  ${CENTRAL_HOST}"
echo "  binary:   ${ZENOH_BIN}"
echo "  config:   ${RUNTIME_CONFIG}"
echo "  domain:   ${ROS_DOMAIN_ID}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  distro:   ${ROS_DISTRO}"
echo "  ROS_LOCALHOST_ONLY: (unset — required for Nav2/SLAM participant count)"
echo ""

exec "${ZENOH_BIN}" -c "$RUNTIME_CONFIG"
