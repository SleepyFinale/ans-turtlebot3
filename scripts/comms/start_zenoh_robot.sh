#!/usr/bin/env bash
# Start Zenoh client + ros2dds bridge on a robot (Clyde first).
# Usage:
#   ./scripts/comms/start_zenoh_robot.sh [robot] [central_tailscale_host]
# Defaults: robot=clyde, central=reverie
#
# Copy this script + config/zenoh/ to the robot if needed:
#   scp -r scripts/comms config/zenoh clyde@clyde:~/turtlebot3/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer workspace root two levels up (central-computer or turtlebot3 layout).
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROBOT="$(echo "${1:-clyde}" | tr '[:upper:]' '[:lower:]')"
CENTRAL_HOST="${2:-${ZENOH_CENTRAL_HOST:-reverie}}"

case "$ROBOT" in
  clyde)
    BASE_CONFIG="${WORKSPACE_DIR}/config/zenoh/clyde_client.json5"
    DOMAIN_ID=80
    ;;
  *)
    echo "ERROR: robot Zenoh client config currently shipped for 'clyde' only (got: ${ROBOT})."
    exit 1
    ;;
esac

if [[ ! -f "$BASE_CONFIG" ]]; then
  echo "ERROR: missing config: $BASE_CONFIG"
  echo "From central: scp -r scripts/comms config/zenoh ${ROBOT}@${ROBOT}:~/turtlebot3/"
  exit 1
fi

# shellcheck source=zenoh_bridge_path.bash
source "${SCRIPT_DIR}/zenoh_bridge_path.bash"
if ! ZENOH_BIN="$(resolve_zenoh_bridge_ros2dds)"; then
  echo "ERROR: zenoh-bridge-ros2dds not found."
  echo "  On Ubuntu 22.04 use the musl standalone installer (apt needs glibc 2.38+):"
  echo "    ${WORKSPACE_DIR}/scripts/comms/install_zenoh_bridge.sh"
  exit 1
fi

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/zenoh_bridge_${ROBOT}"
mkdir -p "$RUNTIME_DIR"
RUNTIME_CONFIG="${RUNTIME_DIR}/client.json5"

# Rewrite connect endpoint to the chosen central Tailscale host.
python3 - "$BASE_CONFIG" "$RUNTIME_CONFIG" "$CENTRAL_HOST" <<'PY'
import pathlib, re, sys
src, dst, host = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), sys.argv[3]
text = src.read_text(encoding="utf-8")
endpoint = f'tcp/{host}:7447'
text2, n = re.subn(
    r'endpoints:\s*\[[^\]]*\]',
    f'endpoints: ["{endpoint}"]',
    text,
    count=1,
)
if n != 1:
    raise SystemExit("Failed to patch connect.endpoints in Zenoh client config")
dst.write_text(text2, encoding="utf-8")
print(f"Wrote {dst} with connect endpoint {endpoint}")
PY

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DOMAIN_ID}"
# Do NOT default ROS_LOCALHOST_ONLY=1 on the robot. CycloneDDS localhost mode
# has a tiny MaxAutoParticipantIndex; Nav2/SLAM + helpers exhaust it
# ("Failed to find a free participant index for domain N").
# Leave unset so local nodes use normal Cyclone discovery; Zenoh still
# tunnels only the allow-listed topics over Tailscale.
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
echo "  robot:    ${ROBOT}"
echo "  central:  ${CENTRAL_HOST}"
echo "  binary:   ${ZENOH_BIN}"
echo "  config:   ${RUNTIME_CONFIG}"
echo "  domain:   ${ROS_DOMAIN_ID}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  distro:   ${ROS_DISTRO}"
echo "  ROS_LOCALHOST_ONLY: (unset — required for Nav2/SLAM participant count)"
echo ""

exec "${ZENOH_BIN}" -c "$RUNTIME_CONFIG"
