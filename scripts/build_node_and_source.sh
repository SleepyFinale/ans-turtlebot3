#!/usr/bin/env bash
# Build, source, and prepare turtlebot3_ws for use (ROS 2 Humble).
# Usage:
#   ./scripts/build_and_source.sh          # build then source
#   ./scripts/build_and_source.sh --source # only source (no build)
#   ./scripts/build_and_source.sh --clean  # clean, then build, then source

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Source ROS 2 if not already in environment (e.g. ros2 not in path)
if ! command -v ros2 &>/dev/null; then
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found; ensure ROS 2 is installed."
  fi
fi

cd "${WS_DIR}"

echo "Building workspace (colcon build --packages-select turtlebot3_node --symlink-install --parallel-workers 1)..."
colcon build --packages-select turtlebot3_node --symlink-install --parallel-workers 1

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  echo "Sourcing ${WS_DIR}/install/setup.bash"
  source "${WS_DIR}/install/setup.bash"
else
  echo "Error: install/setup.bash not found. Build may have failed." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL}"
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"

echo "Done. Workspace is built and sourced. Use 'source ${WS_DIR}/install/setup.bash' in new shells."
