#!/usr/bin/env bash
# Clean, build, and source turtlebot3_ws (ROS 2 Humble).
# Default: remove build/install/log, run colcon build, then source install/setup.bash.
# Usage:
#   ./scripts/clean_rebuild.sh          # clean, build, then source
#   ./scripts/clean_rebuild.sh --no-clean  # build then source (no clean)
#   ./scripts/clean_rebuild.sh --source   # only source (no clean, no build)

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

# Parallel workers: use COLCON_PARALLEL_JOBS if set, else 1 (override with COLCON_PARALLEL_JOBS=N for more).
PARALLEL_JOBS="${COLCON_PARALLEL_JOBS:-1}"

if [ "${1:-}" = "--source" ]; then
  echo "Skipping clean and build (--source only)."
else
  if [ "${1:-}" != "--no-clean" ]; then
    echo "Cleaning build, install, log..."
    rm -rf build install log
    # Clear stale workspace paths so colcon does not warn about missing install dirs
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    echo "Skipping clean (--no-clean)."
  fi
  echo "Building workspace (colcon build --symlink-install --parallel-workers ${PARALLEL_JOBS})..."
  colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}"
fi

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
