#!/usr/bin/env bash
# Rebuild only the packages needed for: ros2 launch turtlebot3_bringup robot.launch.py
# (turtlebot3_bringup, turtlebot3_node, turtlebot3_description, ld08_driver for LDS-02).
# No clean step; uses parallel workers. Much faster than clean_rebuild.sh.
# Usage: ./scripts/minimal_rebuild.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# Source ROS 2 if not already in environment
if ! command -v ros2 &>/dev/null; then
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found; ensure ROS 2 is installed."
  fi
fi

cd "${WS_DIR}"

PARALLEL_JOBS="${COLCON_PARALLEL_JOBS:-$(nproc 2>/dev/null || echo 1)}"

echo "Building minimal set for robot.launch.py (turtlebot3_description, turtlebot3_node, ld08_driver, turtlebot3_bringup)..."
colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}" \
  --packages-select turtlebot3_description turtlebot3_node ld08_driver turtlebot3_bringup \
  --allow-overriding turtlebot3_bringup turtlebot3_description turtlebot3_node

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  echo "Sourcing ${WS_DIR}/install/setup.bash"
  source "${WS_DIR}/install/setup.bash"
else
  echo "Error: install/setup.bash not found. Build may have failed." >&2
  exit 1
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL}"
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "Done. Run: ros2 launch turtlebot3_bringup robot.launch.py"
echo "Use 'source ${WS_DIR}/install/setup.bash' in new shells."
