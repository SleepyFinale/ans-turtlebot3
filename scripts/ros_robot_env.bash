#!/usr/bin/env bash

# Base ROS 2 Humble underlay
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  return 1 2>/dev/null || exit 1
fi

# Workspace overlay
WS_DIR="${WS_DIR:-$HOME/turtlebot3_ws}"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "ERROR: ${WS_DIR}/install/setup.bash not found."
  echo "Make sure you are in the correct workspace and have run: colcon build"
  return 1 2>/dev/null || exit 1
fi

echo "ROS 2 Humble and workspace environment loaded from:"
echo "  Underlay: /opt/ros/humble"
echo "  Overlay : ${WS_DIR}"
