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
WS_DIR="${WS_DIR:-$HOME/turtlebot3}"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "ERROR: ${WS_DIR}/install/setup.bash not found."
  echo "Make sure you are in the correct workspace and have run: colcon build"
  return 1 2>/dev/null || exit 1
fi

# Fleet defaults for TAMU/Zenoh + Nav2/SLAM on the Pi:
# CycloneDDS is required by zenoh-bridge-ros2dds; leave ROS_LOCALHOST_ONLY unset
# so Nav2/SLAM do not exhaust Cyclone's MaxAutoParticipantIndex.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

echo "ROS 2 Humble and workspace environment loaded from:"
echo "  Underlay: /opt/ros/humble"
echo "  Overlay : ${WS_DIR}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  ROS_LOCALHOST_ONLY: (unset)"
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
else
  echo "  ROS_DOMAIN_ID: (unset; source scripts/env/ros_domain_profile.bash for bridged mode)"
fi
