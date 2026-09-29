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
# CycloneDDS is required by zenoh-bridge-ros2dds. DDS itself stays on loopback
# (see CYCLONEDDS_URI) so a Wi-Fi roam does not spam ddsi_udp_conn_write.
# Do not set ROS_LOCALHOST_ONLY=1: that path uses Cyclone's small default
# participant cap. The URI raises MaxAutoParticipantIndex instead.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
_cyclone_cfg="${WS_DIR}/config/cyclonedds/robot_localhost.xml"
if [ -f "${_cyclone_cfg}" ]; then
  export CYCLONEDDS_URI="file://${_cyclone_cfg}"
fi
unset _cyclone_cfg

# Overlay the tf2 build that runs transform callbacks without holding
# transformable_requests_mutex_. Stock Humble 0.25.23 deadlocks that mutex
# against Buffer::waitForTransform, and Nav2's laser callback then freezes
# map->odom for the life of the controller.
_tf2_fix_lib="${HOME}/tf2_deadlock_fix/install/tf2/lib"
if [ -f "${_tf2_fix_lib}/libtf2.so" ]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${_tf2_fix_lib}:"*) ;;
    *) export LD_LIBRARY_PATH="${_tf2_fix_lib}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
fi
unset _tf2_fix_lib

# Per-robot ROS_DOMAIN_ID (USER -> blinky=05, pinky=22, inky=19, clyde=80).
# Bringup/SLAM must share this domain with the Zenoh robot bridge.
_ros_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${_ros_env_dir}/ros_domain_profile.bash" ]; then
  # shellcheck disable=SC1091
  source "${_ros_env_dir}/ros_domain_profile.bash"
fi
unset _ros_env_dir

echo "ROS 2 Humble and workspace environment loaded from:"
echo "  Underlay: /opt/ros/humble"
echo "  Overlay : ${WS_DIR}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  ROS_LOCALHOST_ONLY: (unset)"
echo "  CYCLONEDDS_URI: ${CYCLONEDDS_URI:-unset}"
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
else
  echo "  ROS_DOMAIN_ID: (unset)"
fi
