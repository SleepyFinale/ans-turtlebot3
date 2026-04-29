#!/usr/bin/env bash
#
# Set ROS_DOMAIN_ID using stable per-robot fleet mapping.
# Usage:
#   source scripts/env/ros_domain_profile.bash
#   source scripts/env/ros_domain_profile.bash pinky
#
# Detection order when no explicit arg is provided:
#   1) ROBOT_NAME / TURTLEBOT3_ROBOT_NAME
#   2) ROS_NAMESPACE (e.g. /pinky)
#   3) USER (common on SBCs)
#   4) HOSTNAME
_ros_domain_robot="${1:-}"
if [ -z "${_ros_domain_robot}" ]; then
  if [ -n "${ROBOT_NAME:-}" ]; then
    _ros_domain_robot="${ROBOT_NAME}"
  elif [ -n "${TURTLEBOT3_ROBOT_NAME:-}" ]; then
    _ros_domain_robot="${TURTLEBOT3_ROBOT_NAME}"
  elif [ -n "${ROS_NAMESPACE:-}" ]; then
    _ros_domain_robot="${ROS_NAMESPACE#/}"
  elif [ -n "${USER:-}" ]; then
    _ros_domain_robot="${USER}"
  else
    _ros_domain_robot="${HOSTNAME:-}"
  fi
fi

_ros_domain_robot="$(echo "${_ros_domain_robot}" | tr '[:upper:]' '[:lower:]')"

case "${_ros_domain_robot}" in
  blinky) export ROS_DOMAIN_ID=05 ;;
  pinky) export ROS_DOMAIN_ID=22 ;;
  inky) export ROS_DOMAIN_ID=19 ;;
  clyde) export ROS_DOMAIN_ID=80 ;;
  *)
    echo "WARN: unknown robot '${_ros_domain_robot}'"
    echo "Valid robots: blinky, pinky, inky, clyde"
    if [ -n "${ROS_DOMAIN_ID:-}" ]; then
      echo "Keeping existing ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
      return 0 2>/dev/null || exit 0
    fi
    echo "No existing ROS_DOMAIN_ID set; pass robot name explicitly:"
    echo "  source scripts/env/ros_domain_profile.bash pinky"
    return 1 2>/dev/null || exit 1
    ;;
esac

echo "ROS domain profile loaded: robot=${_ros_domain_robot}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
