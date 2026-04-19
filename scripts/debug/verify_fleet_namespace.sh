#!/usr/bin/env bash
# Print host identity and optional ROS 2 topic hints for fleet namespace debugging.
# Run on a robot SBC or on the central PC (same ROS_DOMAIN_ID as the fleet).
#
# Usage:
#   ./scripts/verify_fleet_namespace.sh
#   FLEET_ROBOTS="pinky clyde" ./scripts/verify_fleet_namespace.sh
#   ./scripts/verify_fleet_namespace.sh --ros   # requires sourced ROS + workspace

set -euo pipefail

ROBOTS="${FLEET_ROBOTS:-blinky pinky inky clyde}"
DO_ROS=false
for a in "$@"; do
  if [[ "$a" == "--ros" ]]; then
    DO_ROS=true
  fi
done

echo "=========================================="
echo "  Fleet namespace / identity (local host)"
echo "=========================================="
echo ""
echo "  uname -n (kernel hostname): $(uname -n 2>/dev/null || echo '?')"
if command -v hostnamectl >/dev/null 2>&1; then
  echo "  hostnamectl:"
  hostnamectl status 2>/dev/null | sed 's/^/    /' || true
else
  echo "  hostnamectl: (not installed)"
fi
echo "  USER=${USER:-}  LOGNAME=${LOGNAME:-}"
echo "  HOSTNAME=${HOSTNAME:-<unset>}  HOST=${HOST:-<unset>}"
echo ""
echo "  In ans-turtlebot3, navigation2_slam default robot_name uses HOSTNAME/HOST"
echo "  if set; otherwise USER/LOGNAME. An image hostname of 'ubuntu' often means"
echo "  the ROS namespace follows the Linux login name (e.g. pinky vs clyde)."
echo ""

if [[ "$DO_ROS" == true ]]; then
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not in PATH. Source /opt/ros/humble/setup.bash and workspace install/setup.bash."
    exit 1
  fi
  echo "=========================================="
  echo "  ROS 2 graph (this shell, ROS_DOMAIN_ID)"
  echo "=========================================="
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo ""
  for r in $ROBOTS; do
    if ros2 topic list 2>/dev/null | grep -q "^/${r}/tf$"; then
      echo "  /${r}/tf: present"
      ros2 topic info "/${r}/tf" 2>/dev/null | sed 's/^/    /' || true
    else
      echo "  /${r}/tf: (not listed from this machine)"
    fi
    echo ""
  done
else
  echo "Tip: run with --ros after sourcing ROS + workspace to inspect /<robot>/tf."
fi

echo "=========================================="
echo "  Motion test (from CENTRAL PC, fleet running)"
echo "=========================================="
echo "  ros2 topic pub -1 /pinky/cmd_vel geometry_msgs/msg/Twist \\"
echo "    '{linear: {x: 0.05}, angular: {z: 0.0}}'"
echo "  See which physical robot moves; repeat for /clyde/cmd_vel."
echo ""
