#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

ROBOT_NAME="${1:-${ROBOT_NAME:-${USER:-robot}}}"
SESSION_TS="$(date +%Y%m%d-%H%M%S)"
BASE_OUT="${HOME}/.ros/nav2_debug/${ROBOT_NAME}"
BAG_OUT="${BASE_OUT}/bag-${SESSION_TS}"
JSONL_HINT="${BASE_OUT}/session-*.jsonl"

mkdir -p "$BASE_OUT"

echo "=========================================="
echo " Nav2 Debug Capture"
echo "=========================================="
echo " Robot name   : ${ROBOT_NAME}"
echo " Bag output   : ${BAG_OUT}"
echo " JSONL output : ${JSONL_HINT}"
echo ""
echo "Make sure navigation2_slam.launch.py is started with:"
echo "  enable_debug_logging:=true"
echo "  debug_log_dir:=${HOME}/.ros/nav2_debug"
echo ""
echo "Press Ctrl+C to stop recording."
echo "=========================================="
echo ""

ros2 bag record -o "$BAG_OUT" \
  "/${ROBOT_NAME}/map" \
  "/${ROBOT_NAME}/map_updates" \
  "/${ROBOT_NAME}/global_costmap/costmap_raw" \
  "/${ROBOT_NAME}/global_costmap/costmap_updates" \
  "/${ROBOT_NAME}/plan" \
  "/${ROBOT_NAME}/cmd_vel" \
  "/${ROBOT_NAME}/cmd_vel_nav" \
  "/${ROBOT_NAME}/odom" \
  "/${ROBOT_NAME}/tf" \
  "/${ROBOT_NAME}/tf_static" \
  "/${ROBOT_NAME}/navigate_to_pose/_action/status" \
  "/${ROBOT_NAME}/navigate_to_pose/_action/feedback" \
  "/${ROBOT_NAME}/navigate_to_pose/_action/result" \
  "/${ROBOT_NAME}/goal_pose" \
  --include-hidden-topics
