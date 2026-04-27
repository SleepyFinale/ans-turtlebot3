#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This file lives in scripts/debug — workspace root is two levels up (colcon install/).
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

ROBOT_NAME="${1:-${ROBOT_NAME:-${USER:-robot}}}"
SESSION_TS="$(date +%Y%m%d-%H%M%S)"
BASE_OUT="${WORKSPACE_DIR}/logs/${ROBOT_NAME}"
BAG_OUT="${BASE_OUT}/bag-${SESSION_TS}"
DB3_OUT="${BAG_OUT}/bag-${SESSION_TS}_0.db3"
JSONL_OUT="${BASE_OUT}/session-${SESSION_TS}.jsonl"
DB3_REL="turtlebot3/logs/${ROBOT_NAME}/bag-${SESSION_TS}/bag-${SESSION_TS}_0.db3"
JSONL_REL="turtlebot3/logs/${ROBOT_NAME}/session-${SESSION_TS}.jsonl"

mkdir -p "$BASE_OUT"

# If debug logger already started and created a nearby session file with a
# different timestamp, rename it so bag + JSONL share the same SESSION_TS.
if [[ ! -f "$JSONL_OUT" ]]; then
  LATEST_JSONL="$(ls -1t "${BASE_OUT}"/session-*.jsonl 2>/dev/null | sed -n '1p' || true)"
  if [[ -n "$LATEST_JSONL" && "$LATEST_JSONL" != "$JSONL_OUT" ]]; then
    mv "$LATEST_JSONL" "$JSONL_OUT"
  fi
fi

ANALYZE_CMD="python3 ${WORKSPACE_DIR}/scripts/debug/analyze_nav2_bag_stop.py \"\$(dirname \"${DB3_OUT}\")\" && python3 ${WORKSPACE_DIR}/scripts/debug/analyze_nav2_debug_session.py \"${JSONL_OUT}\""

echo "=========================================="
echo " Nav2 Debug Capture"
echo "=========================================="
echo " Robot name   : ${ROBOT_NAME}"
echo " Bag DB3 path : ${DB3_OUT}"
echo " JSONL path   : ${JSONL_OUT}"
echo " Bag DB3(repo): ${DB3_REL}"
echo " JSONL (repo) : ${JSONL_REL}"
echo ""
echo " Analyze command (copy/paste):"
echo " ${ANALYZE_CMD}"
echo ""
echo "Make sure navigation2_slam.launch.py is started with:"
echo "  enable_debug_logging:=true"
echo "  debug_log_dir:=${WORKSPACE_DIR}/logs"
echo ""
echo "Press Ctrl+C to stop recording."
echo "=========================================="
echo ""

# Namespaced topics: robot SLAM + Nav2. Fleet mode also publishes merged map + full TF on
# global /map and /tf (see navigation_launch_multirobot.py); record those too or bags
# replay without the world→robot chain.
# Omit global_costmap/costmap: some stacks advertise multiple types on that name and rosbag2
# rejects the topic; costmap_raw + costmap_updates are enough for postmortems.
ros2 bag record -o "$BAG_OUT" \
  "/tf" \
  "/tf_static" \
  "/map" \
  "/map_updates" \
  "/clock" \
  "/${ROBOT_NAME}/map" \
  "/${ROBOT_NAME}/map_updates" \
  "/${ROBOT_NAME}/local_costmap/costmap" \
  "/${ROBOT_NAME}/local_costmap/costmap_raw" \
  "/${ROBOT_NAME}/local_costmap/costmap_updates" \
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
  "/rosout" \
  --include-hidden-topics
