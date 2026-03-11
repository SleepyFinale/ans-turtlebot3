#!/usr/bin/env bash
# Rebuild only the workspace packages needed for:
#   - ros2 launch turtlebot3_bringup robot.launch.py
#   - ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False use_rviz:=False
# This targets only TurtleBot3 + lidar driver packages for speed (no clean step).
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

# Parallel workers: use COLCON_PARALLEL_JOBS if set, else 1 (override with COLCON_PARALLEL_JOBS=N for more).
PARALLEL_JOBS="${COLCON_PARALLEL_JOBS:-1}"

echo "Building minimal set for TurtleBot3 bringup + Navigation2 SLAM..."
echo "Packages: turtlebot3_description, turtlebot3_node, turtlebot3_bringup, turtlebot3_navigation2, ld08_driver, coin_d4_driver"
colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}" \
  --packages-select \
    turtlebot3_description \
    turtlebot3_node \
    turtlebot3_bringup \
    turtlebot3_navigation2 \
    ld08_driver \
    coin_d4_driver \
  --allow-overriding \
    turtlebot3_description \
    turtlebot3_node \
    turtlebot3_bringup \
    turtlebot3_navigation2

export TURTLEBOT3_WS="${WS_DIR}"
export WS_DIR="${WS_DIR}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL}"

echo "Loading ROS 2 + workspace environment via scripts/ros_robot_env.bash..."
source "${SCRIPT_DIR}/ros_robot_env.bash"

echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "Done. You can now run:"
echo "  ros2 launch turtlebot3_bringup robot.launch.py"
echo "  ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False use_rviz:=False"
echo "Use 'source ${WS_DIR}/install/setup.bash' (or scripts/ros_robot_env.bash) in new shells."
