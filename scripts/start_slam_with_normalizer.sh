#!/bin/bash
# Launch SLAM Toolbox with laser scan normalizer to handle variable scan readings.
# NOTE: This script is intended for **single-robot** setups only.
# For multi-robot, namespaced operation (Blinky + Pinky + ... in one domain),
# prefer running SLAM via:
#   ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "Starting SLAM with Laser Scan Normalizer"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Start laser scan normalizer (fixes variable reading counts)"
echo "  2. Start SLAM Toolbox with fast config (0.5s map updates)"
echo ""
echo "Press Ctrl+C to stop both processes"
echo ""

# Get absolute path to fast SLAM config
SLAM_CONFIG="$(pwd)/src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml"

# Check if config exists
if [ ! -f "$SLAM_CONFIG" ]; then
    echo "ERROR: SLAM config not found at: $SLAM_CONFIG"
    exit 1
fi

# Start normalizer in background (publish_every_n_scans:=2 reduces SLAM message filter queue overflow)
echo "Starting laser scan normalizer..."
python3 src/turtlebot3/turtlebot3_navigation2/scripts/normalize_laser_scan.py --ros-args -p publish_every_n_scans:=2 &
NORMALIZER_PID=$!

# Wait a moment for normalizer to start
sleep 2

# Start SLAM Toolbox with normalized scan
echo "Starting SLAM Toolbox with fast config..."
echo "Using normalized scan topic: /scan_normalized"
echo ""

# Real robot default: do NOT use sim time (no /clock).
# Override by setting USE_SIM_TIME=1 (or true) before running this script.
USE_SIM_TIME_VALUE="False"
if [ "${USE_SIM_TIME}" = "1" ] || [ "${USE_SIM_TIME}" = "true" ] || [ "${USE_SIM_TIME}" = "True" ]; then
    USE_SIM_TIME_VALUE="True"
fi
echo "use_sim_time: ${USE_SIM_TIME_VALUE}"

ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_CONFIG" \
    scan_topic:=/scan_normalized \
    use_sim_time:="${USE_SIM_TIME_VALUE}"

# Cleanup: kill normalizer when SLAM exits
echo "Stopping laser scan normalizer..."
kill $NORMALIZER_PID 2>/dev/null
