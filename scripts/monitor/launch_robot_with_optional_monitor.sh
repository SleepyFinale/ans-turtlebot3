#!/usr/bin/env bash
set -euo pipefail

WITH_MONITOR=0
MONITOR_IFACE="wlan0"
MONITOR_INTERVAL="1.0"
MONITOR_OUTPUT=""
MONITOR_LINK_MBPS=""
ROS_LOG_OUTPUT=""
LAUNCH_PACKAGE="turtlebot3_bringup"
LAUNCH_FILE="robot.launch.py"

usage() {
  cat <<'EOF'
Usage:
  launch_robot_with_optional_monitor.sh [options] [-- <ros2 launch args...>]

Options:
  --with-monitor           Enable bottleneck monitoring while robot.launch.py runs
  --iface <name>           Monitor interface (default: wlan0)
  --interval <seconds>     Monitor sample interval (default: 1.0)
  --output <path>          Monitor CSV output path
  --link-mbps <value>      Monitor link speed override (Mbps)
  --ros-log-output <path>  Save ros2 launch console output to this file
  --launch-package <pkg>   ros2 launch package (default: turtlebot3_bringup)
  --launch-file <file>     ros2 launch file (default: robot.launch.py)
  --help                   Show this help

Examples:
  # Normal bringup (no monitor)
  ./scripts/monitor/launch_robot_with_optional_monitor.sh

  # Bringup with monitor
  ./scripts/monitor/launch_robot_with_optional_monitor.sh --with-monitor

  # Bringup with monitor and custom launch args
  ./scripts/monitor/launch_robot_with_optional_monitor.sh --with-monitor -- robot_name:=pinky

  # Nav2 SLAM with monitor + timing log
  ./scripts/monitor/launch_robot_with_optional_monitor.sh --with-monitor --launch-package turtlebot3_navigation2 --launch-file navigation2_slam.launch.py
EOF
}

LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --with-monitor)
      WITH_MONITOR=1
      shift
      ;;
    --iface)
      MONITOR_IFACE="${2:-}"
      shift 2
      ;;
    --interval)
      MONITOR_INTERVAL="${2:-}"
      shift 2
      ;;
    --output)
      MONITOR_OUTPUT="${2:-}"
      shift 2
      ;;
    --link-mbps)
      MONITOR_LINK_MBPS="${2:-}"
      shift 2
      ;;
    --ros-log-output)
      ROS_LOG_OUTPUT="${2:-}"
      shift 2
      ;;
    --launch-package)
      LAUNCH_PACKAGE="${2:-}"
      shift 2
      ;;
    --launch-file)
      LAUNCH_FILE="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      while [ $# -gt 0 ]; do
        LAUNCH_ARGS+=("$1")
        shift
      done
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

MONITOR_SCRIPT="./scripts/monitor/pi_bottleneck_monitor.sh"
if [ "$WITH_MONITOR" -eq 1 ] && [ ! -x "$MONITOR_SCRIPT" ]; then
  echo "Monitor script not found or not executable: $MONITOR_SCRIPT" >&2
  exit 1
fi

MONITOR_PID=""
cleanup() {
  if [ -n "$MONITOR_PID" ] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    echo "Stopping bottleneck monitor (pid: $MONITOR_PID)..."
    kill -TERM "$MONITOR_PID" 2>/dev/null || true

    # Give the monitor a short grace period to flush and exit.
    for _ in 1 2 3 4 5; do
      if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
        break
      fi
      sleep 0.2
    done

    if kill -0 "$MONITOR_PID" 2>/dev/null; then
      echo "Monitor did not exit on TERM; forcing stop..."
      kill -KILL "$MONITOR_PID" 2>/dev/null || true
    fi

    wait "$MONITOR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "$WITH_MONITOR" -eq 1 ]; then
  mkdir -p "scripts/monitor/logs"
  if [ -z "$MONITOR_OUTPUT" ] && [ -z "$ROS_LOG_OUTPUT" ]; then
    _log_stamp="$(date +%Y%m%d_%H%M%S)"
    MONITOR_OUTPUT="scripts/monitor/logs/pi_bottleneck_${_log_stamp}.csv"
    ROS_LOG_OUTPUT="scripts/monitor/logs/ros_launch_${_log_stamp}.log"
  else
    if [ -z "$MONITOR_OUTPUT" ]; then
      MONITOR_OUTPUT="scripts/monitor/logs/pi_bottleneck_$(date +%Y%m%d_%H%M%S).csv"
    fi
    if [ -z "$ROS_LOG_OUTPUT" ]; then
      ROS_LOG_OUTPUT="scripts/monitor/logs/ros_launch_$(date +%Y%m%d_%H%M%S).log"
    fi
  fi

  MONITOR_CMD=("$MONITOR_SCRIPT" "--iface" "$MONITOR_IFACE" "--interval" "$MONITOR_INTERVAL")
  MONITOR_CMD+=("--output" "$MONITOR_OUTPUT")
  if [ -n "$MONITOR_LINK_MBPS" ]; then
    MONITOR_CMD+=("--link-mbps" "$MONITOR_LINK_MBPS")
  fi

  echo "Starting bottleneck monitor..."
  "${MONITOR_CMD[@]}" &
  MONITOR_PID=$!
  sleep 0.2
  if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
    echo "Failed to start monitor." >&2
    exit 1
  fi
fi

echo "Running: ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE ${LAUNCH_ARGS[*]:-}"
LAUNCH_EXIT_CODE=0

if [ "$WITH_MONITOR" -eq 1 ]; then
  mkdir -p "$(dirname "$ROS_LOG_OUTPUT")"
  echo "Saving ros2 launch output to: $(basename "$ROS_LOG_OUTPUT")"

  set +e
  ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" 2>&1 | tee "$ROS_LOG_OUTPUT"
  LAUNCH_EXIT_CODE=${PIPESTATUS[0]}
  set -e
else
  set +e
  ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"
  LAUNCH_EXIT_CODE=$?
  set -e
fi

if [ "$WITH_MONITOR" -eq 1 ]; then
  echo "Analyze with:"
  echo "  ./scripts/monitor/analyze_bottleneck_log.py $(basename "$MONITOR_OUTPUT") --ros-log $(basename "$ROS_LOG_OUTPUT")"
fi

exit "$LAUNCH_EXIT_CODE"
