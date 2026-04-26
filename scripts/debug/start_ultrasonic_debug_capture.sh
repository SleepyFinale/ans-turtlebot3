#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

ROBOT_NAME="${1:-${ROBOT_NAME:-${USER:-robot}}}"
SESSION_TS="$(date +%Y%m%d-%H%M%S)"
BASE_OUT="${WORKSPACE_DIR}/logs/${ROBOT_NAME}"
JSONL_HINT="${BASE_OUT}/ultrasonic-session-*.jsonl"

mkdir -p "$BASE_OUT"

echo "=========================================="
echo " Ultrasonic Debug Capture"
echo "=========================================="
echo " Robot name   : ${ROBOT_NAME}"
echo " Workspace    : ${WORKSPACE_DIR}"
echo " JSONL output : ${JSONL_HINT}"
echo ""
echo "Start this after bringup + navigation launch."
echo "Press Ctrl+C to stop capture."
echo "=========================================="
echo ""

run_logger() {
  local exe_name="$1"
  ros2 run turtlebot3_navigation2 "$exe_name" \
    --ros-args \
    -r __ns:="/${ROBOT_NAME}" \
    -p robot_name:="${ROBOT_NAME}" \
    -p output_dir:="${WORKSPACE_DIR}/logs" \
    -p log_rate_hz:=5.0
}

find_logger_executable() {
  local executables
  if ! executables="$(ros2 pkg executables turtlebot3_navigation2 2>/dev/null)"; then
    return 1
  fi
  if printf '%s\n' "$executables" | rg -q "turtlebot3_navigation2 ultrasonic_debug_logger.py"; then
    echo "ultrasonic_debug_logger.py"
    return 0
  fi
  if printf '%s\n' "$executables" | rg -q "turtlebot3_navigation2 ultrasonic_debug_logger$"; then
    echo "ultrasonic_debug_logger"
    return 0
  fi
  return 1
}

LOGGER_EXE="$(find_logger_executable || true)"
if [[ -n "$LOGGER_EXE" ]]; then
  run_logger "$LOGGER_EXE"
  exit $?
fi

LOGGER_SRC="${WORKSPACE_DIR}/src/turtlebot3/turtlebot3_navigation2/scripts/ultrasonic_debug_logger.py"
if [[ -f "$LOGGER_SRC" ]]; then
  echo "[fallback] ros2 run executable not found; running source script directly: $LOGGER_SRC"
  exec python3 "$LOGGER_SRC" \
    --ros-args \
    -r __ns:="/${ROBOT_NAME}" \
    -p robot_name:="${ROBOT_NAME}" \
    -p output_dir:="${WORKSPACE_DIR}/logs" \
    -p log_rate_hz:=5.0
fi

echo "ERROR: Could not launch ultrasonic_debug_logger via ros2 run or source fallback."
echo "Try rebuilding and sourcing:"
echo "  cd ${WORKSPACE_DIR} && ./scripts/clean_rebuild.sh"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
exit 1
