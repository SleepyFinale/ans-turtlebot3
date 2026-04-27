#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/local_setup.bash 2>/dev/null || source install/setup.bash 2>/dev/null || true

ROBOT_NAME="${1:-${ROBOT_NAME:-${USER:-robot}}}"
SESSION_TS="$(date +%Y%m%d-%H%M%S)"
BASE_OUT="${WORKSPACE_DIR}/logs/${ROBOT_NAME}"
JSONL_OUT="${BASE_OUT}/ultrasonic-session-${SESSION_TS}.jsonl"
JSONL_REL="turtlebot3/logs/${ROBOT_NAME}/ultrasonic-session-${SESSION_TS}.jsonl"

mkdir -p "$BASE_OUT"

# If logger already created a nearby ultrasonic-session file with a different
# timestamp, rename it so capture timestamp and JSONL timestamp match.
if [[ ! -f "$JSONL_OUT" ]]; then
  LATEST_JSONL="$(ls -1t "${BASE_OUT}"/ultrasonic-session-*.jsonl 2>/dev/null | sed -n '1p' || true)"
  if [[ -n "$LATEST_JSONL" && "$LATEST_JSONL" != "$JSONL_OUT" ]]; then
    mv "$LATEST_JSONL" "$JSONL_OUT"
  fi
fi

ANALYZE_CMD="python3 ${WORKSPACE_DIR}/scripts/debug/analyze_ultrasonic_debug_session.py \"${JSONL_OUT}\""

echo "=========================================="
echo " Ultrasonic Debug Capture"
echo "=========================================="
echo " Robot name   : ${ROBOT_NAME}"
echo " JSONL path   : ${JSONL_OUT}"
echo " JSONL (repo) : ${JSONL_REL}"
echo ""
echo " Analyze command (copy/paste):"
echo " ${ANALYZE_CMD}"
echo ""
echo "Start this after bringup + navigation launch."
echo "Press Ctrl+C to stop capture."
echo "=========================================="
echo ""

run_logger() {
  local exe_name="$1"
  DEBUG_SESSION_TS="${SESSION_TS}" ros2 run turtlebot3_navigation2 "$exe_name" \
    --ros-args \
    -r __ns:="/${ROBOT_NAME}" \
    -p robot_name:="${ROBOT_NAME}" \
    -p output_dir:="${WORKSPACE_DIR}/logs" \
    -p log_rate_hz:=5.0
}

run_installed_logger() {
  local exe_path="$1"
  exec env DEBUG_SESSION_TS="${SESSION_TS}" "$exe_path" \
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

# Prefer workspace-local installed executable directly. This avoids wrong overlay
# selection when ros2 pkg resolves turtlebot3_navigation2 from another workspace.
INSTALLED_LOGGER_EXE="${WORKSPACE_DIR}/install/turtlebot3_navigation2/lib/turtlebot3_navigation2/ultrasonic_debug_logger.py"
if [[ -f "$INSTALLED_LOGGER_EXE" ]]; then
  chmod +x "$INSTALLED_LOGGER_EXE" 2>/dev/null || true
  echo "[info] Using workspace-installed logger: $INSTALLED_LOGGER_EXE"
  run_installed_logger "$INSTALLED_LOGGER_EXE"
fi

LOGGER_EXE="$(find_logger_executable || true)"
if [[ -z "$LOGGER_EXE" ]]; then
  echo "[info] ultrasonic_debug_logger not found in current overlay."
  echo "[info] Rebuilding turtlebot3_navigation2 so ros2 run can use installed executable..."
  if colcon build --packages-select turtlebot3_navigation2 --symlink-install --allow-overriding turtlebot3_navigation2; then
    source "${WORKSPACE_DIR}/install/local_setup.bash" 2>/dev/null || source "${WORKSPACE_DIR}/install/setup.bash"
    if [[ -f "$INSTALLED_LOGGER_EXE" ]]; then
      chmod +x "$INSTALLED_LOGGER_EXE" 2>/dev/null || true
      echo "[info] Using workspace-installed logger after rebuild: $INSTALLED_LOGGER_EXE"
      run_installed_logger "$INSTALLED_LOGGER_EXE"
    fi
    LOGGER_EXE="$(find_logger_executable || true)"
  fi
fi

if [[ -n "$LOGGER_EXE" ]]; then
  run_logger "$LOGGER_EXE"
  exit $?
fi

LOGGER_SRC="${WORKSPACE_DIR}/src/turtlebot3/turtlebot3_navigation2/scripts/ultrasonic_debug_logger.py"
if [[ -f "$LOGGER_SRC" ]]; then
  echo "[fallback] ros2 run executable still not found after rebuild; running source script directly: $LOGGER_SRC"
  exec env DEBUG_SESSION_TS="${SESSION_TS}" python3 "$LOGGER_SRC" \
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
