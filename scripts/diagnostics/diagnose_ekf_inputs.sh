#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_name> [seconds]"
  echo "Example: $0 blinky 8"
  exit 1
fi

robot_name="$1"
sample_seconds="${2:-8}"

if ! [[ "$sample_seconds" =~ ^[0-9]+$ ]]; then
  echo "seconds must be an integer, got: $sample_seconds"
  exit 1
fi

odom_topic="/${robot_name}/odom"
imu_topic="/${robot_name}/imu"
gps_topic="/${robot_name}/odometry/gps"
did_daemon_retry=0

check_topic() {
  local topic="$1"
  if ros2 topic info "$topic" >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

restart_ros2_daemon_once() {
  if [[ "${did_daemon_retry}" -eq 1 ]]; then
    return 1
  fi
  did_daemon_retry=1
  echo "Detected ROS context/daemon issue. Restarting ros2 daemon once and retrying..."
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 1
  return 0
}

run_topic_hz() {
  local topic="$1"
  local output

  set +e
  output="$(timeout "$((sample_seconds + 2))" ros2 topic hz "$topic" -w "$sample_seconds" 2>&1)"
  local rc=$?
  set -e

  if [[ $rc -eq 0 ]]; then
    echo "${output}"
    return 0
  fi

  if [[ "${output}" == *"rcl node's context is invalid"* ]] || [[ "${output}" == *"failed to create guard condition"* ]]; then
    echo "${output}"
    if restart_ros2_daemon_once; then
      set +e
      output="$(timeout "$((sample_seconds + 2))" ros2 topic hz "$topic" -w "$sample_seconds" 2>&1)"
      rc=$?
      set -e
      echo "${output}"
      return $rc
    fi
  else
    echo "${output}"
  fi

  return $rc
}

echo "=== EKF input diagnostics for robot: ${robot_name} ==="
echo "Sampling window: ${sample_seconds}s"
echo

for topic in "$odom_topic" "$imu_topic" "$gps_topic"; do
  if check_topic "$topic"; then
    echo "--- ros2 topic hz ${topic} ---"
    run_topic_hz "$topic" || true
    echo
  else
    echo "--- ${topic} ---"
    echo "Topic not currently available (this is expected for GPS in indoor mode)."
    echo
  fi
done

echo "--- EKF diagnostics sample ---"
timeout 5 ros2 topic echo "/diagnostics" --once || true
echo
if [[ "${did_daemon_retry}" -eq 1 ]]; then
  echo "Note: ros2 daemon restart was required during this diagnostic run."
  echo "If this repeats often, open a fresh shell and re-source ROS + workspace setup."
  echo
fi
echo "Done."
