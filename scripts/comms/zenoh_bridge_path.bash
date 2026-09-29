#!/usr/bin/env bash
# Resolve path to zenoh-bridge-ros2dds (standalone install preferred over apt).
# Sourced by start_zenoh_*.sh — prints absolute path on stdout, returns 0/1.
#
# Search order:
#   1) ZENOH_BRIDGE_ROS2DDS env override
#   2) <workspace>/third_party/zenoh/zenoh-bridge-ros2dds
#   3) command -v zenoh-bridge-ros2dds
#   4) ~/.local/bin/zenoh-bridge-ros2dds

_zenoh_comms_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_zenoh_workspace="$(cd "${_zenoh_comms_dir}/../.." && pwd)"

resolve_zenoh_bridge_ros2dds() {
  local candidate
  if [[ -n "${ZENOH_BRIDGE_ROS2DDS:-}" && -x "${ZENOH_BRIDGE_ROS2DDS}" ]]; then
    echo "${ZENOH_BRIDGE_ROS2DDS}"
    return 0
  fi
  candidate="${_zenoh_workspace}/third_party/zenoh/zenoh-bridge-ros2dds"
  if [[ -x "$candidate" ]]; then
    echo "$candidate"
    return 0
  fi
  if candidate="$(command -v zenoh-bridge-ros2dds 2>/dev/null)"; then
    echo "$candidate"
    return 0
  fi
  candidate="${HOME}/.local/bin/zenoh-bridge-ros2dds"
  if [[ -x "$candidate" ]]; then
    echo "$candidate"
    return 0
  fi
  return 1
}
