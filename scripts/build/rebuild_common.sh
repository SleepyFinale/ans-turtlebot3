#!/usr/bin/env bash
# Unified rebuild entrypoint for turtlebot3 workspace.
# Usage:
#   ./scripts/build/rebuild_common.sh minimal
#   ./scripts/build/rebuild_common.sh clean
#   ./scripts/build/rebuild_common.sh clean --no-clean
#   ./scripts/build/rebuild_common.sh clean --source

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

MODE="${1:-}"
if [[ "${MODE}" != "minimal" && "${MODE}" != "clean" ]]; then
  echo "Usage: $0 <minimal|clean> [--no-clean] [--source]"
  exit 1
fi
shift

NO_CLEAN=false
SOURCE_ONLY=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-clean)
      NO_CLEAN=true
      ;;
    --source)
      SOURCE_ONLY=true
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 <minimal|clean> [--no-clean] [--source]"
      exit 1
      ;;
  esac
  shift
done

if [[ "${MODE}" == "minimal" && "${NO_CLEAN}" == "true" ]]; then
  echo "INFO: --no-clean is ignored in minimal mode."
fi

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found."
    exit 1
  fi
fi

cd "${WS_DIR}"
if [[ ! -d src ]]; then
  echo "ERROR: expected workspace root with src/ at ${WS_DIR}"
  exit 1
fi

if [[ "${SOURCE_ONLY}" == "true" ]]; then
  echo "Skipping clean/build (--source)."
else
  if [[ "${MODE}" == "clean" && "${NO_CLEAN}" != "true" ]]; then
    echo "Cleaning workspace artifacts (build/, install/, log/)..."
    rm -rf build install log
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi

  PARALLEL_JOBS="${COLCON_PARALLEL_JOBS:-1}"
  if [[ "${MODE}" == "minimal" ]]; then
    MINIMAL_PKGS=(
      turtlebot3_description
      turtlebot3_node
      turtlebot3_bringup
      turtlebot3_navigation2
      ld08_driver
      coin_d4_driver
    )

    for pkg in "${MINIMAL_PKGS[@]}"; do
      cache="${WS_DIR}/build/${pkg}/CMakeCache.txt"
      if [[ -f "${cache}" ]]; then
        home_dir="$(rg -n '^CMAKE_HOME_DIRECTORY:INTERNAL=' "${cache}" | sed -n '1s/^[0-9]*:CMAKE_HOME_DIRECTORY:INTERNAL=//p')"
        if [[ -n "${home_dir}" && "${home_dir}" != "${WS_DIR}/"* ]]; then
          echo "Removing stale build/${pkg} (CMAKE_HOME_DIRECTORY was ${home_dir})"
          rm -rf "${WS_DIR}/build/${pkg}"
        fi
      fi
    done

    echo "Running minimal rebuild (workers=${PARALLEL_JOBS})..."
    colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}" \
      --packages-select \
      "${MINIMAL_PKGS[@]}" \
      --allow-overriding \
      turtlebot3_description \
      turtlebot3_node \
      turtlebot3_bringup \
      turtlebot3_navigation2
  else
    echo "Running clean/full rebuild (workers=${PARALLEL_JOBS})..."
    colcon build --symlink-install --parallel-workers "${PARALLEL_JOBS}"
  fi
fi

export TURTLEBOT3_WS="${WS_DIR}"
export WS_DIR="${WS_DIR}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL}"

echo "Loading ROS 2 + workspace environment via scripts/env/ros_robot_env.bash..."
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env/ros_robot_env.bash"

echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "Done."
