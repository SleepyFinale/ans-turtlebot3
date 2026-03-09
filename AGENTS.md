# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is a ROS 2 Humble colcon workspace for a fleet of TurtleBot3 Burger robots. Since the Cloud VM runs Ubuntu 24.04, the development environment uses Docker to provide the required Ubuntu 22.04 + ROS 2 Humble stack.

### Docker-based Development

The VM's host OS is Ubuntu 24.04 (which ships ROS 2 Jazzy), but this codebase targets **ROS 2 Humble on Ubuntu 22.04**. All build, test, and run commands must be executed inside the `turtlebot3-humble-dev` Docker container.

**Starting Docker (if not already running):**
```bash
sudo dockerd &>/tmp/dockerd.log &
sleep 3
```

**Building the workspace:**
```bash
sudo docker run --rm \
  -v /workspace/src:/root/turtlebot3_ws/src \
  -v /workspace/scripts:/root/turtlebot3_ws/scripts \
  turtlebot3-humble-dev \
  bash -c "source /opt/ros/humble/setup.bash && cd /root/turtlebot3_ws && colcon build --symlink-install --parallel-workers \$(nproc)"
```

**Running an interactive container (for testing, launching nodes, etc.):**
```bash
sudo docker run -it --rm --network host \
  -e TURTLEBOT3_MODEL=burger -e LDS_MODEL=LDS-02 -e ROS_DOMAIN_ID=30 \
  -v /workspace/src:/root/turtlebot3_ws/src \
  -v /workspace/scripts:/root/turtlebot3_ws/scripts \
  turtlebot3-humble-dev \
  bash
# Then inside the container:
# source /opt/ros/humble/setup.bash && cd /root/turtlebot3_ws && colcon build --symlink-install && source install/setup.bash
```

**Running colcon test:**
```bash
sudo docker run --rm \
  -v /workspace/src:/root/turtlebot3_ws/src \
  -v /workspace/scripts:/root/turtlebot3_ws/scripts \
  turtlebot3-humble-dev \
  bash -c "source /opt/ros/humble/setup.bash && cd /root/turtlebot3_ws && colcon build --symlink-install && colcon test && colcon test-result --verbose"
```

### Key Gotchas

- **No physical hardware**: The Cloud VM has no TurtleBot3 robot, OpenCR, or LiDAR. Nodes like `turtlebot3_ros` and lidar drivers will fail at runtime if they try to open serial ports (`/dev/ttyACM0`, `/dev/ttyUSB0`). Use `robot_state_publisher` for URDF/TF verification.
- **`ros2 topic pub --once` blocks** if no subscriber exists for the target topic. Use `--times N` instead, or skip the command if no subscriber node is running.
- **`turtlebot3_state_publisher.launch.py`** requires a `namespace` launch argument when called standalone (it's normally invoked through `robot.launch.py` which declares the default). Pass `namespace:=myns` or launch via `robot.launch.py` instead.
- **Build artifacts are ephemeral** in `--rm` containers. For interactive testing, use a named container (`--name tb3-dev`) or build + test in a single `bash -c` invocation.
- **Rebuild scripts** (`scripts/clean_rebuild.sh`, `scripts/minimal_rebuild.sh`) reference `~` as `~/turtlebot3_ws`. Inside Docker, the workspace is at `/root/turtlebot3_ws`, which matches.
- The packages have no formal ament_lint or test suites registered; `colcon test` completes with 0 tests. Lint checks are limited to `python3 -m py_compile` and `bash -n` syntax checks.
- The `Dockerfile.dev` at the repo root is for Cloud Agent use; it is a simplified version of `src/turtlebot3/docker/humble/Dockerfile` without the libcamera/camera-ros steps (not needed on x86_64 Cloud VMs).
