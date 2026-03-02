# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is a ROS 2 colcon workspace for a TurtleBot3 Burger robot fleet (robots: Blinky, Pinky, Inky, Clyde). It contains 8 ROS 2 packages: `turtlebot3_node`, `turtlebot3_bringup`, `turtlebot3_description`, `turtlebot3_teleop`, `turtlebot3_example`, `ld08_driver`, `coin_d4_driver`, and the meta-package `turtlebot3`.

See the root `README.md` for full setup/build instructions and fleet reference.

### Build environment (Ubuntu 24.04 / Jazzy)

The Cloud VM runs Ubuntu 24.04 (Noble). The workspace is designed for ROS 2 Humble (Ubuntu 22.04), but builds and runs correctly under **ROS 2 Jazzy** on Noble. The update script installs Jazzy and builds with Jazzy.

**Compiler gotcha**: The default C++ compiler on this VM is Clang 18, which cannot find GCC's `libstdc++` headers. You **must** set `CC=gcc CXX=g++` before running `colcon build`, or the build will fail with `fatal error: 'memory' file not found`. The `libstdc++.so` symlink at `/usr/lib/x86_64-linux-gnu/libstdc++.so` must also exist (the update script creates it).

### Key commands

```bash
# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=30

# Build (always use gcc)
export CC=gcc CXX=g++
colcon build --symlink-install --parallel-workers $(nproc)

# Quick rebuild (only robot.launch.py deps)
./scripts/minimal_rebuild.sh

# Full clean rebuild
./scripts/clean_rebuild.sh

# Run tests
colcon test && colcon test-result --verbose

# Launch state publisher (no hardware needed)
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=true namespace:=tb3
```

### Hardware-dependent nodes

`ros2 launch turtlebot3_bringup robot.launch.py` requires physical USB connections to the OpenCR motor controller (`/dev/ttyACM0`) and LiDAR sensor (`/dev/ttyUSB0`). It will not run in the Cloud VM. Use `turtlebot3_state_publisher.launch.py` to test the workspace without hardware.

### No linting or automated tests

This workspace does not configure linting tools (no `ament_lint`, `flake8`, `ruff`, etc.) and has 0 automated tests (`colcon test` passes trivially). Testing is done against physical hardware.
