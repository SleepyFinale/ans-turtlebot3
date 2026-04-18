################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY behavior_trees launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/map_wire_compressed_republisher.py
    scripts/normalize_laser_scan.py
    scripts/nav2_motion_debug_logger.py
    scripts/nav2_lethal_watch.py
    scripts/standalone_world_map_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
