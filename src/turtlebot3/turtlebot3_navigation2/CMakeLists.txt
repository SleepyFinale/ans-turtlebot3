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
    scripts/map_throttle_relay.py
    scripts/scan_costmap_relay.py
    scripts/startup_map_seeder.py
    scripts/normalize_laser_scan.py
    scripts/ultrasonic_triangulation_blob.py
    scripts/ultrasonic_debug_logger.py
    scripts/nav2_motion_debug_logger.py
    scripts/nav2_lethal_watch.py
    scripts/nav2_retrace_escape.py
    scripts/nav2_controller_collision_watch.py
    scripts/ultrasonic_cmd_vel_enforcer.py
    scripts/standalone_world_map_tf.py
    scripts/namespace_tf_to_global_tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
