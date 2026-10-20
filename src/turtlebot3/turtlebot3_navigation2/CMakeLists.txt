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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

################################################################################
# Build
################################################################################
add_executable(stereo_pair_throttle src/stereo_pair_throttle.cpp)
ament_target_dependencies(stereo_pair_throttle rclcpp sensor_msgs)

################################################################################
# Install
################################################################################
install(
  TARGETS stereo_pair_throttle
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY behavior_trees launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/map_wire_compressed_republisher.py
    scripts/startup_map_seeder.py
    scripts/normalize_laser_scan.py
    scripts/ultrasonic_triangulation_blob.py
    scripts/ultrasonic_debug_logger.py
    scripts/nav2_motion_debug_logger.py
    scripts/nav2_lethal_watch.py
    scripts/nav2_retrace_escape.py
    scripts/nav2_controller_collision_watch.py
    scripts/ultrasonic_cmd_vel_enforcer.py
    scripts/orca_shadow_advisor.py
    scripts/standalone_world_map_tf.py
    scripts/stereo_common.py
    scripts/stereo_calibrate.py
    scripts/stereo_refine_extrinsics.py
    scripts/stereo_fit_floor.py
    scripts/stereo_fit_offset.py
    scripts/stereo_depth_node.py
    scripts/stereo_cmd_vel_enforcer.py
    scripts/stereo_visual_odometry.py
    scripts/stuck_detector.py
    scripts/stuck_snapshot.py
    scripts/low_obstacle_memory.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
