#!/usr/bin/env python3
#
# Wrapper launch file to start TurtleBot3 bringup with a per-robot namespace.
# This keeps the existing robot.launch.py as the single source of truth while
# adding a convenient robot_name argument for multi-robot setups.
#
# Usage examples:
#   ros2 launch turtlebot3_bringup robot_namespaced.launch.py robot_name:=blinky
#   ros2 launch turtlebot3_bringup robot_namespaced.launch.py namespace:=blinky
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Default robot name from Linux username (so you normally don't need to
    # pass robot_name/namespace on the command line).
    default_robot_name = os.environ.get("USER", "")

    # namespace: explicit namespace for the robot (takes precedence if set)
    namespace = LaunchConfiguration("namespace")
    # robot_name: logical robot identifier (e.g. blinky, pinky, inky, clyde)
    robot_name = LaunchConfiguration("robot_name")
    # robot.launch.py optionally auto-starts a legacy, non-namespaced SLAM + normalizer.
    # Default this wrapper to *not* start it (namespaced SLAM should be started separately).
    start_slam_with_normalizer = LaunchConfiguration(
        "start_slam_with_normalizer", default="false"
    )

    # Use namespace if non-empty, otherwise fall back to robot_name.
    # This keeps the interface flexible while ensuring a single effective
    # namespace string is passed down to robot.launch.py.
    effective_namespace = PythonExpression(
        ['"', namespace, '" if "', namespace, '" != "" else "', robot_name, '"']
    )

    bringup_share = get_package_share_directory("turtlebot3_bringup")
    robot_launch = os.path.join(bringup_share, "launch", "robot.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value=default_robot_name,
                description=(
                    "Logical robot name to use as the namespace when "
                    "namespace is not explicitly provided (e.g. blinky, pinky)."
                ),
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description=(
                    "Explicit ROS namespace for this robot. If non-empty, "
                    "this value takes precedence over robot_name."
                ),
            ),
            DeclareLaunchArgument(
                "start_slam_with_normalizer",
                default_value="false",
                description=(
                    "If true, robot.launch.py will run scripts/start_slam_with_normalizer.sh "
                    "(legacy single-robot, non-namespaced). Default false for multi-robot namespaces."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    "namespace": effective_namespace,
                    "start_slam_with_normalizer": start_slam_with_normalizer,
                }.items(),
            ),
        ]
    )

