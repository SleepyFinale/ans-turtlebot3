#!/usr/bin/env python3
#
# Thin wrapper: delegates to navigation2_slam.launch.py with robot_name.
#
# Kept for backward compatibility.  Prefer calling navigation2_slam.launch.py
# directly with robot_name:=<name>.
#
# Usage:
#   ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py \
#       robot_name:=blinky use_sim_time:=false use_rviz:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    default_robot_name = os.environ.get('USER', '')

    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')

    effective_namespace = PythonExpression(
        ['"', namespace, '" if "', namespace, '" != "" else "', robot_name, '"']
    )

    nav2_slam_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch', 'navigation2_slam.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value=default_robot_name,
            description='Robot name used as namespace (e.g. blinky, pinky)'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Explicit namespace (overrides robot_name if set)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 if true'),
        DeclareLaunchArgument(
            'wait_for_tf', default_value='true',
            description='Wait for TF before starting Nav2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_slam_launch),
            launch_arguments={
                'robot_name': effective_namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'wait_for_tf': LaunchConfiguration('wait_for_tf'),
            }.items(),
        ),
    ])
