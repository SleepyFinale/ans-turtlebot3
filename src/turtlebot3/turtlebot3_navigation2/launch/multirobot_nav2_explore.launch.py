# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Multi-robot Nav2 + Explorer for Blinky and Pinky
# Both robots explore autonomously on the merged map. Requires multirobot_slam running.
#
# Prerequisites: multirobot_slam.launch.py, domain bridges, tf_relay
# Usage: ROS_DOMAIN_ID=50 ros2 launch turtlebot3_navigation2 multirobot_nav2_explore.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_navigation2')
    launch_dir = os.path.join(pkg_dir, 'launch')
    param_dir = os.path.join(pkg_dir, 'param', 'humble')
    explore_config = os.path.join(
        get_package_share_directory('explore_lite'), 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'rviz', 'tb3_navigation2.rviz')

    # TF remappings: Nav2 nodes in namespace use relative "tf" (=/blinky/tf or /pinky/tf)
    # which only has robot odom. Redirect to global /tf for full tree (map->odom->base).
    # Map remapping: ensure namespaced nodes subscribe to global /map from map_merge.
    nav_remappings = [
        ('tf', '/tf'), ('tf_static', '/tf_static'),
        ('map', '/map'), ('map_updates', '/map_updates'),
    ]

    # Use RewrittenYaml (like nav2 bringup) so costmap/controller find params under namespace
    autostart = LaunchConfiguration('autostart', default='true')
    param_substitutions = {'use_sim_time': use_sim_time, 'autostart': autostart}
    blinky_params_path = os.path.join(param_dir, 'burger_multirobot_blinky.yaml')
    pinky_params_path = os.path.join(param_dir, 'burger_multirobot_pinky.yaml')
    blinky_configured = ParameterFile(
        RewrittenYaml(
            source_file=blinky_params_path,
            root_key='blinky',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    pinky_configured = ParameterFile(
        RewrittenYaml(
            source_file=pinky_params_path,
            root_key='pinky',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Blinky: container + Nav2
    blinky_group = GroupAction([
        PushRosNamespace('blinky'),
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[blinky_configured, {'use_sim_time': use_sim_time, 'autostart': autostart}],
            remappings=nav_remappings,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch_multirobot.py')),
            launch_arguments={
                'namespace': 'blinky',
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': blinky_params_path,
                'use_composition': 'True',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])

    # Pinky: container + Nav2
    pinky_group = GroupAction([
        PushRosNamespace('pinky'),
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[pinky_configured, {'use_sim_time': use_sim_time, 'autostart': autostart}],
            remappings=nav_remappings,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch_multirobot.py')),
            launch_arguments={
                'namespace': 'pinky',
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': pinky_params_path,
                'use_composition': 'True',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])

    # Explorer remappings: use global /map and /tf; each explores with its own base frame
    explore_remap_blinky = [
        ('map', '/map'),
        ('map_updates', '/map_updates'),
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
    ]
    explore_remap_pinky = [
        ('map', '/map'),
        ('map_updates', '/map_updates'),
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
    ]

    workspace_dir = os.path.expanduser('~/turtlebot3_ws')
    wait_tf_script = os.path.join(workspace_dir, 'scripts', 'wait_for_tf_multirobot.py')

    wait_for_tf = LaunchConfiguration('wait_for_tf', default='true')
    wait_process = ExecuteProcess(
        cmd=['python3', wait_tf_script],
        output='screen',
        name='wait_for_tf_multirobot',
    )

    explore_blinky = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        namespace='blinky',
        output='screen',
        parameters=[
            explore_config,
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'blinky/base_footprint'},
            {'costmap_topic': '/map'},
        ],
        remappings=explore_remap_blinky,
    )
    explore_pinky = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        namespace='pinky',
        output='screen',
        parameters=[
            explore_config,
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'pinky/base_footprint'},
            {'costmap_topic': '/map'},
        ],
        remappings=explore_remap_pinky,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    nav2_actions = [blinky_group, pinky_group, explore_blinky, explore_pinky, rviz_node]

    def on_wait_exit(event, context):
        """Only start Nav2 on success; shutdown launch if TF wait timed out."""
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason='TF wait timed out. Ensure multirobot_slam, domain bridges, '
                       'and tf_relay are running before start_multirobot_nav2_explore.'
            ))]
        return nav2_actions

    # When wait_for_tf: run wait script first; start Nav2 + Explore only after TF is ready
    wait_then_nav2 = GroupAction([
        wait_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_process,
                on_exit=on_wait_exit,
            )
        ),
    ], condition=IfCondition(wait_for_tf))

    # When wait_for_tf=false: start Nav2 + Explore immediately
    nav2_immediate = GroupAction(nav2_actions, condition=UnlessCondition(wait_for_tf))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument(
            'wait_for_tf', default_value='true',
            description='Wait for map->robot/base_footprint TF before starting Nav2'),
        wait_then_nav2,
        nav2_immediate,
    ])
