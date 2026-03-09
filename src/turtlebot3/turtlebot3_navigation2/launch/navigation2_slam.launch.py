#!/usr/bin/env python3
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Namespacing-aware SLAM + Nav2 launch for TurtleBot3.
#
# Single-robot (no namespace):
#   ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
#       use_sim_time:=False use_rviz:=False
#
# Multi-robot (with namespace):
#   ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
#       robot_name:=blinky use_sim_time:=False use_rviz:=False

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def _yaml_replace(content, key, old_val, new_val):
    """Replace 'key: old_val' with 'key: new_val' in YAML text (preserves formatting)."""
    pattern = r'(\b' + re.escape(key) + r':\s*)' + re.escape(old_val) + r'(\s*(?:#.*)?$)'
    return re.sub(pattern, r'\g<1>' + new_val + r'\2', content, flags=re.MULTILINE)


def _generate_nav2_params(source_file, namespace):
    """Generate a modified Nav2 params file with namespace-prefixed frame names.

    Uses text-based replacement to preserve the exact YAML formatting that
    Nav2 expects (yaml.dump can alter list styles, quoting, and key order
    which breaks DWB critic loading).
    """
    with open(source_file) as f:
        content = f.read()

    if namespace:
        p = namespace
        content = _yaml_replace(content, 'robot_base_frame', 'base_footprint',
                                f'{p}/base_footprint')
        content = _yaml_replace(content, 'base_frame_id', '"base_footprint"',
                                f'"{p}/base_footprint"')
        content = _yaml_replace(content, 'base_frame', 'base_footprint',
                                f'{p}/base_footprint')
        content = _yaml_replace(content, 'global_frame', 'odom', f'{p}/odom')
        content = _yaml_replace(content, 'odom_frame_id', '"odom"', f'"{p}/odom"')
        content = _yaml_replace(content, 'odom_frame', 'odom', f'{p}/odom')
        content = _yaml_replace(content, 'odom_topic', '/odom', 'odom')
        content = _yaml_replace(content, 'odom_topic', 'odom', 'odom')

    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='nav2_params_')
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


def _launch_setup(context):
    """OpaqueFunction that resolves substitutions and builds the launch actions."""
    ns = LaunchConfiguration('effective_namespace').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    wait_for_tf_str = LaunchConfiguration('wait_for_tf').perform(context)

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    slam_params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'humble', 'mapper_params_online_async_fast.yaml')

    workspace_dir = os.path.expanduser(
        os.environ.get('TURTLEBOT3_WS', '~/turtlebot3_ws'))
    wait_tf_script = os.path.join(workspace_dir, 'scripts', 'wait_for_tf.py')

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz', 'tb3_navigation2.rviz')

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    actions = []

    # --- Laser scan normalizer ---
    normalizer_params = {
        'input_topic': 'scan',
        'output_topic': 'scan_normalized',
    }
    if ns:
        normalizer_params['frame_id_prefix'] = ns

    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='normalize_laser_scan.py',
        name='laser_scan_normalizer',
        namespace=ns if ns else None,
        parameters=[normalizer_params],
        remappings=tf_remappings,
        output='screen',
    ))

    # --- SLAM Toolbox ---
    slam_overrides = {
        'use_sim_time': use_sim_time_str.lower() == 'true',
    }
    if ns:
        slam_overrides['odom_frame'] = f'{ns}/odom'
        slam_overrides['base_frame'] = f'{ns}/base_footprint'
        slam_overrides['map_frame'] = 'map'

    actions.append(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=ns if ns else None,
        output='screen',
        parameters=[slam_params, slam_overrides],
        remappings=tf_remappings + [('/scan', 'scan_normalized')],
    ))

    # --- Wait for TF ---
    wait_tf_env = dict(os.environ)
    wait_tf_env['TF_WAIT_ODOM_ONLY'] = 'true'
    if ns:
        wait_tf_env['TF_WAIT_ODOM_FRAME'] = f'{ns}/odom'
        wait_tf_env['TF_WAIT_BASE_FRAMES'] = f'{ns}/base_footprint,{ns}/base_link'
        wait_tf_env['TF_WAIT_NAMESPACE'] = ns

    if wait_for_tf_str.lower() == 'true':
        actions.append(ExecuteProcess(
            cmd=['python3', wait_tf_script],
            output='screen',
            env=wait_tf_env,
        ))

    # --- Nav2 with frame-rewritten params ---
    nav2_params_file = _generate_nav2_params(params_file, ns)

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_launch_file_dir, '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items(),
    )

    if ns:
        nav2_group = GroupAction([
            PushRosNamespace(ns),
            nav2_include,
        ])
    else:
        nav2_group = nav2_include

    if wait_for_tf_str.lower() == 'true':
        actions.append(TimerAction(period=3.0, actions=[nav2_group]))
    else:
        actions.append(nav2_group)

    # --- RViz (optional) ---
    if use_rviz_str.lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time_str.lower() == 'true'}],
            output='screen',
        ))

    return actions


def generate_launch_description():
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        default_param_file = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param', ROS_DISTRO, param_file_name)
    else:
        default_param_file = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param', param_file_name)

    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')

    effective_namespace = PythonExpression(
        ['"', namespace, '" if "', namespace, '" != "" else "', robot_name, '"']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='',
            description='Robot name used as namespace (e.g. blinky, pinky)'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Explicit namespace (overrides robot_name if set)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_param_file,
            description='Full path to Nav2 params YAML file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 if true'),
        DeclareLaunchArgument(
            'wait_for_tf', default_value='true',
            description='Wait for TF tree before starting Nav2'),
        DeclareLaunchArgument(
            'effective_namespace', default_value=effective_namespace,
            description='(internal) resolved namespace'),
        OpaqueFunction(function=_launch_setup),
    ])
