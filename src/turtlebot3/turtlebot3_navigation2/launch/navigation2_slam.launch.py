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
#
# With central PC (start_central.sh: map_merge + tf_relay + explorer), enable
# global TF + merged map so Nav2 goals in frame `map` match the central stack:
#   use_central_tf_map:=true

import os
import tempfile

import yaml

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

DEFAULT_ROBOT_NAME = os.environ.get('USER') or os.environ.get('LOGNAME') or 'robot'


def _rewrite_frame(d, key, old_val, new_val):
    """Recursively find parameters matching key=old_val and replace with new_val."""
    for k, v in list(d.items()):
        if k == key and v == old_val:
            d[k] = new_val
        elif isinstance(v, dict):
            _rewrite_frame(v, key, old_val, new_val)


def _generate_nav2_params(source_file, namespace):
    """Generate a modified Nav2 params file with namespace-prefixed frame names."""
    with open(source_file) as f:
        params = yaml.safe_load(f)

    if namespace:
        # Rewrite common frame / topic params to be namespace-aware.
        _rewrite_frame(params, 'robot_base_frame', 'base_footprint',
                        f'{namespace}/base_footprint')
        _rewrite_frame(params, 'base_frame_id', 'base_footprint',
                        f'{namespace}/base_footprint')
        _rewrite_frame(params, 'base_frame', 'base_footprint',
                        f'{namespace}/base_footprint')
        _rewrite_frame(params, 'global_frame', 'odom', f'{namespace}/odom')
        _rewrite_frame(params, 'odom_frame_id', 'odom', f'{namespace}/odom')
        _rewrite_frame(params, 'odom_frame', 'odom', f'{namespace}/odom')
        _rewrite_frame(params, 'odom_topic', '/odom', 'odom')
        _rewrite_frame(params, 'topic', '/scan_normalized', 'scan_normalized')
        # Ensure the global costmap static layer subscribes to the
        # per-robot map topic (e.g. /pinky/map) instead of a local
        # /<namespace>/global_costmap/map topic.
        _rewrite_frame(params, 'map_topic', '/map', f'/{namespace}/map')
        _rewrite_frame(params, 'map_topic', 'map', f'/{namespace}/map')

    # Force BT navigator to use local no-backup behavior trees so disabling
    # the backup behavior plugin does not break lifecycle activation.
    bt_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'behavior_trees')
    bt_to_pose = os.path.join(
        bt_dir, 'navigate_to_pose_w_replanning_and_recovery_no_backup.xml')
    bt_through_poses = os.path.join(
        bt_dir, 'navigate_through_poses_w_replanning_and_recovery_no_backup.xml')
    if 'bt_navigator' in params and 'ros__parameters' in params['bt_navigator']:
        bt_params = params['bt_navigator']['ros__parameters']
        bt_params['default_nav_to_pose_bt_xml'] = bt_to_pose
        bt_params['default_nav_through_poses_bt_xml'] = bt_through_poses
        # Keep compatibility with older key used by some Nav2 configs.
        bt_params['default_bt_xml_filename'] = bt_to_pose

    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='nav2_params_')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    return path


def _launch_setup(context):
    """OpaqueFunction that resolves substitutions and builds the launch actions."""
    ns = LaunchConfiguration('effective_namespace').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    wait_for_tf_str = LaunchConfiguration('wait_for_tf').perform(context)
    enable_debug_logging_str = LaunchConfiguration('enable_debug_logging').perform(context)
    debug_log_dir = LaunchConfiguration('debug_log_dir').perform(context)
    debug_log_rate_hz = LaunchConfiguration('debug_log_rate_hz').perform(context)
    use_central_tf_map_str = LaunchConfiguration('use_central_tf_map').perform(
        context)

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    turtlebot3_nav2_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch')

    slam_params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'humble', 'mapper_params_online_async_fast.yaml')

    workspace_dir = os.path.expanduser(
        os.environ.get('TURTLEBOT3_WS', '~/turtlebot3_ws'))
    repo_logs_dir = os.path.join(workspace_dir, 'logs')
    wait_tf_script = os.path.join(workspace_dir, 'scripts', 'wait_for_tf.py')
    expanded_debug_log_dir = os.path.expanduser(debug_log_dir)
    legacy_debug_root = os.path.expanduser('~/.ros/nav2_debug')
    if expanded_debug_log_dir == legacy_debug_root:
        resolved_debug_log_dir = repo_logs_dir
    else:
        resolved_debug_log_dir = expanded_debug_log_dir


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

    # slam_toolbox reads parameters from YAML based on the node's fully
    # qualified name when running under a namespace. Without rewriting the
    # YAML top-level key, it can silently fall back to defaults (notably
    # `map_update_interval`), making the map publish/update very slowly.
    slam_params_file = slam_params
    if ns:
        with open(slam_params) as f:
            slam_params_dict = yaml.safe_load(f)

        # Example: for namespace "pinky" and node name "slam_toolbox",
        # rewrite `slam_toolbox:` -> `pinky/slam_toolbox:`.
        if 'slam_toolbox' in slam_params_dict:
            slam_params_dict[f'{ns}/slam_toolbox'] = slam_params_dict.pop('slam_toolbox')

        fd, path = tempfile.mkstemp(suffix='.yaml', prefix='slam_params_')
        with os.fdopen(fd, 'w') as f:
            yaml.dump(slam_params_dict, f, default_flow_style=False)
        slam_params_file = path

    actions.append(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=ns if ns else None,
        output='screen',
        parameters=[slam_params_file, slam_overrides],
        remappings=tf_remappings + [
            ('/scan', 'scan_normalized'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
            ('/map_updates', 'map_updates'),
        ],
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

    use_multirobot_nav_launch = (
        use_central_tf_map_str.lower() in ('1', 'true', 'yes'))
    if use_multirobot_nav_launch:
        nav2_launch_source = PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_nav2_launch_dir, 'navigation_launch_multirobot.py'))
    else:
        nav2_launch_source = PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/navigation_launch.py'])

    nav2_include = IncludeLaunchDescription(
        nav2_launch_source,
        launch_arguments={
            'namespace': ns,
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

    # --- Structured Nav2 debug logger (optional) ---
    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='nav2_motion_debug_logger.py',
        name='nav2_motion_debug_logger',
        namespace=ns if ns else None,
        parameters=[{
            'robot_name': ns if ns else DEFAULT_ROBOT_NAME,
            'output_dir': resolved_debug_log_dir,
            'log_rate_hz': float(debug_log_rate_hz),
            'map_frame': 'map',
            'base_frame_candidates': (
                f'{ns}/base_footprint,{ns}/base_link,base_footprint,base_link'
                if ns else 'base_footprint,base_link'
            ),
        }],
        remappings=tf_remappings,
        output='screen',
        condition=IfCondition(enable_debug_logging_str),
    ))

    # --- RViz (optional) ---
    if use_rviz_str.lower() == 'true':
        # RViz config uses absolute topic names like `/map` and `/map_updates`.
        # When running SLAM/Nav2 under a namespace (e.g. `/pinky`), remap those
        # topics so the displayed map matches the robot's published map.
        rviz_remappings = []
        if ns:
            rviz_remappings = [
                ('/map', f'/{ns}/map'),
                ('/map_updates', f'/{ns}/map_updates'),
            ]

        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time_str.lower() == 'true'}],
            remappings=rviz_remappings,
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
            'robot_name', default_value=DEFAULT_ROBOT_NAME,
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
            'enable_debug_logging', default_value='false',
            description='Enable structured Nav2 motion debug logger'),
        DeclareLaunchArgument(
            'debug_log_dir', default_value='~/turtlebot3_ws/logs',
            description='Directory for nav2_motion_debug_logger JSONL output'),
        DeclareLaunchArgument(
            'debug_log_rate_hz', default_value='5.0',
            description='Debug logger sampling rate in Hz'),
        DeclareLaunchArgument(
            'use_central_tf_map', default_value='false',
            description='If true, Nav2 remaps tf/tf_static and map to global /tf, '
                        '/map (required with start_central.sh + map_merge + explorer).'),
        DeclareLaunchArgument(
            'effective_namespace', default_value=effective_namespace,
            description='(internal) resolved namespace'),
        OpaqueFunction(function=_launch_setup),
    ])
