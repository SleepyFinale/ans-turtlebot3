# Nav2 servers for one namespaced TurtleBot3 (included by navigation2_slam.launch.py).
#
# Differences from nav2_bringup/navigation_launch.py (Apache 2.0):
# - Nav2 reads this robot's TF on /<robot>/tf and /<robot>/tf_static.
# - One process per server. A shared component container crashed under load
#   on the Pi and hung on Ctrl+C.
# - Lifecycle bonds are off, so a CPU spike cannot make the lifecycle manager
#   tear the stack down.
# - No smoother_server or waypoint_follower. The behavior trees never call
#   SmoothPath and the central PC only sends NavigateToPose and
#   ComputePathToPose, so they were two idle processes (~0.05 core, 40 MB and
#   two more DDS participants) on the Pi.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'velocity_smoother'
    ]

    # Inside the robot namespace these resolve to /<robot>/tf and
    # /<robot>/tf_static, where bringup and slam_toolbox publish.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True),
        allow_substs=True)

    def server(package, executable, extra_remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + list(extra_remappings))

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # Velocity pipeline: controller -> cmd_vel_nav -> smoother ->
        # cmd_vel_smoothed, and behaviors -> cmd_vel_behavior. cmd_vel_mux
        # (navigation2_slam.launch.py) picks one of those or a stuck_detector
        # reflex and is the only node that publishes cmd_vel.
        server('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_behavior')]),
        server('nav2_bt_navigator', 'bt_navigator'),
        server('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                # 0 disables bond heartbeats; see the header.
                'bond_timeout': 0.0,
            }]),
    ])
