#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


_GENERIC_DEFAULT_HOSTNAMES = frozenset({
    'ubuntu', 'raspberrypi', 'raspberry', 'debian', 'linaro-alip', 'localhost', 'omap',
})


def _default_robot_name():
    """Default namespace when ``robot_name`` is not passed explicitly.

    Use hostname when it identifies the robot; ignore stock image defaults so
    ``USER`` selects the namespace when ``/etc/hostname`` is still ``ubuntu``.export TURTLEBOT3_MODEL=burger
source scripts/ros_domain_profile.bash
source scripts/ros_robot_env.bash
ros2 launch turtlebot3_bringup robot.launch.py
    """

    def _short(raw):
        if not raw:
            return ''
        return raw.split('.')[0].strip()

    for key in ('HOSTNAME', 'HOST'):
        h = _short(os.environ.get(key, ''))
        if h and h.lower() not in _GENERIC_DEFAULT_HOSTNAMES:
            return h
    try:
        k = _short(socket.gethostname())
        if k and k.lower() not in _GENERIC_DEFAULT_HOSTNAMES:
            return k
    except OSError:
        pass
    if os.environ.get('USER') == 'root' and os.environ.get('SUDO_USER'):
        return os.environ['SUDO_USER']
    return os.environ.get('USER') or os.environ.get('LOGNAME') or 'robot'


DEFAULT_ROBOT_NAME = _default_robot_name()
LIDAR_USB_PORT = '/dev/tb3_lidar'

def launch_gps_nodes(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace_override = LaunchConfiguration('namespace').perform(context)
    effective_ns = namespace_override if namespace_override else robot_name
    gps_port_1 = LaunchConfiguration('gps_port_1').perform(context).strip()
    gps_port_2 = LaunchConfiguration('gps_port_2').perform(context).strip()
    gps_baud_1 = int(LaunchConfiguration('gps_baud_1').perform(context))
    gps_baud_2 = int(LaunchConfiguration('gps_baud_2').perform(context))

    gps_ports = []
    if gps_port_1:
        gps_ports.append(gps_port_1)
    if gps_port_2:
        gps_ports.append(gps_port_2)
    nodes = []
    if not gps_ports:
        print('[robot.launch.py] WARNING: No GPS serial ports configured; starting without nmea_serial_driver nodes.')
    else:
        print(f'[robot.launch.py] GPS ports selected: {gps_ports}')
        print(f'[robot.launch.py] GPS baud rates selected: {[gps_baud_1, gps_baud_2][:len(gps_ports)]}')

    gps_bauds = [gps_baud_1, gps_baud_2]
    for i, port in enumerate(gps_ports[:2]):

        nodes.append(
            Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name=f'gps_{i+1}',
                parameters=[{
                    'port': port,
                    'baud': gps_bauds[i],
                    'frame_id': f'{effective_ns}/gps_link'
                }],
                remappings=[
                    ('fix', f'gps{i+1}/fix')
                ],
                output='screen'
            )
        )

    return nodes

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace', default='')

    effective_namespace = PythonExpression(
        ['"', namespace, '" if "', namespace, '" != "" else "', robot_name, '"']
    )

    usb_port = LaunchConfiguration('usb_port', default='/dev/opencr')
    lidar_port = LaunchConfiguration('lidar_port', default=LIDAR_USB_PORT)

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
        ekf_param_dir = LaunchConfiguration(
            'ekf_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                "ekf" + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('coin_d4_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_slam_with_normalizer = LaunchConfiguration('start_slam_with_normalizer', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=lidar_port,
            description='Connected USB port for lidar (recommended: /dev/tb3_lidar)'),
        DeclareLaunchArgument(
            'gps_port_1',
            default_value='/dev/gps1',
            description='GPS 1 serial device path (recommended: /dev/gps1)'),
        DeclareLaunchArgument(
            'gps_port_2',
            default_value='/dev/gps2',
            description='GPS 2 serial device path (recommended: /dev/gps2)'),
        DeclareLaunchArgument(
            'gps_baud_1',
            default_value='115200',
            description='GPS 1 baud rate (tested default: 115200)'),
        DeclareLaunchArgument(
            'gps_baud_2',
            default_value='115200',
            description='GPS 2 baud rate (tested default: 115200)'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'robot_name',
            default_value=DEFAULT_ROBOT_NAME,
            description=(
                'Namespace; default is hostname if not a stock image name '
                '(ubuntu, raspberrypi, …), else login name (USER)'
            )),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Explicit namespace (overrides robot_name if set)'),

        DeclareLaunchArgument(
            'start_slam_with_normalizer',
            default_value=start_slam_with_normalizer,
            description='If true, also run scripts/start_slam_with_normalizer.sh'),

        PushRosNamespace(effective_namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': effective_namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': lidar_port,
                              'frame_id': 'base_scan',
                              'namespace': effective_namespace}.items(),
        ),

        OpaqueFunction(function=launch_gps_nodes),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': effective_namespace}],
            arguments=['-i', usb_port],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'),

        ExecuteProcess(
            condition=IfCondition(start_slam_with_normalizer),
            cmd=[
                'bash',
                os.path.join(
                    os.path.expanduser('~'),
                    'turtlebot3_ws',
                    'scripts',
                    'start_slam_with_normalizer.sh'
                )
            ],
            output='screen',
            additional_env={
                'USE_SIM_TIME': use_sim_time
            },
        ),

        Node(
            package='turtlebot3_bringup',
            executable='auto_gps_fusion.py',
            name='gps_driver',
            output='screen',
            parameters=[{'namespace': namespace}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[ekf_param_dir,{'namespace': namespace}],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            remappings=[('gps/fix','fix'), ('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[ekf_param_dir,{'namespace': namespace}],
        ),
    ])
