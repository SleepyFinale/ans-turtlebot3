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
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
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
    ``USER`` selects the namespace when ``/etc/hostname`` is still ``ubuntu``.
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


def validate_opencr_port(context, *args, **kwargs):
    """Warn early when the OpenCR symlink does not resolve to a live tty device."""
    port = LaunchConfiguration('usb_port').perform(context).strip()
    if not port:
        print('[robot.launch.py] WARNING: usb_port is empty; turtlebot3_node may fail to connect to OpenCR.')
        return []

    if not os.path.exists(port):
        print(f'[robot.launch.py] WARNING: OpenCR port does not exist: {port}')
        print('[robot.launch.py]          Verify udev rules and that /dev/opencr points to a live ttyACM device.')
        return []

    real_port = os.path.realpath(port)
    try:
        mode = os.stat(real_port).st_mode
        if not stat.S_ISCHR(mode):
            print(f'[robot.launch.py] WARNING: OpenCR target is not a character device: {real_port}')
    except OSError as exc:
        print(f'[robot.launch.py] WARNING: OpenCR stat failed for {real_port}: {exc}')
    return []


def launch_gps_nodes(context, *args, **kwargs):
    """Launch up to two GPS serial drivers after filtering invalid or duplicate ports."""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace_override = LaunchConfiguration('namespace').perform(context)
    effective_ns = namespace_override if namespace_override else robot_name
    gps_port_1 = LaunchConfiguration('gps_port_1').perform(context).strip()
    gps_port_2 = LaunchConfiguration('gps_port_2').perform(context).strip()
    gps_enable_1 = LaunchConfiguration('gps_enable_1').perform(context).strip().lower() in ('1', 'true', 'yes', 'on')
    gps_enable_2 = LaunchConfiguration('gps_enable_2').perform(context).strip().lower() in ('1', 'true', 'yes', 'on')
    gps_baud_1 = int(LaunchConfiguration('gps_baud_1').perform(context))
    gps_baud_2 = int(LaunchConfiguration('gps_baud_2').perform(context))
    outdoor_mode = LaunchConfiguration('outdoor').perform(context).strip().lower() in ('1', 'true', 'yes', 'on')

    if not outdoor_mode:
        print('[robot.launch.py] Outdoor mode disabled (outdoor:=false); skipping all GPS serial drivers.')
        return []

    # Keep GPS selection deterministic so outdoor runs do not accidentally open
    # the same USB-UART twice through two different symlink names.
    configured_ports = [
        (1, gps_port_1, gps_baud_1, gps_enable_1),
        (2, gps_port_2, gps_baud_2, gps_enable_2),
    ]
    gps_ports = []
    seen_real_targets = {}
    for gps_idx, port, baud, enabled in configured_ports:
        if not enabled:
            print(f'[robot.launch.py] GPS {gps_idx} disabled by launch arg gps_enable_{gps_idx}:={enabled}.')
            continue
        if not port:
            print(f'[robot.launch.py] GPS {gps_idx} port is empty; skipping.')
            continue
        if not os.path.exists(port):
            print(f'[robot.launch.py] WARNING: GPS {gps_idx} port does not exist: {port}; skipping.')
            continue

        real_port = os.path.realpath(port)
        try:
            mode = os.stat(real_port).st_mode
            if not stat.S_ISCHR(mode):
                print(f'[robot.launch.py] WARNING: GPS {gps_idx} target is not a serial character device: {real_port}; skipping.')
                continue
        except OSError as exc:
            print(f'[robot.launch.py] WARNING: GPS {gps_idx} stat failed for {real_port}: {exc}; skipping.')
            continue

        dup_idx = seen_real_targets.get(real_port)
        if dup_idx is not None:
            print(
                f'[robot.launch.py] WARNING: GPS {gps_idx} ({port}) resolves to the same device as '
                f'GPS {dup_idx} ({real_port}). Skipping GPS {gps_idx} to avoid serial contention.'
            )
            continue
        seen_real_targets[real_port] = gps_idx
        gps_ports.append((gps_idx, port, real_port, baud))

    nodes = []
    if not gps_ports:
        print('[robot.launch.py] WARNING: No GPS serial ports configured; starting without nmea_serial_driver nodes.')
    else:
        selected_ports = [f'gps{idx}:{port} -> {real_port}' for idx, port, real_port, _ in gps_ports]
        selected_bauds = [f'gps{idx}:{baud}' for idx, _, _, baud in gps_ports]
        print(f'[robot.launch.py] GPS ports selected: {selected_ports}')
        print(f'[robot.launch.py] GPS baud rates selected: {selected_bauds}')

    for gps_idx, port, _, baud in gps_ports[:2]:

        nodes.append(
            Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name=f'gps_{gps_idx}',
                parameters=[{
                    'port': port,
                    'baud': baud,
                    'frame_id': f'{effective_ns}/gps_link'
                }],
                remappings=[
                    ('fix', f'gps{gps_idx}/fix')
                ],
                output='screen'
            )
        )

    return nodes

def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def launch_base_and_fusion(context, *args, **kwargs):
    """turtlebot3_node plus the EKF that fuses wheels, IMU, stereo VO (and GPS outdoors).

    With use_ekf:=true (default) the EKF owns odom -> base_footprint and the
    odom topic; turtlebot3_node publishes plain wheel odometry on odom/wheel
    with no TF and without substituting the IMU heading, so the EKF gets
    wheels and IMU as two independent measurements. use_ekf:=false restores
    the stock driver: IMU heading inside wheel odometry, driver-owned TF.
    """
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace_override = LaunchConfiguration('namespace').perform(context)
    ns = namespace_override if namespace_override else robot_name
    use_ekf = _truthy(context, 'use_ekf')
    outdoor = _truthy(context, 'outdoor')
    usb_port = LaunchConfiguration('usb_port').perform(context)
    tb3_param_dir = LaunchConfiguration('tb3_param_dir').perform(context)
    ekf_param_dir = LaunchConfiguration('ekf_param_dir').perform(context)
    ekf_gps_param_dir = os.path.join(os.path.dirname(ekf_param_dir), 'ekf_gps.yaml')
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    base_params = [tb3_param_dir, {'namespace': ns}]
    base_remaps = list(tf_remaps)
    if use_ekf:
        base_params.append({'odometry.publish_tf': False, 'odometry.use_imu': False})
        base_remaps.append(('odom', 'odom/wheel'))

    actions = [Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=base_params,
        arguments=['-i', usb_port],
        remappings=base_remaps,
        output='screen')]

    if not use_ekf:
        print('[robot.launch.py] use_ekf:=false; turtlebot3_node publishes odom and its TF directly.')
        if outdoor:
            print('[robot.launch.py] WARNING: outdoor GPS needs the EKF; GPS is read but not fused.')
        return actions

    ekf_files = [ekf_param_dir] + ([ekf_gps_param_dir] if outdoor else [])
    frames = {
        'map_frame': f'{ns}/map',
        'odom_frame': f'{ns}/odom',
        'base_link_frame': f'{ns}/base_footprint',
        'world_frame': f'{ns}/odom',
    }
    print(f'[robot.launch.py] EKF fusing wheels + IMU + stereo VO{" + GPS" if outdoor else ""} -> {ns}/odom')
    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=tf_remaps + [('odometry/filtered', 'odom')],
        parameters=ekf_files + [dict(frames, frequency=float(
            LaunchConfiguration('ekf_frequency').perform(context)))],
    ))
    if outdoor:
        actions.append(Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            # navsat_transform consumes the fused fix topic from auto_gps_fusion
            # and the EKF output, and feeds odometry/gps back into the EKF.
            remappings=tf_remaps + [('gps/fix', 'fix'), ('odometry/filtered', 'odom')],
            parameters=[ekf_gps_param_dir, {
                'frequency': float(LaunchConfiguration('navsat_frequency').perform(context)),
                'world_frame': f'{ns}/odom',
            }],
        ))
    return actions


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
    outdoor = LaunchConfiguration('outdoor', default='false')
    start_slam_with_normalizer = LaunchConfiguration('start_slam_with_normalizer', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'outdoor',
            default_value=outdoor,
            description='If true, enable all GPS-related nodes for outdoor operation'),

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
            'gps_enable_1',
            default_value='true',
            description='Enable GPS 1 nmea_serial_driver node'),
        DeclareLaunchArgument(
            'gps_enable_2',
            default_value='true',
            description='Enable GPS 2 nmea_serial_driver node'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description=(
                'Fuse wheels, IMU, stereo visual odometry (and GPS outdoors) in an EKF that '
                'publishes odom and odom->base_footprint. false = stock wheel+IMU odometry')),
        DeclareLaunchArgument(
            'ekf_frequency',
            # The wheels and IMU arrive at 20 Hz (OpenCR read loop), so 30 Hz
            # only added predictions every /tf and odom listener had to take.
            default_value='20.0',
            description='EKF output frequency (Hz); also the odom->base_footprint TF rate'),
        DeclareLaunchArgument(
            'ekf_param_dir',
            default_value=ekf_param_dir,
            description='EKF parameter file; ekf_gps.yaml next to it is added outdoors'),
        DeclareLaunchArgument(
            'navsat_frequency',
            default_value='4.0',
            description='navsat_transform frequency (Hz)'),

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
            description='Deprecated compatibility flag (no-op; use navigation2_slam.launch.py)'),

        # Everything below runs under the effective robot namespace so sensors,
        # TF, and the hardware node agree on the same <robot>/... graph.
        PushRosNamespace(effective_namespace),

        # Robot model frames are xacro-prefixed with the same namespace so URDF
        # links line up with namespaced TF published by bringup and Nav2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': effective_namespace}.items(),
        ),

        # LiDAR driver stays modular because the fleet can switch between LDS-01,
        # LDS-02, and LDS-03 while keeping the rest of bringup unchanged.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': lidar_port,
                              'frame_id': 'base_scan',
                              'namespace': effective_namespace}.items(),
        ),

        # Hardware validation happens before TurtleBot3/OpenCR startup so bad
        # udev state is visible in the launch log immediately.
        OpaqueFunction(function=validate_opencr_port),
        OpaqueFunction(function=launch_gps_nodes),

        OpaqueFunction(function=launch_base_and_fusion),

        LogInfo(
            condition=IfCondition(start_slam_with_normalizer),
            msg=(
                '[robot.launch.py] start_slam_with_normalizer is deprecated and now a no-op. '
                'Launch Nav2 + SLAM explicitly via: '
                'ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py'
            ),
        ),

        Node(
            package='turtlebot3_bringup',
            executable='auto_gps_fusion.py',
            name='gps_driver',
            condition=IfCondition(outdoor),
            output='screen',
            # Publish a single fused "fix" topic so downstream localization nodes
            # do not need to know whether one or two receivers are active.
            parameters=[{'namespace': namespace}],
        ),
    ])
