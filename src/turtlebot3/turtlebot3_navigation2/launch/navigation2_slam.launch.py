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
#   fleet_mode:=true
#
# Optional: nav2_use_composition (default false on Pi), nav2_use_isolated_container (default false),
# fleet_map_relay_hz, nav2_use_local_slam_map, scan_costmap_max_hz (costmap-only scan throttle),
# slam_toolbox_mode (async|sync). Robot-side README: ans-turtlebot3 repo root.

import os
import socket
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')

# Stock image hostnames are not unique per robot; prefer login name for the default namespace.
_GENERIC_DEFAULT_HOSTNAMES = frozenset({
    'ubuntu', 'raspberrypi', 'raspberry', 'debian', 'linaro-alip', 'localhost', 'omap',
})


def _default_robot_name():
    """Default namespace when ``robot_name`` is not passed explicitly.

    Use ``HOSTNAME``/``HOST`` or the kernel hostname when it identifies the robot
    (e.g. ``pinky``). Ignore common unflashy defaults (``ubuntu``, ``raspberrypi``)
    so the same launch command on every Pi can rely on ``USER`` (``pinky`` vs ``clyde``).
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


def _make_wait_tf_exit_handler(nav2_group, delay_sec: float):
    """Start Nav2 only when wait_for_tf exits 0; otherwise shut the launch down."""

    def _handler(event, context):
        if not isinstance(event, ProcessExited):
            return None
        if event.returncode == 0:
            if delay_sec > 0.0:
                return [
                    TimerAction(
                        period=float(delay_sec),
                        actions=[nav2_group],
                    )
                ]
            return [nav2_group]
        rc = int(event.returncode)
        reason = (
            f'wait_for_tf.py exited with code {rc} '
            '(0=ok, 1=map-odom timeout, 2=odom-base timeout, '
            '3=map_merge map->robot/map timeout, 4=full-chain timeout, '
            '5=post-settle lost map->base). Nav2 will not start.'
        )
        return [
            LogInfo(msg=reason),
            Shutdown(reason=reason),
        ]

    return _handler


def _rewrite_frame(d, key, old_val, new_val):
    """Recursively find parameters matching key=old_val and replace with new_val."""
    for k, v in list(d.items()):
        if k == key and v == old_val:
            d[k] = new_val
        elif isinstance(v, dict):
            _rewrite_frame(v, key, old_val, new_val)


def _generate_nav2_params(
    source_file,
    namespace,
    fleet_mode: bool,
    *,
    fleet_use_local_slam_map: bool = False,
    fleet_map_relay: bool = False,
    costmap_scan_relay: bool = False,
):
    """Generate a modified Nav2 params file with namespace-prefixed frame names.

    When namespace is set and fleet_mode is False (robot alone), SLAM uses
    frame {namespace}/map; Nav2 global frames and map_topic follow that.
    When fleet_mode is True, Nav2 uses world frame ``map`` and merged ``/map``.
    When fleet_mode and fleet_use_local_slam_map, Nav2 uses local SLAM ``/{ns}/map``
    (no network ``/map`` subscription) while TF can remain global via remappings.
    When fleet_map_relay, static layer ``map_topic`` points at ``/map_relay``.
    """
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
        # Costmap nodes are nested (e.g. /pinky/local_costmap/local_costmap). A
        # relative laser topic "scan_normalized" would wrongly resolve to
        # /pinky/local_costmap/scan_normalized (no publisher). Normalizer and
        # SLAM publish at /pinky/scan_normalized — use an absolute topic.
        # Optional relay publishes scan_costmap at a lower rate for costmaps only.
        scan_abs = (
            f'/{namespace}/scan_costmap'
            if costmap_scan_relay
            else f'/{namespace}/scan_normalized'
        )
        _rewrite_frame(params, 'topic', '/scan_normalized', scan_abs)
        _rewrite_frame(params, 'topic', 'scan_normalized', scan_abs)
        _rewrite_frame(params, 'scan_topic', '/scan_normalized', scan_abs)
        _rewrite_frame(params, 'scan_topic', 'scan_normalized', scan_abs)
        if not fleet_mode:
            # Standalone namespaced robot: SLAM map frame is {ns}/map.
            _rewrite_frame(params, 'global_frame', 'map',
                            f'{namespace}/map')
            _rewrite_frame(params, 'global_frame_id', 'map',
                            f'{namespace}/map')
            # Global costmap static layer uses this robot's SLAM map topic.
            _rewrite_frame(params, 'map_topic', '/map', f'/{namespace}/map')
            _rewrite_frame(params, 'map_topic', 'map', f'/{namespace}/map')
        elif fleet_mode and fleet_use_local_slam_map:
            # Fleet TF from central, but costmaps use local SLAM map (less DDS load).
            _rewrite_frame(params, 'global_frame', 'map',
                            f'{namespace}/map')
            _rewrite_frame(params, 'global_frame_id', 'map',
                            f'{namespace}/map')
            _rewrite_frame(params, 'map_topic', '/map', f'/{namespace}/map')
            _rewrite_frame(params, 'map_topic', 'map', f'/{namespace}/map')

    if namespace and fleet_mode and fleet_map_relay and not fleet_use_local_slam_map:
        _rewrite_frame(params, 'map_topic', '/map', '/map_relay')
        _rewrite_frame(params, 'map_topic', 'map', '/map_relay')

    # Use a custom navigate-to-pose tree that proactively clears costmaps
    # around planner/controller failures and keeps backup/spin/wait recoveries.
    bt_tree_path = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery_with_lethal_escape.xml',
    )
    bt_nav_params = params.get('bt_navigator', {}).get('ros__parameters', {})
    if isinstance(bt_nav_params, dict):
        bt_nav_params['default_bt_xml_filename'] = bt_tree_path

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
    enable_lethal_watch_str = LaunchConfiguration('enable_lethal_watch').perform(context)
    enable_retrace_escape_str = LaunchConfiguration('enable_retrace_escape').perform(context)
    enable_controller_collision_watch_str = LaunchConfiguration(
        'enable_controller_collision_watch').perform(context)
    debug_log_dir = LaunchConfiguration('debug_log_dir').perform(context)
    debug_log_rate_hz = LaunchConfiguration('debug_log_rate_hz').perform(context)
    fleet_mode_str = LaunchConfiguration('fleet_mode').perform(context)
    auto_fleet_wait_timeout_str = LaunchConfiguration(
        'auto_fleet_wait_timeout_sec').perform(context)
    auto_fleet_tf_max_age_str = LaunchConfiguration(
        'auto_fleet_tf_max_age_sec').perform(context)
    use_central_tf_map_str = LaunchConfiguration('use_central_tf_map').perform(
        context)
    fleet_map_relay_hz_str = LaunchConfiguration('fleet_map_relay_hz').perform(
        context)
    nav2_use_local_slam_map_str = LaunchConfiguration(
        'nav2_use_local_slam_map').perform(context)
    nav2_use_composition_str = LaunchConfiguration('nav2_use_composition').perform(
        context)
    nav2_container_name_str = LaunchConfiguration('nav2_container_name').perform(
        context)
    slam_toolbox_mode_str = LaunchConfiguration('slam_toolbox_mode').perform(
        context)
    container_sigterm_timeout_str = LaunchConfiguration(
        'container_sigterm_timeout').perform(context)
    container_sigkill_timeout_str = LaunchConfiguration(
        'container_sigkill_timeout').perform(context)
    nav2_use_isolated_container_str = LaunchConfiguration(
        'nav2_use_isolated_container').perform(context)
    fleet_tf_map_wait_timeout_str = LaunchConfiguration(
        'fleet_tf_map_wait_timeout_sec').perform(context)
    scan_costmap_max_hz_str = LaunchConfiguration(
        'scan_costmap_max_hz').perform(context)

    fleet_mode_norm = fleet_mode_str.lower()
    fleet_auto_mode = fleet_mode_norm == 'auto'
    fleet_active = fleet_mode_norm in ('1', 'true', 'yes') or fleet_auto_mode
    if use_central_tf_map_str.lower() in ('1', 'true', 'yes'):
        fleet_active = True

    try:
        fleet_map_relay_hz = float(fleet_map_relay_hz_str)
    except ValueError:
        fleet_map_relay_hz = 0.0
    fleet_use_local_nav_map = (
        fleet_active and bool(ns) and
        nav2_use_local_slam_map_str.lower() in ('1', 'true', 'yes'))
    # Throttled merged-map relay is incompatible with Nav2 on local /<robot>/map.
    fleet_map_relay = (
        fleet_active and bool(ns) and fleet_map_relay_hz > 0.0
        and not fleet_use_local_nav_map)

    try:
        scan_costmap_max_hz = float(scan_costmap_max_hz_str)
    except ValueError:
        scan_costmap_max_hz = 0.0
    costmap_scan_relay = bool(ns) and scan_costmap_max_hz > 0.0

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    turtlebot3_nav2_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch')

    if slam_toolbox_mode_str.lower() == 'sync':
        slam_params = os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config', 'mapper_params_online_sync.yaml')
        slam_toolbox_executable = 'sync_slam_toolbox_node'
    else:
        slam_params = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param', 'humble', 'mapper_params_online_async_fast.yaml')
        slam_toolbox_executable = 'async_slam_toolbox_node'

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
        output='screen',
    ))

    if costmap_scan_relay:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='scan_costmap_relay.py',
            name='scan_costmap_relay',
            namespace=ns,
            parameters=[{
                'input_topic': 'scan_normalized',
                'output_topic': 'scan_costmap',
                'max_hz': scan_costmap_max_hz,
            }],
            output='screen',
        ))

    # Fleet Nav2 listens on global /tf; robot + SLAM publish on /{ns}/tf only.
    if ns and fleet_active:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='namespace_tf_to_global_tf_relay.py',
            name='namespace_tf_to_global_tf_relay',
            parameters=[{'robot_namespace': ns}],
            output='screen',
        ))

    # Standalone namespaced robot: SLAM uses {ns}/map but goals / BT often use
    # world frame "map". Publish map -> {ns}/map on /{ns}/tf_static (Python
    # broadcaster + periodic refresh; tf2_ros CLI under launch was unreliable).
    # With fleet_mode:=true, Nav2 uses global /tf; this launch republishes
    # /{ns}/tf -> /tf so the robot works without central. Central can still add
    # world frames on /tf when the bridge is up.
    if ns and not fleet_active:
        # tf2_ros.StaticTransformBroadcaster uses absolute "/tf_static". Same
        # remapping as Nav2/slam so the bridge publishes on /<ns>/tf_static.
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='standalone_world_map_tf.py',
            name='standalone_world_map_tf',
            namespace=ns,
            parameters=[{'child_frame': f'{ns}/map'}],
            remappings=tf_remappings,
            output='screen',
        ))

    # Fleet mode: zlib-compressed map side channel for central relay (/{ns}/map_wire_z).
    if ns and fleet_active:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='map_wire_compressed_republisher.py',
            name='map_wire_compressed_republisher',
            namespace=ns,
            parameters=[{
                'input_topic': 'map',
                'output_topic': 'map_wire_z',
                # Fleet bridge contract: low-rate, compressed map side channel.
                'max_publish_hz': 1.0,
                'compression_level': 3,
            }],
            output='screen',
        ))

    # Throttle merged /map (and updates) for Nav2 when Wi‑Fi is constrained.
    if fleet_map_relay:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='map_throttle_relay.py',
            name='map_throttle_relay',
            parameters=[{
                'input_map_topic': '/map',
                'output_map_topic': '/map_relay',
                'input_updates_topic': '/map_updates',
                'output_updates_topic': '/map_updates_relay',
                'max_map_hz': fleet_map_relay_hz,
            }],
            output='screen',
        ))

    # --- SLAM Toolbox ---
    slam_overrides = {
        'use_sim_time': use_sim_time_str.lower() == 'true',
    }
    if ns:
        slam_overrides['odom_frame'] = f'{ns}/odom'
        slam_overrides['base_frame'] = f'{ns}/base_footprint'
        slam_overrides['map_frame'] = f'{ns}/map'

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
        executable=slam_toolbox_executable,
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
    wait_tf_env['TF_WAIT_STABLE_SAMPLES'] = '2'
    if ns:
        wait_tf_env['TF_WAIT_ODOM_FRAME'] = f'{ns}/odom'
        wait_tf_env['TF_WAIT_BASE_FRAMES'] = f'{ns}/base_footprint,{ns}/base_link'
        wait_tf_env['TF_WAIT_NAMESPACE'] = ns
        wait_tf_env['TF_WAIT_MAP_FRAME'] = f'{ns}/map'

    fleet_require_global_tf_before_nav2 = (
        LaunchConfiguration('fleet_require_global_tf_before_nav2').perform(context).lower()
        in ('1', 'true', 'yes')
    )

    if fleet_active and ns and not fleet_auto_mode and fleet_require_global_tf_before_nav2:
        wait_tf_env['TF_WAIT_FLEET_WORLD_MAP_FRAME'] = f'{ns}/map'
        wait_tf_env['TF_WAIT_FLEET_MAP_TIMEOUT_SEC'] = fleet_tf_map_wait_timeout_str
        # Hold briefly while map->base stays valid so Nav2 lifecycle does not race a
        # vanishing root `map` frame (see stabilize_fleet plan: activation race).
        wait_tf_env['TF_WAIT_FLEET_POST_SETTLE_SEC'] = '2.0'

    # Gated Nav2 start: must reference the same ExecuteProcess below for OnProcessExit.
    wait_tf_proc = None
    if wait_for_tf_str.lower() == 'true':
        wait_tf_proc = ExecuteProcess(
            cmd=['python3', wait_tf_script],
            output='screen',
            env=wait_tf_env,
        )
        actions.append(wait_tf_proc)

    # --- Nav2 with frame-rewritten params ---
    nav2_params_file = _generate_nav2_params(
        params_file, ns, fleet_active,
        fleet_use_local_slam_map=fleet_use_local_nav_map,
        fleet_map_relay=fleet_map_relay,
        costmap_scan_relay=costmap_scan_relay,
    )

    if fleet_map_relay:
        nav_map_topic_str = '/map_relay'
        nav_map_updates_str = '/map_updates_relay'
    elif fleet_use_local_nav_map:
        nav_map_topic_str = f'/{ns}/map'
        nav_map_updates_str = f'/{ns}/map_updates'
    else:
        nav_map_topic_str = '/map'
        nav_map_updates_str = '/map_updates'

    if ns:
        if fleet_use_local_nav_map:
            map_frame_for_nav = f'{ns}/map'
        elif fleet_active:
            map_frame_for_nav = 'map'
        else:
            map_frame_for_nav = f'{ns}/map'
    else:
        map_frame_for_nav = 'map'

    use_multirobot_nav_launch = fleet_active
    if use_multirobot_nav_launch:
        nav2_launch_source = PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_nav2_launch_dir, 'navigation_launch_multirobot.py'))
    else:
        nav2_launch_source = PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/navigation_launch.py'])

    nav2_launch_args = {
        'namespace': ns,
        'use_sim_time': use_sim_time_str,
        'params_file': nav2_params_file,
        'autostart': 'True',
    }
    # navigation_launch_multirobot spawns rclcpp component_container_isolated; stock
    # navigation_launch.py does not — only enable composition for fleet (multirobot).
    if fleet_active:
        nav2_launch_args['use_composition'] = nav2_use_composition_str
        nav2_launch_args['container_name'] = nav2_container_name_str
    if use_multirobot_nav_launch:
        nav2_launch_args['nav_map_topic'] = nav_map_topic_str
        nav2_launch_args['nav_map_updates_topic'] = nav_map_updates_str
        nav2_launch_args['container_sigterm_timeout'] = container_sigterm_timeout_str
        nav2_launch_args['container_sigkill_timeout'] = container_sigkill_timeout_str
        nav2_launch_args['use_isolated_container'] = nav2_use_isolated_container_str

    nav2_include = IncludeLaunchDescription(
        nav2_launch_source,
        launch_arguments=nav2_launch_args.items(),
    )

    if ns:
        nav2_group = GroupAction([
            PushRosNamespace(ns),
            nav2_include,
        ])
    else:
        nav2_group = nav2_include

    if fleet_auto_mode and ns:
        # In auto mode, keep startup order flexible: robots can start before
        # the central stack. Delay Nav2 bringup until global TF chain exists.
        auto_wait_env = dict(os.environ)
        auto_wait_env['TF_WAIT_ODOM_ONLY'] = 'false'
        auto_wait_env['TF_WAIT_TIMEOUT_SEC'] = auto_fleet_wait_timeout_str
        auto_wait_env['TF_WAIT_MAX_AGE_SEC'] = auto_fleet_tf_max_age_str
        auto_wait_env['TF_WAIT_STABLE_SAMPLES'] = '3'
        auto_wait_env['TF_WAIT_MAP_FRAME'] = 'map'
        auto_wait_env['TF_WAIT_ODOM_FRAME'] = f'{ns}/odom'
        auto_wait_env['TF_WAIT_BASE_FRAMES'] = (
            f'{ns}/base_footprint,{ns}/base_link'
        )
        # Empty namespace means the waiter listens to global /tf only.
        auto_wait_env['TF_WAIT_NAMESPACE'] = ''
        auto_wait_proc = ExecuteProcess(
            cmd=['python3', wait_tf_script],
            output='screen',
            env=auto_wait_env,
        )
        actions.append(auto_wait_proc)
        auto_delay = 3.0 if wait_for_tf_str.lower() == 'true' else 0.0
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=auto_wait_proc,
                on_exit=_make_wait_tf_exit_handler(nav2_group, auto_delay),
            )
        ))
    elif wait_for_tf_str.lower() == 'true' and wait_tf_proc is not None:
        # Nav2 must not start on a fixed delay from launch: TimerAction(3s) raced ahead
        # of wait_for_tf and let the Nav2 stack load/activate before TF was ready
        # (local_costmap: odom/base_footprint in different trees).
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=wait_tf_proc,
                on_exit=_make_wait_tf_exit_handler(nav2_group, 1.0),
            )
        ))
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
            'map_frame': map_frame_for_nav,
            'base_frame_candidates': (
                f'{ns}/base_footprint,{ns}/base_link,base_footprint,base_link'
                if ns else 'base_footprint,base_link'
            ),
        }],
        # tf2_ros.TransformListener hard-subscribes to /tf and /tf_static. Remapping those
        # to namespaced topics breaks fleet stacks that publish the world TF on global /tf
        # (same rationale as navigation_launch_multirobot.py).
        output='screen',
        condition=IfCondition(enable_debug_logging_str),
    ))

    # --- Publish whether the robot is in Nav2 lethal costmap space ---
    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='nav2_lethal_watch.py',
        name='nav2_lethal_watch',
        namespace=ns if ns else None,
        parameters=[{
            'robot_name': ns if ns else DEFAULT_ROBOT_NAME,
            'map_frame': map_frame_for_nav,
            'base_frame': (
                f'{ns}/base_footprint' if ns else 'base_footprint'),
            'costmap_topic': 'global_costmap/costmap',
            'publish_topic': 'nav2_lethal_inflation',
            'lethal_cost_threshold': 100,
            'publish_hz': 4.0,
        }],
        output='screen',
        condition=IfCondition(enable_lethal_watch_str),
    ))

    # --- Memory-based retrace escape for lethal-start / progress timeout ---
    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='nav2_retrace_escape.py',
        name='nav2_retrace_escape',
        namespace=ns if ns else None,
        parameters=[
            nav2_params_file,
            {
                'robot_name': ns if ns else DEFAULT_ROBOT_NAME,
                'map_frame': map_frame_for_nav,
                'base_frame': (
                    f'{ns}/base_footprint' if ns else 'base_footprint'),
                'nav2_status_topic': 'navigate_to_pose/_action/status',
                'nav2_cancel_service': 'navigate_to_pose/_action/cancel_goal',
                'lethal_topic': 'nav2_lethal_inflation',
                'retrace_active_topic': 'nav2_retrace_active',
            },
        ],
        output='screen',
        condition=IfCondition(enable_retrace_escape_str),
    ))

    # --- controller_server "collision ahead" -> Bool for central arrival probe ---
    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='nav2_controller_collision_watch.py',
        name='nav2_controller_collision_watch',
        namespace=ns if ns else None,
        parameters=[{
            'rosout_topic': '/rosout',
            'node_name_substring': 'controller_server',
            'message_substring': 'collision ahead',
            'hold_sec': 0.75,
            'publish_hz': 4.0,
            'publish_topic': 'nav2_collision_ahead',
        }],
        output='screen',
        condition=IfCondition(enable_controller_collision_watch_str),
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
            description=(
                'Namespace; default is hostname if it is not a stock image name '
                '(ubuntu, raspberrypi, …), else login name (USER)'
            )),
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
            'use_rviz', default_value='false',
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
            'enable_lethal_watch', default_value='false',
            description='Publish /<robot>/nav2_lethal_inflation from global costmap'),
        DeclareLaunchArgument(
            'enable_retrace_escape', default_value='true',
            description=(
                'Enable robot-side memory retrace helper; publishes '
                '/<robot>/nav2_retrace_active and sends retreat goals on lethal/stall.')),
        DeclareLaunchArgument(
            'enable_controller_collision_watch', default_value='true',
            description=(
                'Publish /<robot>/nav2_collision_ahead from controller_server '
                'rosout lines (e.g. RPP collision ahead)')),
        DeclareLaunchArgument(
            'fleet_mode', default_value='false',
            description=('Fleet topology mode: true=merged /map + inject map TF from '
                         'central, false=standalone namespaced map/TF (default), '
                         'auto=wait for central global TF/map before starting Nav2.')),
        DeclareLaunchArgument(
            'fleet_require_global_tf_before_nav2', default_value='false',
            description=(
                'When fleet_mode=true, block Nav2 startup until global map->robot/map TF exists. '
                'Default false keeps robot control loops local-first and resilient to central jitter.')),
        DeclareLaunchArgument(
            'auto_fleet_wait_timeout_sec', default_value='300.0',
            description=('When fleet_mode=auto, max seconds to wait for central '
                         'global TF/map before proceeding.')),
        DeclareLaunchArgument(
            'auto_fleet_tf_max_age_sec', default_value='0.6',
            description=('When fleet_mode=auto, require fresh global TF '
                         '(map->odom and odom->base) age <= this value. '
                         'Set <= 0 to disable freshness gating.')),
        DeclareLaunchArgument(
            'use_central_tf_map', default_value='false',
            description='Deprecated alias for fleet_mode; if true, enables global /tf and /map.'),
        DeclareLaunchArgument(
            'fleet_map_relay_hz', default_value='1.5',
            description=(
                'Fleet only: if > 0, throttle merged /map to /map_relay at this max rate (Hz). '
                'Reduces DDS load on Wi‑Fi; requires merged /map from central. Disabled automatically '
                'when nav2_use_local_slam_map:=true (local /<robot>/map). Set 0 to disable relay '
                'when using merged /map without throttling.')),
        DeclareLaunchArgument(
            'fleet_tf_map_wait_timeout_sec', default_value='120.0',
            description=(
                'Fleet only (fleet_mode true, not auto): after odom->base TF, max seconds to wait '
                'for map_merge transform map->/<robot>/map on global /tf before starting Nav2.')),
        DeclareLaunchArgument(
            'nav2_use_local_slam_map', default_value='false',
            description=(
                'Fleet only: if true, Nav2 costmaps use local /<robot>/map from SLAM instead of '
                'merged /map (merged-map throttle relay is turned off automatically). '
                'Helps when map->robot/map alignment is wrong; explorer goals still use world map.')),
        DeclareLaunchArgument(
            'nav2_use_composition', default_value='false',
            description=(
                'When fleet_mode is true: if true, load Nav2 in one rclcpp component container '
                '(higher segfault risk on Pi under load). Default false uses separate processes.'
            )),
        DeclareLaunchArgument(
            'nav2_container_name', default_value='nav2_container',
            description='Component container name for Nav2 composition.'),
        DeclareLaunchArgument(
            'container_sigterm_timeout',
            default_value='5',
            description=(
                'Fleet Nav2 only: seconds after SIGINT before launch escalates the '
                'component container to SIGTERM (Nav2 lifecycle teardown is slow).')),
        DeclareLaunchArgument(
            'container_sigkill_timeout',
            default_value='5',
            description=(
                'Fleet Nav2 only: seconds after SIGTERM before launch SIGKILLs the '
                'component container.')),
        DeclareLaunchArgument(
            'nav2_use_isolated_container',
            default_value='false',
            description=(
                'Fleet Nav2 only: if true, use component_container_isolated (can hang '
                'on shutdown; see ros2/rclcpp#2083). Default false uses component_container.')),
        DeclareLaunchArgument(
            'slam_toolbox_mode', default_value='async',
            description='slam_toolbox node: async (default) or sync (mapper_params_online_sync).'),
        DeclareLaunchArgument(
            'scan_costmap_max_hz', default_value='0',
            description=(
                'Namespaced robots only: if > 0, relay scan_normalized -> scan_costmap at this '
                'max rate (Hz) for Nav2 costmaps; SLAM stays on full-rate scan_normalized. '
                '0 disables (default). Try 5–7.5 on Pi fleet to cut message_filters load.')),
        DeclareLaunchArgument(
            'effective_namespace', default_value=effective_namespace,
            description='(internal) resolved namespace'),
        OpaqueFunction(function=_launch_setup),
    ])
