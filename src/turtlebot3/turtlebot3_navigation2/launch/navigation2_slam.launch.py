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
# Default (fleet, driven by the central PC over Zenoh):
#   ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py
#
# Bench test without the central PC (adds a static map -> <robot>/map so RViz
# goals in frame `map` work):
#   ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py fleet_mode:=false
#
# Everything runs under the robot namespace (default: USER when the hostname is
# a stock name such as `ubuntu`). Nav2 plans on this robot's own SLAM map and
# TF (/<robot>/map, /<robot>/tf). The central PC never sends TF or maps back,
# so the robot keeps driving if the link drops.

import copy
import os
import socket
import tempfile
from typing import Optional

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
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


def _robot_sensors(workspace_dir: str, namespace: str) -> dict:
    """Optional-hardware flags from config/sensors/<namespace>.yaml ({} if absent)."""
    path = os.path.join(workspace_dir, 'config', 'sensors', f'{(namespace or "robot").strip("/")}.yaml')
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def _stereo_calibration_usable(path: str) -> bool:
    """True when the stereo yaml passes the shared physical sanity checks."""
    try:
        import importlib.util
        common_path = os.path.join(
            get_package_prefix('turtlebot3_navigation2'),
            'lib',
            'turtlebot3_navigation2',
            'stereo_common.py',
        )
        spec = importlib.util.spec_from_file_location('stereo_common_gate', common_path)
        if spec is None or spec.loader is None:
            return False
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return bool(module.calibration_is_usable(path))
    except Exception:
        return False


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
    """Recursively find parameters matching key=old_val and replace with new_val, including lists."""
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                if isinstance(v, str):
                    if v == old_val:
                        d[k] = new_val
                elif isinstance(v, list):
                    d[k] = [new_val if item == old_val else item for item in v]
            elif isinstance(v, dict) or isinstance(v, list):
                _rewrite_frame(v, key, old_val, new_val)
    elif isinstance(d, list):
        for item in d:
            if isinstance(item, dict) or isinstance(item, list):
                _rewrite_frame(item, key, old_val, new_val)


def _generate_nav2_params(
    source_file,
    namespace,
    *,
    costmap_scan_relay: bool = False,
    nav2_enable_range_layer: bool = False,
    nav2_enable_ultrasonic_blob_layer: bool = False,
    nav2_ultrasonic_blob_on_global: bool = True,
    ultrasonic_emergency_observation_persistence: float = 0.28,
    ultrasonic_side_observation_persistence: float = 0.08,
    ultrasonic_global_emergency_observation_persistence: Optional[float] = None,
    ultrasonic_global_side_observation_persistence: Optional[float] = None,
    ultrasonic_emergency_obstacle_max_range: float = 0.65,
    ultrasonic_side_obstacle_max_range: float = 0.55,
    use_custom_bt_recovery_tree: bool = False,
    nav2_enable_stereo_blob_layer: bool = False,
    # stereo_depth_node marks low obstacles 0.28-1.10 m ahead within the
    # cameras' +-27-37 deg; clearing starts at 0.35 m, where every bearing is
    # inside that band, so a beam never erases a cell stereo did not check.
    stereo_obstacle_max_range: float = 1.4,
    stereo_clear_min_range: float = 0.35,
    low_obstacle_memory: bool = False,
):
    """Generate a modified Nav2 params file with namespace-prefixed frame names.

    With a namespace, every frame gets the ``{namespace}/`` prefix and the
    global costmap reads this robot's SLAM map ``/{namespace}/map``.
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
        ultra_abs = f'/{namespace}/ultrasonic_'
        ultra_list = ['l', 'f', 'r']
        for label in ultra_list:
            _rewrite_frame(params, 'topics', f'/ultrasonic_{label}', ultra_abs+label)
            _rewrite_frame(params, 'topics', f'ultrasonic_{label}', ultra_abs+label)
            _rewrite_frame(params, 'range_topic', f'/ultrasonic_{label}', ultra_abs+label)
            _rewrite_frame(params, 'range_topic', f'ultrasonic_{label}', ultra_abs+label)
        # SLAM map frame is {ns}/map; the global costmap reads this robot's map.
        _rewrite_frame(params, 'global_frame', 'map', f'{namespace}/map')
        _rewrite_frame(params, 'global_frame_id', 'map', f'{namespace}/map')
        _rewrite_frame(params, 'map_topic', '/map', f'/{namespace}/map')
        _rewrite_frame(params, 'map_topic', 'map', f'/{namespace}/map')

    if nav2_enable_range_layer:
        for costmap_root in ('local_costmap', 'global_costmap'):
            costmap_node = params.get(costmap_root, {}).get(costmap_root, {}).get('ros__parameters', {})
            if not isinstance(costmap_node, dict):
                continue
            plugins = costmap_node.get('plugins')
            if isinstance(plugins, list) and 'range_layer' not in plugins:
                plugins.append('range_layer')
            elif not isinstance(plugins, list):
                costmap_node['plugins'] = ['obstacle_layer', 'inflation_layer', 'range_layer']
            range_topics = ['ultrasonic_l', 'ultrasonic_f', 'ultrasonic_r']
            if namespace:
                range_topics = [
                    f'/{namespace}/ultrasonic_l',
                    f'/{namespace}/ultrasonic_f',
                    f'/{namespace}/ultrasonic_r',
                ]
            costmap_node['range_layer'] = {
                'plugin': 'nav2_costmap_2d::RangeSensorLayer',
                'enabled': True,
                'topics': range_topics,
                'clear_threshold': 0.20,
                'mark_threshold': 0.80,
                'clear_on_max_reading': True,
                'no_readings_timeout': 0.5,
                'phi': 1.2,
                'inflate_cone': 1.0,
            }

    if nav2_enable_ultrasonic_blob_layer:
        blob_emergency_topic = 'ultrasonic_blob_scan_emergency'
        blob_side_topic = 'ultrasonic_blob_scan_side'
        if namespace:
            blob_emergency_topic = f'/{namespace}/ultrasonic_blob_scan_emergency'
            blob_side_topic = f'/{namespace}/ultrasonic_blob_scan_side'

        costmap_targets = ['local_costmap']
        if nav2_ultrasonic_blob_on_global:
            costmap_targets.append('global_costmap')

        global_em_persist = ultrasonic_global_emergency_observation_persistence
        if global_em_persist is None:
            global_em_persist = ultrasonic_emergency_observation_persistence
        global_side_persist = ultrasonic_global_side_observation_persistence
        if global_side_persist is None:
            global_side_persist = ultrasonic_side_observation_persistence

        for costmap_root in costmap_targets:
            costmap_node = params.get(costmap_root, {}).get(costmap_root, {}).get('ros__parameters', {})
            if not isinstance(costmap_node, dict):
                continue
            obstacle = costmap_node.get('obstacle_layer')
            if not isinstance(obstacle, dict):
                continue
            if costmap_root == 'local_costmap':
                em_persist = ultrasonic_emergency_observation_persistence
                side_persist = ultrasonic_side_observation_persistence
            else:
                em_persist = global_em_persist
                side_persist = global_side_persist

            obs_sources = str(obstacle.get('observation_sources', 'scan')).split()
            for source in ('ultrasonic_blob_emergency', 'ultrasonic_blob_side'):
                if source not in obs_sources:
                    obs_sources.append(source)
            obstacle['observation_sources'] = ' '.join(obs_sources)
            obstacle['ultrasonic_blob_emergency'] = {
                'topic': blob_emergency_topic,
                'max_obstacle_height': 2.0,
                'clearing': False,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': ultrasonic_emergency_obstacle_max_range,
                'raytrace_min_range': 0.02,
                'obstacle_max_range': ultrasonic_emergency_obstacle_max_range,
                'obstacle_min_range': 0.02,
                'observation_persistence': em_persist,
                'expected_update_rate': 0.0
            }
            obstacle['ultrasonic_blob_side'] = {
                'topic': blob_side_topic,
                'max_obstacle_height': 2.0,
                'clearing': False,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': ultrasonic_side_obstacle_max_range,
                'raytrace_min_range': 0.02,
                'obstacle_max_range': ultrasonic_side_obstacle_max_range,
                'obstacle_min_range': 0.02,
                'observation_persistence': side_persist,
                'expected_update_rate': 0.0
            }

    if nav2_enable_stereo_blob_layer:
        # Stereo gets its own ObstacleLayer in both costmaps. Sharing the lidar
        # layer would let lidar beams, which pass over a low object, clear it.
        # The global costmap is fixed to the map frame, so its marks are the
        # robot's memory: an object seen on route A stays until stereo sees
        # floor through it again, and the planner and the explorer's path
        # precheck route around it meanwhile. The SLAM map is never touched.
        prefix = f'/{namespace}/' if namespace else ''
        stereo_layer = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'stereo_mark stereo_clear',
            'footprint_clearing_enabled': True,
            # Marks: confirmed low obstacles and drop edges.
            'stereo_mark': {
                'topic': f'{prefix}stereo_blob_scan',
                'data_type': 'LaserScan',
                'marking': True,
                'clearing': False,
                'max_obstacle_height': 2.0,
                'obstacle_min_range': 0.05,
                'obstacle_max_range': stereo_obstacle_max_range,
                'observation_persistence': 0.0,
                'expected_update_rate': 0.0,
                'inf_is_valid': False,
            },
            # Clears: beams where the floor was matched. It starts at
            # stereo_clear_min_range, the nearest range stereo checks for
            # obstacles, so closer remembered cells are not erased by
            # a beam that never looked at them.
            'stereo_clear': {
                'topic': f'{prefix}stereo_clear_scan',
                'data_type': 'LaserScan',
                'marking': False,
                'clearing': True,
                'max_obstacle_height': 2.0,
                'raytrace_min_range': stereo_clear_min_range,
                'raytrace_max_range': stereo_obstacle_max_range,
                'observation_persistence': 0.0,
                'expected_update_rate': 0.0,
                'inf_is_valid': False,
            },
        }
        for section in ('local_costmap', 'global_costmap'):
            costmap_node = params.get(section, {}).get(section, {}).get('ros__parameters', {})
            if not isinstance(costmap_node, dict):
                continue
            plugins = list(costmap_node.get('plugins', []))
            if 'stereo_layer' not in plugins:
                # Before inflation so stereo marks get the same safety halo.
                at = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
                plugins.insert(at, 'stereo_layer')
            costmap_node['plugins'] = plugins
            costmap_node['stereo_layer'] = copy.deepcopy(stereo_layer)

    if low_obstacle_memory:
        # low_obstacle_memory.py keeps its own record of low obstacles and
        # bumps and republishes it every second as low_obstacle_points, so it
        # is back in the costmap right after Nav2's recoveries clear the
        # layers. As an ordinary obstacle layer before inflation it gets the
        # stock costs: lethal on the cells, inscribed (253) within the robot
        # radius, which spin and backup still accept. (A keepout filter mask
        # can only say lethal or <= 251, and its lethal ring trapped the robot
        # next to an obstacle.) stereo_clear_scan clears it like stereo_layer.
        prefix = f'/{namespace}/' if namespace else '/'
        memory_layer = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'memory_mark memory_clear',
            'footprint_clearing_enabled': True,
            'memory_mark': {
                'topic': f'{prefix}low_obstacle_points',
                'data_type': 'PointCloud2',
                'marking': True,
                'clearing': False,
                'min_obstacle_height': 0.0,
                'max_obstacle_height': 2.0,
                # Points are in the map frame, so "range" is from the map
                # origin; the node already limits them to 6 m of the robot.
                'obstacle_min_range': 0.0,
                'obstacle_max_range': 1000.0,
                'observation_persistence': 0.0,
                'expected_update_rate': 0.0,
            },
            'memory_clear': {
                'topic': f'{prefix}stereo_clear_scan',
                'data_type': 'LaserScan',
                'marking': False,
                'clearing': True,
                'max_obstacle_height': 2.0,
                'raytrace_min_range': stereo_clear_min_range,
                'raytrace_max_range': stereo_obstacle_max_range,
                'observation_persistence': 0.0,
                'expected_update_rate': 0.0,
                'inf_is_valid': False,
            },
        }
        for section in ('local_costmap', 'global_costmap'):
            costmap_node = params.get(section, {}).get(section, {}).get('ros__parameters', {})
            if not isinstance(costmap_node, dict):
                continue
            plugins = list(costmap_node.get('plugins', []))
            if 'memory_layer' not in plugins:
                at = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
                plugins.insert(at, 'memory_layer')
            costmap_node['plugins'] = plugins
            costmap_node['memory_layer'] = copy.deepcopy(memory_layer)

    if use_custom_bt_recovery_tree:
        # Optional custom BT; baseline mode keeps stock Nav2 behavior tree.
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
    enable_ultrasonic_cmd_vel_enforcer_str = LaunchConfiguration(
        'enable_ultrasonic_cmd_vel_enforcer').perform(context)
    ultrasonic_stop_hold_sec_str = LaunchConfiguration(
        'ultrasonic_stop_hold_sec').perform(context)
    ultrasonic_stop_guarded_max_blob_dist_m_str = LaunchConfiguration(
        'ultrasonic_stop_guarded_max_blob_dist_m').perform(context)
    retrace_collision_retry_window_sec_str = LaunchConfiguration(
        'retrace_collision_retry_window_sec').perform(context)
    retrace_collision_retry_min_events_str = LaunchConfiguration(
        'retrace_collision_retry_min_events').perform(context)
    retrace_collision_retry_cooldown_sec_str = LaunchConfiguration(
        'retrace_collision_retry_cooldown_sec').perform(context)
    retrace_retry_zone_radius_m_str = LaunchConfiguration(
        'retrace_retry_zone_radius_m').perform(context)
    retrace_retry_zone_decay_sec_str = LaunchConfiguration(
        'retrace_retry_zone_decay_sec').perform(context)
    retrace_retry_zone_hard_block_sec_str = LaunchConfiguration(
        'retrace_retry_zone_hard_block_sec').perform(context)
    retrace_retry_zone_extra_events_max_str = LaunchConfiguration(
        'retrace_retry_zone_extra_events_max').perform(context)
    retrace_retry_zone_repeated_hits_min_str = LaunchConfiguration(
        'retrace_retry_zone_repeated_hits_min').perform(context)
    retrace_retry_zone_nonrepeated_extra_events_max_str = LaunchConfiguration(
        'retrace_retry_zone_nonrepeated_extra_events_max').perform(context)
    debug_log_dir = LaunchConfiguration('debug_log_dir').perform(context)
    debug_log_rate_hz = LaunchConfiguration('debug_log_rate_hz').perform(context)
    fleet_mode_str = LaunchConfiguration('fleet_mode').perform(context)
    slam_toolbox_mode_str = LaunchConfiguration('slam_toolbox_mode').perform(
        context)
    scan_costmap_max_hz_str = LaunchConfiguration(
        'scan_costmap_max_hz').perform(context)
    enable_startup_map_seeding_str = LaunchConfiguration(
        'enable_startup_map_seeding').perform(context)
    startup_map_seed_wait_timeout_sec_str = LaunchConfiguration(
        'startup_map_seed_wait_timeout_sec').perform(context)
    startup_map_seed_publish_hz_str = LaunchConfiguration(
        'startup_map_seed_publish_hz').perform(context)
    ultrasonic_profile_str = LaunchConfiguration(
        'ultrasonic_profile').perform(context)
    nav2_enable_range_layer_str = LaunchConfiguration(
        'nav2_enable_range_layer').perform(context)
    ultrasonic_use_left_str = LaunchConfiguration(
        'ultrasonic_use_left').perform(context)
    ultrasonic_use_front_str = LaunchConfiguration(
        'ultrasonic_use_front').perform(context)
    ultrasonic_use_right_str = LaunchConfiguration(
        'ultrasonic_use_right').perform(context)
    ultrasonic_overlap_arbitration_enabled_str = LaunchConfiguration(
        'ultrasonic_overlap_arbitration_enabled').perform(context)
    ultrasonic_overlap_similarity_m_str = LaunchConfiguration(
        'ultrasonic_overlap_similarity_m').perform(context)
    ultrasonic_overlap_side_pair_front_scale_str = LaunchConfiguration(
        'ultrasonic_overlap_side_pair_front_scale').perform(context)
    ultrasonic_hard_max_range_m_str = LaunchConfiguration(
        'ultrasonic_hard_max_range_m').perform(context)
    ultrasonic_triangulation_enabled_str = LaunchConfiguration(
        'ultrasonic_triangulation_enabled').perform(context)
    ultrasonic_triangulation_similarity_m_str = LaunchConfiguration(
        'ultrasonic_triangulation_similarity_m').perform(context)
    ultrasonic_triangulation_blob_radius_m_str = LaunchConfiguration(
        'ultrasonic_triangulation_blob_radius_m').perform(context)
    nav2_enable_ultrasonic_blob_layer_str = LaunchConfiguration(
        'nav2_enable_ultrasonic_blob_layer').perform(context)
    ultrasonic_triangulation_max_age_sec_str = LaunchConfiguration(
        'ultrasonic_triangulation_max_age_sec').perform(context)
    ultrasonic_triangulation_require_pair_agreement_str = LaunchConfiguration(
        'ultrasonic_triangulation_require_pair_agreement').perform(context)
    ultrasonic_disable_scan_fusion_when_blob_str = LaunchConfiguration(
        'ultrasonic_disable_scan_fusion_when_blob').perform(context)
    ultrasonic_front_emergency_range_m_str = LaunchConfiguration(
        'ultrasonic_front_emergency_range_m').perform(context)
    ultrasonic_front_emergency_strict_range_m_str = LaunchConfiguration(
        'ultrasonic_front_emergency_strict_range_m').perform(context)
    ultrasonic_front_emergency_required_streak_str = LaunchConfiguration(
        'ultrasonic_front_emergency_required_streak').perform(context)
    ultrasonic_triangulation_blob_hold_sec_str = LaunchConfiguration(
        'ultrasonic_triangulation_blob_hold_sec').perform(context)
    ultrasonic_hard_block_duration_sec_str = LaunchConfiguration(
        'ultrasonic_hard_block_duration_sec').perform(context)
    ultrasonic_memory_decay_duration_sec_str = LaunchConfiguration(
        'ultrasonic_memory_decay_duration_sec').perform(context)
    ultrasonic_memory_replay_inward_m_str = LaunchConfiguration(
        'ultrasonic_memory_replay_inward_m').perform(context)
    ultrasonic_memory_replay_max_odom_travel_m_str = LaunchConfiguration(
        'ultrasonic_memory_replay_max_odom_travel_m').perform(context)
    ultrasonic_memory_replay_max_odom_yaw_rad_str = LaunchConfiguration(
        'ultrasonic_memory_replay_max_odom_yaw_rad').perform(context)
    ultrasonic_lateral_blob_cancel_min_odom_yaw_rad_str = LaunchConfiguration(
        'ultrasonic_lateral_blob_cancel_min_odom_yaw_rad').perform(context)
    ultrasonic_lateral_blob_cancel_min_abs_blob_angle_rad_str = LaunchConfiguration(
        'ultrasonic_lateral_blob_cancel_min_abs_blob_angle_rad').perform(context)
    ultrasonic_memory_replay_in_decay_str = LaunchConfiguration(
        'ultrasonic_memory_replay_in_decay').perform(context)
    ultrasonic_front_emergency_blob_radius_m_str = LaunchConfiguration(
        'ultrasonic_front_emergency_blob_radius_m').perform(context)
    ultrasonic_front_emergency_cone_scale_str = LaunchConfiguration(
        'ultrasonic_front_emergency_cone_scale').perform(context)
    ultrasonic_front_emergency_hold_sec_str = LaunchConfiguration(
        'ultrasonic_front_emergency_hold_sec').perform(context)
    ultrasonic_front_emergency_close_blob_dist_m_str = LaunchConfiguration(
        'ultrasonic_front_emergency_close_blob_dist_m').perform(context)
    ultrasonic_front_emergency_close_extra_hold_sec_str = LaunchConfiguration(
        'ultrasonic_front_emergency_close_extra_hold_sec').perform(context)
    ultrasonic_front_emergency_close_radius_scale_str = LaunchConfiguration(
        'ultrasonic_front_emergency_close_radius_scale').perform(context)
    ultrasonic_triangulation_similarity_scale_per_m_str = LaunchConfiguration(
        'ultrasonic_triangulation_similarity_scale_per_m').perform(context)
    ultrasonic_triangulation_similarity_max_m_str = LaunchConfiguration(
        'ultrasonic_triangulation_similarity_max_m').perform(context)
    ultrasonic_emergency_observation_persistence_str = LaunchConfiguration(
        'ultrasonic_emergency_observation_persistence_sec').perform(context)
    ultrasonic_side_observation_persistence_str = LaunchConfiguration(
        'ultrasonic_side_observation_persistence_sec').perform(context)
    ultrasonic_global_blob_emergency_persistence_sec_str = LaunchConfiguration(
        'ultrasonic_global_blob_emergency_persistence_sec').perform(context)
    ultrasonic_global_blob_side_persistence_sec_str = LaunchConfiguration(
        'ultrasonic_global_blob_side_persistence_sec').perform(context)
    ultrasonic_emergency_obstacle_max_range_str = LaunchConfiguration(
        'ultrasonic_emergency_obstacle_max_range_m').perform(context)
    ultrasonic_side_obstacle_max_range_str = LaunchConfiguration(
        'ultrasonic_side_obstacle_max_range_m').perform(context)
    nav2_ultrasonic_blob_on_global_str = LaunchConfiguration(
        'nav2_ultrasonic_blob_on_global').perform(context)
    use_custom_bt_recovery_tree_str = LaunchConfiguration(
        'use_custom_bt_recovery_tree').perform(context)
    ultrasonic_cooperative_mode_str = LaunchConfiguration(
        'ultrasonic_cooperative_mode').perform(context)
    orca_mode_str = LaunchConfiguration('orca_mode').perform(context)
    orca_max_linear_scale_str = LaunchConfiguration(
        'orca_max_linear_scale').perform(context)
    orca_stop_time_max_sec_str = LaunchConfiguration(
        'orca_stop_time_max_sec').perform(context)
    orca_min_predicted_separation_m_str = LaunchConfiguration(
        'orca_min_predicted_separation_m').perform(context)
    nav2_enable_stereo_blob_layer_str = LaunchConfiguration(
        'nav2_enable_stereo_blob_layer').perform(context)
    enable_stereo_cmd_vel_enforcer_str = LaunchConfiguration(
        'enable_stereo_cmd_vel_enforcer').perform(context)
    stereo_calibration_file_str = LaunchConfiguration(
        'stereo_calibration_file').perform(context)
    stereo_close_obstacle_m_str = LaunchConfiguration(
        'stereo_close_obstacle_m').perform(context)
    stereo_visual_odometry_str = LaunchConfiguration(
        'stereo_visual_odometry').perform(context)
    low_obstacle_memory_enabled = LaunchConfiguration(
        'low_obstacle_memory').perform(context).lower() in ('1', 'true', 'yes')
    stuck_detector_enabled = LaunchConfiguration(
        'enable_stuck_detector').perform(context).lower() in ('1', 'true', 'yes')
    stuck_snapshot_enabled = LaunchConfiguration(
        'enable_stuck_snapshot').perform(context).lower() in ('1', 'true', 'yes')

    # fleet: the central PC drives this robot (map_wire_z for Zenoh).
    # standalone: no central PC (static map -> <robot>/map for RViz goals).
    # `auto` is accepted as an old spelling of true.
    fleet_active = fleet_mode_str.lower() in ('1', 'true', 'yes', 'auto')

    try:
        scan_costmap_max_hz = float(scan_costmap_max_hz_str)
    except ValueError:
        scan_costmap_max_hz = 0.0
    costmap_scan_relay = bool(ns) and scan_costmap_max_hz > 0.0

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
        os.environ.get('TURTLEBOT3_WS', '~/turtlebot3'))
    repo_logs_dir = os.path.join(workspace_dir, 'logs')
    wait_tf_script = os.path.join(workspace_dir, 'scripts', 'env', 'wait_for_tf.py')
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

    stereo_ns = (ns or '').strip().strip('/')
    if stereo_calibration_file_str.strip():
        stereo_calib_path = os.path.expanduser(stereo_calibration_file_str.strip())
    else:
        stereo_calib_path = os.path.join(
            workspace_dir, 'config', 'stereo', f'{stereo_ns or "robot"}.yaml')
    stereo_calibrated = _stereo_calibration_usable(stereo_calib_path)
    stereo_blob_enabled = (
        nav2_enable_stereo_blob_layer_str.lower() in ('1', 'true', 'yes')
        and stereo_calibrated
    )
    stereo_stop_enabled = (
        enable_stereo_cmd_vel_enforcer_str.lower() in ('1', 'true', 'yes')
        and stereo_calibrated
    )

    actions = []

    # Cameras and the depth node start whenever two capture devices exist.
    # The blob scan stays empty until stereo_calib_path is a real calibration.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_nav2_launch_dir, 'stereo.launch.py')),
        launch_arguments={
            'namespace': ns,
            'calibration_file': stereo_calib_path,
            'close_obstacle_m': stereo_close_obstacle_m_str,
            'visual_odometry': stereo_visual_odometry_str,
        }.items(),
    ))
    actions.append(LogInfo(msg=(
        f'stereo calibration {stereo_calib_path}: '
        f'usable={stereo_calibrated} '
        f'local_costmap={stereo_blob_enabled} stop_gate={stereo_stop_enabled}'
    )))

    # --- Laser scan normalizer ---
    # Keep SLAM/Nav2 fed with a fixed-width scan stream even when the lidar
    # driver varies beam count slightly between packets.
    ultrasonic_profile = ultrasonic_profile_str.strip().lower()
    if ultrasonic_profile not in ('safe', 'aggressive', 'off'):
        ultrasonic_profile = 'safe'
    # Optional hardware is listed per robot in config/sensors/<robot>.yaml.
    # The OpenCR publishes ultrasonic ranges (a constant 3.0 m) even with no
    # sensors fitted, so a robot without them says so there; the fusion,
    # triangulation node and costmap layer are then left out. The
    # triangulation node alone used about a third of a core on Inky.
    has_ultrasonic = _robot_sensors(workspace_dir, ns).get('ultrasonic', True)
    if not has_ultrasonic:
        ultrasonic_profile = 'off'
    ultrasonic_blob_layer = (
        has_ultrasonic and nav2_enable_ultrasonic_blob_layer_str.lower() in ('1', 'true', 'yes'))
    ultrasonic_cooperative_mode = ultrasonic_cooperative_mode_str.lower() in (
        '1', 'true', 'yes')

    normalizer_params = {
        'input_topic': 'scan',
        'output_topic': 'scan_normalized',
        # LDS-02 beam count is not guaranteed to be 360 in this driver path.
        # Auto mode samples startup scan lengths and locks a stable target.
        'auto_target_readings': True,
        'auto_target_sample_scans': 30,
        'auto_target_min_samples': 12,
        'auto_target_outlier_tolerance': 12,
        # Fixed fallback/override when auto lock cannot be determined.
        'target_readings': 228,
        'range_topics': ['ultrasonic_l', 'ultrasonic_f', 'ultrasonic_r'],
        'range_frame_ids': (
            [f'{ns}/ultrasonic_link_left', f'{ns}/ultrasonic_link_front', f'{ns}/ultrasonic_link_right']
            if ns else
            ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right']
        ),
        # Small temporal/median filtering on HC-SR04-style readings reduces
        # transient echoes from glossy surfaces and stale obstacle ghosts.
        'ultrasonic_window_size': 3,
        'ultrasonic_max_age_sec': 0.40,
        'ultrasonic_min_valid_range': 0.02,
        'ultrasonic_max_valid_range': float(ultrasonic_hard_max_range_m_str),
        'ultrasonic_fusion_enabled': ultrasonic_profile != 'off',
        'ultrasonic_lidar_min_override_delta': 0.10,
        'ultrasonic_max_delta_per_update': 0.45,
        'ultrasonic_hysteresis_m': 0.03,
        'ultrasonic_max_hold_sec': 1.2,
        'ultrasonic_hold_epsilon_m': 0.008,
        'ultrasonic_cone_scale': 1.0,
        'ultrasonic_use_left': ultrasonic_use_left_str.lower() in ('1', 'true', 'yes'),
        'ultrasonic_use_front': ultrasonic_use_front_str.lower() in ('1', 'true', 'yes'),
        'ultrasonic_use_right': ultrasonic_use_right_str.lower() in ('1', 'true', 'yes'),
        'ultrasonic_overlap_arbitration_enabled': (
            ultrasonic_overlap_arbitration_enabled_str.lower() in ('1', 'true', 'yes')
        ),
        'ultrasonic_overlap_similarity_m': float(ultrasonic_overlap_similarity_m_str),
        'ultrasonic_overlap_side_pair_front_scale': float(
            ultrasonic_overlap_side_pair_front_scale_str
        ),
    }
    if (
        nav2_enable_ultrasonic_blob_layer_str.lower() in ('1', 'true', 'yes')
        and ultrasonic_disable_scan_fusion_when_blob_str.lower() in ('1', 'true', 'yes')
        and not ultrasonic_cooperative_mode
    ):
        normalizer_params['ultrasonic_fusion_enabled'] = False
    if ultrasonic_profile == 'aggressive':
        normalizer_params['ultrasonic_lidar_min_override_delta'] = 0.04
        normalizer_params['ultrasonic_max_delta_per_update'] = 0.70
        normalizer_params['ultrasonic_hysteresis_m'] = 0.01
        normalizer_params['ultrasonic_cone_scale'] = 1.25
    elif ultrasonic_profile == 'safe':
        normalizer_params['ultrasonic_lidar_min_override_delta'] = 0.12
        normalizer_params['ultrasonic_max_delta_per_update'] = 0.35
        normalizer_params['ultrasonic_hysteresis_m'] = 0.04
        normalizer_params['ultrasonic_cone_scale'] = 0.85
        normalizer_params['ultrasonic_overlap_similarity_m'] = max(
            float(ultrasonic_overlap_similarity_m_str), 0.10
        )
        normalizer_params['ultrasonic_overlap_side_pair_front_scale'] = min(
            float(ultrasonic_overlap_side_pair_front_scale_str), 0.60
        )
    if ultrasonic_cooperative_mode and has_ultrasonic:
        normalizer_params['ultrasonic_fusion_enabled'] = True
        normalizer_params['ultrasonic_max_hold_sec'] = min(
            float(normalizer_params['ultrasonic_max_hold_sec']), 0.40
        )
        normalizer_params['ultrasonic_cone_scale'] = min(
            float(normalizer_params['ultrasonic_cone_scale']), 0.90
        )
    if ns:
        normalizer_params['frame_id_prefix'] = ns
    if costmap_scan_relay:
        # Costmaps get a slower copy of the scan; SLAM keeps the full rate.
        normalizer_params['costmap_output_topic'] = 'scan_costmap'
        normalizer_params['costmap_max_hz'] = scan_costmap_max_hz

    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='normalize_laser_scan.py',
        name='laser_scan_normalizer',
        namespace=ns if ns else None,
        parameters=[normalizer_params],
        remappings=tf_remappings,
        output='screen',
    ))

    # Triangulation turns overlapping ultrasonic returns into compact blob scans
    # that Nav2 can consume through standard obstacle-layer interfaces.
    tri_params = {
            'range_topics': ['ultrasonic_l', 'ultrasonic_f', 'ultrasonic_r'],
            'range_frame_ids': (
                [f'{ns}/ultrasonic_link_left', f'{ns}/ultrasonic_link_front', f'{ns}/ultrasonic_link_right']
                if ns else
                ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right']
            ),
            'base_scan_frame': f'{ns}/base_scan' if ns else 'base_scan',
            'output_scan_topic': 'ultrasonic_blob_scan',
            'output_scan_topic_emergency': 'ultrasonic_blob_scan_emergency',
            'output_scan_topic_side': 'ultrasonic_blob_scan_side',
            'debug_topic': 'ultrasonic_triangulation_debug',
            'triangulation_enabled': (
                ultrasonic_triangulation_enabled_str.lower() in ('1', 'true', 'yes')
            ),
            'max_valid_range_m': float(ultrasonic_hard_max_range_m_str),
            'similarity_m': float(ultrasonic_triangulation_similarity_m_str),
            'similarity_scale_per_m': float(ultrasonic_triangulation_similarity_scale_per_m_str),
            'similarity_max_m': float(ultrasonic_triangulation_similarity_max_m_str),
            'blob_radius_m': float(ultrasonic_triangulation_blob_radius_m_str),
            'max_age_sec': float(ultrasonic_triangulation_max_age_sec_str),
            'require_pair_agreement': (
                ultrasonic_triangulation_require_pair_agreement_str.lower() in ('1', 'true', 'yes')
            ),
            'front_emergency_range_m': float(ultrasonic_front_emergency_range_m_str),
            'front_emergency_strict_range_m': float(ultrasonic_front_emergency_strict_range_m_str),
            'front_emergency_required_streak': int(ultrasonic_front_emergency_required_streak_str),
            'front_emergency_blob_radius_m': float(ultrasonic_front_emergency_blob_radius_m_str),
            'front_emergency_cone_scale': float(ultrasonic_front_emergency_cone_scale_str),
            'front_emergency_head_on_angle_rad': 0.45,
            'front_emergency_head_on_cone_boost': 1.10,
            'front_emergency_head_on_cone_max': 1.62,
            'front_emergency_hold_sec': float(ultrasonic_front_emergency_hold_sec_str),
            'front_emergency_close_blob_dist_m': float(
                ultrasonic_front_emergency_close_blob_dist_m_str
            ),
            'front_emergency_close_extra_hold_sec': float(
                ultrasonic_front_emergency_close_extra_hold_sec_str
            ),
            'front_emergency_close_radius_scale': float(
                ultrasonic_front_emergency_close_radius_scale_str
            ),
            'hard_block_duration_sec': float(ultrasonic_hard_block_duration_sec_str),
            'memory_decay_duration_sec': float(ultrasonic_memory_decay_duration_sec_str),
            'memory_replay_inward_m': float(ultrasonic_memory_replay_inward_m_str),
            'memory_replay_max_odom_travel_m': float(
                ultrasonic_memory_replay_max_odom_travel_m_str
            ),
            'memory_replay_max_odom_yaw_rad': float(
                ultrasonic_memory_replay_max_odom_yaw_rad_str
            ),
            'lateral_blob_cancel_min_odom_yaw_rad': float(
                ultrasonic_lateral_blob_cancel_min_odom_yaw_rad_str
            ),
            'lateral_blob_cancel_min_abs_blob_angle_rad': float(
                ultrasonic_lateral_blob_cancel_min_abs_blob_angle_rad_str
            ),
            'memory_replay_in_decay': (
                ultrasonic_memory_replay_in_decay_str.lower() in ('1', 'true', 'yes')
            ),
            'blob_hold_sec': float(ultrasonic_triangulation_blob_hold_sec_str),
    }
    if ultrasonic_cooperative_mode:
        tri_params['hard_block_duration_sec'] = 0.0
        tri_params['memory_decay_duration_sec'] = 0.0
        tri_params['memory_replay_inward_m'] = 0.0
        tri_params['memory_replay_in_decay'] = False
        tri_params['blob_hold_sec'] = min(float(tri_params['blob_hold_sec']), 0.20)
        tri_params['front_emergency_hold_sec'] = min(
            float(tri_params['front_emergency_hold_sec']), 0.35
        )

    if has_ultrasonic:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='ultrasonic_triangulation_blob.py',
            name='ultrasonic_triangulation_blob',
            namespace=ns if ns else None,
            parameters=[tri_params],
            remappings=tf_remappings,
            output='screen',
        ))
    actions.append(LogInfo(msg=(
        f'sensors: ultrasonic={has_ultrasonic} '
        f'({os.path.join(workspace_dir, "config", "sensors", (ns or "robot") + ".yaml")})')))

    if enable_startup_map_seeding_str.lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='startup_map_seeder.py',
            name='startup_map_seeder',
            namespace=ns if ns else None,
            parameters=[{
                'controller_server_node': 'controller_server',
                'wait_timeout_sec': float(startup_map_seed_wait_timeout_sec_str),
                'publish_hz': float(startup_map_seed_publish_hz_str),
                'cmd_topic': 'cmd_vel',
            }],
            output='screen',
        ))

    # Standalone namespaced robot: SLAM uses {ns}/map but RViz goals use
    # world frame "map". Publish map -> {ns}/map on /{ns}/tf_static. In fleet
    # mode the central PC owns "map" (map_merge), so this stays off.
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

    # Fleet mode: keep a compressed side channel available for central bridging
    # so OccupancyGrid traffic does not have to cross Wi-Fi uncompressed.
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
    # Bringup intentionally gates Nav2 on the minimum TF needed for whichever
    # topology is active, rather than relying on a fixed startup delay.
    wait_tf_env = dict(os.environ)
    wait_tf_env['TF_WAIT_ODOM_ONLY'] = 'true'
    wait_tf_env['TF_WAIT_STABLE_SAMPLES'] = '2'
    if ns:
        wait_tf_env['TF_WAIT_ODOM_FRAME'] = f'{ns}/odom'
        wait_tf_env['TF_WAIT_BASE_FRAMES'] = f'{ns}/base_footprint,{ns}/base_link'
        wait_tf_env['TF_WAIT_NAMESPACE'] = ns
        wait_tf_env['TF_WAIT_MAP_FRAME'] = f'{ns}/map'

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
    # The params file is rewritten per launch so the same YAML can serve
    # standalone robots, namespaced fleet robots, and central-map variants.
    def _optional_same_as_local_persist(raw: str):
        s = (raw or '').strip().lower()
        if s in ('', 'auto', 'use_local', 'same'):
            return None
        return float(s)

    nav2_params_file = _generate_nav2_params(
        params_file, ns,
        costmap_scan_relay=costmap_scan_relay,
        nav2_enable_range_layer=(
            nav2_enable_range_layer_str.lower() in ('1', 'true', 'yes')),
        nav2_enable_ultrasonic_blob_layer=ultrasonic_blob_layer,
        nav2_ultrasonic_blob_on_global=(
            nav2_ultrasonic_blob_on_global_str.lower() in ('1', 'true', 'yes')),
        ultrasonic_emergency_observation_persistence=float(
            ultrasonic_emergency_observation_persistence_str
        ),
        ultrasonic_side_observation_persistence=float(
            ultrasonic_side_observation_persistence_str
        ),
        ultrasonic_global_emergency_observation_persistence=_optional_same_as_local_persist(
            ultrasonic_global_blob_emergency_persistence_sec_str
        ),
        ultrasonic_global_side_observation_persistence=_optional_same_as_local_persist(
            ultrasonic_global_blob_side_persistence_sec_str
        ),
        ultrasonic_emergency_obstacle_max_range=float(
            ultrasonic_emergency_obstacle_max_range_str
        ),
        ultrasonic_side_obstacle_max_range=float(
            ultrasonic_side_obstacle_max_range_str
        ),
        use_custom_bt_recovery_tree=(
            use_custom_bt_recovery_tree_str.lower() in ('1', 'true', 'yes')),
        nav2_enable_stereo_blob_layer=stereo_blob_enabled,
        low_obstacle_memory=low_obstacle_memory_enabled,
    )

    map_frame_for_nav = f'{ns}/map' if ns else 'map'

    nav2_launch_source = PythonLaunchDescriptionSource(
        os.path.join(turtlebot3_nav2_launch_dir, 'navigation_launch_multirobot.py'))
    nav2_launch_args = {
        'namespace': ns,
        'use_sim_time': use_sim_time_str,
        'params_file': nav2_params_file,
        'autostart': 'True',
    }

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

    if wait_tf_proc is not None:
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
        remappings=tf_remappings,
        output='screen',
        condition=IfCondition(enable_debug_logging_str),
    ))

    # --- Publish whether the robot is in Nav2 lethal costmap space ---
    # This gives robot- and central-side helpers a cheap signal that the robot
    # is effectively starting inside inflated / blocked space.
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
        remappings=tf_remappings,
        output='screen',
        condition=IfCondition(enable_lethal_watch_str),
    ))

    # --- Memory-based retrace escape for lethal-start / progress timeout ---
    # This recovery helper is separate from the BT so it can watch action
    # status, costmap state, and recent pose history in one place.
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
                'collision_ahead_topic': 'nav2_collision_ahead',
                'retrace_active_topic': 'nav2_retrace_active',
                'enable_collision_retry_guard': True,
                'collision_retry_window_sec': float(retrace_collision_retry_window_sec_str),
                'collision_retry_min_events': int(retrace_collision_retry_min_events_str),
                'collision_retry_min_goal_age_sec': 6.0,
                'collision_retry_cooldown_sec': float(retrace_collision_retry_cooldown_sec_str),
                'retry_zone_radius_m': float(retrace_retry_zone_radius_m_str),
                'retry_zone_decay_sec': float(retrace_retry_zone_decay_sec_str),
                'retry_zone_hard_block_sec': float(retrace_retry_zone_hard_block_sec_str),
                'retry_zone_extra_events_max': int(retrace_retry_zone_extra_events_max_str),
                'retry_zone_repeated_hits_min': int(retrace_retry_zone_repeated_hits_min_str),
                'retry_zone_nonrepeated_extra_events_max': int(
                    retrace_retry_zone_nonrepeated_extra_events_max_str
                ),
            },
        ],
        remappings=tf_remappings,
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

    # --- Hard-stop cmd_vel override from ultrasonic hazard clusters ---
    # Keep this as a narrow emergency override; normal avoidance should still
    # come from costmaps and the controller whenever possible.
    actions.append(Node(
        package='turtlebot3_navigation2',
        executable='ultrasonic_cmd_vel_enforcer.py',
        name='ultrasonic_cmd_vel_enforcer',
        namespace=ns if ns else None,
        parameters=[{
            'triangulation_debug_topic': 'ultrasonic_triangulation_debug',
            'cmd_vel_topic': 'cmd_vel',
            'publish_hz': 24.0,
            'hold_sec': float(ultrasonic_stop_hold_sec_str),
            'hazard_clusters': [
                'front_emergency',
                'front_emergency_held',
                'front_emergency_replay',
            ],
            'always_stop_clusters': [
                'front_emergency',
                'front_emergency_held',
                'front_emergency_replay',
            ],
            # launch_ros parameter evaluation rejects empty list here (normalized as ()).
            # Use a sentinel cluster that never appears in triangulation output.
            'guarded_stop_clusters': ['__disabled__'],
            'guarded_stop_max_blob_dist_m': float(
                ultrasonic_stop_guarded_max_blob_dist_m_str
            ),
        }],
        output='screen',
        condition=IfCondition(enable_ultrasonic_cmd_vel_enforcer_str),
    ))

    # Stereo stop is armed only after a usable calibration file exists.
    # One bad frame is ignored inside the enforcer (required_streak >= 2).
    if stereo_stop_enabled:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='stereo_cmd_vel_enforcer.py',
            name='stereo_cmd_vel_enforcer',
            namespace=ns if ns else None,
            parameters=[{
                'debug_topic': 'stereo/debug',
                'cmd_vel_topic': 'cmd_vel',
                'publish_hz': 20.0,
                'hold_sec': 0.50,
                'close_obstacle_m': float(stereo_close_obstacle_m_str),
                'required_streak': 2,
            }],
            output='screen',
        ))

    # Stuck detection: cmd_vel vs wheels, wheels vs LiDAR / stereo VO, turn
    # vs gyro. A stall or slip publishes bump_points for the memory below.
    if stuck_detector_enabled:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='stuck_detector.py',
            name='stuck_detector',
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            namespace=ns if ns else None,
            output='screen',
        ))
    if stuck_snapshot_enabled:
        # When "collision ahead" persists, log which layer (LiDAR, stereo,
        # memory) owns the lethal cells in front and save a snapshot. Off by
        # default: listening to /rosout costs about a tenth of a core.
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='stuck_snapshot.py',
            name='stuck_snapshot',
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            namespace=ns if ns else None,
            remappings=tf_remappings,
            output='screen',
        ))

    # Low-obstacle memory: stereo marks (extended behind the visible face)
    # and bumps, fed to both costmaps as a keepout mask.
    if low_obstacle_memory_enabled:
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='low_obstacle_memory.py',
            name='low_obstacle_memory',
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            namespace=ns if ns else None,
            remappings=tf_remappings,
            output='screen',
        ))

    # --- ORCA shadow/advisory module (experimental, off by default) ---
    # It only publishes cmd_vel_orca_advised / orca_shadow_debug; nothing in
    # the drive path reads them, so it runs only when asked for.
    orca_mode = orca_mode_str.strip().lower()
    if orca_mode in ('shadow', 'advisory'):
        actions.append(Node(
            package='turtlebot3_navigation2',
            executable='orca_shadow_advisor.py',
            name='orca_shadow_advisor',
            namespace=ns if ns else None,
            parameters=[{
                'mode': orca_mode,
                'max_linear_scale': float(orca_max_linear_scale_str),
                'stop_time_max_sec': float(orca_stop_time_max_sec_str),
                'min_predicted_separation_m': float(orca_min_predicted_separation_m_str),
                'input_cmd_topic': 'cmd_vel_nav',
                'output_cmd_topic': 'cmd_vel_orca_advised',
                'scan_topic': 'scan_normalized',
                'triangulation_debug_topic': 'ultrasonic_triangulation_debug',
                'nav_path_topic': 'plan',
                'log_topic': 'orca_shadow_debug',
            }],
            output='screen',
        ))

    # --- RViz (optional) ---
    if use_rviz_str.lower() == 'true':
        # RViz config uses absolute topic names like `/map` and `/map_updates`.
        # When running SLAM/Nav2 under a namespace (e.g. `/pinky`), remap those
        # topics so the displayed map matches the robot's published map.
        rviz_remappings = []
        if ns:
            rviz_remappings = tf_remappings + [
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
            'debug_log_dir', default_value='~/turtlebot3/logs',
            description='Directory for nav2_motion_debug_logger JSONL output'),
        DeclareLaunchArgument(
            'debug_log_rate_hz', default_value='5.0',
            description='Debug logger sampling rate in Hz'),
        DeclareLaunchArgument(
            'enable_lethal_watch', default_value='false',
            description=(
                'Publish /<robot>/nav2_lethal_inflation from global costmap. Leave off with the '
                'stuck detector on: stuck_detector publishes the same topic (idle where Nav2 '
                'cannot plan) without reading the costmap.')),
        DeclareLaunchArgument(
            'enable_retrace_escape', default_value='false',
            description=(
                'Enable robot-side memory retrace helper; publishes '
                '/<robot>/nav2_retrace_active and sends retreat goals on lethal/stall.')),
        DeclareLaunchArgument(
            'enable_controller_collision_watch', default_value='false',
            description=(
                'Publish /<robot>/nav2_collision_ahead from controller_server '
                'rosout lines (e.g. RPP collision ahead)')),
        DeclareLaunchArgument(
            'enable_ultrasonic_cmd_vel_enforcer', default_value='false',
            description=(
                'Hard-stop cmd_vel while triangulation reports front_emergency / held / '
                'front_emergency_replay (stops Nav2 forward commands when cost blinks). '
                'Recommended true for repeated low-pipe or bag contacts.')),
        DeclareLaunchArgument(
            'use_custom_bt_recovery_tree', default_value='false',
            description=(
                'If true, force custom recovery BT tree override; default false uses '
                'stock Nav2 tree for baseline-stable behavior.')),
        DeclareLaunchArgument(
            'ultrasonic_cooperative_mode', default_value='true',
            description=(
                'Conservative LiDAR+ultrasonic cooperation profile: keep low-height '
                'evidence active while disabling long replay/hold behavior.')),
        DeclareLaunchArgument(
            'ultrasonic_stop_hold_sec', default_value='0.50',
            description='How long cmd_vel enforcer holds stop after ultrasonic hazard trigger (s).'),
        DeclareLaunchArgument(
            'ultrasonic_stop_guarded_max_blob_dist_m', default_value='0.28',
            description=(
                'Only stop on guarded hazard clusters if blob distance is below this threshold (m). '
                'Lower values reduce stutter in tight spaces.')),
        DeclareLaunchArgument(
            'fleet_mode', default_value='true',
            description=('true: driven by the central PC (publishes map_wire_z for Zenoh). '
                         'false: bench test without the central PC (static map -> <robot>/map).')),
        DeclareLaunchArgument(
            'slam_toolbox_mode', default_value='async',
            description='slam_toolbox node: async (default) or sync (mapper_params_online_sync).'),
        DeclareLaunchArgument(
            'scan_costmap_max_hz', default_value='6.0',
            description=(
                'Namespaced robots only: if > 0, the normalizer also publishes scan_costmap at '
                'this max rate (Hz) for Nav2 costmaps; SLAM stays on full-rate scan_normalized. '
                '0 feeds costmaps scan_normalized. Default 6.0 for Pi load.')),
        DeclareLaunchArgument(
            'ultrasonic_profile', default_value='safe',
            description=(
                'Ultrasonic+l lidar fusion profile for normalize_laser_scan.py: '
                'safe (default), aggressive, or off.')),
        DeclareLaunchArgument(
            'nav2_enable_range_layer', default_value='false',
            description=(
                'If true, inject Nav2 RangeSensorLayer (ultrasonic topics) into local/global '
                'costmaps for A/B testing. Default false keeps scan-fusion-only behavior.')),
        DeclareLaunchArgument(
            'ultrasonic_use_left', default_value='true',
            description='Enable left ultrasonic in scan fusion.'),
        DeclareLaunchArgument(
            'ultrasonic_use_front', default_value='true',
            description='Enable front ultrasonic in scan fusion.'),
        DeclareLaunchArgument(
            'ultrasonic_use_right', default_value='true',
            description='Enable right ultrasonic in scan fusion.'),
        DeclareLaunchArgument(
            'ultrasonic_overlap_arbitration_enabled', default_value='true',
            description=(
                'If true, arbitrate overlapping ultrasonic detections to reduce '
                'duplicate cone inflation (front-vs-side classification).')),
        DeclareLaunchArgument(
            'ultrasonic_overlap_similarity_m', default_value='0.10',
            description=(
                'Meters threshold for treating two ultrasonic readings as the same obstacle.')),
        DeclareLaunchArgument(
            'ultrasonic_overlap_side_pair_front_scale', default_value='0.60',
            description=(
                'When front+side agree, scale front cone width by this factor (0..1).')),
        DeclareLaunchArgument(
            'ultrasonic_hard_max_range_m', default_value='0.50',
            description='Hard max ultrasonic range for fusion/triangulation (meters).'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_enabled', default_value='true',
            description='Enable ultrasonic triangulation blob publisher.'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_similarity_m', default_value='0.07',
            description='Triangulation pair/triple similarity threshold (meters).'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_similarity_scale_per_m', default_value='0.18',
            description=(
                'Additional triangulation similarity tolerance per meter of front range '
                '(supports distance-scaled agreement).')),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_similarity_max_m', default_value='0.12',
            description='Upper bound for triangulation similarity threshold (meters).'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_blob_radius_m', default_value='0.11',
            description=(
                'Base triangulated blob radius (m); slightly larger improves side marking '
                'for low obstacles (pairs with side costmap max range / persistence).')),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_max_age_sec', default_value='0.15',
            description='Max age of ultrasonic samples used for triangulation (seconds).'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_require_pair_agreement', default_value='true',
            description='If true, only publish blobs when front+side pair agreement exists.'),
        DeclareLaunchArgument(
            'ultrasonic_disable_scan_fusion_when_blob', default_value='true',
            description='If true, disable legacy ultrasonic scan fusion when blob layer is enabled.'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_range_m', default_value='0.50',
            description='Enable front-only emergency blob when front range is below this distance (m).'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_strict_range_m', default_value='0.34',
            description=(
                'Within front_emergency_range_m but without side pair agreement: use full '
                'front_emergency only if front range is below this (m); otherwise use narrower '
                'front_hint (side costmap only) so Nav2 can find lateral clearance past soft '
                'obstacles like floor bags.')),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_required_streak', default_value='1',
            description='Consecutive front emergency samples required before publishing emergency blob.'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_blob_radius_m', default_value='0.20',
            description='Blob radius used for front emergency obstacle marking (meters).'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_cone_scale', default_value='1.35',
            description=(
                'Angular width scale for emergency (and close-range side) blob painting in '
                'LaserScan; >1 widens the arc (helps angled approaches to low obstacles).')),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_hold_sec', default_value='1.20',
            description='Hold duration (s) specifically for front_emergency blobs before release.'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_close_blob_dist_m', default_value='0.34',
            description=(
                'When front_emergency blob distance (m) is below this, add extra hold time '
                'and widen blob radius (pipe / final-approach reinforcement).')),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_close_extra_hold_sec', default_value='0.60',
            description='Extra hold (s) after close-range front_emergency when readings drop.'),
        DeclareLaunchArgument(
            'ultrasonic_front_emergency_close_radius_scale', default_value='1.18',
            description='Scale emergency blob radius when blob is inside close distance.'),
        DeclareLaunchArgument(
            'ultrasonic_triangulation_blob_hold_sec', default_value='0.20',
            description='How long to keep last triangulated blob when agreement drops briefly (s).'),
        DeclareLaunchArgument(
            'ultrasonic_emergency_observation_persistence_sec', default_value='0.28',
            description='Local costmap persistence (s) for emergency ultrasonic blobs.'),
        DeclareLaunchArgument(
            'ultrasonic_side_observation_persistence_sec', default_value='0.12',
            description=(
                'Local costmap persistence (s) for side/agreement ultrasonic blobs; '
                'slightly longer helps inscribed cost for go-around past low obstacles.')),
        DeclareLaunchArgument(
            'ultrasonic_emergency_obstacle_max_range_m', default_value='0.65',
            description='Obstacle/raytrace max range (m) for emergency ultrasonic costmap source.'),
        DeclareLaunchArgument(
            'ultrasonic_side_obstacle_max_range_m', default_value='0.55',
            description=(
                'Obstacle/raytrace max range (m) for side ultrasonic costmap source '
                '(wider arc for side triangulation marks).')),
        DeclareLaunchArgument(
            'retrace_collision_retry_window_sec', default_value='8.0',
            description='Window length (s) for counting collision-ahead retries.'),
        DeclareLaunchArgument(
            'retrace_collision_retry_min_events', default_value='5',
            description='Base collision-ahead events required before retry-zone retrace triggers.'),
        DeclareLaunchArgument(
            'retrace_collision_retry_cooldown_sec', default_value='12.0',
            description='Cooldown (s) between retry-zone retrace triggers.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_radius_m', default_value='0.45',
            description='Radius (m) used to match repeated collisions to the same retry zone.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_decay_sec', default_value='22.0',
            description='Decay duration (s) for retry-zone suppression after hard phase.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_hard_block_sec', default_value='6.0',
            description='Initial hard-block duration (s) for retry-zone suppression.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_extra_events_max', default_value='5',
            description='Max additional collision events required as retry-zone suppression fades.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_repeated_hits_min', default_value='2',
            description='Minimum zone hit count required before strong retry-zone suppression applies.'),
        DeclareLaunchArgument(
            'retrace_retry_zone_nonrepeated_extra_events_max', default_value='1',
            description='Max extra retry events for non-repeated zones (keeps corner escape responsive).'),
        DeclareLaunchArgument(
            'ultrasonic_hard_block_duration_sec', default_value='0.0',
            description='Duration of hard ultrasonic memory phase after front/front_emergency detection (s).'),
        DeclareLaunchArgument(
            'ultrasonic_memory_decay_duration_sec', default_value='0.0',
            description='Duration of ultrasonic decay-memory phase after hard phase expires (s).'),
        DeclareLaunchArgument(
            'ultrasonic_memory_replay_inward_m', default_value='0.08',
            description=(
                'Pull memory-replay obstacle toward the robot along the stored ray (m). '
                'Hard phase uses full value; decay uses half. Reduces creep-into-pipe when '
                'triangulated blob sits farther than the true frontal range.')),
        DeclareLaunchArgument(
            'ultrasonic_memory_replay_max_odom_travel_m', default_value='0.32',
            description=(
                'Cancel ultrasonic memory/replay after base moves this far in odom (m) '
                'from anchor (e.g. backed away from pipe). Reduces costmap ghost following.')),
        DeclareLaunchArgument(
            'ultrasonic_memory_replay_max_odom_yaw_rad', default_value='0.90',
            description='Cancel memory/replay after this yaw change (rad) from anchor.'),
        DeclareLaunchArgument(
            'ultrasonic_lateral_blob_cancel_min_odom_yaw_rad', default_value='0.32',
            description=(
                'For front_left / front_right / front_hint only: drop held blob and re-anchor '
                'odom after this yaw (rad) so small turns refresh body-fixed cost (go-around). '
                'Set 0 to disable. Skipped while front range is in emergency band or blob angle '
                'is below ultrasonic_lateral_blob_cancel_min_abs_blob_angle_rad. '
                'Does not apply to front_emergency or centered front.')),
        DeclareLaunchArgument(
            'ultrasonic_lateral_blob_cancel_min_abs_blob_angle_rad', default_value='0.48',
            description=(
                'Minimum |atan2(blob_y,blob_x)| (rad) before lateral yaw-cancel may run; keeps '
                'shallow angled pipe edges from losing marks. front_left/right can still cancel '
                'under this angle even if the front range is in the emergency band (bag go-around). '
                'Set 0 to disable this gate.')),
        DeclareLaunchArgument(
            'ultrasonic_memory_replay_in_decay', default_value='false',
            description=(
                'If true, keep publishing memory replay during decay phase; if false, '
                'replay only in hard phase (less phantom cost after escape turns).')),
        DeclareLaunchArgument(
            'orca_mode', default_value='off',
            description=('ORCA experiment: off (default, node not started), shadow (log only), '
                         'advisory (publish cmd_vel_orca_advised; nothing drives from it).')),
        DeclareLaunchArgument(
            'orca_max_linear_scale', default_value='0.55',
            description='Max linear scaling in advisory mode (0..1).'),
        DeclareLaunchArgument(
            'orca_stop_time_max_sec', default_value='0.80',
            description='Maximum stop-hold time commanded by ORCA advisory (seconds).'),
        DeclareLaunchArgument(
            'orca_min_predicted_separation_m', default_value='0.26',
            description='Predicted minimum separation threshold used for ORCA conflict checks (m).'),
        DeclareLaunchArgument(
            'nav2_enable_stereo_blob_layer', default_value='true',
            description=(
                'Inject stereo_blob_scan into the local costmap when '
                'config/stereo/<namespace>.yaml is a usable calibration. '
                'Does nothing without that file, and never writes the global costmap.')),
        DeclareLaunchArgument(
            'enable_stereo_cmd_vel_enforcer', default_value='true',
            description=(
                'Zero cmd_vel on a close stereo obstacle or a confirmed drop. '
                'Starts only when the stereo calibration file is usable.')),
        DeclareLaunchArgument(
            'stereo_calibration_file', default_value='',
            description=(
                'Override for ~/turtlebot3/config/stereo/<namespace>.yaml. '
                'Empty uses that default path.')),
        DeclareLaunchArgument(
            'enable_stuck_snapshot', default_value='false',
            description=(
                'Diagnostic: when "collision ahead" lasts 5 s, log which layer owns the '
                'blocking cells and save ~/.ros/stuck_snapshots/*.npz.')),
        DeclareLaunchArgument(
            'enable_stuck_detector', default_value='true',
            description=(
                'Detect driving without moving (stall, wheel slip, blocked turn); '
                'publishes stuck, stuck/event and bump_points.')),
        DeclareLaunchArgument(
            'low_obstacle_memory', default_value='true',
            description=(
                'Remember stereo low obstacles and bumps in a costmap layer on both costmaps '
                'that is re-marked after Nav2 costmap clears.')),
        DeclareLaunchArgument(
            'stereo_visual_odometry', default_value='true',
            description=(
                'Publish stereo visual odometry on odom/visual for the bringup EKF '
                '(needs a usable stereo calibration).')),
        DeclareLaunchArgument(
            'stereo_close_obstacle_m', default_value='0.45',
            description='Forward range (m) at which a stereo obstacle triggers the stop gate.'),
        DeclareLaunchArgument(
            'nav2_enable_ultrasonic_blob_layer', default_value='true',
            description='Inject ultrasonic_blob_scan into Nav2 obstacle_layer sources.'),
        DeclareLaunchArgument(
            'nav2_ultrasonic_blob_on_global', default_value='false',
            description=(
                'When nav2_enable_ultrasonic_blob_layer is true, also add emergency/side '
                'blob sources to global_costmap so ComputePathToPose detours around low '
                'obstacles (LiDAR may not see).')),
        DeclareLaunchArgument(
            'ultrasonic_global_blob_emergency_persistence_sec', default_value='auto',
            description=(
                'Global costmap observation_persistence for ultrasonic_blob_emergency. '
                'Use auto (default) to match ultrasonic_emergency_observation_persistence_sec; '
                'or a larger float for stabler global plans.')),
        DeclareLaunchArgument(
            'ultrasonic_global_blob_side_persistence_sec', default_value='auto',
            description=(
                'Global costmap observation_persistence for ultrasonic_blob_side. '
                'Use auto to match ultrasonic_side_observation_persistence_sec.')),
        DeclareLaunchArgument(
            'enable_startup_map_seeding', default_value='true',
            description=(
                'If true, run a one-shot startup cmd_vel seeding motion once '
                'controller_server is ACTIVE. Recommended before central start.')),
        DeclareLaunchArgument(
            'startup_map_seed_wait_timeout_sec', default_value='60.0',
            description='Max seconds startup_map_seeder waits for active controller_server.'),
        DeclareLaunchArgument(
            'startup_map_seed_publish_hz', default_value='10.0',
            description='Twist publish rate (Hz) for startup_map_seeder motion sequence.'),
        DeclareLaunchArgument(
            'effective_namespace', default_value=effective_namespace,
            description='(internal) resolved namespace'),
        OpaqueFunction(function=_launch_setup),
    ])
