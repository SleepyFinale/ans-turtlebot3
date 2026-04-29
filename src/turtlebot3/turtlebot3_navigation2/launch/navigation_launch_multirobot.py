# Fleet / central-PC variant of nav2 navigation_launch.py.
# Remaps tf -> /tf so namespaced Nav2 nodes receive the full TF tree from the
# central relay and map_merge: map -> blinky/map -> blinky/odom -> base, etc.
#
# Based on nav2_bringup/navigation_launch.py (Apache 2.0)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav_map_topic = LaunchConfiguration('nav_map_topic')
    nav_map_updates_topic = LaunchConfiguration('nav_map_updates_topic')
    use_isolated_container = LaunchConfiguration('use_isolated_container')

    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother'
    ]

    # bond_timeout <= 0 disables bond heartbeats (nav2_lifecycle_manager). With
    # component_container, preshutdown order is undefined: a node can destroy its bond
    # before the manager stops the bond timer — spurious CRITICAL FAILURE, conflicting
    # lifecycle transitions, and teardown SIGSEGV (-11).
    lifecycle_manager_extra_params = {'bond_timeout': 0.0}

    # Namespaced nodes default to /<ns>/tf (no world map -> ns/map). Use global
    # /tf from the central relay + map_merge (or single-robot map bridge).
    # Map: merged /map on the central PC when map_merge runs.
    # All Nav2 servers in this launch are expected to behave like fleet clients:
    # TF comes from the global graph, while the map topic can be the merged map,
    # a throttled relay, or a robot-local map chosen by the parent launch.
    remappings = [
        ('tf', '/tf'), ('tf_static', '/tf_static'),
        ('map', nav_map_topic), ('map_updates', nav_map_updates_topic),
    ]

    param_substitutions = {'use_sim_time': use_sim_time, 'autostart': autostart}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_composition', default_value='False'))
    ld.add_action(DeclareLaunchArgument('container_name', default_value='nav2_container'))
    ld.add_action(DeclareLaunchArgument('use_respawn', default_value='False'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))
    ld.add_action(DeclareLaunchArgument(
        'nav_map_topic', default_value='/map',
        description='Absolute topic Nav2 subscribes to for OccupancyGrid (e.g. /map_relay when throttling)'))
    ld.add_action(DeclareLaunchArgument(
        'nav_map_updates_topic', default_value='/map_updates',
        description='Absolute topic for map_updates remapping'))
    container_sigterm_timeout = LaunchConfiguration('container_sigterm_timeout')
    container_sigkill_timeout = LaunchConfiguration('container_sigkill_timeout')
    ld.add_action(DeclareLaunchArgument(
        'container_sigterm_timeout',
        default_value='30',
        description=(
            'Seconds after SIGINT before launch sends SIGTERM to the Nav2 '
            'component container (lifecycle shutdown is serialized; default '
            'launch 5s is often too short).')))
    ld.add_action(DeclareLaunchArgument(
        'container_sigkill_timeout',
        default_value='45',
        description=(
            'Seconds after SIGTERM before launch sends SIGKILL to the Nav2 '
            'component container.')))
    ld.add_action(DeclareLaunchArgument(
        'use_isolated_container',
        default_value='false',
        description=(
            'If true, use component_container_isolated (per-component executors). '
            'If false (default), use component_container — avoids known hangs on '
            'Ctrl+C where the isolated manager never exits after Nav2 cleanup '
            '(see ros2/rclcpp#2083 / #2085).')))

    # rclcpp component container when use_composition is true. Default is
    # component_container, not *_isolated, for reliable process exit on shutdown.
    container_common = dict(
        package='rclcpp_components',
        name=container_name,
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        output='screen',
        sigterm_timeout=container_sigterm_timeout,
        sigkill_timeout=container_sigkill_timeout,
    )
    ld.add_action(Node(
        condition=IfCondition(PythonExpression([
            "(", "'", use_composition,
            "'.strip().lower() in ('true', '1', 'yes')) and not (",
            "'", use_isolated_container,
            "'.strip().lower() in ('true', '1', 'yes'))",
        ])),
        executable='component_container',
        **container_common,
    ))
    ld.add_action(Node(
        condition=IfCondition(PythonExpression([
            "(", "'", use_composition,
            "'.strip().lower() in ('true', '1', 'yes')) and (",
            "'", use_isolated_container,
            "'.strip().lower() in ('true', '1', 'yes'))",
        ])),
        executable='component_container_isolated',
        **container_common,
    ))

    # Non-composed mode is the safer default on Raspberry Pi: each Nav2 server
    # gets its own process, which is easier to debug and less fragile on exit.
    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(package='nav2_controller', executable='controller_server', output='screen',
                 respawn=use_respawn, respawn_delay=2.0, parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
            Node(package='nav2_planner', executable='planner_server', name='planner_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                 output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower', output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', output='screen', respawn=use_respawn, respawn_delay=2.0,
                 parameters=[configured_params],
                 arguments=['--ros-args', '--log-level', log_level],
                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 arguments=['--ros-args', '--log-level', log_level],
                 parameters=[{
                     'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': lifecycle_nodes,
                     **lifecycle_manager_extra_params,
                 }]),
        ])

    # Composed mode keeps the same logical stack but loads the servers into one
    # container for users willing to trade debuggability for lower process count.
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer',
                          name='controller_server', parameters=[configured_params],
                          remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(package='nav2_smoother', plugin='nav2_smoother::SmootherServer',
                          name='smoother_server', parameters=[configured_params],
                          remappings=remappings),
            ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer',
                          name='planner_server', parameters=[configured_params],
                          remappings=remappings),
            ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
                          name='behavior_server', parameters=[configured_params],
                          remappings=remappings),
            ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
                          name='bt_navigator', parameters=[configured_params],
                          remappings=remappings),
            ComposableNode(package='nav2_waypoint_follower',
                          plugin='nav2_waypoint_follower::WaypointFollower',
                          name='waypoint_follower', parameters=[configured_params],
                          remappings=remappings),
            ComposableNode(package='nav2_velocity_smoother',
                          plugin='nav2_velocity_smoother::VelocitySmoother',
                          name='velocity_smoother', parameters=[configured_params],
                          remappings=remappings +
                          [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(package='nav2_lifecycle_manager',
                          plugin='nav2_lifecycle_manager::LifecycleManager',
                          name='lifecycle_manager_navigation',
                          parameters=[{
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'node_names': lifecycle_nodes,
                              **lifecycle_manager_extra_params,
                          }]),
        ])

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    return ld
