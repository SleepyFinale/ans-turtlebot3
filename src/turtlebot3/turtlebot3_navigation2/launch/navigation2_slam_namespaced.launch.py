#!/usr/bin/env python3
#
# Namespaced SLAM + Nav2 launch for a single TurtleBot3 robot.
# Runs laser scan normalizer, SLAM Toolbox, and Nav2 inside a per-robot
# namespace so that topics and TF frames are isolated per robot.
#
# Usage examples:
#   ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py \\
#       robot_name:=blinky use_sim_time:=false use_rviz:=false
#   ros2 launch turtlebot3_navigation2 navigation2_slam_namespaced.launch.py \\
#       namespace:=pinky wait_for_tf:=true
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
ROS_DISTRO = os.environ.get("ROS_DISTRO")


def generate_launch_description():
    # Default robot name from Linux username (so you normally don't need to
    # pass robot_name/namespace on the command line).
    default_robot_name = os.environ.get("USER", "")

    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")

    # If namespace is empty, fall back to robot_name.
    effective_namespace = PythonExpression(
        ['"', namespace, '" if "', namespace, '" != "" else "', robot_name, '"']
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    wait_for_tf = LaunchConfiguration("wait_for_tf", default="true")

    # Nav2 params: reuse the standard per-model YAML, but allow override.
    param_file_name = TURTLEBOT3_MODEL + ".yaml"
    if ROS_DISTRO == "humble":
        default_param_file = os.path.join(
            get_package_share_directory("turtlebot3_navigation2"),
            "param",
            ROS_DISTRO,
            param_file_name,
        )
    else:
        default_param_file = os.path.join(
            get_package_share_directory("turtlebot3_navigation2"),
            "param",
            param_file_name,
        )
    params_file = LaunchConfiguration(
        "params_file",
        default=default_param_file,
    )

    # Paths to helper scripts and Nav2 bringup.
    workspace_dir = os.path.expanduser(
        os.environ.get("TURTLEBOT3_WS", "~/turtlebot3_ws")
    )
    wait_tf_script = os.path.join(workspace_dir, "scripts", "wait_for_tf.py")
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    # Laser scan normalizer: run in the robot namespace so topics become:
    #   /<robot>/scan           (input)
    #   /<robot>/scan_normalized (output)
    normalizer_node = Node(
        package="turtlebot3_navigation2",
        executable="normalize_laser_scan.py",
        name="laser_scan_normalizer",
        namespace=effective_namespace,
        parameters=[
            {
                # Use relative topic names so they are resolved within the namespace.
                "input_topic": "scan",
                "output_topic": "scan_normalized",
                # Prefix frame_id to <namespace>/base_scan so each robot has unique scan frame.
                "frame_id_prefix": effective_namespace,
            }
        ],
        output="screen",
    )

    # SLAM Toolbox: also run inside the namespace, consuming the normalized scan.
    slam_params = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "param",
        "humble",
        "mapper_params_online_async_fast.yaml",
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=effective_namespace,
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # Within the namespace, this resolves to /<robot>/scan_normalized.
            ("/scan", "scan_normalized"),
        ],
    )

    # Wait for a usable TF tree before starting Nav2.
    wait_tf_env = dict(os.environ)
    # In SLAM mode, wait only for odom->base_*; map->odom will appear from SLAM.
    wait_tf_env["TF_WAIT_ODOM_ONLY"] = "true"
    # Allow overriding map/odom/base frames via env if needed; defaults are fine for
    # the per-robot tree when running inside a namespace.

    wait_tf_process = ExecuteProcess(
        cmd=["python3", wait_tf_script],
        output="screen",
        env=wait_tf_env,
        condition=IfCondition(wait_for_tf),
    )

    # Nav2 stack: include nav2_bringup's navigation_launch.py inside the same namespace
    # so actions, topics, and services are all under /<robot>/...
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, "/navigation_launch.py"]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )
    nav2_launch = TimerAction(
        period=3.0,
        actions=[
            GroupAction(
                [
                    PushRosNamespace(effective_namespace),
                    nav2_include,
                ]
            )
        ],
        condition=IfCondition(wait_for_tf),
    )

    # RViz is optional; typically run on the central computer instead.
    rviz_config_dir = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "rviz",
        "tb3_navigation2.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value=default_robot_name,
                description=(
                    "Logical robot name to use as the namespace when "
                    "namespace is not explicitly provided (e.g. blinky, pinky)."
                ),
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description=(
                    "Explicit ROS namespace for this robot. If non-empty, "
                    "this value takes precedence over robot_name."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_param_file,
                description="Full path to Nav2 params YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz2 on the robot if true.",
            ),
            DeclareLaunchArgument(
                "wait_for_tf",
                default_value="true",
                description=(
                    "Wait for odom->base_* TF to be ready before starting Nav2."
                ),
            ),
            normalizer_node,
            slam_node,
            wait_tf_process,
            nav2_launch,
            rviz_node,
        ]
    )

