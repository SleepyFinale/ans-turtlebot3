#!/usr/bin/env python3
"""Start a stereo pair when two V4L capture devices are present.

Metadata-only nodes (the extra /dev/video* entries USB cameras create) are
ignored. With fewer than two capture devices this launch starts nothing, so
robots that do not have a pair are unchanged.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_stereo_common():
    path = os.path.join(
        get_package_prefix('turtlebot3_navigation2'),
        'lib',
        'turtlebot3_navigation2',
        'stereo_common.py',
    )
    spec = importlib.util.spec_from_file_location('stereo_common', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Unable to load stereo helper at {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip().strip('/')
    calib_override = LaunchConfiguration('calibration_file').perform(context)
    width = int(LaunchConfiguration('image_width').perform(context))
    height = int(LaunchConfiguration('image_height').perform(context))
    pixel_format = LaunchConfiguration('pixel_format').perform(context).strip() or 'YUYV'
    swap = LaunchConfiguration('swap_cameras').perform(context).strip().lower() in (
        '1', 'true', 'yes',
    )
    close_obstacle_m = float(LaunchConfiguration('close_obstacle_m').perform(context))

    common = _load_stereo_common()
    devices = common.list_capture_devices()
    if len(devices) < 2:
        return [LogInfo(
            msg=(
                f'stereo: found {len(devices)} capture device(s) {devices}; '
                'not starting cameras or the depth node'
            )
        )]

    left_dev, right_dev = devices[0], devices[1]
    if swap:
        left_dev, right_dev = right_dev, left_dev

    calib_path = common.calibration_file_path(ns, calib_override)
    prefix = f'{ns}/' if ns else ''
    left_frame = f'{prefix}stereo_left_optical_frame'
    right_frame = f'{prefix}stereo_right_optical_frame'
    left_ns = f'{prefix}stereo_left'
    right_ns = f'{prefix}stereo_right'

    def _camera(device: str, namespace: str, frame_id: str, name: str) -> Node:
        return Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name=name,
            namespace=namespace,
            parameters=[{
                'video_device': device,
                'pixel_format': pixel_format,
                'output_encoding': 'rgb8',
                'image_size': [width, height],
                'camera_frame_id': frame_id,
            }],
            output='screen',
        )

    depth_params = {
        'left_image_topic': 'stereo_left/image_raw',
        'right_image_topic': 'stereo_right/image_raw',
        'calibration_file': calib_path,
        'left_frame': left_frame,
        'right_frame': right_frame,
        'parent_frame': f'{prefix}base_footprint',
        'image_width': width,
        'image_height': height,
        'close_obstacle_m': close_obstacle_m,
    }

    return [
        LogInfo(msg=(
            f'stereo: left={left_dev} right={right_dev} '
            f'size={width}x{height} calibration={calib_path}'
        )),
        _camera(left_dev, left_ns, left_frame, 'v4l2_camera'),
        _camera(right_dev, right_ns, right_frame, 'v4l2_camera'),
        Node(
            package='turtlebot3_navigation2',
            executable='stereo_depth_node.py',
            name='stereo_depth_node',
            namespace=ns if ns else None,
            parameters=[depth_params],
            output='screen',
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace. Empty runs the pair at the root.'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description=(
                'Override path for config/stereo/<namespace>.yaml. '
                'Empty uses ~/turtlebot3/config/stereo/<namespace>.yaml.'),
        ),
        DeclareLaunchArgument(
            'image_width', default_value='320',
            description='Capture width passed to both cameras.'),
        DeclareLaunchArgument(
            'image_height', default_value='240',
            description='Capture height passed to both cameras.'),
        DeclareLaunchArgument(
            'pixel_format', default_value='YUYV',
            description='V4L pixel format for both cameras.'),
        DeclareLaunchArgument(
            'swap_cameras', default_value='false',
            description='Swap the sorted capture devices if left/right are reversed.'),
        DeclareLaunchArgument(
            'close_obstacle_m', default_value='0.45',
            description='Range (m) at which consecutive stereo obstacles count as a close stop.'),
        OpaqueFunction(function=_setup),
    ])
