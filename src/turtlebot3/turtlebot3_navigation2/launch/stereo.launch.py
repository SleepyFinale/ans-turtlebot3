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
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# numpy's OpenBLAS starts a busy-waiting thread per core in every process.
# On a loaded Pi those threads fight SLAM and Nav2 and made a 4x4 inverse
# take 14 ms; the stereo nodes only do small matrix work.
_SINGLE_THREAD_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


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


def _calibration_focus(path: str):
    """Fixed focus (left, right) stored with the calibration, or None for each.

    Both Arducams default to continuous autofocus. The lens moves while the
    robot drives, which blurs the image and changes the focal length the
    calibration measured, so a calibrated pair runs at a fixed focus.
    """
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except Exception:
        return None, None
    out = []
    for key in ('left_focus_absolute', 'right_focus_absolute'):
        try:
            out.append(int(data[key]))
        except (KeyError, TypeError, ValueError):
            out.append(None)
    return out[0], out[1]


def _prepare_command(devices_and_focus):
    """One shell step that readies each camera before its driver opens it.

    UVC cameras time out (and can wedge until a USB reset) when controls are
    written while the driver is starting, so this runs alone and the camera
    nodes start after it exits. focus_absolute is ignored while autofocus is
    on, and v4l2_camera writes focus_absolute before autofocus, so autofocus
    has to be off already when the node starts.

    It also waits until each camera answers a query, then a second more. A
    camera that had just been touched (the focus writes, USB autosuspend)
    refused the driver's open with "Device or resource busy" in three of six
    launches on Inky, and v4l2_camera never retries: stereo then ran on one
    camera for the whole run.
    """
    steps = []
    for device, focus in devices_and_focus:
        steps.append(
            f'for i in $(seq 1 25); do v4l2-ctl -d {device} --get-fmt-video >/dev/null 2>&1 && break; '
            f'sleep 0.2; done'
        )
        if focus is None:
            continue
        steps.append(
            f'timeout 5 v4l2-ctl -d {device} -c focus_automatic_continuous=0 && '
            f'timeout 5 v4l2-ctl -d {device} -c focus_absolute={int(focus)} || '
            f'echo "could not lock focus on {device}; it stays on autofocus" >&2'
        )
    steps.append('sleep 1')
    return ExecuteProcess(cmd=['bash', '-c', '; '.join(steps)], output='screen')


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
    visual_odometry = LaunchConfiguration('visual_odometry').perform(context).strip().lower() in (
        '1', 'true', 'yes',
    )

    common = _load_stereo_common()
    cameras = common.list_stereo_cameras()
    if len(cameras) < 2:
        return [LogInfo(
            msg=(
                f'stereo: found {len(cameras)} camera(s) {cameras}; '
                'not starting cameras or the depth node'
            )
        )]

    left_cam, right_cam = cameras[0], cameras[1]
    if swap:
        left_cam, right_cam = right_cam, left_cam
    left_dev = left_cam['path']
    right_dev = right_cam['path']

    calib_path = common.calibration_file_path(ns, calib_override)
    prefix = f'{ns}/' if ns else ''
    left_frame = f'{prefix}stereo_left_optical_frame'
    right_frame = f'{prefix}stereo_right_optical_frame'
    left_ns = f'{prefix}stereo_left'
    right_ns = f'{prefix}stereo_right'

    left_focus, right_focus = _calibration_focus(calib_path)

    def _camera(device: str, namespace: str, frame_id: str, name: str, focus) -> ComposableNode:
        params = {
            'video_device': device,
            'pixel_format': pixel_format,
            # Every stereo consumer reads mono8. Asking the driver for it skips
            # an rgb8 conversion and sends a third of the bytes per frame.
            'output_encoding': 'mono8',
            'image_size': [width, height],
            'camera_frame_id': frame_id,
        }
        if focus is not None:
            # v4l2_camera writes every control back to its default on start,
            # which turns autofocus on again unless these are given.
            params['focus_automatic_continuous'] = False
            params['focus_absolute'] = int(focus)
        return ComposableNode(
            package='v4l2_camera',
            plugin='v4l2_camera::V4L2Camera',
            name=name,
            namespace=namespace,
            parameters=[params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    depth_params = {
        # Matched pairs from stereo_pair_throttle, not the raw 30 fps streams.
        'left_image_topic': 'stereo_left/image_pair',
        'right_image_topic': 'stereo_right/image_pair',
        'calibration_file': calib_path,
        'left_frame': left_frame,
        'right_frame': right_frame,
        'parent_frame': f'{prefix}base_footprint',
        'image_width': width,
        'image_height': height,
        'close_obstacle_m': close_obstacle_m,
        # Visual odometry runs inside the depth node on the pairs it already
        # decodes (odom/visual for the bringup EKF); a separate subscriber to
        # both cameras cost more than a core on the Pi.
        'visual_odometry': visual_odometry,
        'odom_frame': f'{prefix}odom',
    }

    nodes = [
        # Both cameras and the pair throttle share one process with
        # intra-process comms: the 60 frames/s reach the throttle as pointers
        # instead of 2.3 MB/s of serialized images over loopback DDS, and two
        # fewer processes run. Only the 3 Hz pairs leave the container.
        ComposableNodeContainer(
            name='stereo_container',
            namespace=ns if ns else '',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                _camera(left_dev, left_ns, left_frame, 'v4l2_camera', left_focus),
                _camera(right_dev, right_ns, right_frame, 'v4l2_camera', right_focus),
                # The cameras only offer 30 fps. Pairing and throttling them in
                # C++ keeps the Python depth node from waking 60 times a
                # second. 3 Hz covers the depth node (2.5 Hz) and visual
                # odometry (3 Hz).
                ComposableNode(
                    package='turtlebot3_navigation2',
                    plugin='StereoPairThrottle',
                    name='stereo_pair_throttle',
                    namespace=ns if ns else '',
                    parameters=[{'rate_hz': 3.0, 'slop_sec': 0.03}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
        Node(
            package='turtlebot3_navigation2',
            executable='stereo_depth_node.py',
            additional_env=_SINGLE_THREAD_BLAS,
            name='stereo_depth_node',
            namespace=ns if ns else None,
            parameters=[depth_params],
            output='screen',
        ),
    ]
    prepare = _prepare_command([(left_dev, left_focus), (right_dev, right_focus)])
    info = LogInfo(msg=(
        f'stereo: left={left_dev} ({left_cam["card"]}) '
        f'right={right_dev} ({right_cam["card"]}) '
        f'size={width}x{height} calibration={calib_path} '
        f'visual_odometry={visual_odometry} '
        f'focus={left_focus if left_focus is not None else "auto"}/'
        f'{right_focus if right_focus is not None else "auto"}'
    ))
    return [
        info,
        prepare,
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=nodes)),
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
        DeclareLaunchArgument(
            'visual_odometry', default_value='true',
            description='Publish stereo visual odometry (odom/visual) from the depth node.'),
        OpaqueFunction(function=_setup),
    ])
