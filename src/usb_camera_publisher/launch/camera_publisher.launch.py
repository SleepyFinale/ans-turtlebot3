from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='320',
        description='Camera width'
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='240',
        description='Camera height'
    )
    fps_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        Node(
            package='usb_camera_publisher',
            executable='usb_camera_publisher_node',
            name='usb_camera_publisher_node',
            output='screen',
            parameters=[{
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'framerate': LaunchConfiguration('framerate')
            }]
        )
    ])