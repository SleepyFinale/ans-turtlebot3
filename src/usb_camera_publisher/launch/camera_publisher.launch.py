from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera_publisher',
            executable='usb_camera_publisher_node',
            name='usb_camera_publisher_node',
            output='screen'
        )
    ])