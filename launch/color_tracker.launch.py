from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='color_tracker', executable='color_detector', name='color_detector'),
        Node(package='color_tracker', executable='turtle_controller', name='turtle_controller'),
    ])
