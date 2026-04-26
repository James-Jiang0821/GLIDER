"""Launches the can bridge node in isolation for CAN/Teensy bring-up and testing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    glider_params = os.path.join(
        get_package_share_directory('glider_ros'),
        'config', 'glider_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='glider_ros',
            executable='can_bridge_node',
            name='can_bridge_node',
            parameters=[glider_params],
            output='screen',
        ),
    ])
