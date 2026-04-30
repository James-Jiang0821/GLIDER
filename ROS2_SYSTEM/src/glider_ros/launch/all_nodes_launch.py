"""Launches the full glider node graph: 
drivers, adapters, safety, and controller
no bridge and state manager
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    glider_params = os.path.join(
        get_package_share_directory('glider_ros'),
        'config', 'glider_params.yaml'
    )

    return LaunchDescription([
        #--- Drivers ---
        Node(
            package='glider_ros',
            executable='imu_minimu_node',
            name='imu_minimu_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='gnss_maxm10s_i2c_node',
            name='gnss_maxm10s_i2c_node',
            output='screen'
        ),
        LifecycleNode(
            package='glider_ros',
            executable='communication_iridium_node',
            name='communication_iridium_node',
            namespace='',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='sonar_ping_node',
            name='sonar_ping_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='pressure_keller_node',
            name='pressure_keller_node',
            parameters=[glider_params],
            output='screen'
        ),

        #--- Adapters ---
        Node(
            package='glider_ros',
            executable='imu_adapter_node',
            name='imu_adapter_node',
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='sonar_adapter_node',
            name='sonar_adapter_node',
            parameters=[glider_params],
            output='screen'
        ),
        Node(
            package='glider_ros',
            executable='telemetry_adapter_node',
            name='telemetry_adapter_node',
            parameters=[glider_params],
            output='screen'
        ),

        #--- Safety ---
        Node(
            package='glider_ros',
            executable='safety_node',
            name='safety_node',
            parameters=[glider_params],
            output='screen'
        ),

        #--- Controller ---
        LifecycleNode(
            package='glider_ros',
            executable='controller_node',
            name='controller_node',
            namespace='',
            parameters=[glider_params],
            output='screen'
        ),
    ])
