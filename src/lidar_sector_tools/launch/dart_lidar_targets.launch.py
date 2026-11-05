import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('lidar_sector_tools'),
        'config', 'dart_lidar_params.yaml')
    return LaunchDescription([
        Node(
            package='lidar_sector_tools',
            executable='dart_lidar_targets',
            name='dart_lidar_targets',
            parameters=[params_file],
            output='screen',
        ),
    ])
