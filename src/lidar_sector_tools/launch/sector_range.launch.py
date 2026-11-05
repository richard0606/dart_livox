import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('lidar_sector_tools'),
        'config', 'sector_params.yaml')
    return LaunchDescription([
        Node(
            package='lidar_sector_tools',
            executable='sector_range_node',
            name='sector_range',
            parameters=[params_file],
            output='screen',
        ),
    ])
