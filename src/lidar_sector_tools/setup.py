from setuptools import setup

package_name = 'lidar_sector_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sector_range.launch.py',
            'launch/dart_lidar_targets.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/sector_params.yaml',
            'config/dart_lidar_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rich',
    maintainer_email='rich@todo.todo',
    description='Livox MID-360 扇区测距与空中目标检测',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sector_range_node = lidar_sector_tools.sector_range_node:main',
            'dart_lidar_targets = lidar_sector_tools.dart_lidar_targets:main',
        ],
    },
)
