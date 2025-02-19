from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
TERRAIN_MAPPING_CONFIG = os.path.join(CONFIG_FOLDER, 'terrain_mapping_config.yaml')

def PointCloudRGBFilterNode(name):
    return Node(
        package='selqie_perception',
        executable='pointcloud_rgb_filter_node',
        name=f'{name}_pointcloud_rgb_filter_node',
        output='screen',
        parameters=[TERRAIN_MAPPING_CONFIG],
        remappings=[
            ('points/in', 'stereo/points2'),
            ('points/out', f'points/{name}')
        ]
    )

def TerrainMappingNode(use_sim_time : str):
    return Node(
        package='terrain_mapping',
        executable='terrain_mapping_node',
        name='terrain_mapping_node',
        output='screen',
        parameters=[TERRAIN_MAPPING_CONFIG, {'use_sim_time': use_sim_time}],
        # prefix=['xterm -e gdb -ex run --args']
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        PointCloudRGBFilterNode('rock'),
        PointCloudRGBFilterNode('wall'),
        TerrainMappingNode(use_sim_time),
    ])