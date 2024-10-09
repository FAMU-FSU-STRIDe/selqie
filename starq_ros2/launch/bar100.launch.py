from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'starq_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

BAR100_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'bar100_config.yaml')

def Bar100Node():
    return Node(
        package='bar100_ros2',
        executable='bar100_node',
        name='bar100_node',
        output='screen',
        parameters=[BAR100_CONFIG_FILE],
    )

def generate_launch_description():
    return LaunchDescription([
        Bar100Node(),
    ])