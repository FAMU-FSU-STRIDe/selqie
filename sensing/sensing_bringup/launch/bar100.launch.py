import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sensing_bringup'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
BAR100_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'bar100.yaml')

def generate_launch_description():
    # Return the launch description with the node configuration
    return LaunchDescription([
        Node(
            package='bar100_driver',
            executable='bar100_node',
            name='bar100_node',
            output='screen',
            parameters=[BAR100_CONFIG_FILE],
        ),
        Node(
            package='bar100_driver',
            executable='depth2pose_node',
            name='depth2pose_node',
            output='screen',
            parameters=[BAR100_CONFIG_FILE],
        ),
    ])