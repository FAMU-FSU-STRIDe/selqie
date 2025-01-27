from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

def RVIZ2Node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(CONFIG_FOLDER, 'rviz_config.rviz')],
        parameters=[{
            'use_sim_time': True
        }]
    )

def generate_launch_description():
    return LaunchDescription([
        RVIZ2Node()
    ])