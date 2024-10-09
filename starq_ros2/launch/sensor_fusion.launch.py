from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'starq_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

EKF_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'ekf_config.yaml')

def RobotLocalizationNode():
    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[EKF_CONFIG_FILE],
    )

def generate_launch_description():
    return LaunchDescription([
        # Robot Localization Node
        RobotLocalizationNode(),
    ])