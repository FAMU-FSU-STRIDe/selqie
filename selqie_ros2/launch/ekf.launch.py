from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
LAUNCH_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch')

EKF_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'ekf_config.yaml')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        )
    )

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