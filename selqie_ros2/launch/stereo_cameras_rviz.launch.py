from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
LAUNCH_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch')

def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        )
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
    ])