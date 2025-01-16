from launch import LaunchDescription

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
LAUNCH_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        )
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('actuation.launch.py'),
        IncludeLaunchFile('imu.launch.py'),
        IncludeLaunchFile('bar100.launch.py'),
        IncludeLaunchFile('lights.launch.py'),
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('ekf.launch.py'),
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        # IncludeLaunchFile('visualization.launch.py'),
    ])