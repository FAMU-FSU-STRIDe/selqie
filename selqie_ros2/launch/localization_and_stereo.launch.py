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
        # Static Transforms Launch File
        IncludeLaunchFile('tf.launch.py'),
        # IMU Launch File
        IncludeLaunchFile('imu.launch.py'),
        # Bar100 Launch File
        IncludeLaunchFile('bar100.launch.py'),
        # EKF Launch File
        IncludeLaunchFile('ekf.launch.py'),
        # Stereo Cameras Launch File
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        # RViz Launch File
        IncludeLaunchFile('visualization.launch.py'),
    ])