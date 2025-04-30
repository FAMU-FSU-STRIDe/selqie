import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

EKF_LAUNCH_FILE = os.path.join(
        get_package_share_directory('localization_bringup'), 'launch', 'ekf.launch.py')

def EKFLaunch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(EKF_LAUNCH_FILE)
    )

def generate_launch_description():
    return LaunchDescription([
        EKFLaunch(),
    ])