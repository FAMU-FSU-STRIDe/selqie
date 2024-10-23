from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

MICROSTRAIN_LAUNCH_FILE = os.path.join(get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
IMU_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'imu_config.yaml')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def MicroStrainIMULaunch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
        launch_arguments={
            'configure': 'true',
            'activate': 'true',
            'params_file': IMU_CONFIG_FILE,
            'namespace': '/',
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        MicroStrainIMULaunch(),
    ])