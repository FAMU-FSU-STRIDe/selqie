from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

IMU_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'imu_config.yaml')

MICROSTRAIN_LAUNCH_FILE = os.path.join(get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')

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
    
def DeadReckoningNode():
    return Node(
        package='selqie_localization',
        executable='imu_dead_reckoning',
        name='dead_reckoning_node',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'publish_tf': True,
        }]
    )

def generate_launch_description():
    return LaunchDescription([
        MicroStrainIMULaunch(),
        DeadReckoningNode(),
    ])