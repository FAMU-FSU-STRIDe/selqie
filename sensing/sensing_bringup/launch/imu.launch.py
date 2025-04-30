import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'sensing_bringup'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
IMU_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'imu.yaml')

MICROSTRAIN_LAUNCH_FILE = os.path.join(
    get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
IMU_CALIBRATION_FILE = os.path.join(CONFIG_FOLDER, 'imu_calibration.txt')

def generate_launch_description():
    # Return the launch description with the node configuration
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
            launch_arguments={
                'configure': 'true',
                'activate': 'true',
                'params_file': IMU_CONFIG_FILE,
                'namespace': '/',
            }.items()
        ),
        Node(
            package='imu_calibration',
            executable='imu_calibration_node',
            name='imu_calibration_node',
            output='screen',
            parameters=[IMU_CONFIG_FILE, {
                'calibration_file': IMU_CALIBRATION_FILE
            }],
        )
    ])