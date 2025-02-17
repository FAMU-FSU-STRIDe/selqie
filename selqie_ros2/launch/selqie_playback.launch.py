from launch import LaunchDescription

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
LAUNCH_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch')
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
IMU_CALIBRATION_FILE = os.path.join(CONFIG_FOLDER, 'imu_calibration.txt')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'playback': 'true'
            }.items()
    )
    
from launch_ros.actions import Node
def Depth2PoseNode():
    return Node(
        package='bar100_ros2',
        executable='depth2pose_node',
        name='depth2pose_node',
        output='screen',
        parameters=[{
            'z_variance': 2.89,
            'use_sim_time': True
        }],
    )
    
def IMUCalibrationNode():
    return Node(
        package='selqie_localization',
        executable='imu_calibration_node',
        name='imu_calibration_node',
        output='screen',
        parameters=[{
            'sample_size': 500,
            'calibration_file': IMU_CALIBRATION_FILE,
            'use_sim_time': True
        }],
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('ekf.launch.py'),
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        IncludeLaunchFile('gaits.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        IncludeLaunchFile('gait_planning.launch.py'),
        IncludeLaunchFile('terrain_mapping.launch.py'),
        IncludeLaunchFile('urdf.launch.py'),
        Depth2PoseNode(),
        IMUCalibrationNode(),
    ])