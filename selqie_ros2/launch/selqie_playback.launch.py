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
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
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
    
def IMUBiasCorrectionNode():
    return Node(
        package='selqie_localization',
        executable='imu_bias_correction_node',
        name='imu_bias_correction_node',
        output='screen',
        parameters=[{
            'bias': [0.0, 0.0, -9.81],
            'use_sim_time': True
        }],
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('tf.launch.py'),
        # IncludeLaunchFile('ekf.launch.py'),
        IncludeLaunchFile('stereo_cameras_disparity_playback.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        Depth2PoseNode(),
        IMUBiasCorrectionNode(),
    ])