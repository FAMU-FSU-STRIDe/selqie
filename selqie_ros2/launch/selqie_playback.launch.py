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
    
from launch_ros.actions import Node
def Depth2PoseNode():
    return Node(
        package='bar100_ros2',
        executable='depth2pose_node',
        name='depth2pose_node',
        output='screen',
        parameters=[{
            'z_variance': 2.89,
        }],
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('ekf.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        # Depth2PoseNode(),
    ])