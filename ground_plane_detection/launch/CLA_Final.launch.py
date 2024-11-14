from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
SELQIE_PACKAGE_NAME = 'selqie_ros2'
LAUNCH_FOLDER = os.path.join(get_package_share_directory(SELQIE_PACKAGE_NAME), 'launch')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        )
    )
    
def GroundPlaneDetectionNode():
    return Node(
        package='ground_plane_detection',
        executable='ground_plane_detection_node',
        name='ground_plane_detection_node',
        output='screen',
        parameters=[{
            'sample_size': 100,
            'range_min': 0.1,
            'range_max': 2.0,
            'plane_width': 2.5,
        }],
        remappings=[
            ('/points', '/stereo/points2'),
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        GroundPlaneDetectionNode()
    ])