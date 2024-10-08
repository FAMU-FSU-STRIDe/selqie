import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'starq_ros2'
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch', name)
        )
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('sensors.launch.py'),
        IncludeLaunchFile('stereo_cameras.launch.py'),
        IncludeLaunchFile('stereo_disparity.launch.py'),
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        # IncludeLaunchFile('sensor_fusion.launch.py'),
        # IncludeLaunchFile('slam.launch.py')
    ])