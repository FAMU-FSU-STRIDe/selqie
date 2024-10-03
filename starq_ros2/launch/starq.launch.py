import os
import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = 'starq_ros2'
def IncludeLaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ament_index_python.packages.get_package_share_directory(PACKAGE_NAME), 'launch', name)
        )
    )

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('actuation.launch.py'),
        IncludeLaunchFile('imu.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
    ])