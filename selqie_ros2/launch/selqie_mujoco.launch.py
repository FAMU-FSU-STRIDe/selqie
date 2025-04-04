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

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchFile('mujoco.launch.py'),
        IncludeLaunchFile('legs.launch.py'),
        IncludeLaunchFile('gaits.launch.py'),
        IncludeLaunchFile('local_planning.launch.py'),
        IncludeLaunchFile('gait_planning.launch.py'),
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        IncludeLaunchFile('urdf.launch.py'),
    ])