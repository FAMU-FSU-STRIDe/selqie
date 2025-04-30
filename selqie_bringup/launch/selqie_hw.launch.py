import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

LAUNCH_FOLDER = os.path.join(get_package_share_directory('selqie_bringup'), 'launch')

def LaunchFile(name : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

def generate_launch_description():
    return LaunchDescription([
        LaunchFile('actuation.launch.py'),
        # LaunchFile('sensing.launch.py'),
        # LaunchFile('vision.launch.py'),
        LaunchFile('leg_control.launch.py'),
        # LaunchFile('mapping.launch.py'),
        # LaunchFile('planning.launch.py'),
        # LaunchFile('tf.launch.py'),
    ])