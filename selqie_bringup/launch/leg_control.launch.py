import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

FIVEBAR_LAUNCH_FILE = os.path.join(
        get_package_share_directory('leg_control_bringup'), 'launch', 'fivebar.launch.py')

def FivebarLaunch(leg_name : str, motor0 : str, motor1 : str, flip_y : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FIVEBAR_LAUNCH_FILE),
        launch_arguments={
            'leg_name': leg_name,
            'motor0': motor0,
            'motor1': motor1,
            'flip_y': flip_y
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        FivebarLaunch('FL', '0', '1', 'true'),
        FivebarLaunch('RL', '2', '3', 'true'),
        FivebarLaunch('RR', '4', '5', 'false'),
        FivebarLaunch('FR', '6', '7', 'false'),
    ])