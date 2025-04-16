import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

CAN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'can.launch.py')

ODRIVE_LAUNCH_FILE = os.path.join(
        get_package_share_directory('actuation_bringup'), 'launch', 'odrive.launch.py')

ODRIVE_GEAR_RATIOS = '6.0'

def CanLaunch(interface : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CAN_LAUNCH_FILE),
        launch_arguments={
            'interface': interface
        }.items()
    )

def ODriveLaunch(odrive_id : str, interface : str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ODRIVE_LAUNCH_FILE),
        launch_arguments={
            'odrive_id': odrive_id,
            'interface': interface,
            'gear_ratio': ODRIVE_GEAR_RATIOS
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        CanLaunch('can0'),
        CanLaunch('can1'),
        ODriveLaunch('0', 'can0'),
        ODriveLaunch('1', 'can0'),
        ODriveLaunch('2', 'can0'),
        ODriveLaunch('3', 'can0'),
        ODriveLaunch('4', 'can1'),
        ODriveLaunch('5', 'can1'),
        ODriveLaunch('6', 'can1'),
        ODriveLaunch('7', 'can1'),
    ])