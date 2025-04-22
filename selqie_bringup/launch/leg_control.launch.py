import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

FIVEBAR_LAUNCH_FILE = os.path.join(
        get_package_share_directory('leg_control_bringup'), 'launch', 'fivebar.launch.py')

STRIDE_GENERATION_LAUNCH_FILE = os.path.join(
        get_package_share_directory('leg_control_bringup'), 'launch', 'stride_generation.launch.py')

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

def StrideGenerationLaunch(use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(STRIDE_GENERATION_LAUNCH_FILE),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

def generate_launch_description():
    # Use sim time argument for the stride generation node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        sim_time_arg,
        FivebarLaunch('FL', '0', '1', 'true'),
        FivebarLaunch('RL', '2', '3', 'true'),
        FivebarLaunch('RR', '4', '5', 'false'),
        FivebarLaunch('FR', '6', '7', 'false'),
        StrideGenerationLaunch(use_sim_time)
    ])