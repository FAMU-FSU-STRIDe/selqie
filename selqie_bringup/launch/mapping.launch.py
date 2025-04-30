import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

TERRAIN_MAPPING_LAUNCH_FILE = os.path.join(
        get_package_share_directory('mapping_bringup'), 'launch', 'terrain_mapping.launch.py')

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(TERRAIN_MAPPING_LAUNCH_FILE),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        )
    ])