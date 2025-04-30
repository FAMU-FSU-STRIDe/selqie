import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

URDF_LAUNCH_FILE = os.path.join(
        get_package_share_directory('ui_bringup'), 'launch', 'urdf.launch.py')

RVIZ_LAUNCH_FILE = os.path.join(
        get_package_share_directory('ui_bringup'), 'launch', 'rviz.launch.py')

SELQIE_URDF_FILE = os.path.join(get_package_share_directory('selqie_models'), 'models', 'selqie_v1', 'selqie.urdf')

def URDFLaunch(urdf_path, use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(URDF_LAUNCH_FILE),
        launch_arguments={
            'urdf_path': urdf_path,
            'use_sim_time': use_sim_time
        }.items()
    )
    
def RVizLaunch(use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(RVIZ_LAUNCH_FILE),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
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
        URDFLaunch(SELQIE_URDF_FILE, use_sim_time),
        RVizLaunch(use_sim_time)
    ])