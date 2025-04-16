import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

MUJOCO_LAUNCH_FILE = os.path.join(
        get_package_share_directory('simulation_bringup'), 'launch', 'mujoco.launch.py')

SELQIE_MUJOCO_SCENE = os.path.join(
        get_package_share_directory('selqie_models'), 'models', 'selqie_v1', 'scene.xml')

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MUJOCO_LAUNCH_FILE),
            launch_arguments={
                'model_path': SELQIE_MUJOCO_SCENE,
                'frame_rate': '60.0'
            }.items()
        )
    ])