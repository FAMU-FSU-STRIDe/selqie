from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
MODEL_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'model')

def MuJoCoNode():
    return Node(
        package='mujoco_ros2',
        executable='mujoco_node',
        name='mujoco',
        output='screen',
        parameters=[{
            'model_path': os.path.join(MODEL_FOLDER, 'scene.xml'),
            'frame_rate': 60.0
        }],
        remappings=[(f'motor{i}/command', f'odrive{i}/command') for i in range(8)] + 
                   [(f'motor{i}/estimate', f'odrive{i}/estimate') for i in range(8)] + 
                   [(f'motor{i}/config', f'odrive{i}/config') for i in range(8)]
    )

def generate_launch_description():
    return LaunchDescription([
        MuJoCoNode(),
    ])