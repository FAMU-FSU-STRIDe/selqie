from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
    
def Walk2DNode():
    return Node(
        package='stride_maker',
        executable='walk2d_node',
        name='walk2d_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'walk2d_config.yaml')],
    )

def WalkingPlannerNode():
    return Node(
        package='local_planning',
        executable='walking_planner_node',
        name='walking_planner_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'walking_planner_config.yaml')]
    )
    
def SwimNode():
    return Node(
        package='stride_maker',
        executable='swim_node',
        name='swim_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'swim_config.yaml')],
    )

def generate_launch_description():
    return LaunchDescription([
        Walk2DNode(),
        WalkingPlannerNode(),
        SwimNode(),
    ])