from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
    
def GaitPlanningNode():
    return Node(
        package='gait_planning',
        executable='gait_planning_node',
        name='gait_planning_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'gait_planning_config.yaml')]
    )

def generate_launch_description():
    return LaunchDescription([
        GaitPlanningNode()
    ])