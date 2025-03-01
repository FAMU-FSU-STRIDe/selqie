from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
LOCAL_PLANNING_CONFIG = os.path.join(CONFIG_FOLDER, 'local_planning_config.yaml')
    
def WalkingPlannerNode(use_sim_time : str):
    return Node(
        package='local_planning',
        executable='walking_planner_node',
        name='walking_planner_node',
        parameters=[LOCAL_PLANNING_CONFIG, {'use_sim_time': use_sim_time}]
    )

def SwimmingPlannerNode(use_sim_time : str):
    return Node(
        package='local_planning',
        executable='swimming_planner_node',
        name='swimming_planner_node',
        parameters=[LOCAL_PLANNING_CONFIG, {'use_sim_time': use_sim_time}]
    )
    
def JumpingPlannerNode(use_sim_time : str):
    return Node(
        package='local_planning',
        executable='jumping_planner_node',
        name='jumping_planner_node',
        parameters=[LOCAL_PLANNING_CONFIG, {'use_sim_time': use_sim_time}]
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        WalkingPlannerNode(use_sim_time),
        SwimmingPlannerNode(use_sim_time),
        JumpingPlannerNode(use_sim_time)
    ])