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
    
def CommandVelocityFilterNode(use_sim_time : str):
    return Node(
        package='local_planning',
        executable='command_velocity_filter_node',
        name='command_velocity_filter_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'command_velocity_filter.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

def Walk2DNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='walk2d_node',
        name='walk2d_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'walk2d_config.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

def WalkingPlannerNode(use_sim_time : str):
    return Node(
        package='local_planning',
        executable='walking_planner_node',
        name='walking_planner_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'walking_planner_config.yaml'),
                    {'use_sim_time': use_sim_time}]
    )
    
def SwimNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='swim_node',
        name='swim_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'swim_config.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

def JumpNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='jump_node',
        name='jump_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'jump_config.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        CommandVelocityFilterNode(use_sim_time),
        Walk2DNode(use_sim_time),
        WalkingPlannerNode(use_sim_time),
        SwimNode(use_sim_time),
        JumpNode(use_sim_time),
    ])