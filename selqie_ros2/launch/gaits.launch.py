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

COMMAND_VELOCITY_FILTER_CONFIG = os.path.join(CONFIG_FOLDER, 'command_velocity_filter.yaml')
WALK_CONFIG = os.path.join(CONFIG_FOLDER, 'walk_config.yaml')
SWIM_CONFIG = os.path.join(CONFIG_FOLDER, 'swim_config.yaml')
JUMP_CONFIG = os.path.join(CONFIG_FOLDER, 'jump_config.yaml')

def CommandVelocityFilterNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='command_velocity_filter_node',
        name='command_velocity_filter_node',
        parameters=[COMMAND_VELOCITY_FILTER_CONFIG, {'use_sim_time': use_sim_time}]
    )

def WalkNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='walk_node',
        name='walk_node',
        parameters=[WALK_CONFIG, {'use_sim_time': use_sim_time}],
        # prefix=['xterm -e gdb -ex run --args']
    )
    
def SwimNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='swim_node',
        name='swim_node',
        parameters=[SWIM_CONFIG, {'use_sim_time': use_sim_time}],
    )

def JumpNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='jump_node',
        name='jump_node',
        parameters=[JUMP_CONFIG, {'use_sim_time': use_sim_time}],
    )

def SinkNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='sink_node',
        name='sink_node',
        parameters=[{'use_sim_time': use_sim_time}],
    )

def StandNode(use_sim_time : str):
    return Node(
        package='stride_maker',
        executable='stand_node',
        name='stand_node',
        parameters=[{'use_sim_time': use_sim_time}],
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        CommandVelocityFilterNode(use_sim_time),
        WalkNode(use_sim_time),
        SwimNode(use_sim_time),
        JumpNode(use_sim_time),
        SinkNode(use_sim_time),
        StandNode(use_sim_time),
    ])