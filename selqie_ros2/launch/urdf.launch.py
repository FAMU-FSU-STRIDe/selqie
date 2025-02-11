from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
MODEL_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'model')
URDF_FILE = os.path.join(MODEL_FOLDER, 'selqie.urdf')

def RobotStatePublisherNode(use_sim_time : str):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', URDF_FILE])
        }]
    )
    
def SELQIERVizNode(use_sim_time : str):
    return Node(
        package='selqie_ros2',
        executable='selqie_rviz.py',
        name='selqie_rviz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        RobotStatePublisherNode(use_sim_time),
        SELQIERVizNode(use_sim_time)
    ])