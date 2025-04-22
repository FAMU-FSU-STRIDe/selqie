import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

RVIZ_CONFIG_FILE = os.path.join(
        get_package_share_directory('ui_bringup'), 'config', 'config.rviz')

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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', RVIZ_CONFIG_FILE],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])