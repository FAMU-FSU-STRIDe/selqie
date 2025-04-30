import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

GAIT_PLANNING_CONFIG = os.path.join(
    get_package_share_directory('planning_bringup'), 'config', 'gait_planning.yaml')

def generate_launch_description():
    # Use sim time argument for the planning node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Return the launch description with the node configuration
    return LaunchDescription([
        sim_time_arg,
        Node(
            package='gait_planning',
            executable='gait_planning_node',
            name=f'gait_planning_node',
            output='screen',
            parameters=[GAIT_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        )
    ])