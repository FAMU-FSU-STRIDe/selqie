import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

LOCAL_PLANNING_CONFIG = os.path.join(
    get_package_share_directory('planning_bringup'), 'config', 'local_planning.yaml')

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
            package='local_planning',
            executable='walk_planning_node',
            name=f'walk_planning_node',
            output='screen',
            parameters=[LOCAL_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='local_planning',
            executable='jump_planning_node',
            name=f'jump_planning_node',
            output='screen',
            parameters=[LOCAL_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='local_planning',
            executable='swim_planning_node',
            name=f'swim_planning_node',
            output='screen',
            parameters=[LOCAL_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='local_planning',
            executable='sink_planning_node',
            name=f'sink_planning_node',
            output='screen',
            parameters=[LOCAL_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='local_planning',
            executable='stand_planning_node',
            name=f'stand_planning_node',
            output='screen',
            parameters=[LOCAL_PLANNING_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
    ])