import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

STRIDE_GENERATION_CONFIG = os.path.join(
    get_package_share_directory('leg_control_bringup'), 'config', 'stride_generation.yaml')

def generate_launch_description():
    # Use sim time argument for the stride generation node
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
            package='stride_generation',
            executable='walk_stride_node',
            name=f'walk_stride_node',
            output='screen',
            parameters=[STRIDE_GENERATION_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='stride_generation',
            executable='jump_stride_node',
            name=f'jump_stride_node',
            output='screen',
            parameters=[STRIDE_GENERATION_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='stride_generation',
            executable='swim_stride_node',
            name=f'swim_stride_node',
            output='screen',
            parameters=[STRIDE_GENERATION_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='stride_generation',
            executable='sink_stride_node',
            name=f'sink_stride_node',
            output='screen',
            parameters=[STRIDE_GENERATION_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='stride_generation',
            executable='stand_stride_node',
            name=f'stand_stride_node',
            output='screen',
            parameters=[STRIDE_GENERATION_CONFIG, 
                        {'use_sim_time': use_sim_time}]
        ),
    ])