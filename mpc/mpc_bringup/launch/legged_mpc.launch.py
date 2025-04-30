import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

LEGGED_MPC_CONFIG_FILE = os.path.join(get_package_share_directory('mpc_bringup'), 
                                      'config', 'legged_mpc.yaml')

def generate_launch_description():
    # Return the launch description with the node configuration
    return LaunchDescription([
        Node(
            package='legged_mpc',
            executable='legged_mpc_node',
            name='legged_mpc_node',
            output='screen',
            parameters=[LEGGED_MPC_CONFIG_FILE],
        ),
        Node(
            package='legged_mpc',
            executable='body_trajectory_node',
            name='body_trajectory_node',
            output='screen',
            parameters=[LEGGED_MPC_CONFIG_FILE],
        ),
        Node(
            package='legged_mpc',
            executable='foothold_planner_node',
            name='foothold_planner_node',
            output='screen',
            parameters=[LEGGED_MPC_CONFIG_FILE],
        ),
        Node(
            package='legged_mpc',
            executable='swing_leg_node',
            name='swing_leg_node',
            output='screen',
            parameters=[LEGGED_MPC_CONFIG_FILE],
        ),
    ])