from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Path argument for the URDF
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        description='Path to the URDF'
    )
    # Get the URDF path from the launch configuration
    urdf_path = LaunchConfiguration('urdf_path')
    
    # Use sim time argument for the stride generation node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        urdf_path_arg,
        sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),
        Node(
            package='selqie_ui',
            executable='selqie_joint_publisher',
            name='selqie_joint_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])