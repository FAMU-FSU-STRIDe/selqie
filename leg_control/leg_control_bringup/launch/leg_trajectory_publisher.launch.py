from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the leg name from the launch configuration
    leg_name = LaunchConfiguration('leg_name').perform(context)
    
    return [
        Node(
            package='leg_trajectory_publisher',
            executable='leg_trajectory_publisher_node',
            name=f'leg{leg_name}_trajectory_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('leg/trajectory', f'leg{leg_name}/trajectory'),
                ('leg/command', f'leg{leg_name}/command'),
            ],
        ),
    ]

def generate_launch_description():
    # Use sim time argument for the stride generation node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Leg name argument for the leg
    leg_name_arg = DeclareLaunchArgument(
        'leg_name',
        default_value='FL',
        description='The name of the leg (default: FL)'
    )
    
    # Return the launch description with the node configuration
    return LaunchDescription([
        sim_time_arg,
        leg_name_arg,
        OpaqueFunction(function=launch_setup)
    ])