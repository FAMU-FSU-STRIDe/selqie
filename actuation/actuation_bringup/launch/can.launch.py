from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Interface argument for the CAN bus
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='The CAN interface to use (default: can0)'
    )
    # Get the interface from the launch configuration
    interface = LaunchConfiguration('interface')

    # Return the launch description with the node configuration
    return LaunchDescription([
        interface_arg,
        Node(
            package='can_bus',
            executable='can_bus_node',
            name=f'{interface}_node',
            output='screen',
            parameters=[{'interface': interface}],
            remappings=[
                ('can/tx', f'{interface}/tx'),
                ('can/rx', f'{interface}/rx')
            ],
        ),
    ])