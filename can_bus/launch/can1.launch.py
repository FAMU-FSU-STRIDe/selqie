from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_bus',
            executable='can_bus_node',
            name='can1',
            output='screen',
            parameters=[{
                'interface': 'can1'
            }],
            remappings=[
                ('can/tx', 'can1/tx'),
                ('can/rx', 'can1/rx')
            ]
        )
    ])