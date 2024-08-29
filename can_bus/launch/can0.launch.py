from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_bus',
            executable='can_bus_node',
            name='can0',
            output='screen',
            parameters=[{
                'interface': 'can0'
            }]
        )
    ])