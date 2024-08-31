from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_ros2',
            executable='odrive_can_node',
            name='odrive_can',
            output='screen',
            parameters=[{
                'id': 0
            }],
            remappings=[
                ('can/tx', 'can0/tx'),
                ('can/rx', 'can0/rx'),
                ('odrive/command', 'odrive0/command'),
                ('odrive/config', 'odrive0/config'),
                ('odrive/estimate', 'odrive0/estimate'),
                ('odrive/info', 'odrive0/info')
            ]
        )
    ])