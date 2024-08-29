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
                ('command', 'motor0/command'),
                ('config', 'motor0/config'),
                ('estimate', 'motor0/estimate'),
                ('info', 'motor0/info')
            ]
        )
    ])