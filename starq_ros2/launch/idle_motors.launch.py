from launch import LaunchDescription
from launch_ros.actions import Node

def IdleMotorNode(id : int):
    return Node(
        package='odrive_ros2',
        executable='odrive_cmd_node',
        name=f'odrive_cmd{id}',
        output='screen',
        namespace='starq',
        parameters=[{
            'cmd': 'idle'
        }],
        remappings=[
            ('config', f'motor{id}/config')
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        IdleMotorNode(0),
        IdleMotorNode(1),
        IdleMotorNode(2),
        IdleMotorNode(3),
        IdleMotorNode(4),
        IdleMotorNode(5),
        IdleMotorNode(6),
        IdleMotorNode(7)
    ])