from launch import LaunchDescription
from launch_ros.actions import Node

def ReadyMotorNode(id : int):
    return Node(
        package='odrive_ros2',
        executable='odrive_cmd_node',
        name=f'odrive_cmd{id}',
        output='screen',
        namespace='starq',
        parameters=[{
            'cmd': 'ready'
        }],
        remappings=[
            ('config', f'motor{id}/config')
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        ReadyMotorNode(0),
        ReadyMotorNode(1),
        ReadyMotorNode(2),
        ReadyMotorNode(3),
        ReadyMotorNode(4),
        ReadyMotorNode(5),
        ReadyMotorNode(6),
        ReadyMotorNode(7)
    ])