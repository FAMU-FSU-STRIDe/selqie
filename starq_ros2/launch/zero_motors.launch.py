from launch import LaunchDescription
from launch_ros.actions import Node

def ZeroMotorNode(id : int):
    return Node(
        package='odrive_ros2',
        executable='odrive_cmd_node',
        name=f'odrive_cmd{id}',
        output='screen',
        namespace='starq',
        parameters=[{
            'cmd': 'zero'
        }],
        remappings=[
            ('command', f'motor{id}/command')
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        ZeroMotorNode(0),
        ZeroMotorNode(1),
        ZeroMotorNode(2),
        ZeroMotorNode(3),
        ZeroMotorNode(4),
        ZeroMotorNode(5),
        ZeroMotorNode(6),
        ZeroMotorNode(7)
    ])