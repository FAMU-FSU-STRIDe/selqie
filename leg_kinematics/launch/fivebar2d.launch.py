from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leg_kinematics',
            executable='fivebar2d_node',
            name='fivebar2d',
            output='screen',
            parameters=[{
                'id': 0,
                'L1': 0.065,
                'L2': 0.2,
                'flip_y': False # Left: False, Right: True
            }],
            remappings=[
                ('command', 'leg0/command'),
                ('estimate', 'leg0/estimate'),
                ('motor0/command', 'motor0/command'),
                ('motor0/estimate', 'motor0/estimate'),
                ('motor1/command', 'motor1/command'),
                ('motor1/estimate', 'motor1/estimate')
            ]
        )
    ])