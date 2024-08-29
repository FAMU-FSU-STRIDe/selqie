from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leg_kinematics',
            executable='unitree_a1_leg_node',
            name='unitree_a1_leg',
            output='screen',
            parameters=[{
                'D': 0.08505,
                'Lt': 0.2,
                'Lc': 0.2,
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