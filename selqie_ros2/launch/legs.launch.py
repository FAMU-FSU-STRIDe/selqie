from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

def FiveBar2DNode(name : str, id0 : int, id1 : int, flip_y : bool):
    return Node(
        package='leg_kinematics',
        executable='fivebar2d_node',
        name=f'leg{name}',
        parameters=[{
            'flip_y': flip_y # Left: True, Right: False
        }],
        remappings=[
            ('leg/command', f'leg{name}/command'),
            ('leg/estimate', f'leg{name}/estimate'),
            ('leg/trajectory', f'leg{name}/trajectory'),
            ('motor0/command', f'odrive{id0}/command'),
            ('motor0/estimate', f'odrive{id0}/estimate'),
            ('motor1/command', f'odrive{id1}/command'),
            ('motor1/estimate', f'odrive{id1}/estimate')
        ]
    )

def LegTrajectoryPublisherNode(name : str, use_sim_time : str):
    return Node(
        package='leg_kinematics',
        executable='leg_trajectory_publisher_node',
        name=f'leg{name}_trajectory',
        remappings=[
            ('leg/trajectory', f'leg{name}/trajectory'),
            ('leg/command', f'leg{name}/command')
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

def generate_launch_description():
    launch_arg, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_arg,
        FiveBar2DNode('FL', 0, 1, True),
        FiveBar2DNode('RL', 2, 3, True),
        FiveBar2DNode('RR', 4, 5, False),
        FiveBar2DNode('FR', 6, 7, False),
        LegTrajectoryPublisherNode('FL', use_sim_time),
        LegTrajectoryPublisherNode('RL', use_sim_time),
        LegTrajectoryPublisherNode('RR', use_sim_time),
        LegTrajectoryPublisherNode('FR', use_sim_time)
    ])