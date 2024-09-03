from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

STANDING_LEG_POSITION = [0.0, 0.0, -0.164]

def RunLegTrajectoryFile(file : str, num_loops : int, frequency : float):
    return Node(
        package='robot_utils',
        executable='run_leg_trajectory_file',
        name='run_trajectory',
        output='screen',
        parameters=[
            {'file': file},
            {'num_loops': num_loops},
            {'frequency': frequency},
        ]
    )

def generate_launch_description():

    stand = Node(
        package='robot_utils',
        executable='stand',
        name='stand',
        output='screen',
        parameters=[
            {'stand_position': STANDING_LEG_POSITION},
            {'delay': 1.0},
            {'duration': 5.0},
        ]
    )

    walk = RunLegTrajectoryFile(
        file='walk.txt',
        num_loops=10,
        frequency=1.0
    )
    
    jump = RunLegTrajectoryFile(
        file='jump.txt',
        num_loops=1,
        frequency=3.0
    )
    
    crawl = RunLegTrajectoryFile(
        file='crawl.txt',
        num_loops=5,
        frequency=1.0
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '/legFR/command', '/legFL/command', '/legRR/command', '/legRL/command',
                 '/legFR/estimate', '/legFL/estimate', '/legRR/estimate', '/legRL/estimate',
                 '/odrive0/info', '/odrive1/info', '/odrive2/info', '/odrive3/info',
                 '/odrive4/info', '/odrive5/info', '/odrive6/info', '/odrive7/info',],
            output='screen'
        ),
        stand,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=stand,
                on_exit=[walk],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=walk,
                on_exit=[jump],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jump,
                on_exit=[crawl],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=crawl,
                on_exit=[
                    ExecuteProcess(
                        cmd=['bash', '-c', '\'kill -SIGINT $(pgrep -f "ros2 bag record")\''],
                        shell=True,
                        output='screen'
                    )
                ]
            )
        )
    ])