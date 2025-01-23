from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
MODEL_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'model')

def MuJoCoNode():
    return Node(
        package='mujoco_ros2',
        executable='mujoco_node',
        name='mujoco',
        output='screen',
        parameters=[{
            'model_path': os.path.join(MODEL_FOLDER, 'scene.xml'),
            'frame_rate': 60.0
        }],
        remappings=[(f'motor{i}/command', f'odrive{i}/command') for i in range(8)] + 
                   [(f'motor{i}/estimate', f'odrive{i}/estimate') for i in range(8)] + 
                   [(f'motor{i}/config', f'odrive{i}/config') for i in range(8)]
    )

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

def LegTrajectoryPublisherNode(name : str):
    return Node(
        package='leg_kinematics',
        executable='leg_trajectory_publisher_node',
        name=f'leg{name}_trajectory',
        remappings=[
            ('leg/trajectory', f'leg{name}/trajectory'),
            ('leg/command', f'leg{name}/command')
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )
    
def Walk2DNode():
    return Node(
        package='stride_maker',
        executable='walk2d_node',
        name='walk2d_node',
        parameters=[os.path.join(CONFIG_FOLDER, 'walk2d_config.yaml'), {
            'use_sim_time': True
        }],
    )

def WalkingPlannerNode():
    return Node(
        package='local_planning',
        executable='walking_planner_node',
        name='walking_planner_node',
        parameters=[{
            'use_sim_time': True,
            'solve_frequency': 3.0,
            'max_iterations': 500000,
            'publish_all': True
        }]
    )

def RVIZ2Node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(CONFIG_FOLDER, 'rviz_config.rviz')]
    )

def generate_launch_description():
    return LaunchDescription([
        MuJoCoNode(),
        FiveBar2DNode('FL', 0, 1, True),
        FiveBar2DNode('RL', 2, 3, True),
        FiveBar2DNode('RR', 4, 5, False),
        FiveBar2DNode('FR', 6, 7, False),
        LegTrajectoryPublisherNode('FL'),
        LegTrajectoryPublisherNode('RL'),
        LegTrajectoryPublisherNode('RR'),
        LegTrajectoryPublisherNode('FR'),
        Walk2DNode(),
        WalkingPlannerNode(),
        RVIZ2Node()
    ])