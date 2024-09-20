from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

mujoco_scene_file = get_package_share_directory('mujoco_ros2') + '/models/unitree_a1/scene.xml'
mujoco_frame_rate = 60.0

default_leg_position_left = [0.0, 0.08505, -0.27]
default_leg_position_right = [0.0, -0.08505, -0.27]

unitree_config_folder = get_package_share_directory('unitree_a1_mujoco') + '/config'

def MuJoCoNode():
    return Node(
        package='mujoco_ros2',
        executable='mujoco_node',
        name='mujoco',
        output='screen',
        parameters=[{
            'model_path': mujoco_scene_file,
            'frame_rate': mujoco_frame_rate
        }]
    )

def UnitreeA1LegNode(name : str, id0 : int, id1 : int, id2 : int, flip_y : bool):
    return Node(
        package='leg_kinematics',
        executable='unitree_a1_leg_node',
        name=f'unitree_a1_leg{name}',
        output='screen',
        parameters=[{
            'flip_y': flip_y # Left: False, Right: True
        }],
        remappings=[
            ('leg/command', f'leg{name}/command'),
            ('leg/estimate', f'leg{name}/estimate'),
            ('motor0/command', f'motor{id0}/command'),
            ('motor0/estimate', f'motor{id0}/estimate'),
            ('motor1/command', f'motor{id1}/command'),
            ('motor1/estimate', f'motor{id1}/estimate'),
            ('motor2/command', f'motor{id2}/command'),
            ('motor2/estimate', f'motor{id2}/estimate')
        ]
    )

def SetLegPositionNode(name: str, position):
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', f'/leg{name}/command', 'robot_msgs/LegCommand',
            '{control_mode: 3, pos_setpoint: {x: ' + str(position[0]) + ', y: ' + str(position[1]) + ', z: ' + str(position[2]) + '}}'
        ],
        output='screen'
    )

def LegTrajectoryPublisherNode(name : str):
    return Node(
        package='leg_kinematics',
        executable='leg_trajectory_publisher_node',
        name=f'leg{name}_trajectory',
        remappings=[
            ('leg/trajectory', f'leg{name}/trajectory'),
            ('leg/command', f'leg{name}/command')
        ]
    )

def LeggedMPCNode():
    return Node(
        package='legged_mpc',
        executable='legged_mpc_node',
        name='legged_mpc',
        output='screen',
        parameters=[unitree_config_folder + '/legged_mpc_config.yaml']
    )

def BodyTrajectoryNode():
    return Node(
        package='legged_mpc',
        executable='body_trajectory_node',
        name='body_trajectory',
        output='screen',
        parameters=[unitree_config_folder + '/legged_mpc_config.yaml']
    )

def FootholdPlannerNode():
    return Node(
        package='legged_mpc',
        executable='foothold_planner_node',
        name='foothold_planner',
        output='screen',
        parameters=[unitree_config_folder + '/legged_mpc_config.yaml']
    )

def generate_launch_description():
    return LaunchDescription([
        MuJoCoNode(),
        UnitreeA1LegNode('FR', 0, 1, 2, True),
        UnitreeA1LegNode('FL', 3, 4, 5, False),
        UnitreeA1LegNode('RR', 6, 7, 8, True),
        UnitreeA1LegNode('RL', 9, 10, 11, False),
        # SetLegPositionNode('FR', default_leg_position_right),
        # SetLegPositionNode('FL', default_leg_position_left),
        # SetLegPositionNode('RR', default_leg_position_right),
        # SetLegPositionNode('RL', default_leg_position_left),
        LegTrajectoryPublisherNode('FR'),
        LegTrajectoryPublisherNode('FL'),
        LegTrajectoryPublisherNode('RR'),
        LegTrajectoryPublisherNode('RL'),
        LeggedMPCNode(),
        BodyTrajectoryNode(),
        FootholdPlannerNode()
    ])