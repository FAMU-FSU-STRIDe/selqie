from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

mujoco_models_folder = get_package_share_directory('mujoco_ros2') + '/models/'

default_leg_position_left = [0.0, 0.08505, -0.27]
default_leg_position_right = [0.0, -0.08505, -0.27]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_ros2',
            executable='mujoco_node',
            name='unitree_a1',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'model_path': mujoco_models_folder + 'unitree_a1/scene.xml',
                'frame_rate': 60.0,
            }]
        ),
        Node(
            package='leg_kinematics',
            executable='unitree_a1_leg_node',
            name='unitree_a1_leg_front_right',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'flip_y': True # Left: False, Right: True
            }],
            remappings=[
                ('leg/command', 'legFR/command'),
                ('leg/estimate', 'legFR/estimate'),
                ('motor0/command', 'motor0/command'),
                ('motor0/estimate', 'motor0/estimate'),
                ('motor1/command', 'motor1/command'),
                ('motor1/estimate', 'motor1/estimate'),
                ('motor2/command', 'motor2/command'),
                ('motor2/estimate', 'motor2/estimate')
            ]
        ),
        Node(
            package='leg_kinematics',
            executable='unitree_a1_leg_node',
            name='unitree_a1_leg_front_left',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'flip_y': False # Left: False, Right: True
            }],
            remappings=[
                ('leg/command', 'legFL/command'),
                ('leg/estimate', 'legFL/estimate'),
                ('motor0/command', 'motor3/command'),
                ('motor0/estimate', 'motor3/estimate'),
                ('motor1/command', 'motor4/command'),
                ('motor1/estimate', 'motor4/estimate'),
                ('motor2/command', 'motor5/command'),
                ('motor2/estimate', 'motor5/estimate')
            ]
        ),
        Node(
            package='leg_kinematics',
            executable='unitree_a1_leg_node',
            name='unitree_a1_leg_rear_right',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'flip_y': True # Left: False, Right: True
            }],
            remappings=[
                ('leg/command', 'legRR/command'),
                ('leg/estimate', 'legRR/estimate'),
                ('motor0/command', 'motor6/command'),
                ('motor0/estimate', 'motor6/estimate'),
                ('motor1/command', 'motor7/command'),
                ('motor1/estimate', 'motor7/estimate'),
                ('motor2/command', 'motor8/command'),
                ('motor2/estimate', 'motor8/estimate')
            ]
        ),
        Node(
            package='leg_kinematics',
            executable='unitree_a1_leg_node',
            name='unitree_a1_leg_rear_left',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'flip_y': False # Left: False, Right: True
            }],
            remappings=[
                ('leg/command', 'legRL/command'),
                ('leg/estimate', 'legRL/estimate'),
                ('motor0/command', 'motor9/command'),
                ('motor0/estimate', 'motor9/estimate'),
                ('motor1/command', 'motor10/command'),
                ('motor1/estimate', 'motor10/estimate'),
                ('motor2/command', 'motor11/command'),
                ('motor2/estimate', 'motor11/estimate')
            ]
        ),
        Node(
            package='robot_utils',
            executable='set_leg_position',
            name='leg_stand_front_right',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'position': default_leg_position_right,
                'delay': 1.0
            }],
            remappings=[
                ('leg/command', 'legFR/command')
            ]
        ),
        Node(
            package='robot_utils',
            executable='set_leg_position',
            name='leg_stand_front_left',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'position': default_leg_position_left,
                'delay': 1.0
            }],
            remappings=[
                ('leg/command', 'legFL/command')
            ]
        ),
        Node(
            package='robot_utils',
            executable='set_leg_position',
            name='leg_stand_rear_right',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'position': default_leg_position_right,
                'delay': 1.0
            }],
            remappings=[
                ('leg/command', 'legRR/command')
            ]
        ),
        Node(
            package='robot_utils',
            executable='set_leg_position',
            name='leg_stand_rear_left',
            output='screen',
            namespace='unitree_a1',
            parameters=[{
                'position': default_leg_position_left,
                'delay': 1.0
            }],
            remappings=[
                ('leg/command', 'legRL/command')
            ]
        ),
    ])