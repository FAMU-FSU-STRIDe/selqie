from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

LEG_NAME = "FL"
MOTOR_ID0 = 0
MOTOR_ID1 = 1
MOTOR_ID2 = 2
FLIP_Y_AXIS = False

def UnitreeA1LegNode(name : str, id0 : int, id1 : int, id2 : int, flip_y : bool):
    return ComposableNode(
        package='leg_kinematics',
        plugin='leg_kinematics::UnitreeA1LegNode',
        name=f'unitree_a1_leg{name}',
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

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='leg_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                UnitreeA1LegNode(LEG_NAME, MOTOR_ID0, MOTOR_ID1, MOTOR_ID2, FLIP_Y_AXIS),
            ],
        ),
    ])