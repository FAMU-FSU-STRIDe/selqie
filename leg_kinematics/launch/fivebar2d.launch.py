from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

LEG_NAME = "FL"
ODRIVE_ID0 = 0
ODRIVE_ID1 = 1
FLIP_Y_AXIS = False

def FiveBar2DNode(name : str, id0 : int, id1 : int, flip_y : bool):
    return ComposableNode(
        package='leg_kinematics',
        plugin='leg_kinematics::FiveBar2DNode',
        name=f'fivebar2d{name}',
        parameters=[{
            'flip_y': flip_y # Left: False, Right: True
        }],
        remappings=[
            ('leg/command', f'leg{name}/command'),
            ('leg/estimate', f'leg{name}/estimate'),
            ('motor0/command', f'odrive{id0}/command'),
            ('motor0/estimate', f'odrive{id0}/estimate'),
            ('motor1/command', f'odrive{id1}/command'),
            ('motor1/estimate', f'odrive{id1}/estimate')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='leg_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                FiveBar2DNode(LEG_NAME, ODRIVE_ID0, ODRIVE_ID1, FLIP_Y_AXIS),
            ],
        ),
    ])