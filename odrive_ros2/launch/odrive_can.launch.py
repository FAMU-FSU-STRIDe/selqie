from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ODRIVE_ID = 0
CAN_INTERFACE = "can0"

def ODriveCanNode(id : int, ifc : str):
    return ComposableNode(
        package='odrive_ros2',
        plugin='odrive_ros2::ODriveCanNode',
        name=f'odrive_can{id}',
        parameters=[{
            'id': id,
            'gear_ratio': 1.0
        }],
        remappings=[
            ('can/tx', ifc + '/tx'),
            ('can/rx', ifc + '/rx'),
            ('odrive/command', f'odrive{id}/command'),
            ('odrive/config', f'odrive{id}/config'),
            ('odrive/estimate', f'odrive{id}/estimate'),
            ('odrive/info', f'odrive{id}/info')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='odrive_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ODriveCanNode(ODRIVE_ID, CAN_INTERFACE),
            ],
        ),
    ])