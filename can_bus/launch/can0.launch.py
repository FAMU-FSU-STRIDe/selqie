from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def CanBusNode(ifc : str):
    return ComposableNode(
        package='can_bus',
        plugin='can_bus::CanBusNode',
        name=ifc,
        parameters=[{
            'interface': ifc
        }],
        remappings=[
            ('can/tx', ifc + '/tx'),
            ('can/rx', ifc + '/rx')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='can0_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                CanBusNode("can0"),
            ],
        ),
    ])