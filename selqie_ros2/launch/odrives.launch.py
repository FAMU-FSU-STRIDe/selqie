from launch import LaunchDescription
from launch_ros.actions import Node

GEAR_RATIO = 6.0

def CanBusNode(ifc : str):
    return Node(
        package='can_bus',
        executable='can_bus_node',
        name=ifc,
        parameters=[{
            'interface': ifc
        }],
        remappings=[
            ('can/tx', ifc + '/tx'),
            ('can/rx', ifc + '/rx')
        ]
    )

def ODriveCanNode(id : int, ifc : str):
    return Node(
        package='odrive_ros2',
        executable='odrive_can_node',
        name=f'odrive{id}',
        parameters=[{
            'id': id,
            'gear_ratio': GEAR_RATIO
        }],
        remappings=[
            ('can/tx', ifc + '/tx'),
            ('can/rx', ifc + '/rx'),
            ('odrive/command', f'odrive{id}/command'),
            ('odrive/config', f'odrive{id}/config'),
            ('odrive/estimate', f'odrive{id}/estimate'),
            ('odrive/info', f'odrive{id}/info')
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        CanBusNode('can0'),
        CanBusNode('can1'),
        ODriveCanNode(0, 'can0'),
        ODriveCanNode(1, 'can0'),
        ODriveCanNode(2, 'can0'),
        ODriveCanNode(3, 'can0'),
        ODriveCanNode(4, 'can1'),
        ODriveCanNode(5, 'can1'),
        ODriveCanNode(6, 'can1'),
        ODriveCanNode(7, 'can1'),
    ])