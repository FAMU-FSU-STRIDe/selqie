from launch import LaunchDescription
from launch_ros.actions import Node

default_leg_position = [0.0, 0.0, -0.18914]

def CanNode(ifc : int):
    return Node(
        package='can_bus',
        executable='can_bus_node',
        name=f'can{ifc}',
        output='screen',
        namespace='starq',
        parameters=[{
            'interface': f'can{ifc}'
        }],
        remappings=[
            ('can/tx', f'can{ifc}/tx'),
            ('can/rx', f'can{ifc}/rx')
        ]
    )

def ODriveNode(id : int, ifc : int):
    return Node(
        package='odrive_ros2',
        executable='odrive_can_node',
        name=f'odrive_can{id}',
        output='screen',
        namespace='starq',
        parameters=[{
            'id': id
        }],
        remappings=[
            ('can/tx', f'can{ifc}/tx'),
            ('can/rx', f'can{ifc}/rx'),
            ('command', f'motor{id}/command'),
            ('config', f'motor{id}/config'),
            ('estimate', f'motor{id}/estimate'),
            ('info', f'motor{id}/info')
        ]
    )

def FiveBar2DNode(name : str, id0 : int, id1 : int, flip_y : bool):
    return Node(
        package='leg_kinematics',
        executable='unitree_a1_leg_node',
        name=f'unitree_a1_leg{name}',
        output='screen',
        namespace='starq',
        parameters=[{
            'flip_y': flip_y # Left: False, Right: True
        }],
        remappings=[
            ('command', f'leg{name}/command'),
            ('estimate', f'leg{name}/estimate'),
            ('motor0/command', f'motor{id0}/command'),
            ('motor0/estimate', f'motor{id0}/estimate'),
            ('motor1/command', f'motor{id1}/command'),
            ('motor1/estimate', f'motor{id1}/estimate')
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        CanNode(0),
        CanNode(1),
        ODriveNode(0, 0),
        ODriveNode(1, 0),
        ODriveNode(2, 0),
        ODriveNode(3, 0),
        ODriveNode(4, 1),
        ODriveNode(5, 1),
        ODriveNode(6, 1),
        ODriveNode(7, 1),
        FiveBar2DNode('FL', 0, 1, True),
        FiveBar2DNode('RL', 2, 3, True),
        FiveBar2DNode('RR', 4, 5, False),
        FiveBar2DNode('FR', 6, 7, False)
    ])