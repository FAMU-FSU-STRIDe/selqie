import os
import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

GEAR_RATIO = 6.0

MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 
                                       'launch', 'microstrain_launch.py')

IMU_CONFIG_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('starq_ros2'),
                               'config', 'imu_cv7.yml')

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
    
def MicroStrainIMULaunch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
        launch_arguments={
            'configure': 'true',
            'activate': 'true',
            'params_file': IMU_CONFIG_FILE,
            'namespace': '/',
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='starq_actuation_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                CanBusNode("can0"),
                CanBusNode("can1"),
                ODriveCanNode(0, "can0"),
                ODriveCanNode(1, "can0"),
                ODriveCanNode(2, "can0"),
                ODriveCanNode(3, "can0"),
                ODriveCanNode(4, "can1"),
                ODriveCanNode(5, "can1"),
                ODriveCanNode(6, "can1"),
                ODriveCanNode(7, "can1"),
                FiveBar2DNode('FL', 0, 1, True),
                FiveBar2DNode('RL', 2, 3, True),
                FiveBar2DNode('RR', 4, 5, False),
                FiveBar2DNode('FR', 6, 7, False),
            ],
        ),
        MicroStrainIMULaunch()
    ])