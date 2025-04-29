from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ODrive ID argument for the ODrive motor
    odrive_id_arg = DeclareLaunchArgument(
        'odrive_id',
        default_value='0',
        description='The ODrive ID to use (default: 0)'
    )
    # Get the ODrive ID from the launch configuration
    odrive_id = LaunchConfiguration('odrive_id')

    # Interface argument for the CAN bus
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='The CAN interface to use (default: can0)'
    )
    # Get the interface from the launch configuration
    interface = LaunchConfiguration('interface')

    # Gear ratio argument for the ODrive motor
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='The gear ratio for the ODrive motor (default: 1.0)'
    )
    # Get the gear ratio from the launch configuration
    gear_ratio = LaunchConfiguration('gear_ratio')

    # Return the launch description with the node configuration
    return LaunchDescription([
        odrive_id_arg,
        interface_arg,
        gear_ratio_arg,
        Node(
            package='odrive_control',
            executable='odrive_can_node',
            name=f'odrive{odrive_id}_node',
            output='screen',
            parameters=[{
                'id': odrive_id,
                'gear_ratio': gear_ratio
            }],
            remappings=[
                ('can/tx', f'{interface}/tx'),
                ('can/rx', f'{interface}/rx'),
                ('motor/command', f'motor{odrive_id}/command'),
                ('motor/config', f'motor{odrive_id}/config'),
                ('motor/estimate', f'motor{odrive_id}/estimate'),
                ('motor/info', f'motor{odrive_id}/info')
            ],
        ),
    ])