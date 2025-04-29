from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Get the ODrive ID from the launch configuration
    odrive_id = LaunchConfiguration('odrive_id').perform(context)
    
    # Get the interface from the launch configuration
    interface = LaunchConfiguration('interface').perform(context)
    
    # Get the gear ratio from the launch configuration
    gear_ratio = LaunchConfiguration('gear_ratio').perform(context)
    
    return [
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
    ]

def generate_launch_description():
    # ODrive ID argument for the ODrive motor
    odrive_id_arg = DeclareLaunchArgument(
        'odrive_id',
        default_value='0',
        description='The ODrive ID to use (default: 0)'
    )

    # Interface argument for the CAN bus
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='The CAN interface to use (default: can0)'
    )

    # Gear ratio argument for the ODrive motor
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='The gear ratio for the ODrive motor (default: 1.0)'
    )

    # Return the launch description with the node configuration
    return LaunchDescription([
        odrive_id_arg,
        interface_arg,
        gear_ratio_arg,
        OpaqueFunction(function=launch_setup)
    ])