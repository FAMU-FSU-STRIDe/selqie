import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

FIVEBAR_CONFIG_FILE = os.path.join(get_package_share_directory('sensing_bringup'), 'config', 'bar100.yaml')

def generate_launch_description():
    # Leg name argument for the leg
    leg_name_arg = DeclareLaunchArgument(
        'leg_name',
        default_value='FL',
        description='The name of the leg (default: FL)'
    )
    # Get the leg name from the launch configuration
    leg_name = TextSubstitution(text=LaunchConfiguration('leg_name'))
    
    # Motor id argument for the front motor
    motor0_arg = DeclareLaunchArgument(
        'motor0',
        default_value='0',
        description='The motor number for motor0 (default: 0)'
    )
    # Get the motor id from the launch configuration
    motor0 = TextSubstitution(text=LaunchConfiguration('motor0'))
    
    # Motor id argument for the back motor
    motor1_arg = DeclareLaunchArgument(
        'motor1',
        default_value='1',
        description='The motor number for motor1 (default: 1)'
    )
    # Get the motor id from the launch configuration
    motor1 = TextSubstitution(text=LaunchConfiguration('motor1'))
    
    # Flip y-axis argument for the leg
    flip_y_arg = DeclareLaunchArgument(
        'flip_y',
        default_value='true',
        description='Flip the y-axis (left: true, right: false) (default: true)'
    )
    # Get the flip y-axis configuration from the launch configuration
    flip_y = LaunchConfiguration('flip_y')
    
    # Return the launch description with the node configuration
    return LaunchDescription([
        leg_name_arg,
        motor0_arg,
        motor1_arg,
        flip_y_arg,
        Node(
            package='leg_kinematics',
            executable='fivebar2d_node',
            name=f'fivebar2d_node',
            output='screen',
            parameters=[FIVEBAR_CONFIG_FILE,
                        {'flip_y': flip_y}],
            remappings=[
                ('leg/command', f'leg{leg_name}/command'),
                ('leg/estimate', f'leg{leg_name}/estimate'),
                ('motor0/command', f'motor{motor0}/command'),
                ('motor0/estimate', f'motor{motor0}/estimate'),
                ('motor1/command', f'motor{motor1}/command'),
                ('motor1/estimate', f'motor{motor1}/estimate'),
            ],
        ),
    ])