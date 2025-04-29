import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Identifier of the point cloud
    id_arg = DeclareLaunchArgument(
        'id',
        default_value='filtered',
        description='Identifier of the point cloud'
    )
    # Get the identifier from the launch configuration
    id = LaunchConfiguration('id')
    
    # RGB argument for the point cloud RGB filter node
    rgb_arg = DeclareLaunchArgument(
        'rgb',
        default_value='[0.0, 0.0, 0.0]',
        description='RGB color of the point cloud in the format [R, G, B]'
    )
    # Get the RGB color from the launch configuration
    rgb = LaunchConfiguration('rgb')
    
    # RGB deviation argument for the point cloud RGB filter node
    rgb_deviation_arg = DeclareLaunchArgument(
        'rgb_deviation',
        default_value='[0.0, 0.0, 0.0]',
        description='RGB deviation of the point cloud in the format [R, G, B]'
    )
    # Get the RGB deviation from the launch configuration
    rgb_deviation = LaunchConfiguration('rgb_deviation')

    # Return the launch description with the node configuration
    return LaunchDescription([
        id_arg,
        rgb_arg,
        rgb_deviation_arg,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                        {
                            'rgb': rgb,
                            'rgb_deviation': rgb_deviation,
                        }],
            remappings=[('points/in', 'stereo/points2'),
                        ('points/out', f'points/{id}')],
        ),
    ])