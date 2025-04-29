import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

MARKER_LOCALIZATION_CONFIG = os.path.join(
    get_package_share_directory('localization_bringup'), 'config', 'marker_localization.yaml')

def generate_launch_description():
    # Use sim time argument for the marker localization node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Pointcloud ID argument for the marker localization node
    id_arg = DeclareLaunchArgument(
        'id',
        default_value='filtered',
        description='Identifier of the point cloud'
    )
    # Get the identifier from the launch configuration
    id = LaunchConfiguration('id')
    
    # Marker x position argument for the marker localization node
    marker_x_arg = DeclareLaunchArgument(
        'marker_x',
        default_value='0.0',
        description='X position of the marker in meters'
    )
    # Get the marker x position from the launch configuration
    marker_x = LaunchConfiguration('marker_x')
    
    # Marker y position argument for the marker localization node
    marker_y_arg = DeclareLaunchArgument(
        'marker_y',
        default_value='0.0',
        description='Y position of the marker in meters'
    )
    # Get the marker y position from the launch configuration
    marker_y = LaunchConfiguration('marker_y')
    
    # Marker z position argument for the marker localization node
    marker_z_arg = DeclareLaunchArgument(
        'marker_z',
        default_value='0.0',
        description='Z position of the marker in meters'
    )
    # Get the marker z position from the launch configuration
    marker_z = LaunchConfiguration('marker_z')

    # Return the launch description with the node configuration
    return LaunchDescription([
        sim_time_arg,
        id_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[MARKER_LOCALIZATION_CONFIG, 
                        {
                            'use_sim_time': use_sim_time,
                            'marker_x': marker_x,
                            'marker_y': marker_y,
                            'marker_z': marker_z
                        }],
            remappings=[('points/marker', f'points/{id}'),
                        ('pose/marker', f'pose/{id}')]
        ),
    ])