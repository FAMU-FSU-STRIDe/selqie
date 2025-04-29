import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

MARKER_LOCALIZATION_CONFIG = os.path.join(
    get_package_share_directory('localization_bringup'), 'config', 'marker_localization.yaml')

def launch_setup(context, *args, **kwargs):
    # Get the identifier from the launch configuration
    id = LaunchConfiguration('id').perform(context)
    
    # Get the marker x position from the launch configuration
    marker_x = LaunchConfiguration('marker_x').perform(context)
    
    # Get the marker y position from the launch configuration
    marker_y = LaunchConfiguration('marker_y').perform(context)

    # Get the marker z position from the launch configuration
    marker_z = LaunchConfiguration('marker_z').perform(context)
    
    return [
        Node(
            package='marker_localization',
            executable='marker_localization_node',
            name=f'{id}_localization_node',
            output='screen',
            parameters=[MARKER_LOCALIZATION_CONFIG, {
                'marker_x': float(marker_x),
                'marker_y': float(marker_y),
                'marker_z': float(marker_z)
            }],
            remappings=[('points/marker', f'points/{id}'),
                        ('pose/marker', f'pose/{id}')]
        ),
    ]

def generate_launch_description():    
    # Pointcloud ID argument for the marker localization node
    id_arg = DeclareLaunchArgument(
        'id',
        default_value='marker',
        description='Identifier of the point cloud'
    )

    # Marker x position argument for the marker localization node
    marker_x_arg = DeclareLaunchArgument(
        'marker_x',
        default_value='0.0',
        description='X position of the marker in meters'
    )
    
    # Marker y position argument for the marker localization node
    marker_y_arg = DeclareLaunchArgument(
        'marker_y',
        default_value='0.0',
        description='Y position of the marker in meters'
    )

    # Marker z position argument for the marker localization node
    marker_z_arg = DeclareLaunchArgument(
        'marker_z',
        default_value='0.0',
        description='Z position of the marker in meters'
    )

    # Return the launch description with the node configuration
    return LaunchDescription([
        id_arg,
        marker_x_arg,
        marker_y_arg,
        marker_z_arg,
        OpaqueFunction(function=launch_setup),
    ])