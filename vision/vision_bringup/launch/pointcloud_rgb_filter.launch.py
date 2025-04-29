from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Get the identifier from the launch configuration
    id = LaunchConfiguration('id').perform(context)
    
    # Get the RGB color from the launch configuration
    rgb = LaunchConfiguration('rgb').perform(context)
    
    # Get the RGB deviation from the launch configuration
    rgb_deviation = LaunchConfiguration('rgb_deviation').perform(context)
    
    return [
        Node(
            package='pointcloud_filters',
            executable='rgb_filter_node',
            name=f'{id}_rgb_filter_node',
            output='screen',
            parameters=[{
                'rgb': [int(c) for c in rgb.strip('[]').split(',')],
                'rgb_deviation': [int(d) for d in rgb_deviation.strip('[]').split(',')],
            }],
            remappings=[('points/in', 'stereo/points2'),
                        ('points/out', f'points/{id}')],
        ),
    ]

def generate_launch_description():
    # Identifier of the point cloud
    id_arg = DeclareLaunchArgument(
        'id',
        default_value='marker',
        description='Identifier of the point cloud'
    )

    # RGB argument for the point cloud RGB filter node
    rgb_arg = DeclareLaunchArgument(
        'rgb',
        default_value='[0 0 0]',
        description='RGB color of the point cloud in the format [R, G, B]'
    )

    # RGB deviation argument for the point cloud RGB filter node
    rgb_deviation_arg = DeclareLaunchArgument(
        'rgb_deviation',
        default_value='[0 0 0]',
        description='RGB deviation of the point cloud in the format [R, G, B]'
    )

    # Return the launch description with the node configuration
    return LaunchDescription([
        id_arg,
        rgb_arg,
        rgb_deviation_arg,
        OpaqueFunction(function=launch_setup),
    ])