from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

ROCK_RGB = [144, 111, 102]
ROCK_RGB_DEVIATION = [20, 15, 15]

WALL_RGB = [117, 166, 171]
WALL_RGB_DEVIATION = [30, 20, 25]

def PointCloudRGBFilterNode(name, rgb, rgb_deviation):
    return Node(
        package='selqie_perception',
        executable='pointcloud_rgb_filter_node',
        name='pointcloud_rgb_filter_node',
        output='screen',
        parameters=[{
            'r': rgb[0],
            'g': rgb[1],
            'b': rgb[2],
            'r_deviation': rgb_deviation[0],
            'g_deviation': rgb_deviation[1],
            'b_deviation': rgb_deviation[2],
        }],
        remappings=[
            ('points/in', '/stereo/points2'),
            ('points/out', f'/stereo/points/{name}')
        ]
    )

def DefaultMapNode(use_sim_time : str):
    return Node(
        package='terrain_mapping',
        executable='default_map_node',
        name='default_map_node',
        output='screen',
        parameters=[{
            'map_frame_id': 'map',
            'map_resolution': 0.1,
            'map_length': 10.0,
            'use_sim_time': use_sim_time
        }],
    )

def GroundLevelMapNode(use_sim_time : str):
    return Node(
        package='terrain_mapping',
        executable='ground_level_node',
        name='ground_level_node',
        output='screen',
        parameters=[{
            'robot_height': 0.20,
            'frequency': 2.0,
            'use_sim_time': use_sim_time
        }],
    )

def PointCloudMapNode(layer_name, use_sim_time : str):
    return Node(
        package='terrain_mapping',
        executable='pointcloud_node',
        name='pointcloud_node',
        output='screen',
        parameters=[{
            'layer_name': 'map',
            'frequency': 2.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[('points', f'/stereo/points/{layer_name}')]
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        PointCloudRGBFilterNode('rock', ROCK_RGB, ROCK_RGB_DEVIATION),
        PointCloudRGBFilterNode('wall', WALL_RGB, WALL_RGB_DEVIATION),
        DefaultMapNode(use_sim_time),
        GroundLevelMapNode(use_sim_time),
        PointCloudMapNode('rock', use_sim_time),
        PointCloudMapNode('wall', use_sim_time)
    ])