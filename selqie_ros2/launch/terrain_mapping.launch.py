from launch import LaunchDescription
from launch_ros.actions import Node

def TerrainMappingNode():
    return Node(
        package='terrain_mapping',
        executable='terrain_mapping_node',
        name='terrain_mapping_node',
        output='screen',
        parameters=[{
            'map_frame_id': 'odom',
            'map_resolution': 0.1,
            'map_length': 10.0,
        }],
        remappings=[
            ('/points', '/stereo/points2'),
            ('/map', '/map/raw')
        ]
    )
def generate_launch_description():
    return LaunchDescription([
        TerrainMappingNode(),
    ])