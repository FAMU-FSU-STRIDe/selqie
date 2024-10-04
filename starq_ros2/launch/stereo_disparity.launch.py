from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def StereoDisparityContainer():
    return ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='stereo',
        composable_node_descriptions= [
            # Separate mono and color for the left camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer',
                namespace='stereo/left'
            ),
            # Separate mono and color for the right camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer',
                namespace='stereo/right'
            ),
            # Mono image rectification for the left camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono',
                namespace='stereo/left',
                remappings=[
                    ('image', 'image_mono'),
                    ('image_rect', 'image_rect')
                ],
            ),
            # Mono image rectification for the right camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono',
                namespace='stereo/right',
                remappings=[
                    ('image', 'image_mono'),
                    ('image_rect', 'image_rect')
                ],
            ),
            # Color image rectification for the left camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color',
                namespace='stereo/left',
                remappings=[
                    ('image', 'image_color'),
                    ('image_rect', 'image_rect_color')
                ],
            ),
            # Color image rectification for the right camera
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color',
                namespace='stereo/right',
                remappings=[
                    ('image', 'image_color'),
                    ('image_rect', 'image_rect_color')
                ],
            ),
            # Disparity Map
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                namespace='stereo',
                parameters=[{
                    'approximate_sync': True
                }]
            ),
            # Point cloud 
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                namespace='stereo',
                parameters=[{
                    'approximate_sync': True,
                    'use_color': True
                }]
            ),
        ]
    )


def generate_launch_description():
    return LaunchDescription([
        #   Stereo Disparity container
        StereoDisparityContainer(),
    ])
