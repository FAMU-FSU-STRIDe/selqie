import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'starq_ros2'
config_folder = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

LEFT_CAMERA_INFO_URL = 'file://' + config_folder + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + config_folder + '/calibration_right.yaml'

def ExploreHDCameraNode(device, camera_name, cam_info_url):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=camera_name + "_camera",
        namespace='stereo/' + camera_name,
        parameters=[{
            'video_device': device,
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'camera_name': 'narrow_stereo/' + camera_name,
            'camera_info_url': cam_info_url,
            'frame_id': 'camera_' + camera_name
        }]
    )

def StereoCameraContainer():
    return ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='stereo/',
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
    ),


def generate_launch_description():
    return LaunchDescription([
        # Left camera node
        ExploreHDCameraNode('/dev/video4', 'left', LEFT_CAMERA_INFO_URL),
        # Right camera node
        ExploreHDCameraNode('/dev/video0', 'right', RIGHT_CAMERA_INFO_URL),
        #   Image rectification container
        StereoCameraContainer(),
    ])
