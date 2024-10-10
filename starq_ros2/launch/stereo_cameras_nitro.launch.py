from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'starq_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

CAMERA_CONFIG = os.path.join(CONFIG_FOLDER, 'camera_config.yaml')

LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

def ComposableUSBCameraNode(device, camera_name, cam_info_url):
    return ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name=camera_name + "_camera",
        namespace='stereo/' + camera_name,
        parameters=[{
            'video_device': device,
            'camera_name': 'narrow_stereo/' + camera_name,
            'camera_info_url': cam_info_url,
            'frame_id': 'camera_' + camera_name,
        }, CAMERA_CONFIG],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info_raw'),
        ]
    )

def ComposableSyncStereoNode():
    return ComposableNode(
        package='isaac_ros_nitros_topic_tools',
        plugin='nvidia::isaac_ros::nitros::NitrosCameraDrop',
        name='nitros_camera_drop',
        namespace='stereo',
        remappings=[
            ('image_1', 'left/image_raw'),
            ('camera_info_1', 'left/camera_info_raw'),
            ('image_2', 'right/image_raw'),
            ('camera_info_2', 'right/camera_info_raw'),
            ('image_1_drop', 'left/image'),
            ('camera_info_1_drop', 'left/camera_info'),
            ('image_2_drop', 'right/image'),
            ('camera_info_2_drop', 'right/camera_info'),
        ],
        parameters=[{
            'mode': 'stereo',
        }]
    )

def ComposableRectifyNode(camera_name):
    return ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify_' + camera_name,
        namespace='stereo/' + camera_name,
        remappings=[
            ('crop/image', 'image_rect')
            ('crop/camera_info', 'camera_info_rect')
        ],
        parameters=[{
            'output_width': 640,
            'output_height': 480,
        }]
    )

def ComposableDisparityNode():
    return ComposableNode(
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
        namespace='stereo',
        parameters=[{
            'max_disparity': 64,
            'backends': 'CUDA',
        }]
    )

def ComposablePointCloudNode():
    return ComposableNode(
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::PointCloudNode',
        namespace='stereo',
        parameters=[{
            'use_color': True,
        }]
    )

def StereoCameraContainer():
    return ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='stereo',
        composable_node_descriptions= [
            # Left camera node
            ComposableUSBCameraNode('/dev/video4', 'left', LEFT_CAMERA_INFO_URL),
            # Right camera node
            ComposableUSBCameraNode('/dev/video0', 'right', RIGHT_CAMERA_INFO_URL),
            # Sync stereo node
            ComposableSyncStereoNode(),
            # Rectify left camera node
            ComposableRectifyNode('left'),
            # Rectify right camera node
            ComposableRectifyNode('right'),
            # Disparity node
            ComposableDisparityNode(),
            # Point cloud node
            ComposablePointCloudNode(),
        ]
    )


def generate_launch_description():
    return LaunchDescription([
        # Stereo Camera container
        StereoCameraContainer(),
    ])
