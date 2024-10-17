from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'starq_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

def ExploreHDCameraNode(device, camera_name, cam_info_url):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=camera_name + "_camera",
        namespace='stereo/' + camera_name,
        parameters=[{
            'video_device': device,
            'camera_name': 'narrow_stereo/' + camera_name,
            'camera_info_url': cam_info_url,
            'frame_id': 'camera_' + camera_name,
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'io_method': 'mmap',
            'av_device_format': 'YUV422P',
            'brightness': -1,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        # Left camera node
        ExploreHDCameraNode('/dev/video4', 'left', LEFT_CAMERA_INFO_URL),
        # Right camera node
        ExploreHDCameraNode('/dev/video0', 'right', RIGHT_CAMERA_INFO_URL),
    ])