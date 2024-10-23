from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

def ComposableStereoCameraNode():
    return ComposableNode(
        package='stereo_usb_cam',
        plugin='stereo_usb_cam::StereoUsbCam',
        name="stereo_camera",
        namespace='stereo',
        parameters=[{
            'width': 640,
            'height': 480,
            'framerate': 30.0,
            'left_video_device': '/dev/video4',
            'right_video_device': '/dev/video0',
            'left_frame_id': 'camera_left',
            'right_frame_id': 'camera_right',
            'left_camera_info_url': LEFT_CAMERA_INFO_URL,
            'right_camera_info_url': RIGHT_CAMERA_INFO_URL,
            'left_camera_name': 'narrow_stereo/left',
            'right_camera_name': 'narrow_stereo/right',
        }],
    )

def generate_launch_description():
    return LaunchDescription([
        #   Stereo cameras container
        ComposableNodeContainer(
            name='stereo_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='stereo',
            composable_node_descriptions= [
                # Stereo camera node
                ComposableStereoCameraNode(),
            ])
    ])
