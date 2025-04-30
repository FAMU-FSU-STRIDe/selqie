import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

CONFIG_FOLDER = os.path.join(get_package_share_directory('vision_bringup'), 'config')
STEREO_USB_CAM_CONFIG = os.path.join(CONFIG_FOLDER, 'stereo_usb_cam.yaml')
LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='stereo_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='stereo',
            composable_node_descriptions= [
                ComposableNode(
                    package='stereo_usb_cam',
                    plugin='stereo_usb_cam::StereoUsbCam',
                    name="stereo_usb_cam_node",
                    namespace='stereo',
                    parameters=[STEREO_USB_CAM_CONFIG, {
                        'left_camera_info_url': LEFT_CAMERA_INFO_URL,
                        'right_camera_info_url': RIGHT_CAMERA_INFO_URL,
                    }],
                ),
            ]
        )
    ])