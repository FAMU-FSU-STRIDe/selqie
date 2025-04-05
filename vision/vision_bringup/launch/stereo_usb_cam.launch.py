import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'vision_bringup'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

STEREO_USB_CAM_CONFIG = os.path.join(CONFIG_FOLDER, 'stereo_usb_cam.yaml')

LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

STEREO_IMAGE_PROC_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'stereo_image_proc.yaml')

def generate_launch_description():
    # Playback mode argument for stereo_usb_cam
    playback_mode_arg = DeclareLaunchArgument(
        'playback_mode',
        default_value='false',
        description='Enable or disable playback mode for stereo_usb_cam (default: false)'
    )
    playback_mode = LaunchConfiguration('playback_mode')

    # If in playback mode, use sim time
    use_sim_time = playback_mode

    return LaunchDescription([
        playback_mode_arg,
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
                        'playback': playback_mode,
                        'use_sim_time': use_sim_time,
                    }],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_left_node',
                    namespace='stereo/left',
                    remappings=[
                        ('image', 'image_raw'),
                        ('image_rect', 'image_rect')
                    ],
                    parameters=[{
                        'use_sim_time' : use_sim_time
                    }]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_right_node',
                    namespace='stereo/right',
                    remappings=[
                        ('image', 'image_raw'),
                        ('image_rect', 'image_rect')
                    ],
                    parameters=[{
                        'use_sim_time' : use_sim_time
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    namespace='stereo',
                    parameters=[STEREO_IMAGE_PROC_CONFIG_FILE, {
                        'use_sim_time' : use_sim_time
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name='point_cloud_node',
                    namespace='stereo',
                    parameters=[STEREO_IMAGE_PROC_CONFIG_FILE, {
                        'use_sim_time' : use_sim_time
                    }],
                    remappings=[
                        ('left/image_rect_color', 'left/image_rect'),
                        ('right/image_rect_color', 'right/image_rect')
                    ]
                )
            ]
        )
    ])