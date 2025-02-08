from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseSimTime():
    launch_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return launch_arg, use_sim_time

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

LEFT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_left.yaml'
RIGHT_CAMERA_INFO_URL = 'file://' + CONFIG_FOLDER + '/calibration_right.yaml'

DISPARITY_CONFIG_FILE = os.path.join(CONFIG_FOLDER, 'disparity_config.yaml')
    
def ComposableStereoCameraNode(use_sim_time):
    return ComposableNode(
        package='stereo_usb_cam',
        plugin='stereo_usb_cam::StereoUsbCam',
        name="stereo_camera",
        namespace='stereo',
        parameters=[{
            'playback': True,
            'left_frame_id': 'camera_left',
            'right_frame_id': 'camera_right',
            'left_camera_info_url': LEFT_CAMERA_INFO_URL,
            'right_camera_info_url': RIGHT_CAMERA_INFO_URL,
            'left_camera_name': 'narrow_stereo/left',
            'right_camera_name': 'narrow_stereo/right',
            'use_sim_time': use_sim_time
        }]
    )
    
def ComposableRectifyNode(camera_name, use_sim_time):
    return ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_mono',
        namespace='stereo/' + camera_name,
        remappings=[
            ('image', 'image_raw'),
            ('image_rect', 'image_rect')
        ],
        parameters=[{
            'use_sim_time' : use_sim_time
        }]
    )
    
def ComposableDisparityNode(use_sim_time):
    return ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::DisparityNode',
        namespace='stereo',
        parameters=[DISPARITY_CONFIG_FILE, {
            'use_sim_time' : use_sim_time
        }]
    )
    
def ComposablePointCloudNode(use_sim_time):
    return ComposableNode(
        package='stereo_image_proc',
        plugin='stereo_image_proc::PointCloudNode',
        namespace='stereo',
        parameters=[{
            'use_color': True,
            'approximate_sync': True,
            'use_sim_time' : use_sim_time
        }],
        remappings=[
            ('left/image_rect_color', 'left/image_rect'),
            ('right/image_rect_color', 'right/image_rect')
        ]
    )

def generate_launch_description():
    launch_args, use_sim_time = UseSimTime()
    return LaunchDescription([
        launch_args,
        ComposableNodeContainer(
            name='stereo_disparity_container',
            package='rclcpp_components',
            executable='component_container_mt',
            namespace='stereo',
            composable_node_descriptions= [
                # Stereo camera node
                ComposableStereoCameraNode(use_sim_time),
                # Image rectification for the left camera
                ComposableRectifyNode('left', use_sim_time),
                # Image rectification for the right camera
                ComposableRectifyNode('right', use_sim_time),
                # Disparity Map
                ComposableDisparityNode(use_sim_time),
                # Point cloud 
                ComposablePointCloudNode(use_sim_time),
            ],
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
        )
    ])
