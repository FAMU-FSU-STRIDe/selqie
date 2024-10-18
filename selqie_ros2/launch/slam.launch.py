from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')

SLAM_CONFIG_FILE = os.path.join(CONFIG_FOLDER, '/slam_config.yaml')

def cuVSLAMContainer():
    return ComposableNodeContainer(
        name='slam_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='slam',
        composable_node_descriptions= [
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                parameters=[SLAM_CONFIG_FILE],
                remappings=[('visual_slam/image_0', '/stereo/left/image_rect'),
                            ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                            ('visual_slam/image_1', '/stereo/right/image_rect'),
                            ('visual_slam/camera_info_1', '/stereo/right/camera_info'),
                            ('visual_slam/imu', '/imu/data')
                ]
            )
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        #  SLAM container
        cuVSLAMContainer(),
    ])
