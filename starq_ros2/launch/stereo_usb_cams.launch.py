from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Left camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='left_camera',
            namespace='stereo/left',
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/left',
                'camera_info_url': 'file:///home/nvidia/slam_ws/starq_camera_slam/left.yaml',
                'frame_id': 'camera_left'
            }]
        ),
        # Right camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='right_camera',
            namespace='stereo/right',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/right',
                'camera_info_url': 'file:///home/nvidia/slam_ws/starq_camera_slam/right.yaml',
                'frame_id': 'camera_right'
            }]
        )
    ])
