import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def CameraCalibrationNode():
    return Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name="camera_calibration",
        output='screen',
        arguments=[
            '--approximate', '0.1',
            '--size', '12x9',
            '--square', '0.020',
        ],
        remappings=[
            ('left', 'stereo/left/image_raw'),
            ('right', 'stereo/right/image_raw'),
            ('left_camera', 'stereo/left'),
            ('right_camera', 'stereo/right'),
        ]
    )


def generate_launch_description():
    return LaunchDescription([
        # Camera Calibration Node
        CameraCalibrationNode(),
    ])
