from launch import LaunchDescription
from launch_ros.actions import Node

def CameraCalibrationNode(checker_size : str, square_size : str):
    return Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name="camera_calibration",
        output='screen',
        arguments=[
            '--size', checker_size,
            '--square', square_size,
            '--approximate', '0.05',
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
        # CameraCalibrationNode('32x22', '0.025'),
        CameraCalibrationNode('12x9', '0.020'),
    ])
