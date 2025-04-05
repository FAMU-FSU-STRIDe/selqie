from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Checkerboard size argument for camera calibration
    checker_size_arg = DeclareLaunchArgument(
        'checker_size',
        default_value='12x9',
        description='The size of the checkerboard (default: 12x9)'
    )
    checker_size = LaunchConfiguration('checker_size')

    # Square size argument for camera calibration
    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.020',
        description='The size of each square in meters (default: 0.020)'
    )
    square_size = LaunchConfiguration('square_size')

    return LaunchDescription([
        checker_size_arg,
        square_size_arg,
        Node(
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
        ),
    ])
