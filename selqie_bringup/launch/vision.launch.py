import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

STEREO_USB_CAM_LAUNCH_FILE = os.path.join(
        get_package_share_directory('vision_bringup'), 'launch', 'stereo_usb_cam.launch.py')

LIGHT_PWM_PIN = 18

def generate_launch_description():
    # Playback mode argument for stereo_usb_cam
    playback_mode_arg = DeclareLaunchArgument(
        'playback_mode',
        default_value='false',
        description='Enable or disable playback mode for stereo_usb_cam (default: false)'
    )
    playback_mode = LaunchConfiguration('playback_mode')
    
    return LaunchDescription([
        playback_mode_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(STEREO_USB_CAM_LAUNCH_FILE),
            launch_arguments={'playback_mode': playback_mode}.items()
        ),
        Node(
            package='jetson_drivers',
            executable='gpio_node',
            name='camera_light_node',
            output='screen',
            parameters=[{
                'gpio_pin': LIGHT_PWM_PIN,
                'is_pwm': True,
            }],
            remappings=[
                ('gpio/out', 'lights/pwm'),
            ]
        )
    ])