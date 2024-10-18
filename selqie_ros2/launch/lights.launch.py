from launch import LaunchDescription
from launch_ros.actions import Node

def LightsNode():
    return Node(
        package='jetson_ros2',
        executable='gpio_node',
        name='camera_light_node',
        output='screen',
        parameters=[{
            'gpio_pin': 18,
            'is_pwm': True,
        }],
        remappings=[
            ('gpio/out', 'light/pwm'),
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        LightsNode(),
    ])