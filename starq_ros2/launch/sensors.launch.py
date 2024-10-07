import os
import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 
                                       'launch', 'microstrain_launch.py')

IMU_CONFIG_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('starq_ros2'),
                               'config', 'imu_cv7.yml')

def MicroStrainIMULaunch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
        launch_arguments={
            'configure': 'true',
            'activate': 'true',
            'params_file': IMU_CONFIG_FILE,
            'namespace': '/',
        }.items()
    )

def CameraLightNode():
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

def Bar100Node():
    return Node(
        package='bar100_ros2',
        executable='bar100_node',
        name='bar100_node',
        output='screen',
        parameters=[{
            'i2c-bus': 7,
            'frequency': 20.0,
        }],
    )

def generate_launch_description():
    return LaunchDescription([
        MicroStrainIMULaunch(),
        CameraLightNode(),
        Bar100Node(),
    ])