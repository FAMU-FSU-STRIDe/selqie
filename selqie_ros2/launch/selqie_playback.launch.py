from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def UseExtendedBag():
    launch_arg = DeclareLaunchArgument(
        'use_bag_ext', 
        default_value='false', 
        description='Use rosbag with extended topic remappings'
    )
    use_bag_ext = LaunchConfiguration('use_bag_ext')
    return launch_arg, use_bag_ext

import os
from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'selqie_ros2'
LAUNCH_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'launch')
CONFIG_FOLDER = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
IMU_CALIBRATION_FILE = os.path.join(CONFIG_FOLDER, 'imu_calibration.txt')

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def IncludeLaunchFile(name : str, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_FOLDER, name)
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'playback': 'true'
            }.items(),
        condition=condition
    )
    
from launch_ros.actions import Node
def Depth2PoseNode():
    return Node(
        package='bar100_ros2',
        executable='depth2pose_node',
        name='depth2pose_node',
        output='screen',
        parameters=[{
            'z_variance': 2.89,
            'use_sim_time': True
        }],
    )
    
def IMUCalibrationNode():
    return Node(
        package='selqie_localization',
        executable='imu_calibration_node',
        name='imu_calibration_node',
        output='screen',
        parameters=[{
            'sample_size': 500,
            'calibration_file': IMU_CALIBRATION_FILE,
            'use_sim_time': True
        }],
    )

def OdomTfPublisher(condition=None):
    return Node(
        package='selqie_ros2',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        condition=condition
    )

from launch.conditions import IfCondition, UnlessCondition
def generate_launch_description():
    launch_arg, use_bag_ext = UseExtendedBag()
    return LaunchDescription([
        launch_arg,
        IncludeLaunchFile('tf.launch.py'),
        IncludeLaunchFile('ekf.launch.py', UnlessCondition(use_bag_ext)),
        IncludeLaunchFile('stereo_cameras_disparity.launch.py'),
        IncludeLaunchFile('gaits.launch.py'),
        IncludeLaunchFile('visualization.launch.py'),
        IncludeLaunchFile('gait_planning.launch.py'),
        IncludeLaunchFile('local_planning.launch.py', IfCondition(use_bag_ext)),
        IncludeLaunchFile('terrain_mapping.launch.py'),
        IncludeLaunchFile('urdf.launch.py'),
        Depth2PoseNode(),
        IMUCalibrationNode(),
        OdomTfPublisher(IfCondition(use_bag_ext))
    ])