import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

starq_config_folder = os.path.join(get_package_share_directory('starq_ros2'), 'config')

def RobotLocalizationNode():
    return Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[starq_config_folder + '/ekf_config.yaml']
    )

def generate_launch_description():
    return LaunchDescription([
        # Robot Localization Node
        RobotLocalizationNode()
    ])