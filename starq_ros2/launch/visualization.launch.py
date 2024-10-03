import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

starq_config_folder = os.path.join(get_package_share_directory('starq_ros2'), 'config')

def RVIZ2Node():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', starq_config_folder + '/rviz_config.rviz']
    )

def generate_launch_description():
    return LaunchDescription([
        RVIZ2Node()
    ])