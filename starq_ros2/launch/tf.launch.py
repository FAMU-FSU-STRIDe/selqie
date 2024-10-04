from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_FRAME = 'base_link'

IMU_POSITION = [0, 0, 0]
IMU_ORIENTATION = [0, 0, 0, 1]
IMU_FRAME = 'imu_link'

def StaticTransform(pos, ori, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[str(pos[0]), str(pos[1]), str(pos[2]), str(ori[0]), str(ori[1]), str(ori[2]), str(ori[3]), parent, child]
    )

def generate_launch_description():
    return LaunchDescription([
        StaticTransform(IMU_POSITION, IMU_ORIENTATION, ROBOT_FRAME, IMU_FRAME)
        # IMU to Camera Link
        # Camera Link to Camera Left
        # Camera Link to Camera Right
    ])