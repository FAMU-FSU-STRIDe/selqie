from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_FRAME = 'base_link'

IMU_POSITION = [0, 0, 0]
IMU_ORIENTATION = [0, 0, 0]
IMU_FRAME = 'imu_link'

CAMERA_LINK_POSITION = [0.38, 0, 0.3345]
CAMERA_LINK_ORIENTATION = [0, 0, 0]
CAMERA_LINK_FRAME = 'camera_link'

CAMERA_LEFT_POSITION = [0, 0.359, 0]
CAMERA_LEFT_ORIENTATION = [-1.5707, 0, -1.5707]
CAMERA_LEFT_FRAME = 'camera_left'

CAMERA_RIGHT_POSITION = [0, -0.359, 0]
CAMERA_RIGHT_ORIENTATION = [-1.5707, 0, -1.5707]
CAMERA_RIGHT_FRAME = 'camera_right'

BAR100_POSITION = [0, 0, 0.1]
BAR100_ORIENTATION = [0, 0, 0]
BAR100_FRAME = 'bar100_link'

def StaticTransform(pos, ori, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(pos[0]),
            '--y', str(pos[1]),
            '--z', str(pos[2]),
            '--roll', str(ori[0]),
            '--pitch', str(ori[1]),
            '--yaw', str(ori[2]),
            '--frame-id', parent,
            '--child-frame-id', child
        ]
    )

def generate_launch_description():
    return LaunchDescription([
        # Base Link to IMU Link
        StaticTransform(IMU_POSITION, IMU_ORIENTATION, ROBOT_FRAME, IMU_FRAME),
        # IMU to Camera Link
        StaticTransform(CAMERA_LINK_POSITION, CAMERA_LINK_ORIENTATION, ROBOT_FRAME, CAMERA_LINK_FRAME),
        # Camera Link to Camera Left
        StaticTransform(CAMERA_LEFT_POSITION, CAMERA_LEFT_ORIENTATION, CAMERA_LINK_FRAME, CAMERA_LEFT_FRAME),
        # Camera Link to Camera Right
        StaticTransform(CAMERA_RIGHT_POSITION, CAMERA_RIGHT_ORIENTATION, CAMERA_LINK_FRAME, CAMERA_RIGHT_FRAME),
        # Base Link to Bar100 Link
        StaticTransform(BAR100_POSITION, BAR100_ORIENTATION, ROBOT_FRAME, BAR100_FRAME)
    ])