from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

MAP_FRAME = 'map'

ODOM_FRAME = 'odom'
INITIAL_ODOM_POSITION = [0, 0, 0]
INITIAL_ODOM_ORIENTATION = [0, 0, 0]

ROBOT_FRAME = 'base_link'
INITIAL_ROBOT_POSITION = [0, 0, 0]
INITIAL_ROBOT_ORIENTATION = [0, 0, 0]

IMU_FRAME = 'imu_link'
IMU_POSITION = [0, 0, 0]
IMU_ORIENTATION = [3.14159, 0, 0]

CAMERA_LINK_FRAME = 'camera_link'
CAMERA_LINK_POSITION = [0.315, 0, 0.075]
CAMERA_LINK_ORIENTATION = [0, 0, 0]

CAMERA_LEFT_FRAME = 'camera_left'
CAMERA_LEFT_POSITION = [0, 0.065, 0]
CAMERA_LEFT_ORIENTATION = [-1.5707, 0, -1.5707]

CAMERA_RIGHT_FRAME = 'camera_right'
CAMERA_RIGHT_POSITION = [0, -0.065, 0]
CAMERA_RIGHT_ORIENTATION = [-1.5707, 0, -1.5707]

def StaticTransformNode(pos, ori, parent, child, use_sim_time):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(pos[0]), '--y', str(pos[1]), '--z', str(pos[2]),
            '--roll', str(ori[0]), '--pitch', str(ori[1]), '--yaw', str(ori[2]),
            '--frame-id', parent, '--child-frame-id', child
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

def generate_launch_description():
    # Use sim time argument for the stride generation node
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Get the use sim time from the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        sim_time_arg,
        # Odom to Map
        StaticTransformNode(INITIAL_ODOM_POSITION, INITIAL_ODOM_ORIENTATION, MAP_FRAME, ODOM_FRAME, use_sim_time),
        # Base Link to Odom
        StaticTransformNode(INITIAL_ROBOT_POSITION, INITIAL_ROBOT_ORIENTATION, ODOM_FRAME, ROBOT_FRAME, use_sim_time),
        # Base Link to IMU Link
        StaticTransformNode(IMU_POSITION, IMU_ORIENTATION, ROBOT_FRAME, IMU_FRAME, use_sim_time),
        # IMU to Camera Link
        StaticTransformNode(CAMERA_LINK_POSITION, CAMERA_LINK_ORIENTATION, ROBOT_FRAME, CAMERA_LINK_FRAME, use_sim_time),
        # Camera Link to Camera Left
        StaticTransformNode(CAMERA_LEFT_POSITION, CAMERA_LEFT_ORIENTATION, CAMERA_LINK_FRAME, CAMERA_LEFT_FRAME, use_sim_time),
        # Camera Link to Camera Right
        StaticTransformNode(CAMERA_RIGHT_POSITION, CAMERA_RIGHT_ORIENTATION, CAMERA_LINK_FRAME, CAMERA_RIGHT_FRAME, use_sim_time)
    ])