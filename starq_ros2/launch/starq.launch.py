import os
import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

GEAR_RATIO = 6.0

MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 
                                       'launch', 'microstrain_launch.py')

IMU_CONFIG_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('starq_ros2'),
                               'config', 'imu_cv7.yml')

def CanNode(ifc : int):
    return Node(
        package='can_bus',
        executable='can_bus_node',
        name=f'can{ifc}',
        output='screen',
        parameters=[{
            'interface': f'can{ifc}'
        }],
        remappings=[
            ('can/tx', f'can{ifc}/tx'),
            ('can/rx', f'can{ifc}/rx')
        ]
    )

def ODriveNode(id : int, ifc : int):
    return Node(
        package='odrive_ros2',
        executable='odrive_can_node',
        name=f'odrive_can{id}',
        output='screen',
        parameters=[{
            'id': id,
            'gear_ratio': GEAR_RATIO
        }],
        remappings=[
            ('can/tx', f'can{ifc}/tx'),
            ('can/rx', f'can{ifc}/rx'),
            ('odrive/command', f'odrive{id}/command'),
            ('odrive/config', f'odrive{id}/config'),
            ('odrive/estimate', f'odrive{id}/estimate'),
            ('odrive/info', f'odrive{id}/info')
        ]
    )

def FiveBar2DNode(name : str, id0 : int, id1 : int, flip_y : bool):
    return Node(
        package='leg_kinematics',
        executable='fivebar2d_node',
        name=f'fivebar2d{name}',
        output='screen',
        parameters=[{
            'flip_y': flip_y # Left: False, Right: True
        }],
        remappings=[
            ('leg/command', f'leg{name}/command'),
            ('leg/estimate', f'leg{name}/estimate'),
            ('motor0/command', f'odrive{id0}/command'),
            ('motor0/estimate', f'odrive{id0}/estimate'),
            ('motor1/command', f'odrive{id1}/command'),
            ('motor1/estimate', f'odrive{id1}/estimate')
        ]
    )
    
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

def generate_launch_description():
    return LaunchDescription([
        CanNode(0),
        CanNode(1),
        ODriveNode(0, 0),
        ODriveNode(1, 0),
        ODriveNode(2, 0),
        ODriveNode(3, 0),
        ODriveNode(4, 1),
        ODriveNode(5, 1),
        ODriveNode(6, 1),
        ODriveNode(7, 1),
        FiveBar2DNode('FL', 0, 1, True),
        FiveBar2DNode('RL', 2, 3, True),
        FiveBar2DNode('RR', 4, 5, False),
        FiveBar2DNode('FR', 6, 7, False),
        MicroStrainIMULaunch(),
        # Left camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='left_camera',
            namespace='stereo/left',
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/left',
                'camera_info_url': 'file:///home/nvidia/slam_ws/starq_camera_slam/left.yaml',
                'frame_id': 'camera_left'
            }]
        ),
        # Right camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='right_camera',
            namespace='stereo/right',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/right',
                'camera_info_url': 'file:///home/nvidia/slam_ws/starq_camera_slam/right.yaml',
                'frame_id': 'camera_right'
            }]
        )
    ])