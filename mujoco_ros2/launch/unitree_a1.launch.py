from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

mujoco_models_folder = get_package_share_directory('mujoco_ros2') + '/models/'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_ros2',
            executable='mujoco_node',
            name='unitree_a1',
            output='screen',
            parameters=[{
                'model_path': mujoco_models_folder + 'unitree_a1/scene.xml',
                'frame_rate': 60.0,
            }]
        )
    ])