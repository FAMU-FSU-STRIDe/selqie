from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Model path argument for the MuJoCo model
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        description='Path to the MuJoCo model file'
    )
    # Get the model path from the launch configuration
    model_path = LaunchConfiguration('model_path')

    # Frame rate argument for the simulation
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='60.0',
        description='Frame rate for the simulation'
    )
    # Get the frame rate from the launch configuration
    frame_rate = LaunchConfiguration('frame_rate')

    # Return the launch description with the node configuration
    return LaunchDescription([
        model_path_arg,
        frame_rate_arg,
        Node(
            package='mujoco_ros2',
            executable='mujoco_node',
            name='mujoco',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'frame_rate': frame_rate,
            }]
        ),
    ])