import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from threading import Thread, Event
import subprocess

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from robot_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

NUM_MOTORS = 8
LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
DEFAULT_LEG_POSITION = [0.0, 0.0, -0.18914]
TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('selqie_ros2'), 'trajectories')
ROSBAG_RECORD_TOPICS = ['/legFR/command', '/legFL/command', '/legRR/command', '/legRL/command',
                        '/legFR/estimate', '/legFL/estimate', '/legRR/estimate', '/legRL/estimate',
                        '/odrive0/info', '/odrive1/info', '/odrive2/info', '/odrive3/info',
                        '/odrive4/info', '/odrive5/info', '/odrive6/info', '/odrive7/info',]
DEFAULT_GAINS = [20.0, 0.25, 0.0]

def qos_fast():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

def qos_reliable():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )

class SELQIE(Node):
    def __init__(self, name="robot"):
        super().__init__(name)
        self._stop_event = Event()

        self._init_motors()
        self._init_legs()
        self._init_localization()
        self._init_control()
        self._init_vision()

        self.rosbag_process = None

        # Start ROS spinning in a background thread
        self._spin_thread = Thread(target=self._spin)
        self._spin_thread.start()

    def _init_motors(self):
        """Initialize the motor publishers and subscribers."""
        self.motor_command_publishers = []
        for i in range(NUM_MOTORS):
            self.motor_command_publishers.append(self.create_publisher(MotorCommand, f'odrive{i}/command', qos_fast()))

        self.motor_config_publishers = []
        for i in range(NUM_MOTORS):
            self.motor_config_publishers.append(self.create_publisher(ODriveConfig, f'odrive{i}/config', qos_reliable()))

        self.motor_estimates = [MotorEstimate() for _ in range(NUM_MOTORS)]
        self.motor_estimate_subscribers = []
        for i in range(NUM_MOTORS):
            motor_estimate_callback = lambda msg, i=i: self.motor_estimates.__setitem__(i, msg)
            self.motor_estimate_subscribers.append(self.create_subscription(MotorEstimate, f'odrive{i}/estimate', motor_estimate_callback, qos_fast()))

        self.motor_infos = [ODriveInfo() for _ in range(NUM_MOTORS)]
        self.motor_info_subscribers = []
        for i in range(NUM_MOTORS):
            motor_info_callback = lambda msg, i=i: self.motor_infos.__setitem__(i, msg)
            self.motor_info_subscribers.append(self.create_subscription(ODriveInfo, f'odrive{i}/info', motor_info_callback, qos_fast()))

    def _init_legs(self):
        """Initialize the leg publishers and subscribers."""
        self.leg_command_publishers = []
        for i in range(len(LEG_NAMES)):
            self.leg_command_publishers.append(self.create_publisher(LegCommand, f'leg{LEG_NAMES[i]}/command', qos_fast()))

        self.leg_estimates = [LegEstimate() for _ in range(len(LEG_NAMES))]
        self.leg_estimate_subscribers = []
        for i in range(len(LEG_NAMES)):
            leg_estimate_callback = lambda msg, i=i: self.leg_estimates.__setitem__(i, msg)
            self.leg_estimate_subscribers.append(self.create_subscription(LegEstimate, f'leg{LEG_NAMES[i]}/estimate', leg_estimate_callback, qos_fast()))

        self.leg_trajectory_publishers = []
        for i in range(len(LEG_NAMES)):
            self.leg_trajectory_publishers.append(self.create_publisher(LegTrajectory, f'leg{LEG_NAMES[i]}/trajectory', qos_reliable()))

    def _init_localization(self):
        """Initialize the odometry subscriber."""
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, qos_reliable())

    def _init_control(self):
        """Initialize the control publishers and subscribers."""
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())

        self.gait_pub = self.create_publisher(String, 'gait', qos_reliable())

        self.gait = String()
        gait_callback = lambda msg: setattr(self, 'gait', msg)
        self.gait_sub = self.create_subscription(String, 'gait', gait_callback, qos_reliable())

    def _init_vision(self):
        """Initialize the camera and light publishers and subscribers."""
        self.lights_pwm_pub = self.create_publisher(Float32, 'lights/pwm', qos_reliable())

        self.camera_left_image = Image()
        camera_left_callback = lambda msg: setattr(self, 'camera_left_image', msg)
        self.camera_left_sub = self.create_subscription(Image, 'stereo/left/image_raw', camera_left_callback, qos_fast())

        self.camera_right_image = Image()
        camera_right_callback = lambda msg: setattr(self, 'camera_right_image', msg)
        self.camera_right_sub = self.create_subscription(Image, 'stereo/right/image_raw', camera_right_callback, qos_fast())

    def _spin(self):
        """ROS2 spinning in a background thread."""
        while not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        """Stop the ROS2 spinning thread and clean up."""
        self._stop_event.set()
        self._spin_thread.join()
        self.destroy_node()

    def send_motor_config(self, motor_idx, config : ODriveConfig):
        """Send an ODriveConfig message to the motor."""
        self.motor_config_publishers[motor_idx].publish(config)

    def send_motor_command(self, motor_idx, command : MotorCommand):
        """Send a MotorCommand message to the motor."""
        self.motor_command_publishers[motor_idx].publish(command)

    def get_motor_info(self, motor_idx) -> ODriveInfo:
        """Get the latest ODriveInfo message from the motor."""
        return self.motor_infos[motor_idx]
    
    def get_motor_estimate(self, motor_idx) -> MotorEstimate:
        """Get the latest MotorEstimate message from the motor."""
        return self.motor_estimates[motor_idx]
    
    def send_leg_command(self, leg_idx, command : LegCommand):
        """Send a LegCommand message to the leg."""
        self.leg_command_publishers[leg_idx].publish(command)

    def get_leg_estimate(self, leg_idx) -> LegEstimate:
        """Get the latest LegEstimate message from the leg."""
        return self.leg_estimates[leg_idx]
    
    def send_leg_trajectory(self, leg_idx, trajectory : LegTrajectory):
        """Send a LegTrajectory message to the leg."""
        self.leg_trajectory_publishers[leg_idx].publish(trajectory)

    def get_localization(self) -> Odometry:
        """Get the latest Odometry message."""
        return self.odom
    
    def send_control_velocity(self, cmd_vel : Twist):
        """Send a Twist message to the cmd_vel topic."""
        self.cmd_vel_pub.publish(cmd_vel)
    
    def send_control_gait(self, gait : String):
        """Send a String message to the gait topic."""
        self.gait_pub.publish(gait)

    def get_control_gait(self) -> String:
        """Get the latest gait message."""
        return self.gait
    
    def send_vision_lights_pwm(self, pwm : Float32):
        """Send a Float32 message to the lights pwm topic."""
        self.lights_pwm_pub.publish(pwm)

    def get_vision_camera_left(self) -> Image:
        """Get the latest image from the left camera."""
        return self.camera_left_image

    def get_vision_camera_right(self) -> Image:
        """Get the latest image from the right camera."""
        return self.camera_right_image
    
    def is_recording(self) -> bool:
        return self.rosbag_process is not None

    def start_recording(self, output_folder):
        if self.is_recording():
            return
        self.rosbag_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', output_folder] + ROSBAG_RECORD_TOPICS, stdin=subprocess.DEVNULL)

    def stop_recording(self):
        if not self.is_recording():
            return
        self.rosbag_process.terminate()
        self.rosbag_process.wait()
        self.rosbag_process = None