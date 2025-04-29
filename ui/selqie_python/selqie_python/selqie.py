import os
import math
from threading import Thread, Event
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Empty, String, Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from actuation_msgs.msg import *
from leg_control_msgs.msg import *
from robot_localization.srv import SetPose

def QOS_FAST() -> QoSProfile:
    """Get a QoSProfile with best-effort reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

def QOS_RELIABLE() -> QoSProfile:
    """Get a QoSProfile with reliable reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )

def QUAT2EUL(q : Quaternion) -> list[float]:
    """Convert a Quaternion message to Euler angles."""
    q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return [roll, pitch, yaw]

def EUL2QUAT(eul) -> Quaternion:
    """Convert Euler angles to a Quaternion message."""
    cy = math.cos(eul[2] * 0.5)
    sy = math.sin(eul[2] * 0.5)
    cp = math.cos(eul[1] * 0.5)
    sp = math.sin(eul[1] * 0.5)
    cr = math.cos(eul[0] * 0.5)
    sr = math.sin(eul[0] * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class SELQIE(Node):
    """The main class for the SELQIE robot ROS2 interface."""

    ######################
    ### Initialization ###
    ######################

    def __init__(self, name="selqie"):
        super().__init__(name)
        self._stop_event = Event()
        
    def init(self):
        """Initialize all SELQIE components"""
        self.init_motors()
        self.init_legs()
        self.init_sensors()
        self.init_localization()
        self.init_mapping()
        self.init_control()
        self.init_vision()
        self.init_recording()

    def init_motors(self):
        """Initialize the motor publishers and subscribers."""
        self.NUM_MOTORS = 8
        self.DEFAULT_MOTOR_GAINS = [50.0, 0.025, 0.05]

        self._motor_command_publishers = []
        for i in range(self.NUM_MOTORS):
            self._motor_command_publishers.append(self.create_publisher(MotorCommand, f'motor{i}/command', QOS_RELIABLE()))

        self._motor_config_publishers = []
        for i in range(self.NUM_MOTORS):
            self._motor_config_publishers.append(self.create_publisher(MotorConfig, f'motor{i}/config', QOS_RELIABLE()))

        self._motor_estimates = [MotorEstimate() for _ in range(self.NUM_MOTORS)]
        self._motor_estimate_subscribers = []
        for i in range(self.NUM_MOTORS):
            motor_estimate_callback = lambda msg, i=i: self._motor_estimates.__setitem__(i, msg)
            self._motor_estimate_subscribers.append(self.create_subscription(MotorEstimate, f'motor{i}/estimate', motor_estimate_callback, QOS_FAST()))

        self._motor_infos = [MotorInfo() for _ in range(self.NUM_MOTORS)]
        self._motor_info_subscribers = []
        for i in range(self.NUM_MOTORS):
            motor_info_callback = lambda msg, i=i: self._motor_infos.__setitem__(i, msg)
            self._motor_info_subscribers.append(self.create_subscription(MotorInfo, f'motor{i}/info', motor_info_callback, QOS_FAST()))

    def init_legs(self):
        """Initialize the leg publishers and subscribers."""
        self.LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
        self.NUM_LEGS = len(self.LEG_NAMES)
        self.DEFAULT_LEG_POSITION = [0.0, 0.0, -0.18914]
        self.TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('leg_trajectory_publisher'), 'trajectories')

        self._leg_command_publishers = []
        for i in range(self.NUM_LEGS):
            self._leg_command_publishers.append(self.create_publisher(LegCommand, f'leg{self.LEG_NAMES[i]}/command', QOS_RELIABLE()))

        self._leg_estimates = [LegEstimate() for _ in range(self.NUM_LEGS)]
        self._leg_estimate_subscribers = []
        for i in range(self.NUM_LEGS):
            leg_estimate_callback = lambda msg, i=i: self._leg_estimates.__setitem__(i, msg)
            self._leg_estimate_subscribers.append(self.create_subscription(LegEstimate, f'leg{self.LEG_NAMES[i]}/estimate', leg_estimate_callback, QOS_FAST()))

        self._leg_trajectory_publishers = []
        for i in range(self.NUM_LEGS):
            self._leg_trajectory_publishers.append(self.create_publisher(LegTrajectory, f'leg{self.LEG_NAMES[i]}/trajectory', QOS_RELIABLE()))

    def init_sensors(self):
        """Initialize the sensor publishers and subscribers."""
        self._imu = Imu()
        imu_callback = lambda msg: setattr(self, '_imu', msg)
        self._imu_sub = self.create_subscription(Imu, 'imu', imu_callback, QOS_RELIABLE())

        self._pressure = Float32()
        pressure_callback = lambda msg: setattr(self, '_pressure', msg)
        self._pressure_sub = self.create_subscription(Float32, 'bar100/pressure', pressure_callback, QOS_RELIABLE())

        self._water_temperature = Float32()
        temperature_callback = lambda msg: setattr(self, '_water_temperature', msg)
        self._temperature_sub = self.create_subscription(Float32, 'bar100/temperature', temperature_callback, QOS_RELIABLE())

    def init_localization(self):
        """Initialize the localization publishers and subscribers."""
        self._odom = Odometry()
        odom_callback = lambda msg: setattr(self, '_odom', msg)
        self._odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, QOS_RELIABLE())

        self._set_pose_client = self.create_client(SetPose, 'set_pose')

        self._imu_calibrate_pub = self.create_publisher(Empty, 'imu/calibrate', QOS_RELIABLE())

    def init_mapping(self):
        """Initialize the mapping publishers and subscribers"""
        self._reset_map_pub = self.create_publisher(Empty, "map/reset", QOS_RELIABLE())

    def init_control(self):
        """Initialize the control publishers and subscribers."""
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', QOS_RELIABLE())

        self._goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', QOS_RELIABLE())

        self._gait_pub = self.create_publisher(String, 'gait', QOS_RELIABLE())

        self._gait = String()
        gait_callback = lambda msg: setattr(self, '_gait', msg)
        self._gait_sub = self.create_subscription(String, 'gait', gait_callback, QOS_RELIABLE())

    def init_vision(self):
        """Initialize the camera and light publishers and subscribers."""
        self._lights_pwm_pub = self.create_publisher(Float32, 'lights/pwm', QOS_RELIABLE())

        self._camera_left_image = Image()
        camera_left_callback = lambda msg: setattr(self, '_camera_left_image', msg)
        self._camera_left_sub = self.create_subscription(Image, 'stereo/left/image_raw', camera_left_callback, QOS_FAST())

        self._camera_right_image = Image()
        camera_right_callback = lambda msg: setattr(self, '_camera_right_image', msg)
        self._camera_right_sub = self.create_subscription(Image, 'stereo/right/image_raw', camera_right_callback, QOS_FAST())

    def init_recording(self):
        self.ROSBAG_RECORD_TOPICS = ["motor0/estimate","motor1/estimate","motor2/estimate","motor3/estimate","motor4/estimate", "motor5/estimate", "motor6/estimate", "motor7/estimate",
                                     "motor0/info", "motor1/info", "motor2/info", "motor3/info", "motor4/info", "motor5/info", "motor6/info", "motor7/info",
                                     "legFL/command", "legRL/command", "legRR/command", "legFR/command",
                                     "stereo/left/image_raw", "stereo/right/image_raw", "lights/pwm",
                                     "imu/data", "bar100/depth", "bar100/temperature",
                                     "gait", "cmd_vel/raw", "cmd_vel", "goal_pose", "goal_pose/local",
                                     "gait/transition", "gait/vel_estimate", "gait_planner/path",
                                     "odom", "set_pose", "walk_planner/path"]
        self.ROSBAG_SAVE_FOLDER = '/home/selqie/rosbags'
        self._rosbag_process = None

    ########################
    ### ROS2 Spin Thread ###
    ########################
    
    def _spin_loop(self):
        """ROS2 spinning in a background thread."""
        while not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def spin(self):
        """Start the ROS2 spinning loop."""
        rclpy.spin(self)
        
    def spin_background(self):
        """Start the ROS2 spinning thread."""
        self._spin_thread = Thread(target=self._spin_loop)
        self._spin_thread.start()
        

    def stop(self):
        """Stop the ROS2 spinning thread and clean up."""
        self._stop_event.set()
        self._spin_thread.join()
        self.destroy_node()

    #######################
    ### Motor Functions ###
    #######################

    def send_motor_config(self, motor_idx : int, config : MotorConfig):
        """Send an MotorConfig message to the motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        self._motor_config_publishers[motor_idx].publish(config)

    def set_motor_state(self, motor_idx : int, state : int):
        """Set the state of the motor."""
        config = MotorConfig()
        config.axis_state = state
        self.send_motor_config(motor_idx, config)

    def set_motor_idle(self, motor_idx : int):
        """Set the motor to AXIS_STATE_IDLE."""
        self.set_motor_state(motor_idx, MotorConfig.AXIS_STATE_IDLE)
    
    def set_motor_ready(self, motor_idx : int):
        """Set the motor to AXIS_STATE_CLOSED_LOOP_CONTROL."""
        self.set_motor_state(motor_idx, MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL)

    def set_motor_clear_errors(self, motor_idx : int):
        """Clear the errors on the motor."""
        config = MotorConfig()
        config.clear_errors = True
        self.send_motor_config(motor_idx, config)
    
    def set_motor_gains(self, motor_idx : int, p_gain : float, v_gain : float, v_int_gain : float):
        """Set the gains on the motor."""
        config = MotorConfig()
        config.pos_gain = p_gain
        config.vel_gain = v_gain
        config.vel_int_gain = v_int_gain
        self.send_motor_config(motor_idx, config)

    def set_motor_gains_default(self, motor_idx : int):
        """Set the default gains on the motor."""
        self.set_motor_gains(motor_idx, *self.DEFAULT_MOTOR_GAINS)

    def send_motor_command(self, motor_idx : int, command : MotorCommand):
        """Send a MotorCommand message to the motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        self._motor_command_publishers[motor_idx].publish(command)

    def set_motor_position(self, motor_idx : int, pos : float):
        """Set the position of the motor."""
        command = MotorCommand()
        command.control_mode = MotorCommand.CONTROL_MODE_POSITION
        command.pos_setpoint = pos
        self.send_motor_command(motor_idx, command)

    def get_motor_info(self, motor_idx : int) -> MotorInfo:
        """Get the latest MotorInfo message from the motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        return self._motor_infos[motor_idx]
    
    def get_motor_error_name(self, motor_idx : int) -> str:
        """Get the name of the error on the motor."""
        motor_info = self.get_motor_info(motor_idx)
        for attr_name in dir(MotorInfo):
            if attr_name.startswith("AXIS_ERROR_"):
                error_value = getattr(MotorInfo, attr_name)
                if error_value == motor_info.axis_error:
                    return attr_name
        return f"MULTIPLE_ERRORS ({motor_info.axis_error})"
    
    def get_motor_estimate(self, motor_idx : int) -> MotorEstimate:
        """Get the latest MotorEstimate message from the motor."""
        if motor_idx < 0 or motor_idx >= self.NUM_MOTORS:
            raise ValueError(f"Motor index {motor_idx} out of range")
        return self._motor_estimates[motor_idx]
    
    #####################
    ### Leg Functions ###
    #####################

    def send_leg_command(self, leg_idx : int, command : LegCommand):
        """Send a LegCommand message to the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        self._leg_command_publishers[leg_idx].publish(command)

    def set_leg_position(self, leg_idx : int, x : float, y : float, z : float):
        """Set the position of the leg."""
        command = LegCommand()
        command.control_mode = LegCommand.CONTROL_MODE_POSITION
        command.pos_setpoint.x = x
        command.pos_setpoint.y = y
        command.pos_setpoint.z = z
        self.send_leg_command(leg_idx, command)

    def set_leg_force(self, leg_idx : int, fx : float, fy : float, fz : float):
        """Set the force of the leg."""
        command = LegCommand()
        command.control_mode = LegCommand.CONTROL_MODE_FORCE
        command.force_setpoint.x = fx
        command.force_setpoint.y = fy
        command.force_setpoint.z = fz
        self.send_leg_command(leg_idx, command)

    def set_leg_position_default(self, leg_idx : int):
        """Set the leg to the default position."""
        self.set_leg_position(leg_idx, *self.DEFAULT_LEG_POSITION)

    def get_leg_estimate(self, leg_idx : int) -> LegEstimate:
        """Get the latest LegEstimate message from the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        return self._leg_estimates[leg_idx]
    
    def send_leg_trajectory(self, leg_idx : int, trajectory : LegTrajectory):
        """Send a LegTrajectory message to the leg."""
        if leg_idx < 0 or leg_idx >= self.NUM_LEGS:
            raise ValueError(f"Leg index {leg_idx} out of range")
        self._leg_trajectory_publishers[leg_idx].publish(trajectory)
    
    def get_leg_trajectories_from_file(self, rel_file : str, frequency : float) -> list[LegTrajectory]:
        """Get a list of LegTrajectory messages from a file."""
        file = os.path.join(self.TRAJECTORIES_FOLDER, rel_file)
        if not os.path.exists(file):
            raise FileNotFoundError(f'File {file} does not exist')
        leg_trajectories = [LegTrajectory() for _ in range(self.NUM_LEGS)]
        with open(file) as f:
            for line in f:
                parts = line.split()
                if len(parts) != 13:
                    raise ValueError(f'Invalid file line: {line}')
                time = float(parts[0]) / 1000.0 / frequency
                leg_id = int(parts[1])
                msg = LegCommand()
                msg.control_mode = int(parts[2])
                msg.pos_setpoint.x = float(parts[4])
                msg.pos_setpoint.y = float(parts[5])
                msg.pos_setpoint.z = float(parts[6])
                msg.vel_setpoint.x = float(parts[7])
                msg.vel_setpoint.y = float(parts[8])
                msg.vel_setpoint.z = float(parts[9])
                msg.force_setpoint.x = float(parts[10])
                msg.force_setpoint.y = float(parts[11])
                msg.force_setpoint.z = float(parts[12])
                if (leg_id > self.NUM_LEGS) or (leg_id < 0):
                    raise ValueError(f'Expected leg ids between 0 and {self.NUM_LEGS - 1}')
                leg_trajectories[leg_id].timing.append(time)
                leg_trajectories[leg_id].commands.append(msg)
        return leg_trajectories
    
    def run_leg_trajectories(self, trajectories : list[LegTrajectory]):
        """Run a list of LegTrajectory messages."""
        for i in range(len(trajectories)):
            if trajectories[i] is not None:
                self.send_leg_trajectory(i, trajectories[i])

    ############################
    ### Sensor Data Functions ##
    ############################

    def get_imu(self) -> Imu:
        """Get the latest Imu message."""
        return self._imu
    
    def get_pressure(self) -> Float32:
        """Get the latest depth message."""
        return self._pressure
    
    def get_water_temperature(self) -> Float32:
        """Get the latest water temperature message."""
        return self._water_temperature

    ##############################
    ### Localization Functions ###
    ##############################

    def get_localization(self) -> Odometry:
        """Get the latest Odometry message."""
        return self._odom
    
    def send_localization_set_pose(self, pose : PoseStamped):
        """Send a PoseStamped message to the set_pose service."""
        req = SetPose.Request()
        req.pose = pose
        self._set_pose_client.call_async(req)

    def set_localization_pose(self, x : float, y : float, z : float, theta : float):
        """Set the pose of the robot."""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = z
        pose.pose.pose.orientation = EUL2QUAT([0.0, 0.0, theta])
        self.send_localization_set_pose(pose)

    def set_localization_pose_zero(self):
        """Set the pose of the robot to zero."""
        self.set_localization_pose(0.0, 0.0, 0.0, 0.0)

    def send_localization_calibrate_imu(self):
        """Send an Empty message to the imu/calibrate topic."""
        self._imu_calibrate_pub.publish(Empty())
    
    #########################
    ### Mapping Functions ###
    #########################

    def send_mapping_reset(self):
        self._reset_map_pub.publish(Empty())

    #########################
    ### Control Functions ###
    #########################

    def send_control_command_velocity(self, cmd_vel : Twist):
        """Send a Twist message to the cmd_vel topic."""
        self._cmd_vel_pub.publish(cmd_vel)

    def set_control_command_velocity(self, linear_x : float, linear_z : float, angular_z : float):
        """Set the linear x, z, and angular z velocities of the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.z = angular_z
        self.send_control_command_velocity(cmd_vel)
    
    def send_control_goal_pose(self, goal_pose : PoseStamped):
        """Send a PoseStamped message to the goal_pose topic."""
        self._goal_pose_pub.publish(goal_pose)

    def set_control_goal_pose(self, x : float, y : float, theta : float):
        """Set the goal pose of the robot."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        self.send_control_goal_pose(goal_pose)
    
    def send_control_gait(self, gait : String):
        """Send a String message to the gait topic."""
        self._gait_pub.publish(gait)
        
    def set_control_gait(self, gait : str):
        """Set the gait of the robot."""
        msg = String()
        msg.data = gait
        self.send_control_gait(msg)

    def get_control_gait(self) -> String:
        """Get the latest gait message."""
        return self._gait
    
    ########################
    ### Vision Functions ###
    ########################

    def send_vision_lights_pwm(self, pwm : Float32):
        """Send a Float32 message to the lights pwm topic."""
        if pwm.data < 0.0 or pwm.data > 100.0:
            raise ValueError(f"Invalid PWM value {pwm.data}")
        self._lights_pwm_pub.publish(pwm)

    def set_vision_lights_brightness(self, brightness : float):
        """Set the brightness of the lights."""
        pwm = Float32()
        pwm.data = (1100.0 + 8.0 * brightness) / 200.0
        self.send_vision_lights_pwm(pwm)

    def get_vision_camera_left(self) -> Image:
        """Get the latest image from the left camera."""
        return self._camera_left_image

    def get_vision_camera_right(self) -> Image:
        """Get the latest image from the right camera."""
        return self._camera_right_image
    
    ################################
    ### Data Recording Functions ###
    ################################

    def is_recording(self) -> bool:
        """Check if the rosbag recording process is running."""
        return self._rosbag_process is not None

    def start_recording(self):
        """Start recording rosbag data to the specified output folder."""
        if self.is_recording():
            return
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self._rosbag_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', 
                                                 os.path.join(self.ROSBAG_SAVE_FOLDER, timestamp)] 
                                                 + self.ROSBAG_RECORD_TOPICS, stdin=subprocess.DEVNULL)

    def stop_recording(self):
        """Stop the rosbag recording process."""
        if not self.is_recording():
            return
        self._rosbag_process.terminate()
        self._rosbag_process.wait()
        self._rosbag_process = None