import time
import threading
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Imu
from robot_msgs.msg import *

from robot_utils.utils.robot_util_functions import *
from robot_utils.utils.starq_util_functions import *

INF_LOOP = -1
NUM_MOTORS = 8
JOY_DT = 1.0 / 1000.0
STANDING_LEG_POSITION = [0.0, 0.0, -0.18914]
STANDING_GAINS = [10.0, 0.1, 0.2]
LANDING_LEG_POSITION = [0.0, 0.0, -0.18914]
LANDING_GAINS = [5.0, 0.05, 0.10]

class STARQGaitJoystick(Node):
    def __init__(self):
        super().__init__('starq_gait_joystick')

        self.declare_parameter('num_motors', NUM_MOTORS)
        self.num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value
        
        self.declare_parameter('leg_names', ['FL', 'RL', 'RR', 'FR'])
        self.leg_names = self.get_parameter('leg_names').get_parameter_value().string_array_value
        num_legs = len(self.leg_names)
        
        self.declare_parameter('standing_leg_position', STANDING_LEG_POSITION)
        self.standing_leg_position = self.get_parameter('standing_leg_position').get_parameter_value().double_array_value
        
        self.declare_parameter('walk_trajectory_file', 'walk.txt')
        self.walk_file = self.get_parameter('walk_trajectory_file').get_parameter_value().string_value
        self.declare_parameter('walk_frequency', 1.0)
        self.walk_freq = self.get_parameter('walk_frequency').get_parameter_value().double_value
        self.walk_trajectory = get_trajectories_from_file(self.walk_file, self.walk_freq, num_legs)
        
        self.declare_parameter('jump_trajectory_file', 'jump.txt')
        self.jump_file = self.get_parameter('jump_trajectory_file').get_parameter_value().string_value
        self.declare_parameter('jump_frequency', 0.75)
        self.jump_freq = self.get_parameter('jump_frequency').get_parameter_value().double_value
        self.jump_trajectory = get_trajectories_from_file(self.jump_file, self.jump_freq, num_legs)

        self.declare_parameter('swim_trajectory_file', 'swim.txt')
        self.swim_file = self.get_parameter('swim_trajectory_file').get_parameter_value().string_value
        self.declare_parameter('swim_frequency', 3.5)
        self.swim_freq = self.get_parameter('swim_frequency').get_parameter_value().double_value
        self.swim_trajectory = get_trajectories_from_file(self.swim_file, self.swim_freq, num_legs)
        
        self.declare_parameter('crawl_trajectory_file', 'crawl.txt')
        self.crawl_file = self.get_parameter('crawl_trajectory_file').get_parameter_value().string_value
        self.declare_parameter('crawl_frequency', 1.0)
        self.crawl_freq = self.get_parameter('crawl_frequency').get_parameter_value().double_value
        self.crawl_trajectory = get_trajectories_from_file(self.crawl_file, self.crawl_freq, num_legs)
        
        self.odrive_command_pubs = []
        for i in range(self.num_motors):
            self.odrive_command_pubs.append(self.create_publisher(MotorCommand, f'odrive{i}/command', qos_fast()))
            
        self.odrive_config_pubs = []
        for i in range(self.num_motors):
            self.odrive_config_pubs.append(self.create_publisher(MotorConfig, f'odrive{i}/config', qos_reliable()))

        self.leg_traj_pubs = []
        for l in self.leg_names:
            self.leg_traj_pubs.append(self.create_publisher(LegTrajectory, f'leg{l}/trajectory', qos_reliable()))

        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        self.last_msg = None
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.curr_traj = None
        self.curr_loops = 0
        self.curr_freq = 0.0
        self.loop_idx = 0
        
        self.get_logger().info('STARQ Gait Joystick Node Initialized')
        
    def set_trajectory(self, traj, loops : int, frequency : float):
        self.curr_traj = traj
        self.curr_loops = loops
        self.curr_freq = frequency
        self.loop_idx = 0
        
    def run(self):
        while rclpy.ok():
            if self.curr_traj is None:
                time.sleep(0.05)
            elif self.loop_idx != self.curr_loops:
                for i in range(len(self.leg_traj_pubs)):
                    self.leg_traj_pubs[i].publish(self.curr_traj[i])
                time.sleep(1.0 / self.curr_freq)
                self.loop_idx += 1
            else:
                self.set_trajectory(None, 0)

    def joy_callback(self, msg : Joy):
        if self.last_msg is None:
            pass
        elif msg.buttons[0] == 1 and self.last_msg.buttons[0] == 0:
            # 1 : Walk
            self.set_trajectory(self.walk_trajectory, INF_LOOP, self.walk_freq)
            self.get_logger().info('Walking...')
        elif msg.buttons[1] == 1 and self.last_msg.buttons[1] == 0:
            # 2 : Jump
            self.set_trajectory(self.jump_trajectory, 1, self.jump_freq)
            self.get_logger().info('Jumping...')
        elif msg.buttons[2] == 1 and self.last_msg.buttons[2] == 0:
            # 3 : Swim (TODO)
            self.set_trajectory(self.swim_trajectory, INF_LOOP, self.swim_freq)
            self.get_logger().info('Swimming...')
        elif msg.buttons[3] == 1 and self.last_msg.buttons[3] == 0:
            # 4 : Crawl
            self.set_trajectory(self.crawl_trajectory, INF_LOOP, self.crawl_freq)
            self.get_logger().info('Crawling...')
        elif msg.buttons[4] == 1 and self.last_msg.buttons[4] == 0:
            # LB : Stand
            self.curr_traj = None
            set_leg_states(self.leg_traj_pubs, MotorCommand.CONTROL_MODE_POSITION, STANDING_LEG_POSITION)
            set_motor_gains(self.odrive_config_pubs, STANDING_GAINS)
            self.get_logger().info('Standing...')
        elif msg.buttons[5] == 1 and self.last_msg.buttons[5] == 0:
            # RB : Land
            self.curr_traj = None
            set_leg_states(self.leg_traj_pubs, MotorCommand.CONTROL_MODE_POSITION, LANDING_LEG_POSITION)
            set_motor_gains(self.odrive_config_pubs, LANDING_GAINS)
            self.get_logger().info('Landing...')
        elif msg.buttons[6] == 1 and self.last_msg.buttons[6] == 0:
            # LT
            self.curr_traj = None
            set_motor_positions(self.odrive_command_pubs, 0.0)
            self.get_logger().info('Zeroed ODrives')
        elif msg.buttons[7] == 1 and self.last_msg.buttons[7] == 0:
            # RT
            pass
        elif msg.buttons[8] == 1 and self.last_msg.buttons[8] == 0:
            # 9 : Ready
            self.curr_traj = None
            set_motor_states(self.odrive_config_pubs, MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.get_logger().info('Readied ODrives')
        elif msg.buttons[9] == 1 and self.last_msg.buttons[9] == 0:
            # 10 : Idle
            self.curr_traj = None
            clear_motor_errors(self.odrive_config_pubs)
            self.get_logger().info('Cleared ODrive errors')
            set_motor_states(self.odrive_config_pubs, MotorConfig.AXIS_STATE_IDLE)
            self.get_logger().info('Idled ODrives')
        elif msg.axes[4] >= 0.5 and self.last_msg.axes[4] < 0.5:
            # Cross left
            pass
        elif msg.axes[4] <= -0.5 and self.last_msg.axes[4] > -0.5:
            # Cross right
            pass
        elif msg.axes[5] >= 0.5 and self.last_msg.axes[5] < 0.5:
            # Cross up
            pass
        elif msg.axes[5] <= -0.5 and self.last_msg.axes[5] > -0.5:
            # Cross down
            pass
            
        self.last_msg = msg

    def imu_callback(self, msg : Imu):
        ## TODO: Feedback control here
        # self.get_logger().info(f"roll: {roll}, pitch: {pitch}: yaw: {yaw}")
        pass
            

def main(args=None):
    rclpy.init(args=args)
    node = STARQGaitJoystick()
    thread = threading.Thread(target=node.run)
    thread.start()
    rclpy.spin(node)
    rclpy.shutdown()