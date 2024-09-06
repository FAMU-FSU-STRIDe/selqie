import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_msgs.msg import LegCommand

from robot_utils.utils.robot_util_functions import *
from robot_utils.utils.starq_util_functions import *

class SwimTuningJoystick(Node):
    def __init__(self):
        super().__init__('swim_tuning_joystick')

        self.num_motors = 8
        self.declare_parameter('num_motors', self.num_motors)
        self.num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.declare_parameter('leg_names', self.leg_names)
        self.leg_names = self.get_parameter('leg_names').get_parameter_value().string_array_value

        self.leg_command_pubs = []
        for l in self.leg_names:
            self.leg_command_pubs.append(self.create_publisher(LegCommand, f'leg{l}/command', 10))

        self.last_msg = None
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.running = False
        self.n = 0
        self.df = 0.25
        self.dphi = np.pi/64
        self.dxamp = 0.005
        self.dyamp = 0.001
        self.maxr = 0.216
        self.minr = 0.084
        
        self.swim_traj = SwimTrajectory()
        
        self.get_logger().info('Swim Tuning Joystick Node Initialized')

        self.run()

    def get_default_swim_position(self):
        x0 = self.swim_traj.L0 * np.cos(self.swim_traj.phi)
        y0 = self.swim_traj.L0 * np.sin(self.swim_traj.phi)
        return [x0, 0.0, y0]
    
    def run(self):
        while rclpy.ok():
            if self.running:
                pos = get_swimming_leg_position(self.swim_traj, self.n)
                self.n = (self.n + 1) % self.swim_traj.size
            elif not self.running:
                pos = self.get_default_swim_position()
                self.n = 0

            pnorm = np.linalg.norm(pos)
            if pnorm > self.maxr:
                self.get_logger().warn('Exceeding max radius')
            elif pnorm < self.minr:
                self.get_logger().warn('Exceeding min radius')
            else:
                set_leg_states(self.leg_command_pubs, MotorCommand.CONTROL_MODE_POSITION, pos)

            dt = 1.0 / (self.swim_traj.frequency * self.swim_traj.size)
            rclpy.spin_once(self, timeout_sec=dt)

    def joy_callback(self, msg : Joy):
        if self.last_msg is None:
            pass
        elif msg.buttons[0] == 1 and self.last_msg.buttons[0] == 0:
            # 1 : Toggle running
            self.running = not self.running
            self.get_logger().info(f'Running: {self.running}')
        elif msg.buttons[4] == 1 and self.last_msg.buttons[4] == 0:
            # LB : Decrease phi
            self.swim_traj.phi -= self.dphi
            self.get_logger().info(f'Phi: {self.swim_traj.phi}')
        elif msg.buttons[5] == 1 and self.last_msg.buttons[5] == 0:
            # RB : Increase phi
            self.swim_traj.phi += self.dphi
            self.get_logger().info(f'Phi: {self.swim_traj.phi}')
        elif msg.buttons[6] == 1 and self.last_msg.buttons[6] == 0:
            # LT : Decrease frequency
            self.swim_traj.frequency -= self.df
            self.swim_traj.frequency = max(self.swim_traj.frequency, self.df)
            self.get_logger().info(f'Frequency: {self.swim_traj.frequency}')
        elif msg.buttons[7] == 1 and self.last_msg.buttons[7] == 0:
            # RT : Increase frequency
            self.swim_traj.frequency += self.df
            self.get_logger().info(f'Frequency: {self.swim_traj.frequency}')
        elif msg.axes[4] >= 0.5 and self.last_msg.axes[4] < 0.5:
            # Cross left : Decrease yamp
            self.swim_traj.yamp -= self.dyamp
            self.swim_traj.yamp = max(self.swim_traj.yamp, 0)
            self.get_logger().info(f'Y Amplitude: {self.swim_traj.yamp}')
        elif msg.axes[4] <= -0.5 and self.last_msg.axes[4] > -0.5:
            # Cross right : Increase yamp
            self.swim_traj.yamp += self.dyamp
            self.get_logger().info(f'Y Amplitude: {self.swim_traj.yamp}')
        elif msg.axes[5] >= 0.5 and self.last_msg.axes[5] < 0.5:
            # Cross up : Increase xamp
            self.swim_traj.xamp += self.dxamp
            self.get_logger().info(f'X Amplitude: {self.swim_traj.xamp}')
        elif msg.axes[5] <= -0.5 and self.last_msg.axes[5] > -0.5:
            # Cross down : Decrease xamp
            self.swim_traj.xamp -= self.dxamp
            self.swim_traj.xamp = max(self.swim_traj.xamp, 0)
            self.get_logger().info(f'X Amplitude: {self.swim_traj.xamp}')

        self.last_msg = msg

def main(args=None):
    rclpy.init(args=args)
    SwimTuningJoystick()
    rclpy.shutdown()