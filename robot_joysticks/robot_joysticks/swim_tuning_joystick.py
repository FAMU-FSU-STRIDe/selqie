import time
import numpy as np
import threading
from threading import Lock
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_msgs.msg import LegCommand

class SwimTuningJoystick(Node):
    def __init__(self):
        super().__init__('swim_tuning_joystick')

        self.num_motors = 8
        self.declare_parameter('num_motors', self.num_motors)
        self.num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.declare_parameter('leg_names', self.leg_names)
        self.leg_names = self.get_parameter('leg_names').get_parameter_value().string_array_value

        self.running = False
        self.n = 0

        # Parameters
        self.N = 100
        self.f = 1.0
        self.df = 0.25
        self.L0 = 0.18
        self.phi = -3*np.pi/4
        self.dphi = np.pi/64
        self.xamp = 0.02
        self.dxamp = 0.005
        self.yamp = 0.005
        self.dyamp = 0.001
        self.maxr = 0.216
        self.minr = 0.084

        self.last_time = 0
        self.last_msg = None

        self.leg_command_pubs = []
        for l in self.leg_names:
            self.leg_command_pubs.append(self.create_publisher(LegCommand, f'leg{l}/command', 10))

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        while rclpy.ok():
            if self.running:
                self.update()
                self.n = (self.n + 1) % self.N
            elif not self.running:
                self.set_default()
                self.n = 0
                
            dt = 1.0 / (self.f * self.N)
            time.sleep(dt)
            
            if self.n % (self.N / 10) == 0:
                rclpy.spin_once(self, timeout_sec=dt)

    def set_default(self):
        x0 = self.L0 * np.cos(self.phi)
        y0 = self.L0 * np.sin(self.phi)

        cmd = LegCommand()
        cmd.control_mode = 0x3
        cmd.pos_setpoint.x = x0
        cmd.pos_setpoint.z = y0
        for p in self.leg_command_pubs:
            p.publish(cmd)

    def update(self):
        p0 = np.array([self.L0 * np.cos(self.phi), 
                       self.L0 * np.sin(self.phi)])
        
        theta = self.phi + np.pi / 2
        R = np.array([[np.cos(theta), -np.sin(theta)], 
                      [np.sin(theta), np.cos(theta)]])
        
        pr = np.array([self.xamp * np.cos(2 * np.pi * self.n / self.N), 
                      self.yamp * np.sin(2 * np.pi * self.n / self.N)])
        
        p = p0 + np.matmul(R, pr)

        pnorm = np.linalg.norm(p)
        if pnorm > self.maxr:
            self.get_logger().warn('Exceeding max radius')
        elif pnorm < self.minr:
            self.get_logger().warn('Exceeding min radius')

        cmd = LegCommand()
        cmd.control_mode = 0x3
        cmd.pos_setpoint.x = p[0]
        cmd.pos_setpoint.z = p[1]
        for p in self.leg_command_pubs:
            p.publish(cmd)

    def joy_callback(self, msg : Joy):
        if self.last_msg is None:
            self.last_msg = msg
            return
        
        if msg.buttons[0] == 1 and self.last_msg.buttons[0] == 0:
            # 1
            self.running = not self.running
            self.get_logger().info(f'Running: {self.running}')
        if msg.buttons[4] == 1 and self.last_msg.buttons[4] == 0:
            # LB
            self.phi -= self.dphi
            self.get_logger().info(f'Phi: {self.phi}')
        if msg.buttons[5] == 1 and self.last_msg.buttons[5] == 0:
            # RB
            self.phi += self.dphi
            self.get_logger().info(f'Phi: {self.phi}')
        if msg.buttons[6] == 1 and self.last_msg.buttons[6] == 0:
            # LT
            self.f -= self.df
            self.f = max(self.f, self.df)
            self.get_logger().info(f'Frequency: {self.f}')
        if msg.buttons[7] == 1 and self.last_msg.buttons[7] == 0:
            # RT
            self.f += self.df
            self.get_logger().info(f'Frequency: {self.f}')
        if msg.axes[4] >= 0.5 and self.last_msg.axes[4] < 0.5:
            # Cross left
            self.yamp -= self.dyamp
            self.yamp = max(self.yamp, 0)
            self.get_logger().info(f'Y Amplitude: {self.yamp}')
        if msg.axes[4] <= -0.5 and self.last_msg.axes[4] > -0.5:
            # Cross right
            self.yamp += self.dyamp
            self.get_logger().info(f'Y Amplitude: {self.yamp}')
        if msg.axes[5] >= 0.5 and self.last_msg.axes[5] < 0.5:
            # Cross up
            self.xamp += self.dxamp
            self.get_logger().info(f'X Amplitude: {self.xamp}')
        if msg.axes[5] <= -0.5 and self.last_msg.axes[5] > -0.5:
            # Cross down
            self.xamp -= self.dxamp
            self.xamp = max(self.xamp, 0)
            self.get_logger().info(f'X Amplitude: {self.xamp}')
            
        self.last_msg = msg

def main(args=None):
    rclpy.init(args=args)
    swim_tuning_joystick = SwimTuningJoystick()
    rclpy.shutdown()