import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_utils.utils.ros_util_functions import *
import matplotlib.pyplot as plt
import numpy as np

FREQUENCY = 50.0

class SELQIERobotNode(Node):
    def __init__(self):
        super().__init__('walk_stride_tracking_node')

        self.declare_parameter('linear_velocities', [-0.5, -0.25, 0.0, 0.25, 0.5])
        self.linear_velocities = self.get_parameter('linear_velocities').value
        
        self.declare_parameter('angular_velocities', [-2.0, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 2.0])
        self.angular_velocities = self.get_parameter('angular_velocities').value
        
        self.declare_parameter('sample_time', 30.0)
        self.sample_time = self.get_parameter('sample_time').value

        self.num_samples = len(self.linear_velocities) * len(self.angular_velocities)
        self.duration = self.num_samples * self.sample_time
        
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, qos_reliable())
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())
        
        wait_for_subs([self.cmd_vel_pub])
        
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)
        
        self.vel_x_cmd = []
        self.vel_x = []
        self.yaw_rate_cmd = []
        self.yaw_rate = []

        self.current_sample = 0
        self.current_t = 0.0
        self.current_vel_x = []
        self.current_yaw_rate = []
        
    def timer_callback(self):
        
        if self.current_sample >= self.num_samples:

            plt.figure()
            plt.subplot(2, 1, 1)
            plt.title('Linear Velocity Actual vs Commanded')
            for i in range(len(self.angular_velocities)):
                omega = self.angular_velocities[i]
                vel_x_cmd = []
                vel_x = []
                for j in range(len(self.linear_velocities)):
                    idx = j * len(self.angular_velocities) + i
                    vel_x_cmd.append(self.vel_x_cmd[idx])
                    vel_x.append(self.vel_x[idx])
                plt.plot(self.linear_velocities, vel_x, label=f'Yaw Rate: {omega}')
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.title('Yaw Rate Actual vs Commanded')
            for i in range(len(self.linear_velocities)):
                v = self.linear_velocities[i]
                yaw_rate_cmd = []
                yaw_rate = []
                for j in range(len(self.angular_velocities)):
                    idx = j + i * len(self.angular_velocities)
                    yaw_rate_cmd.append(self.yaw_rate_cmd[idx])
                    yaw_rate.append(self.yaw_rate[idx])
                plt.plot(self.angular_velocities, yaw_rate, label=f'Linear Velocity: {v}')
            plt.legend()
            plt.show()
            
            rclpy.shutdown()

        vel_x_cmd = self.linear_velocities[self.current_sample // len(self.angular_velocities)]
        yaw_rate_cmd = self.angular_velocities[self.current_sample % len(self.angular_velocities)]

        if self.current_t == 0.0:
            twist = Twist()
            twist.linear.x = vel_x_cmd
            twist.angular.z = yaw_rate_cmd
            self.cmd_vel_pub.publish(twist)
            print(f'Sample: {self.current_sample}/{self.num_samples}')
            print(f'Linear Velocity Commanded: {vel_x_cmd}')
            print(f'Yaw Rate Commanded: {yaw_rate_cmd}')
        elif self.current_t >= self.sample_time:
            self.vel_x_cmd.append(vel_x_cmd)
            self.vel_x.append(np.mean(self.current_vel_x))
            self.yaw_rate_cmd.append(yaw_rate_cmd)
            self.yaw_rate.append(np.mean(self.current_yaw_rate))
            self.current_sample += 1
            self.current_t = 0.0
            self.current_vel_x = []
            self.current_yaw_rate = []
            return
        
        self.current_vel_x.append(self.odom.twist.twist.linear.x)
        self.current_yaw_rate.append(self.odom.twist.twist.angular.z)
        self.current_t += 1.0 / FREQUENCY

def main(args=None):
    rclpy.init(args=args)
    node = SELQIERobotNode()
    rclpy.spin(node)
    rclpy.shutdown()