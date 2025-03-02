#!/usr/bin/env python3

import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

FREQUENCY = 50.0

class SELQIERobotNode(Node):
    def __init__(self):
        super().__init__('walk_stride_tracking_node')
        
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, 10)
        
        self.gait_pub = self.create_publisher(String, 'gait', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.declare_parameter('linear_velocity', 0.15)
        self.linear_velocity = self.get_parameter('linear_velocity').value
        
        self.declare_parameter('angular_velocity', 0.1)
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        self.declare_parameter('duration', 65.0)
        self.duration = self.get_parameter('duration').value
        
        time.sleep(2.0)
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.gait_pub.publish(String(data='walk'))
        time.sleep(0.5)
        self.cmd_vel_pub.publish(twist)
        rclpy.spin_once(self)
        self.get_logger().info('Walking...')
        time.sleep(3.0)
        
        self.t = 0.0
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)
        self.get_logger().info('Tracking...')
        
        self.x = []
        self.y = []
        self.vel_x = []
        self.yaw_rate = []
        self.time = []
        
    def timer_callback(self):
        
        if self.t >= self.duration:
            
            self.gait_pub.publish(String())
            
            avg_vel_x = sum(self.vel_x) / len(self.vel_x)
            print(f'Average Velocity: {avg_vel_x}')
            
            avg_yaw_rate = sum(self.yaw_rate) / len(self.yaw_rate)
            print(f'Average Yaw Rate: {avg_yaw_rate}')

            plt.figure()
            plt.plot(self.x, self.y, label='Trajectory')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.show()
            
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time, self.vel_x, label='Actual', linewidth=3)
            plt.plot([0 , self.duration], [self.linear_velocity, self.linear_velocity], label='Commanded', linewidth=3)
            plt.plot([0 , self.duration], [avg_vel_x, avg_vel_x], label='Average', linestyle='--', linewidth=3)
            plt.ylabel('Velocity', fontsize=24, fontdict={'family': 'serif'})
            plt.legend(prop={'family': 'serif', 'size': 14})
            plt.subplot(2, 1, 2)
            plt.plot(self.time, self.yaw_rate, label='Actual', linewidth=3)
            plt.plot([0 , self.duration], [self.angular_velocity, self.angular_velocity], label='Commanded', linewidth=3)
            plt.plot([0 , self.duration], [avg_yaw_rate, avg_yaw_rate], label='Average', linestyle='--', linewidth=3)
            plt.xlabel('Time', fontsize=24, fontdict={'family': 'serif'})
            plt.ylabel('Yaw Rate', fontsize=24, fontdict={'family': 'serif'})
            plt.legend(prop={'family': 'serif', 'size': 14})
            plt.show()
            
            rclpy.shutdown()
            
        self.x.append(self.odom.pose.pose.position.x)
        self.y.append(self.odom.pose.pose.position.y)
        self.vel_x.append(self.odom.twist.twist.linear.x)
        self.yaw_rate.append(self.odom.twist.twist.angular.z)
        self.time.append(self.t)
        self.t += 1.0 / FREQUENCY

def main(args=None):
    rclpy.init(args=args)
    node = SELQIERobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()