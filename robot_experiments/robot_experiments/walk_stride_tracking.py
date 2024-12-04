import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_utils.utils.ros_util_functions import *
import matplotlib.pyplot as plt

FREQUENCY = 50.0

class SELQIERobotNode(Node):
    def __init__(self):
        super().__init__('walk_stride_tracking_node')
        
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, qos_reliable())
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())
        
        self.declare_parameter('linear_velocity', 0.15)
        self.linear_velocity = self.get_parameter('linear_velocity').value
        
        self.declare_parameter('angular_velocity', 0.1)
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        self.declare_parameter('duration', 60.0)
        self.duration = self.get_parameter('duration').value
        
        wait_for_subs([self.cmd_vel_pub])
        
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.cmd_vel_pub.publish(twist)
        
        self.t = 0.0
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)
        
        self.x = []
        self.y = []
        self.vel_x = []
        self.yaw_rate = []
        self.time = []
        
    def timer_callback(self):
        
        if self.t >= self.duration:
            plt.figure()
            plt.plot(self.x, self.y, label='Trajectory')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.show()
            
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time, self.vel_x, label='Actual Velocity')
            plt.plot([0 , self.duration], [self.linear_velocity, self.linear_velocity], label='Commanded Velocity')
            plt.ylabel('Velocity')
            plt.legend()
            plt.subplot(2, 1, 2)
            plt.plot(self.time, self.yaw_rate, label='Actual Yaw Rate')
            plt.plot([0 , self.duration], [self.angular_velocity, self.angular_velocity], label='Commanded Yaw Rate')
            plt.xlabel('Time')
            plt.ylabel('Yaw Rate')
            plt.legend()
            plt.show()
            
            avg_vel_x = sum(self.vel_x) / len(self.vel_x)
            print(f'Average Velocity: {avg_vel_x}')
            
            avg_yaw_rate = sum(self.yaw_rate) / len(self.yaw_rate)
            print(f'Average Yaw Rate: {avg_yaw_rate}')
            
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