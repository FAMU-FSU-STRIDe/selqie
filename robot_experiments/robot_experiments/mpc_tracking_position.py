import rclpy
from rclpy.node import Node
from robot_msgs.msg import *
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from robot_utils.utils.ros_util_functions import *
from robot_utils.utils.motor_util_functions import *
from robot_utils.utils.leg_util_functions import *
from robot_utils.utils.gait_util_functions import *
import numpy as np
import matplotlib.pyplot as plt

DURATION = 20.0
FREQUENCY = 100.0

STAND_HEIGHT = 0.27
AMPLITUDE = 0.05

class UnitreeA1RobotNode(Node):
    def __init__(self):
        super().__init__('mpc_tracking_orientation_node')

        self.cmd_pose_pub = self.create_publisher(Pose, 'cmd_pose', qos_reliable())
        
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, qos_reliable())
        
        self.t = 0.0
        self.timer = self.create_timer(1.0 / FREQUENCY, self.timer_callback)
        
        self.x = []
        self.y = []
        self.x_cmd = []
        self.y_cmd = []
        self.time = []
        
    def timer_callback(self):
        
        if self.t >= DURATION:
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time, self.x, label='Actual X')
            plt.plot(self.time, self.x_cmd, label='Commanded X')
            plt.legend()
            plt.subplot(2, 1, 2)
            plt.plot(self.time, self.y, label='Actual Y')
            plt.plot(self.time, self.y_cmd, label='Commanded Y')
            plt.legend()
            plt.show()
            rclpy.shutdown()
        
        if self.t < DURATION / 4.0:
            x_cmd = 1.0
            y_cmd = 1.0
        elif self.t < DURATION / 2.0:
            x_cmd = -1.0
            y_cmd = 1.0
        elif self.t < 3.0 * DURATION / 4.0:
            x_cmd = -1.0
            y_cmd = -1.0
        else:
            x_cmd = 1.0
            y_cmd = -1.0
        
        cmd_pose = Pose()
        cmd_pose.position.x = x_cmd
        cmd_pose.position.y = y_cmd
        cmd_pose.position.z = STAND_HEIGHT
        cmd_pose.orientation = euler_to_quaternion([0.0, 0.0, 0.0])
        self.cmd_pose_pub.publish(cmd_pose)
        
        eul = quaternion_to_euler(self.odom.pose.pose.orientation)
        self.x.append(self.odom.pose.pose.position.x)
        self.y.append(self.odom.pose.pose.position.y)
        self.time.append(self.t)
        self.x_cmd.append(x_cmd)
        self.y_cmd.append(y_cmd)
        self.t += 1.0 / FREQUENCY
        
def main():
    rclpy.init()
    robot = UnitreeA1RobotNode()
    rclpy.spin(robot)
    robot.destroy_node()

if __name__ == '__main__':
    main()