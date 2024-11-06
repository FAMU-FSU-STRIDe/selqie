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

DURATION = 5.0
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
        
        self.roll = []
        self.pitch = []
        self.roll_cmd = []
        self.pitch_cmd = []
        self.time = []
        
    def timer_callback(self):
        
        if self.t >= DURATION:
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(self.time, self.roll, label='Actual Roll')
            plt.plot(self.time, self.roll_cmd, label='Commanded Roll')
            plt.legend()
            plt.subplot(2, 1, 2)
            plt.plot(self.time, self.pitch, label='Actual Pitch')
            plt.plot(self.time, self.pitch_cmd, label='Commanded Pitch')
            plt.legend()
            
            plt.show()
            rclpy.shutdown()
        
        cos_t = AMPLITUDE * np.cos(2.0 * np.pi * self.t)
        sin_t = AMPLITUDE * np.sin(2.0 * np.pi * self.t)
        
        cmd_pose = Pose()
        cmd_pose.position.x = 0.0
        cmd_pose.position.y = 0.0
        cmd_pose.position.z = STAND_HEIGHT
        cmd_pose.orientation = euler_to_quaternion([cos_t, sin_t, 0.0])
        self.cmd_pose_pub.publish(cmd_pose)
        
        eul = quaternion_to_euler(self.odom.pose.pose.orientation)
        self.roll.append(eul[0])
        self.pitch.append(eul[1])
        self.roll_cmd.append(cos_t)
        self.pitch_cmd.append(sin_t)
        self.time.append(self.t)
        
        self.t += 1.0 / FREQUENCY
        
def main():
    rclpy.init()
    robot = UnitreeA1RobotNode()
    rclpy.spin(robot)
    robot.destroy_node()

if __name__ == '__main__':
    main()