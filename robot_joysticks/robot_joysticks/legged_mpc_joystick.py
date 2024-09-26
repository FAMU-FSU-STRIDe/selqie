import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from robot_msgs.msg import *

from robot_utils.utils.ros_util_functions import *
from robot_utils.utils.leg_util_functions import *
from robot_utils.utils.gait_util_functions import *

STAND_HEIGHT = 0.27
WALK_FREQUENCY = 3.0
MAX_LINEAR_VELOCITY = [0.5, 0.25, 0.5]
MAX_ANGULAR_VELOCITY = [0.5, 0.5, 0.5]
MAX_POSITION = [0.05, 0.05, 0.05]
MAX_ROTATION = [0.1, 0.1, 0.1]

NONE_MODE = -1
POSITION_MODE = 0
VELOCITY_MODE = 1

class LeggedMPCJoystick(Node):
    def __init__(self):
        super().__init__('starq_gait_joystick')

        self.last_msg = None
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_fast())
        self.odom : Odometry = None

        self.stance_pub = self.create_publisher(StancePattern, 'stance_pattern', qos_reliable())

        self.cmd_pose_pub = self.create_publisher(Pose, 'cmd_pose', qos_reliable())
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())
        self.mode = NONE_MODE
        self.base_pos = [0.0, 0.0, STAND_HEIGHT]
        self.base_rot = [0.0, 0.0, 0.0]
        
        self.get_logger().info('Legged MPC Joystick Node Initialized')

    def odom_callback(self, msg : Odometry):
        self.odom = msg

    def joy_callback(self, msg : Joy):
        if self.last_msg is None:
            pass
        elif msg.buttons[0] == 1 and self.last_msg.buttons[0] == 0:
            # 1 : Stand
            self.base_pos, self.base_rot = get_pose(self.odom.pose.pose)
            self.base_pos[2] = STAND_HEIGHT
            self.base_rot[0:2] = [0.0, 0.0]
            set_cmd_pose(self.cmd_pose_pub, self.base_pos, self.base_rot)
            set_stand_stance_pattern(self.stance_pub, self.get_clock())
            self.mode = POSITION_MODE
            self.get_logger().info('Standing')
            pass
        elif msg.buttons[1] == 1 and self.last_msg.buttons[1] == 0:
            # 2 : Walk
            set_cmd_vel(self.cmd_vel_pub, [0.0, 0.0, 0.0])
            set_walk_stance_pattern(self.stance_pub, self.get_clock(), WALK_FREQUENCY)
            self.mode = VELOCITY_MODE
            self.get_logger().info('Walking')
            pass
        elif msg.buttons[2] == 1 and self.last_msg.buttons[2] == 0:
            # 3 : Stop MPC
            set_cmd_pose(self.cmd_pose_pub, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.mode = NONE_MODE
            self.get_logger().info('Stopping MPC')
            pass
        elif msg.buttons[3] == 1 and self.last_msg.buttons[3] == 0:
            # 4
            pass
        elif msg.buttons[4] == 1 and self.last_msg.buttons[4] == 0:
            # LB
            pass
        elif msg.buttons[5] == 1 and self.last_msg.buttons[5] == 0:
            # RB
            pass
        elif msg.buttons[6] == 1 and self.last_msg.buttons[6] == 0:
            # LT
            pass
        elif msg.buttons[7] == 1 and self.last_msg.buttons[7] == 0:
            # RT
            pass
        elif msg.buttons[8] == 1 and self.last_msg.buttons[8] == 0:
            # 9
            pass
        elif msg.buttons[9] == 1 and self.last_msg.buttons[9] == 0:
            # 10
            pass
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

        left_axis_x = msg.axes[1]
        left_axis_y = msg.axes[0]
        right_axis_x = msg.axes[3]

        if self.mode == VELOCITY_MODE:
            vel = [MAX_LINEAR_VELOCITY[0] * left_axis_x, MAX_LINEAR_VELOCITY[1] * left_axis_y, MAX_ANGULAR_VELOCITY[2] * right_axis_x]
            set_cmd_vel(self.cmd_vel_pub, vel)
        elif self.mode == POSITION_MODE:
            delta_pos = [MAX_POSITION[0] * left_axis_x, MAX_POSITION[1] * left_axis_y, 0.0]
            delta_rot = [0.0, 0.0, MAX_ROTATION[2] * right_axis_x]
            cmd_pos = [self.base_pos[i] + delta_pos[i] for i in range(3)]
            cmd_rot = [self.base_rot[i] + delta_rot[i] for i in range(3)]
            set_cmd_pose(self.cmd_pose_pub, cmd_pos, cmd_rot)
            

def main(args=None):
    rclpy.init(args=args)
    node = LeggedMPCJoystick()
    rclpy.spin(node)
    rclpy.shutdown()