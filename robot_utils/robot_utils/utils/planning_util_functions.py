from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from robot_utils.utils.ros_util_functions import *

def set_goal(goal_pub, goal):
    goal_msg = PoseStamped()
    goal_msg.header.stamp = Clock().now().to_msg()
    goal_msg.pose.position.x = goal[0]
    goal_msg.pose.position.y = goal[1]
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation = euler_to_quaternion([0.0, 0.0, goal[2]])
    goal_pub.publish(goal_msg)