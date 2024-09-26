from robot_msgs.msg import *
from geometry_msgs.msg import Pose, Twist
from robot_utils.utils.ros_util_functions import *

def set_cmd_vel(cmd_vel_pub, vel):
    msg = Twist()
    msg.linear.x = vel[0]
    msg.linear.y = vel[1]
    msg.angular.z = vel[2]
    cmd_vel_pub.publish(msg)
    
def set_cmd_pose(cmd_pose_pub, pos, rot):
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation = euler_to_quaternion(rot)
    cmd_pose_pub.publish(pose)

def set_stop_stance_pattern(stance_pub, clock):
    pattern = StancePattern()
    pattern.header.stamp = clock.now().to_msg()
    pattern.frequency = 0.0
    stance_pub.publish(pattern)

def set_stand_stance_pattern(stance_pub, clock):
    pattern = StancePattern()
    pattern.header.stamp = clock.now().to_msg()
    pattern.frequency = 1.0
    pattern.timing = [0.0, 1.0]
    pattern.stance = [0b1111, 0b1111]
    stance_pub.publish(pattern)

def set_walk_stance_pattern(stance_pub, clock, frequency):
    pattern = StancePattern()
    pattern.header.stamp = clock.now().to_msg()
    pattern.frequency = frequency
    pattern.timing = [0.0, 0.45, 0.50, 0.95, 1.0]
    pattern.stance = [0b1111, 0b1010, 0b1111, 0b0101, 0b1111]
    stance_pub.publish(pattern)