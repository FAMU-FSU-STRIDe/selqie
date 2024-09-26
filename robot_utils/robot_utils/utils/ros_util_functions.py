import time
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Quaternion, Pose, Twist

def qos_fast():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

def qos_reliable():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )
    
def wait_for_subs(pubs, timeout=5):
    cstart = time.time()
    while not all(pub.get_subscription_count() != 0 for pub in pubs):
        if time.time() - cstart > timeout:
            return False
        time.sleep(0.05)
    return True

def quaternion_to_euler(q : Quaternion) -> list:
    q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return [roll, pitch, yaw]

def euler_to_quaternion(eul) -> Quaternion:
    cy = math.cos(eul[2] * 0.5)
    sy = math.sin(eul[2] * 0.5)
    cp = math.cos(eul[1] * 0.5)
    sp = math.sin(eul[1] * 0.5)
    cr = math.cos(eul[0] * 0.5)
    sr = math.sin(eul[0] * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def get_pose(pose : Pose):
    return [pose.position.x, pose.position.y, pose.position.z], quaternion_to_euler(pose.orientation)

def get_velocity(twist : Twist):
    return [twist.linear.x, twist.linear.y, twist.linear.z], [twist.angular.x, twist.angular.y, twist.angular.z]
