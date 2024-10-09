import rclpy
from rclpy.node import Node
from robot_utils.utils.ros_util_functions import *
from sensor_msgs.msg import Imu

def imu_callback(msg):
    eul = quaternion_to_euler(msg.orientation)
    print(f'Roll: {eul[0]}, \tPitch: {eul[1]}, \tYaw: {eul[2]}')

def main():
    rclpy.init()
    node = Node('imu_rpy_node')
    imu_sub = node.create_subscription(Imu, '/imu/data', imu_callback, qos_reliable())
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()