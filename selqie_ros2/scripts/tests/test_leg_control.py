#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from robot_msgs.msg import LegCommand

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
def QOS_RELIABLE() -> QoSProfile:
    """Get a QoSProfile with reliable reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )

LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
NUM_LEGS = len(LEG_NAMES)
NUM_POINTS = 500
RADIUS = 0.18
FREQUENCY = 0.25

def main(args=None):
    rclpy.init(args=args)
    
    node = Node('test_leg_control_node')
    
    cmd_pubs = []
    for i in range(NUM_LEGS):
        cmd_pubs.append(node.create_publisher(LegCommand, f'leg{LEG_NAMES[i]}/command', QOS_RELIABLE()))
        
    node.get_logger().info('Waiting for subscribers...')
    while not all(pub.get_subscription_count() != 0 for pub in cmd_pubs):
        time.sleep(0.05)

    i = 0
    def timer_callback():
        nonlocal i
        f = i / NUM_POINTS
        x = RADIUS * math.cos(-2 * math.pi * f - math.pi / 2)
        z = RADIUS * math.sin(-2 * math.pi * f - math.pi / 2)
        print(f'x: {x}, z: {z}')
        cmd = LegCommand()
        cmd.control_mode = LegCommand.CONTROL_MODE_POSITION
        cmd.pos_setpoint.x = x
        cmd.pos_setpoint.z = z
        for j in range(NUM_LEGS):
            cmd_pubs[j].publish(cmd)
        i = (i + 1) % NUM_POINTS
    
    cmd_timer = node.create_timer(1.0 / (FREQUENCY * NUM_POINTS), timer_callback)

    node.get_logger().info('Publishing leg commands...')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()