import rclpy
import time
from rclpy.node import Node
from robot_msgs.msg import LegCommand

def main(args=None):
    rclpy.init(args=args)

    node = Node('stand')

    node.declare_parameter('leg_names', ['FL', 'FR', 'RL', 'RR'])
    leg_names = node.get_parameter('leg_names').get_parameter_value().string_array_value

    node.declare_parameter('stand_position', [0.0, 0.0, -0.15])
    position = node.get_parameter('position').get_parameter_value().double_array_value
    
    node.declare_parameter('delay', 0.0)
    delay = node.get_parameter('delay').get_parameter_value().double_value

    node.declare_parameter('duration', 0.0)
    duration = node.get_parameter('duration').get_parameter_value().double_value

    if (len(position) != 3):
        node.get_logger().error('Invalid position parameter (expected 3 values)')
        return
    
    publishers = []
    for i in range(4):
        publishers.append(node.create_publisher(LegCommand, f'leg{leg_names[i]}/command', 10))
    
    time.sleep(delay)

    msg = LegCommand()
    msg.control_mode = 3
    msg.pos_setpoint.x = position[0]
    msg.pos_setpoint.y = position[1]
    msg.pos_setpoint.z = position[2]

    for i in range(len(leg_names)):
        publishers[i].publish(msg)
    node.get_logger().info('Standing...')

    time.sleep(duration)

    node.destroy_node()
    rclpy.shutdown()