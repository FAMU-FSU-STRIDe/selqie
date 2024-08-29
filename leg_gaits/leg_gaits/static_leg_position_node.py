import rclpy
import time
from rclpy.node import Node
from robot_msgs.msg import LegCommand

def main(args=None):
    rclpy.init(args=args)

    node = Node('static_leg_position_node')

    node.declare_parameter('position', [0.0, 0.0, 0.0])
    position = node.get_parameter('position').get_parameter_value().double_array_value
    
    node.declare_parameter('delay', 0.0)
    delay = node.get_parameter('delay').get_parameter_value().double_value

    if (len(position) != 3):
        node.get_logger().error('Invalid position parameter (expected 3 values)')
        return
    
    time.sleep(delay)

    msg = LegCommand()
    msg.control_mode = 3
    msg.pos_setpoint.x = position[0]
    msg.pos_setpoint.y = position[1]
    msg.pos_setpoint.z = position[2]

    publisher = node.create_publisher(LegCommand, 'command', 1)
    publisher.publish(msg)
    node.get_logger().info('Published static leg position')

    node.destroy_node()
    rclpy.shutdown()