import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotorCommand

def main(args=None):
    rclpy.init(args=args)

    node = Node('zero_motors_node')

    node.declare_parameter('num_motors', 12)
    num_motors = node.get_parameter('num_motors').get_parameter_value().integer_value

    msg = MotorCommand()
    msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
    msg.pos_setpoint = 0.0

    for i in range(num_motors):
        publisher = node.create_publisher(MotorCommand, f'motor{i}/command', 1)
        publisher.publish(msg)
        node.get_logger().info(f'Zeroed motor {i}')

    node.destroy_node()
    rclpy.shutdown()