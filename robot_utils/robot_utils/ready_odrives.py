import rclpy
from rclpy.node import Node
from robot_msgs.msg import MotorConfig

def main(args=None):
    rclpy.init(args=args)

    node = Node('clear_odrive_errors_node')

    node.declare_parameter('num_motors', 8)
    num_motors = node.get_parameter('num_motors').get_parameter_value().integer_value

    msg = MotorConfig()
    msg.axis_state = MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL

    for i in range(num_motors):
        publisher = node.create_publisher(MotorConfig, f'odrive{i}/config', 1)
        publisher.publish(msg)
        node.get_logger().info(f'Readied ODrive {i}')

    node.destroy_node()
    rclpy.shutdown()