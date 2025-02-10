#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from jtop import jtop
from std_msgs.msg import Header
from robot_msgs.msg import JetsonStatus

class JetsonStatusNode(Node):
    def __init__(self):
        super().__init__('jetson_status_publisher')
        self.publisher_ = self.create_publisher(JetsonStatus, 'jetson/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Jetson Status Node Initialized")

    def publish_status(self):
        with jtop() as jetson:
            if not jetson.ok():
                self.get_logger().warn("Unable to read Jetson stats.")
                return

            stats = jetson.stats

            msg = JetsonStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.uptime = stats['uptime'].total_seconds()
            
            # Collect CPU usage for active cores
            cpu_keys = [key for key in stats.keys() if key.startswith('CPU') and isinstance(stats[key], (int, float))]
            msg.cpu_usage = [float(stats[key]) for key in sorted(cpu_keys, key=lambda x: int(x[3:]))]

            msg.ram_usage = float(stats['RAM'])
            msg.swap_usage = float(stats['SWAP'])
            msg.gpu_usage = float(stats['GPU'])
            msg.fan_speed = float(stats['Fan pwmfan0'])
            msg.temp_cpu = float(stats['Temp cpu'])
            msg.temp_gpu = float(stats['Temp gpu'])
            msg.temp_soc = float(stats['Temp soc0'])  # Choose one SOC temp sensor
            msg.temp_tj = float(stats['Temp tj'])
            msg.power_total = float(stats['Power TOT'])
            msg.jetson_clocks = str(stats['jetson_clocks'])
            msg.nvp_model = str(stats['nvp model'])

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JetsonStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down JetsonStatusNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
