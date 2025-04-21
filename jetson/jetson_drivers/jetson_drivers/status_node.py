#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from jetson_msgs.msg import JetsonStatus

from jtop import jtop # Driver for NVIDIA Jetson device information

class JetsonStatusNode(Node):
    """
    Node to publish the status of the NVIDIA Jetson device.
    It uses the jtop library to gather information about CPU, RAM, GPU usage,
    temperature, and other metrics.
    """
    def __init__(self):
        super().__init__('jetson_status_publisher')
        
        # Frequency for publishing status
        self.declare_parameter('frequency', 1.0)
        frequency = self.get_parameter('frequency').value
        
        # Create a publisher for the JetsonStatus message
        self.publisher_ = self.create_publisher(JetsonStatus, 'jetson/status', 10)
        
        # Set up a timer to publish the status at regular intervals
        self.timer = self.create_timer(1.0 / frequency, self.publish_status)
        
        self.get_logger().info("Jetson Status Node Initialized")

    def publish_status(self):
        """Publish the status of the Jetson device using jtop."""
        
        with jtop() as jetson:
            
            # Check if jtop is running and able to read stats
            if not jetson.ok():
                self.get_logger().warn("Unable to read Jetson stats.")
                return

            # Get the stats from jtop
            stats = jetson.stats

            # Create a new JetsonStatus message and populate it with the stats
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

            # Publish the message
            self.publisher_.publish(msg)

# Entry point for the node
def main(args=None):
    rclpy.init(args=args)
    node = JetsonStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle keyboard interrupt (Ctrl+C)
        node.get_logger().info("Shutting down JetsonStatusNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()