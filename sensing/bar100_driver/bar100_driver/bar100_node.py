import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Python package for Keller LD pressure sensors
# https://github.com/bluerobotics/KellerLD-python
from kellerLD import KellerLD

# Bar100 Node class
class Bar100Node(Node):
    def __init__(self):
        super().__init__('bar100_node')

        # Get ROS parameters
        self.declare_parameter('i2c_bus', 1)
        i2c_bus = self.get_parameter('i2c_bus').value

        self.declare_parameter('frequency', 20.0)
        frequency = self.get_parameter('frequency').value

        # Initialize the Keller LD sensor
        self.sensor = KellerLD(i2c_bus)
        self.sensor.init()

        # Create depth and temperature publishers
        self.pressure_pub = self.create_publisher(Float32, 'bar100/pressure', 10)
        self.temperature_pub = self.create_publisher(Float32, 'bar100/temperature', 10)

        # Create timer for publishing data
        self.timer = self.create_timer(1.0 / frequency, self.publish_data)

        self.get_logger().info(f'Bar100 Node Initialized on I2C bus {i2c_bus}')

    # Function to read sensor data and publish it
    # Called periodically by the timer
    def publish_data(self):
        
        # Read sensor data
        self.sensor.read()

        # Publish pressure reading to ROS network
        pressure = self.sensor.pressure() # in bar
        pressure_msg = Float32()
        pressure_msg.data = pressure
        self.pressure_pub.publish(pressure_msg)

        # Publish temperature reading to ROS network
        temperature = self.sensor.temperature()
        temperature_msg = Float32()
        temperature_msg.data = temperature
        self.temperature_pub.publish(temperature_msg)

# Entry point for the node
def main(args=None):
    rclpy.init(args=args)
    bar100_node = Bar100Node()
    try:
        rclpy.spin(bar100_node)
    except KeyboardInterrupt:
        pass
    finally:
        bar100_node.destroy_node()
        rclpy.shutdown()
        