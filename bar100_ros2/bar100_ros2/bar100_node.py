import rclpy
from rclpy.node import Node
from kellerLD import KellerLD
from std_msgs.msg import Float32

class Bar100Node(Node):
    def __init__(self):
        super().__init__('bar100_node')

        self.declare_parameter('i2c_bus', 1)
        i2c_bus = self.get_parameter('i2c_bus').value

        self.declare_parameter('frequency', 20.0)
        frequency = self.get_parameter('frequency').value

        self.declare_parameter('fluid_density', 997.0474)
        self.fluid_density = self.get_parameter('fluid_density').value

        self.declare_parameter('gravity', 9.80665)
        self.gravity = self.get_parameter('gravity').value

        self.declare_parameter('surface_pressure', 1.0)
        self.surface_pressure = self.get_parameter('surface_pressure').value

        self.sensor = KellerLD(i2c_bus)
        self.sensor.init()

        self.depth_pub = self.create_publisher(Float32, 'bar100/depth', 10)
        self.temperature_pub = self.create_publisher(Float32, 'bar100/temperature', 10)

        self.timer = self.create_timer(1.0 / frequency, self.publish_data)

        self.get_logger().info('Bar100 Node initialized')

    def publish_data(self):
        
        self.sensor.read()

        pressure = self.sensor.pressure()
        depth = (self.surface_pressure - pressure) / (self.fluid_density * self.gravity) * 1E5
        depth_msg = Float32()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

        temperature = self.sensor.temperature()
        temperature_msg = Float32()
        temperature_msg.data = temperature
        self.temperature_pub.publish(temperature_msg)

def main(args=None):
    rclpy.init(args=args)
    bar100_node = Bar100Node()
    rclpy.spin(bar100_node)
    bar100_node.destroy_node()
    rclpy.shutdown()