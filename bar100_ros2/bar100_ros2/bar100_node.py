import rclpy
from rclpy.node import Node
# from kellerLD import KellerLD
from std_msgs.msg import Float32

class Bar100Node(Node):
    def __init__(self):
        super().__init__('bar100_node')

        # self.sensor = KellerLD()
        # self.sensor.init()

        self.pressure_pub = self.create_publisher(Float32, 'bar100/pressure', 10)
        self.temperature_pub = self.create_publisher(Float32, 'bar100/temperature', 10)

        self.declare_parameter('frequency', 20.0)
        frequency = self.get_parameter('frequency').value

        self.timer = self.create_timer(1.0 / frequency, self.publish_data)

        self.get_logger().info('Bar100 Node initialized')

    def publish_data(self):

        pressure = 0.0
        temperature = 0.0

        # self.sensor.read()
        # pressure = self.sensor.pressure()
        # temperature = self.sensor.temperature()

        pressure_msg = Float32()
        pressure_msg.data = pressure
        self.pressure_pub.publish(pressure_msg)

        temperature_msg = Float32()
        temperature_msg.data = temperature
        self.temperature_pub.publish(temperature_msg)

def main(args=None):
    rclpy.init(args=args)
    bar100_node = Bar100Node()
    rclpy.spin(bar100_node)
    bar100_node.destroy_node()
    rclpy.shutdown()