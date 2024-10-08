import rclpy
from rclpy.node import Node
from kellerLD import KellerLD
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

class Bar100Node(Node):
    def __init__(self):
        super().__init__('bar100_node')

        self.declare_parameter('i2c-bus', 1)
        i2c_bus = self.get_parameter('i2c-bus').value

        self.declare_parameter('frequency', 20.0)
        frequency = self.get_parameter('frequency').value

        self.declare_parameter('publish_depth_as_pose', False)
        self.publish_depth_as_pose = self.get_parameter('publish_depth_as_pose').value

        if self.publish_depth_as_pose:
            self.declare_parameter('frame_id', 'bar100_link')
            self.frame_id = self.get_parameter('frame_id').value

            self.declare_parameter('fluid_density', 997.0474)
            self.fluid_density = self.get_parameter('fluid_density').value

            self.declare_parameter('gravity', 9.80665)
            self.gravity = self.get_parameter('gravity').value

            self.declare_parameter('surface_pressure', 1.0)
            self.surface_pressure = self.get_parameter('surface_pressure').value

            self.declare_parameter('variance', 2.89)
            self.variance = self.get_parameter('variance').value

            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'bar100/pose', 10)

        self.sensor = KellerLD(i2c_bus)
        self.sensor.init()

        self.pressure_pub = self.create_publisher(Float32, 'bar100/pressure', 10)
        self.temperature_pub = self.create_publisher(Float32, 'bar100/temperature', 10)

        self.timer = self.create_timer(1.0 / frequency, self.publish_data)

        self.get_logger().info('Bar100 Node initialized')

    def publish_data(self):

        pressure = 0.0
        temperature = 0.0

        self.sensor.read()
        pressure = self.sensor.pressure()
        temperature = self.sensor.temperature()

        pressure_msg = Float32()
        pressure_msg.data = pressure
        self.pressure_pub.publish(pressure_msg)

        temperature_msg = Float32()
        temperature_msg.data = temperature
        self.temperature_pub.publish(temperature_msg)

        if self.publish_depth_as_pose:
            depth = (self.surface_pressure - pressure) / (self.fluid_density * self.gravity) * 1E5
            self.publish_pose(depth)

    def publish_pose(self, depth):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.z = depth
        pose.pose.pose.orientation.w = 1.0

        pose.pose.covariance = [0.0] * 36
        pose.pose.covariance[14] = self.variance

        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    bar100_node = Bar100Node()
    rclpy.spin(bar100_node)
    bar100_node.destroy_node()
    rclpy.shutdown()