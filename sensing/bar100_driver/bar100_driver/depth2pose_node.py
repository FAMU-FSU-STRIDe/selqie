import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

# Depth data is converted to a PoseWithCovarianceStamped to be used
# in the robot_localization EKF for state estimation

# Depth2PoseNode class to convert depth readings to pose messages
class Depth2PoseNode(Node):
    def __init__(self):
        super().__init__('depth2pose_node')
        
        # Get ROS parameters

        # Frame ID will be in the 'odom' or 'map' frame 
        # because the pose is relative to an absolute depth
        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value

        # Fresh water density: 997.0474 kg/m^3 at 4 degrees Celsius
        # Salt water density: 1023.6 kg/m^3 at 4 degrees Celsius
        self.declare_parameter('fluid_density', 997.0474)
        self.fluid_density = self.get_parameter('fluid_density').value

        self.declare_parameter('gravity', 9.80665)
        self.gravity = self.get_parameter('gravity').value
        
        # Z Variance for the covariance matrix
        # The sensor has a resolution of ~2cm, so the variance is (0.02)^2 = 0.0004
        self.declare_parameter('z_variance', 0.0004)
        self.z_variance = self.get_parameter('z_variance').value
        
        # Create publishers and subscribers
        self.pressure_sub = self.create_subscription(Float32, 'bar100/pressure', self.depth_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'bar100/pose', 10)
        
    def depth_callback(self, msg):
        # Convert pressure to depth using hydrostatic formula
        depth = (msg.data) / (self.fluid_density * self.gravity) * 1E5 # in m

        # Create pose message
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.z = depth
        pose.pose.covariance = [0.0] * 36  # Initialize covariance matrix to zero
        pose.pose.covariance[14] = self.z_variance # Set the Z variance

        # Publish the pose message
        self.pose_pub.publish(pose)
        
# Entry point for the node
def main(args=None):
    rclpy.init(args=args)
    depth2pose_node = Depth2PoseNode()
    try:
        rclpy.spin(depth2pose_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth2pose_node.destroy_node()
        rclpy.shutdown()