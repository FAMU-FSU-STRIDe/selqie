import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

class Depth2PoseNode(Node):
    def __init__(self):
        super().__init__('depth2pose_node')
        
        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value
        
        self.declare_parameter('z_variance', 2.89)
        self.z_variance = self.get_parameter('z_variance').value
        
        self.depth_sub = self.create_subscription(Float32, 'bar100/depth', self.depth_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'bar100/pose', 10)
        
    def depth_callback(self, msg):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.z = msg.data
        pose.pose.covariance = [0.0] * 36
        pose.pose.covariance[14] = self.z_variance
        self.pose_pub.publish(pose)
        
def main(args=None):
    rclpy.init(args=args)
    depth2pose_node = Depth2PoseNode()
    rclpy.spin(depth2pose_node)
    depth2pose_node.destroy_node()
    rclpy.shutdown()