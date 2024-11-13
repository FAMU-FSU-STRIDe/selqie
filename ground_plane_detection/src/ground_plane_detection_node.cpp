#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class GroundPlaneDetectionNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _ground_coefficients_publisher;

        void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received Point Cloud Message.");
        }

    public:
        GroundPlaneDetectionNode() : Node("ground_plane_detection_node")
        {
            _point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points", 10, std::bind(&GroundPlaneDetectionNode::point_cloud_callback, this, std::placeholders::_1));

            _ground_coefficients_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("ground", 10);

            RCLCPP_INFO(this->get_logger(), "Ground Plane Detection Node Initialized.");
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundPlaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}