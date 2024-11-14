#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

class GroundPlaneDetectionNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _ground_coefficients_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr _ground_plane_publisher;

    int _sample_size;
    double _range_min, _range_max;
    double _plane_width;

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert to PCL Point Cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Generate Random Sample in Range
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_sample;
        std::vector<int> sample_indices(pcl_cloud.size());
        std::iota(sample_indices.begin(), sample_indices.end(), 0);
        std::random_shuffle(sample_indices.begin(), sample_indices.end());
        for (std::size_t i = 0; i < pcl_cloud.size(); i++)
        {
            pcl::PointXYZ point = pcl_cloud[sample_indices[i]];
            if (point.z > _range_min && point.z < _range_max)
            {
                pcl_cloud_sample.push_back(point); // Add to Sample

                if (pcl_cloud_sample.size() >= std::size_t(_sample_size))
                {
                    break;
                }
            }
        }

        // Create A and b Matrices
        Eigen::MatrixX3f A(pcl_cloud_sample.size(), 3);
        Eigen::VectorXf b(pcl_cloud_sample.size());
        for (std::size_t i = 0; i < pcl_cloud_sample.size(); i++)
        {
            A(i, 0) = pcl_cloud_sample[i].x;
            A(i, 1) = pcl_cloud_sample[i].y;
            A(i, 2) = 1.0;
            b(i) = pcl_cloud_sample[i].z;
        }

        // Perform least squares
        Eigen::Vector3f coeffs = A.colPivHouseholderQr().solve(b);

        // Publish Ground Plane Coefficients
        geometry_msgs::msg::Vector3Stamped ground_coefficients;
        ground_coefficients.header = msg->header;
        ground_coefficients.vector.x = coeffs(0);
        ground_coefficients.vector.y = coeffs(1);
        ground_coefficients.vector.z = coeffs(2);
        _ground_coefficients_publisher->publish(ground_coefficients);

        // Publish Ground Plane
        publish_plane(ground_coefficients);

        RCLCPP_INFO(this->get_logger(), "Ground Plane Coefficients: %f %f %f", coeffs(0), coeffs(1), coeffs(2));
    }

    void publish_plane(const geometry_msgs::msg::Vector3Stamped &coeffs)
    {
        geometry_msgs::msg::PolygonStamped ground_plane;
        ground_plane.header = coeffs.header;

        // Create Plane Points
        geometry_msgs::msg::Point32 point;
        point.x = _plane_width / 2.0;
        point.y = _plane_width / 2.0;
        point.z = coeffs.vector.x * point.x + coeffs.vector.y * point.y + coeffs.vector.z;
        ground_plane.polygon.points.push_back(point);

        point.x = -_plane_width / 2.0;
        point.y = _plane_width / 2.0;
        point.z = coeffs.vector.x * point.x + coeffs.vector.y * point.y + coeffs.vector.z;
        ground_plane.polygon.points.push_back(point);

        point.x = -_plane_width / 2.0;
        point.y = -_plane_width / 2.0;
        point.z = coeffs.vector.x * point.x + coeffs.vector.y * point.y + coeffs.vector.z;
        ground_plane.polygon.points.push_back(point);

        point.x = _plane_width / 2.0;
        point.y = -_plane_width / 2.0;
        point.z = coeffs.vector.x * point.x + coeffs.vector.y * point.y + coeffs.vector.z;
        ground_plane.polygon.points.push_back(point);

        point.x = _plane_width / 2.0;
        point.y = _plane_width / 2.0;
        point.z = coeffs.vector.x * point.x + coeffs.vector.y * point.y + coeffs.vector.z;
        ground_plane.polygon.points.push_back(point);

        // Publish Ground Plane
        _ground_plane_publisher->publish(ground_plane);
    }

public:
    GroundPlaneDetectionNode() : Node("ground_plane_detection_node")
    {
        this->declare_parameter("sample_size", 100);
        _sample_size = this->get_parameter("sample_size").as_int();

        this->declare_parameter("range_min", 0.1);
        _range_min = this->get_parameter("range_min").as_double();

        this->declare_parameter("range_max", 5.0);
        _range_max = this->get_parameter("range_max").as_double();

        this->declare_parameter("plane_width", 5.0);
        _plane_width = this->get_parameter("plane_width").as_double();

        _point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10, std::bind(&GroundPlaneDetectionNode::point_cloud_callback, this, std::placeholders::_1));

        _ground_coefficients_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("ground/coeffs", 10);

        _ground_plane_publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>("ground/plane", 10);

        RCLCPP_INFO(this->get_logger(), "Ground Plane Detection Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneDetectionNode>());
    rclcpp::shutdown();
    return 0;
}