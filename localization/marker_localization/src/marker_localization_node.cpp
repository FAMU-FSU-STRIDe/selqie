#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

class MarkerLocalizationNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_publisher;

    std::unique_ptr<tf2_ros::TransformListener> _tf_listener; // Transform listener for TF2
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;              // Buffer for TF2 transforms

    nav_msgs::msg::Odometry::SharedPtr _odom_data;

    int _minumum_point_count = 10;             // Minimum number of points in the point cloud to process
    std::string _robot_frame_id = "base_link"; // Frame ID of the robot
    std::string _map_frame_id = "map";         // Frame ID of the map
    double _marker_x, _marker_y, _marker_z;    // Marker position in the map frame
    double _variance_x = 0.1; // Variance in the x direction
    double _variance_y = 0.1; // Variance in the y direction
    double _variance_z = 0.1; // Variance in the z direction

    void _point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Make sure we have valid odometry data before processing the point cloud
        if (!_odom_data)
        {
            return;
        }

        // Make sure the point cloud is large enough
        if (static_cast<int>(msg->width * msg->height) < _minumum_point_count)
        {
            return;
        }

        // Convert the point cloud message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Cluster cloud points
        tf2::Vector3 centroid(0, 0, 0);
        for (const auto &point : cloud.points)
        {
            centroid += tf2::Vector3(point.x, point.y, point.z);
        }
        centroid /= cloud.points.size();

        // Get the transform from the point cloud frame to the robot frame at the time of the message
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            // Lookup the transform using TF2
            transform = _tf_buffer->lookupTransform(
                _robot_frame_id, msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        // Transform the centroid to the robot frame
        geometry_msgs::msg::Point point_cam;
        point_cam.x = centroid.x();
        point_cam.y = centroid.y();
        point_cam.z = centroid.z();

        // Perform the transformation
        geometry_msgs::msg::Point point_marker;
        tf2::doTransform(point_cam, point_marker, transform);

        // Get the rotation of the robot from the map frame
        const auto &rotation = _odom_data->pose.pose.orientation;
        tf2::Matrix3x3 m_R_r(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));

        // Rotate the marker point into the map frame
        tf2::Vector3 r_p_robot_marker(point_marker.x, point_marker.y, point_marker.z);

        // Get the marker in the map frame
        tf2::Vector3 m_p_map_marker(_marker_x, _marker_y, _marker_z);

        // Calculate the robot position in the map frame
        auto m_p_map_robot = m_p_map_marker - m_R_r * r_p_robot_marker;

        // Convert to pose with covariance
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = _map_frame_id;
        pose.pose.pose.position.x = m_p_map_robot.x();
        pose.pose.pose.position.y = m_p_map_robot.y();
        pose.pose.pose.position.z = m_p_map_robot.z();
        pose.pose.covariance[0] = _variance_x;
        pose.pose.covariance[7] = _variance_y;
        pose.pose.covariance[14] = _variance_z;

        // Publish to the ROS network
        _pose_publisher->publish(pose);
        RCLCPP_INFO(this->get_logger(), "Marker position: (%f, %f, %f)", m_p_map_robot.x(), m_p_map_robot.y(), m_p_map_robot.z());
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom_data = msg;
    }

public:
    MarkerLocalizationNode()
        : Node("marker_localization_node")
    {
        // Get ROS parameters
        this->declare_parameter("marker_x", 0.0);
        this->get_parameter("marker_x", _marker_x);

        this->declare_parameter("marker_y", 0.0);
        this->get_parameter("marker_y", _marker_y);

        this->declare_parameter("marker_z", 0.0);
        this->get_parameter("marker_z", _marker_z);

        this->declare_parameter("robot_frame_id", "base_link");
        this->get_parameter("robot_frame_id", _robot_frame_id);

        this->declare_parameter("map_frame_id", "map");
        this->get_parameter("map_frame_id", _map_frame_id);

        this->declare_parameter("minimum_point_count", 10);
        this->get_parameter("minimum_point_count", _minumum_point_count);

        this->declare_parameter("variance_x", 0.1);
        this->get_parameter("variance_x", _variance_x);

        this->declare_parameter("variance_y", 0.1);
        this->get_parameter("variance_y", _variance_y);

        this->declare_parameter("variance_z", 0.1);
        this->get_parameter("variance_z", _variance_z);

        // Create publishers and subscribers
        _point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/marker", 10, std::bind(&MarkerLocalizationNode::_point_cloud_callback, this, std::placeholders::_1));

        _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MarkerLocalizationNode::_odom_callback, this, std::placeholders::_1));

        _pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose/marker", 10);

        // Create TF buffer and listener
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer, this);

        // Initialize the node and set up subscriptions, publishers, etc.
        RCLCPP_INFO(this->get_logger(), "Marker Localization Node Initialized");
    }
};

// Entry point for the marker localization node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}