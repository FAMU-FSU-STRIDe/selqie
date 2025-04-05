#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

class RGBFilterNode : public rclcpp::Node
{
private:
    int _r, _g, _b;          // Mean RGB value to filter
    int _rdev, _gdev, _bdev; // Deviation for RGB filtering

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub; // Subscription to input point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;    // Publisher for output point cloud
public:
    RGBFilterNode() : Node("pointcloud_rgb_filter_node")
    {
        // Get ROS parameters
        this->declare_parameter("rgb", std::vector<int>{0, 0, 0});
        const auto rgb = this->get_parameter("rgb").as_integer_array();

        this->declare_parameter("rgb_deviation", std::vector<int>{0, 0, 0});
        const auto rgb_deviation = this->get_parameter("rgb_deviation").as_integer_array();

        // Make sure we have exactly 3 values for rgb and rgb_deviation
        assert(rgb.size() == 3);
        assert(rgb_deviation.size() == 3);

        // Assign RGB values and deviations
        _r = rgb[0];
        _g = rgb[1];
        _b = rgb[2];
        _rdev = rgb_deviation[0];
        _gdev = rgb_deviation[1];
        _bdev = rgb_deviation[2];

        // Create subscription and publisher
        _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/in", 10, std::bind(&RGBFilterNode::pointcloud_callback, this, std::placeholders::_1));

        _pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("points/out", 10);

        RCLCPP_INFO(get_logger(), "RGB Filter Node Initialized with parameters: "
                                  "RGB(%d, %d, %d), Deviation(%d, %d, %d)",
                    _r, _g, _b, _rdev, _gdev, _bdev);
    }

    /*
     * Callback function for processing incoming PointCloud2 messages.
     */
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the PointCloud2 message to PCL format
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Apply RGB filtering
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_filtered;
        for (const auto &point : pcl_cloud.points)
        {
            if (point.r >= _r - _rdev && point.r <= _r + _rdev &&
                point.g >= _g - _gdev && point.g <= _g + _gdev &&
                point.b >= _b - _bdev && point.b <= _b + _bdev)
            {
                pcl_cloud_filtered.push_back(point);
            }
        }

        // Convert the filtered PCL cloud back to PointCloud2 message
        sensor_msgs::msg::PointCloud2 msg_out;
        pcl::toROSMsg(pcl_cloud_filtered, msg_out);
        msg_out.header = msg->header;

        // Publish the filtered PointCloud2 message
        _pub->publish(msg_out);
    }
};

// Entry point for the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBFilterNode>());
    rclcpp::shutdown();
    return 0;
}