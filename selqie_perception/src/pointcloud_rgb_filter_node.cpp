#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudRGBFilterNode : public rclcpp::Node
{
private:
    int _r, _g, _b;
    int _rdev, _gdev, _bdev;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
public:
    PointCloudRGBFilterNode() : Node("pointcloud_rgb_filter_node")
    {
        this->declare_parameter("rgb", std::vector<int>{0, 0, 0});
        const auto rgb = this->get_parameter("rgb").as_integer_array();
        assert(rgb.size() == 3);
        _r = rgb[0];
        _g = rgb[1];
        _b = rgb[2];

        this->declare_parameter("rgb_deviation", std::vector<int>{0, 0, 0});
        const auto rgb_deviation = this->get_parameter("rgb_deviation").as_integer_array();
        assert(rgb_deviation.size() == 3);
        _rdev = rgb_deviation[0];
        _gdev = rgb_deviation[1];
        _bdev = rgb_deviation[2];

        _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/in", 10, std::bind(&PointCloudRGBFilterNode::callback, this, std::placeholders::_1));

        _pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("points/out", 10);
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

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

        sensor_msgs::msg::PointCloud2 msg_out;
        pcl::toROSMsg(pcl_cloud_filtered, msg_out);
        msg_out.header = msg->header;
        _pub->publish(msg_out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRGBFilterNode>());
    rclcpp::shutdown();
    return 0;
}