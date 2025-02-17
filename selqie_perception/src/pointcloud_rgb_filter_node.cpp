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
        this->declare_parameter("r", 0);
        this->get_parameter("r", _r);

        this->declare_parameter("g", 0);
        this->get_parameter("g", _g);

        this->declare_parameter("b", 0);
        this->get_parameter("b", _b);

        this->declare_parameter("r_deviation", 0);
        this->get_parameter("r_deviation", _rdev);

        this->declare_parameter("g_deviation", 0);
        this->get_parameter("g_deviation", _gdev);

        this->declare_parameter("b_deviation", 0);
        this->get_parameter("b_deviation", _bdev);

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