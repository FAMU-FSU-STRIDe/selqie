#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudNode : public rclcpp::Node
{
private:
    std::string _layer_name;
    int _thickness;

    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr _map_sub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    pcl::PointCloud<pcl::PointXYZ> _cloud;
    grid_map_msgs::msg::GridMap::SharedPtr _map_msg;

    void _cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!_map_msg)
        {
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _tf_buffer->lookupTransform(
                _map_msg->header.frame_id, msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::fromROSMsg(*msg, cloud_in);
        for (const auto &point : cloud_in.points)
        {
            geometry_msgs::msg::Point point_in;
            point_in.x = point.x;
            point_in.y = point.y;
            point_in.z = point.z;

            geometry_msgs::msg::Point point_out;
            tf2::doTransform(point_in, point_out, transform);

            pcl::PointXYZ point_out_pcl;
            point_out_pcl.x = point_out.x;
            point_out_pcl.y = point_out.y;
            point_out_pcl.z = point_out.z;

            _cloud.push_back(point_out_pcl);
        }
    }

    void _map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        _map_msg = msg;
    }

    void _update_map()
    {
        if (!_map_msg)
        {
            return;
        }

        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromMessage(*_map_msg, grid_map);

        if (!grid_map.exists(_layer_name))
        {
            grid_map.add(_layer_name, 0.0);
        }

        for (const auto &point : _cloud.points)
        {
            grid_map::Index index;
            if (grid_map.getIndex(grid_map::Position(point.x, point.y), index))
            {
                for (int i = -_thickness; i <= _thickness; i++)
                {
                    for (int j = -_thickness; j <= _thickness; j++)
                    {
                        grid_map::Index index_offset = index + grid_map::Index(i, j);
                        if (grid_map.isValid(index_offset))
                        {
                            grid_map.at(_layer_name, index_offset) = 1.0;

                            auto &elevation = grid_map.at("elevation", index_offset);
                            if (std::isnan(elevation) || elevation < point.z)
                            {
                                elevation = point.z;
                            }
                        }
                    }
                }
            }
        }
        _cloud.clear();

        const auto map_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
        map_msg->header.stamp = this->now();
        _map_pub->publish(*map_msg);
    }

public:
    PointCloudNode() : Node("terrain_mapping_node")
    {
        this->declare_parameter<std::string>("layer_name", "visible");
        this->get_parameter("layer_name", _layer_name);

        this->declare_parameter("thickness", 1);
        this->get_parameter("thickness", _thickness);

        this->declare_parameter("frequency", 2.0);
        const double frequency = this->get_parameter("frequency").as_double();

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10, std::bind(&PointCloudNode::_cloud_callback, this, std::placeholders::_1));

        _map_sub = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "map", 10, std::bind(&PointCloudNode::_map_callback, this, std::placeholders::_1));

        _map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(time_t(1E3 / frequency)),
            std::bind(&PointCloudNode::_update_map, this));

        RCLCPP_INFO(this->get_logger(), "PointCloud Mapping Node Initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudNode>());
    rclcpp::shutdown();
    return 0;
}