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

class TerrainMappingNode : public rclcpp::Node
{
private:
    std::string _map_frame_id;

    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_sub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _grid_map_pub;

    std::unique_ptr<grid_map::GridMap> _grid_map;

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _tf_buffer->lookupTransform(_map_frame_id, msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        for (const auto &point : pcl_cloud.points)
        {
            geometry_msgs::msg::Point point_in;
            point_in.x = point.x;
            point_in.y = point.y;
            point_in.z = point.z;

            geometry_msgs::msg::Point point_out;
            tf2::doTransform(point_in, point_out, transform);

            grid_map::Position position(point_out.x, point_out.y);
            if (_grid_map->isInside(position))
            {
                _grid_map->atPosition("elevation", position) = point_out.z;
            }
        }

        const auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(*_grid_map);
        grid_map_msg->header.stamp = msg->header.stamp;
        _grid_map_pub->publish(*grid_map_msg);
    }

public:
    TerrainMappingNode() : Node("terrain_mapping_node")
    {
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->get_parameter("map_frame_id", _map_frame_id);

        this->declare_parameter<double>("map_resolution", 0.1);
        const double map_resolution = this->get_parameter("map_resolution").as_double();

        this->declare_parameter<double>("map_length", 10.0);
        const double map_length = this->get_parameter("map_length").as_double();

        _grid_map = std::make_unique<grid_map::GridMap>();
        _grid_map->setFrameId(_map_frame_id);
        _grid_map->setGeometry(grid_map::Length(map_length, map_length), map_resolution);
        _grid_map->setBasicLayers({"elevation"});
        _grid_map->add("elevation");

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        _point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10, std::bind(&TerrainMappingNode::point_cloud_callback, this, std::placeholders::_1));

        _grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(this->get_logger(), "Terrain Mapping Node Initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainMappingNode>());
    rclcpp::shutdown();
    return 0;
}