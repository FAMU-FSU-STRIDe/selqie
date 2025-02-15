#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

class DefaultMapNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _reset_sub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;

    void _reset_map_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        const std::string frame_id = this->get_parameter("map_frame_id").as_string();
        const double resolution = this->get_parameter("map_resolution").as_double();
        const double length = this->get_parameter("map_length").as_double();

        grid_map::GridMap grid_map;
        grid_map.setFrameId(frame_id);
        grid_map.setGeometry(grid_map::Length(length, length), resolution);
        grid_map.setBasicLayers({"elevation"});
        grid_map.add("elevation");

        const auto map_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
        map_msg->header.stamp = this->now();
        _map_pub->publish(*map_msg);
    }

public:
    DefaultMapNode() : Node("default_map_node")
    {
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->declare_parameter<double>("map_resolution", 0.1);
        this->declare_parameter<double>("map_length", 10.0);

        _reset_sub = this->create_subscription<std_msgs::msg::Empty>(
            "map/reset", 10, std::bind(&DefaultMapNode::_reset_map_callback, this, std::placeholders::_1));

        _map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        RCLCPP_INFO(this->get_logger(), "Terrain Mapping Node Initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DefaultMapNode>());
    rclcpp::shutdown();
    return 0;
}