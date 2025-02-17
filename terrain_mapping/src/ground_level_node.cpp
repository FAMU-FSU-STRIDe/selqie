#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GroundLevelNode : public rclcpp::Node
{
private:
    const std::string _ground_gait = "stand";
    double _robot_height;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr _map_sub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    double _ground_level = 0.0;
    bool _update_level = false;
    grid_map_msgs::msg::GridMap::SharedPtr _map_msg;

    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _update_level = (msg->data == _ground_gait);
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (_update_level)
        {
            _ground_level = msg->pose.pose.position.z - _robot_height;
        }
    }

    void _map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        _map_msg = msg;
    }

    void _update_map()
    {
        if (_map_msg)
        {
            grid_map::GridMap grid_map;
            grid_map::GridMapRosConverter::fromMessage(*_map_msg, grid_map);

            auto &elevation_layer = grid_map["elevation"];
            for (Eigen::Index i = 0; i < elevation_layer.rows(); i++)
            {
                for (Eigen::Index j = 0; j < elevation_layer.cols(); j++)
                {
                    if (std::isnan(elevation_layer(i, j)))
                    {
                        elevation_layer(i, j) = _ground_level;
                    }
                }
            }

            auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
            grid_map_msg->header.stamp = this->now();
            _map_pub->publish(*grid_map_msg);
        }
    }

public:
    GroundLevelNode()
        : Node("ground_level_node")
    {
        this->declare_parameter("robot_height", 0.2);
        this->get_parameter("robot_height", _robot_height);

        this->declare_parameter("frequency", 2.0);
        const double frequency = this->get_parameter("frequency").as_double();

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&GroundLevelNode::_gait_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GroundLevelNode::_odom_callback, this, std::placeholders::_1));

        _map_sub = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "map", 10, std::bind(&GroundLevelNode::_map_callback, this, std::placeholders::_1));

        _map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(time_t(1E3 / frequency)),
            std::bind(&GroundLevelNode::_update_map, this));

        RCLCPP_INFO(get_logger(), "Ground Level Mapping Node Initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundLevelNode>());
    rclcpp::shutdown();
    return 0;
}