#include "local_planning/local_planning_node.hpp"

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<Node>());
    rclcpp::shutdown();
    return 0;
}