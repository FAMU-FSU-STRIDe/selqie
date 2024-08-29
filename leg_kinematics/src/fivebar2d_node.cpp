#include <rclcpp/rclcpp.hpp>

#include "leg_kinematics/leg_kinematics_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("~"));
    rclcpp::shutdown();
    return 0;
}