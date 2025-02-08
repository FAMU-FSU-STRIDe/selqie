#include "stride_maker/stride_maker_node.hpp"

class SinkNode : public StrideMakerNode
{
private:
    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr) override
    {
        const auto traj = make_default_stride(_default_height);
        return {traj, traj, traj, traj};
    }

public:
    SinkNode() : StrideMakerNode("sink")
    {
        RCLCPP_INFO(this->get_logger(), "Sink Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinkNode>());
    rclcpp::shutdown();
    return 0;
}