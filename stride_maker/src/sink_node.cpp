#include "stride_maker/stride_maker_node.hpp"

class SinkNode : public StrideMakerNode
{
private:
    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            return {};
        }

        return {};
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