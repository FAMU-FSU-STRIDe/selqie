#include "stride_maker/stride_maker_node.hpp"

class StandNode : public StrideMakerNode
{
private:
    void update_stride(const geometry_msgs::msg::Twist::SharedPtr) override
    {
        _make_default_stride();
    }

public:
    StandNode() : StrideMakerNode("stand")
    {
        RCLCPP_INFO(this->get_logger(), "Stand Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandNode>());
    rclcpp::shutdown();
    return 0;
}