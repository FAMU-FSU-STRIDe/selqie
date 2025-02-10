#include "stride_maker/stride_maker_node.hpp"

class SinkNode : public StrideMakerNode
{
private:
    void update_stride(const geometry_msgs::msg::Twist::SharedPtr) override
    {
        _make_default_stride();
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