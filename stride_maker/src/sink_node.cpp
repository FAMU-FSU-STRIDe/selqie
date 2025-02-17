#include "stride_maker/stride_maker_node.hpp"

class SinkNode : public StrideMakerNode
{
private:
    double _sink_speed;

    void update_stride(const geometry_msgs::msg::Twist::SharedPtr) override
    {
        _make_default_stride();

        geometry_msgs::msg::TwistWithCovarianceStamped sinking_odometry;
        sinking_odometry.twist.twist.linear.z = -_sink_speed;
        _gait_odometry = {sinking_odometry, sinking_odometry};
    }

public:
    SinkNode() : StrideMakerNode("sink")
    {
        this->declare_parameter("sink_speed", 0.1);
        this->get_parameter("sink_speed", _sink_speed);

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