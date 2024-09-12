#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_trajectory.hpp>

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

using namespace robot_msgs::msg;

class LegTrajectoryPublisherNode : public rclcpp::Node
{
private:
    double max_freq_ = 500.0; // Hz

    rclcpp::Subscription<robot_msgs::msg::LegTrajectory>::SharedPtr _leg_trajectory_sub;
    rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr _leg_command_pub;

public:
    LegTrajectoryPublisherNode() : rclcpp::Node("leg_trajectory_publisher")
    {
        this->declare_parameter("max_frequency", max_freq_);
        this->get_parameter("max_frequency", max_freq_);

        _leg_trajectory_sub = this->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(),
            std::bind(&LegTrajectoryPublisherNode::legTrajectory, this, std::placeholders::_1));

        _leg_command_pub = this->create_publisher<LegCommand>("leg/command", qos_fast());
    }

    void legTrajectory(const LegTrajectory::SharedPtr msg)
    {
        if (msg->timing.size() != msg->commands.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                         msg->timing.size(), msg->commands.size());
            return;
        }

        if (!std::is_sorted(msg->timing.begin(), msg->timing.end()))
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory timing is not sorted");
            return;
        }

        const auto limit_dt = std::chrono::nanoseconds(time_t(1E9 / max_freq_));
        const auto cstart = this->now();
        auto climit = cstart;
        for (std::size_t i = 0; i < msg->commands.size(); i++)
        {
            const auto cnow = this->now();
            const auto delay = std::chrono::nanoseconds(time_t(msg->timing[i] * 1E9));
            if (cnow + delay < climit)
            {
                continue;
            }
            climit += limit_dt;

            const auto cdiff = (cnow - cstart).to_chrono<std::chrono::nanoseconds>();
            if (delay > cdiff)
            {
                rclcpp::sleep_for(delay - cdiff);
            }

            _leg_command_pub->publish(msg->commands[i]);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}