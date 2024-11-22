#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_trajectory.hpp>

#include <thread>

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
    LegTrajectory::SharedPtr _traj;
    bool _update = false;

    rclcpp::Subscription<LegTrajectory>::SharedPtr _leg_trajectory_sub;
    rclcpp::Publisher<LegCommand>::SharedPtr _leg_command_pub;

    std::mutex _mutex;

public:
    LegTrajectoryPublisherNode() : rclcpp::Node("leg_trajectory_publisher")
    {
        _leg_trajectory_sub = this->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(),
            std::bind(&LegTrajectoryPublisherNode::legTrajectory, this, std::placeholders::_1));

        _leg_command_pub = this->create_publisher<LegCommand>("leg/command", qos_fast());

        std::thread(&LegTrajectoryPublisherNode::run, this).detach();
    }

    void legTrajectory(const LegTrajectory::SharedPtr msg)
    {
        if (msg->timing.size() != msg->commands.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                         msg->timing.size(), msg->commands.size());
            return;
        }

        std::sort(msg->timing.begin(), msg->timing.end());

        std::lock_guard<std::mutex> lock(_mutex);
        _traj = msg;
        _update = true;
    }

    void run()
    {
        auto cstart = this->now();
        LegTrajectory traj;
        std::size_t idx = 0;
        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (_update)
                {
                    traj = *_traj;
                    idx = 0;
                    cstart = this->now();
                    _update = false;
                }
            }

            if (idx >= traj.timing.size() || traj.commands.empty())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            const auto delay = std::chrono::nanoseconds(time_t(traj.timing[idx] * 1E9));
            const auto cdiff = (this->now() - cstart).to_chrono<std::chrono::nanoseconds>();
            if (delay > cdiff)
            {
                rclcpp::sleep_for(delay - cdiff);
            }

            _leg_command_pub->publish(traj.commands[idx++]);
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