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
    rclcpp::Subscription<LegTrajectory>::SharedPtr _leg_trajectory_sub;
    rclcpp::Publisher<LegCommand>::SharedPtr _leg_command_pub;
    rclcpp::TimerBase::SharedPtr _publish_timer;

    std::size_t _idx = 0;
    LegTrajectory::SharedPtr _traj;
    rclcpp::Time _start_time;

public:
    LegTrajectoryPublisherNode() : rclcpp::Node("leg_trajectory_publisher")
    {
        _leg_trajectory_sub = this->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(),
            std::bind(&LegTrajectoryPublisherNode::updateTrajectory, this, std::placeholders::_1));

        _leg_command_pub = this->create_publisher<LegCommand>("leg/command", qos_fast());

        _publish_timer = this->create_wall_timer(std::chrono::milliseconds(1),
                                                 std::bind(&LegTrajectoryPublisherNode::publishLegCommand, this));
    }

    void updateTrajectory(const LegTrajectory::SharedPtr msg)
    {
        if (msg->timing.empty())
        {
            _traj = nullptr;
            return;
        }

        if (msg->timing.size() != msg->commands.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                         msg->timing.size(), msg->commands.size());
            return;
        }

        if (!std::is_sorted(msg->timing.begin(), msg->timing.end()))
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory timing not in chronological order");
            return;
        }

        _idx = 0;
        _traj = msg;
        _start_time = this->now();
    }

    void publishLegCommand()
    {
        if (!_traj)
        {
            return;
        }

        using namespace std::chrono;
        const auto delay = milliseconds(time_t(_traj->timing[_idx] * 1E3));
        const auto cdiff = (this->now() - _start_time).to_chrono<milliseconds>();
        if (delay <= cdiff)
        {
            _leg_command_pub->publish(_traj->commands[_idx++]);

            if (_idx >= _traj->timing.size())
            {
                _traj = nullptr;
            }
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