#include <rclcpp/rclcpp.hpp>
#include <leg_control_msgs/msg/leg_command.hpp>
#include <leg_control_msgs/msg/leg_trajectory.hpp>

// Leg command poll rate in nanoseconds
// This is the maximum rate at which leg commands can be generated and sent to the legs
#define LEG_COMMAND_POLL_RATE_NS 1000000UL

/*
 * Fast Quality of Service Protocol
 * Messages can be dropped if neccessary to maintain performance
 * Used for high-frequency data like sensor readings
 */
static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

/*
 * Reliable Quality of Service Protocol
 * Ensures that messages are delivered reliably, even if it means delaying them
 * Used for critical commands or control messages where data loss is unacceptable
 */
static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

// Use namespaces to avoid long names
using namespace leg_control_msgs::msg;

/*
 * LegTrajectoryPublisherNode
 * This node subscribes to a leg trajectory and publishes leg commands
 */
class LegTrajectoryPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<LegTrajectory>::SharedPtr _leg_trajectory_sub; // Subscription to leg trajectory
    rclcpp::Publisher<LegCommand>::SharedPtr _leg_command_pub;          // Publisher for leg commands
    rclcpp::TimerBase::SharedPtr _timer;                                // Timer for publishing leg commands

    std::size_t _idx = 0;           // Current index in the trajectory
    LegTrajectory::SharedPtr _traj; // Pointer to the current leg trajectory
    rclcpp::Time _start_time;       // Start time of the trajectory

    /*
     * Trajectory callback function
     */
    void _trajectory_callback(const LegTrajectory::SharedPtr msg)
    {
        // Use the timing data to activate or deactivate the node
        if (!msg->timing.empty())
        {
            // If not empty, update the trajectory

            // Check if the trajectory is valid
            if (msg->timing.size() != msg->commands.size())
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                             msg->timing.size(), msg->commands.size());
                return;
            }

            // Check if the trajectory is in chronological order
            if (!std::is_sorted(msg->timing.begin(), msg->timing.end()))
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory timing not in chronological order");
                return;
            }

            // Reset the trajectory index and start time
            _idx = 0;
            _traj = msg;
            _start_time = this->now();

            // Activate the node if not already active
            if (!_timer)
            {
                // Create a timer to publish leg commands at the specified rate
                _timer = this->create_wall_timer(
                    std::chrono::nanoseconds(LEG_COMMAND_POLL_RATE_NS),
                    std::bind(&LegTrajectoryPublisherNode::_publish_leg_command, this));
            }
        }
        else
        {
            // If empty, deactivate the node
            if (_timer)
            {
                // Cancel the timer
                _timer->cancel();

                // Reset the shared pointer to the timer
                _timer.reset();
            }
        }
    }

    void _publish_leg_command()
    {
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

public:
    LegTrajectoryPublisherNode() : rclcpp::Node("leg_trajectory_publisher")
    {
        // Create subscriptions and publishers
        _leg_trajectory_sub = this->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(),
            std::bind(&LegTrajectoryPublisherNode::_trajectory_callback, this, std::placeholders::_1));

        _leg_command_pub = this->create_publisher<LegCommand>("leg/command", qos_reliable());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}