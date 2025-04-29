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
    double _start_time;       // Start time of the trajectory

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
            _start_time = this->now().seconds();

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

    /*
     * Publish leg command function
     * This function is called at a fixed rate to publish leg commands
     */
    void _publish_leg_command()
    {
        const auto current_time = this->now().seconds();
        if (current_time < _start_time)
        {
            // If the current time is before the start time, return
            // Happens if the simulation is reset
            return;
        }

        // Get the delay for the current leg command in seconds
        const auto delay = _traj->timing[_idx] / 1E3;

        // Check if the delay greater than the current time
        const auto diff = current_time - _start_time;
        if (delay > diff)
        {
            // If so, return without executing the leg command
            // Essentially waits until the next leg command is ready to be executed
            return;
        }

        // Publish the leg command for the current index and increment the index
        _leg_command_pub->publish(_traj->commands[_idx++]);

        // Check if the index is now greater than the trajectory size
        if (_idx >= _traj->timing.size())
        {
            // If so, deactivate the trajectory publisher

            // Cancel the timer
            _timer->cancel();

            // Reset the shared pointer to the timer
            _timer.reset();
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

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}