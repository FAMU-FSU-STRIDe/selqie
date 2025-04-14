#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <leg_control_msgs/msg/leg_command.hpp>

// Number of legs of the robot
#define NUM_LEGS 4

// Leg command poll rate in nanoseconds
// This is the maximum rate at which leg commands can be generated and sent to the legs
#define LEG_COMMAND_POLL_RATE_NS 1000

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

/*
 * Stride Generation Model Interface
 * This interface defines the methods that any stride generation model must implement
 */
class StrideGenerationModel
{
public:
    /*
     * Get the name of the stride generation model gait
     */
    virtual std::string get_model_name() const = 0;

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    virtual void update_velocity(const geometry_msgs::msg::Twist &velocity) = 0;

    /*
     * Get the number of points in the stride trajectory
     */
    virtual std::size_t get_trajectory_size() const = 0;

    /*
     * Get the time of execution for the leg command at index i along the stride trajectory
     */
    virtual double get_execution_time(const int i) const = 0;

    /*
     * Get the leg command at index i along the stride trajectory
     */
    virtual leg_control_msgs::msg::LegCommand get_leg_command(const int leg, const int i) const = 0;

    /*
     * Get the odometry estimate at index i along the stride trajectory
     */
    virtual geometry_msgs::msg::TwistWithCovarianceStamped get_odometry_estimate(const int i) const = 0;
};

/*
 * Stride Generation Node
 * This class implements the stride generation node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class StrideGenerationNode
{
private:
    rclcpp::Node *_node;           // Pointer to the ROS node
    StrideGenerationModel *_model; // Pointer to the stride generation model

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;                                        // Subscription to current gait
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocity_sub;                                // Subscription to velocity command
    std::array<rclcpp::Publisher<leg_control_msgs::msg::LegCommand>::SharedPtr, NUM_LEGS> _leg_command_pubs; // Publishers for leg commands
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _odometry_pub;              // Publisher for odometry estimates
    rclcpp::TimerBase::SharedPtr _timer;                                                                     // Timer for publishing leg commands and odometry estimates

    int _current_index; // Current index in the stride trajectory

    /*
     * Gait callback function
     * This function is called when a new gait command is received
     */
    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Check if the gait command matches the model name
        if (msg->data == _model->get_model_name())
        {
            // If so, activate the leg command timer if not already active
            if (!_timer)
            {
                // Create a timer at the maximum poll rate
                _timer = _node->create_wall_timer(
                    std::chrono::nanoseconds(LEG_COMMAND_POLL_RATE_NS),
                    std::bind(&StrideGenerationNode::_timer_callback, this));

                // Reset the current index
                _current_index = 0;

                // Give feedback to the user
                RCLCPP_INFO(_node->get_logger(), "Stride Generation node activated gait: %s", msg->data.c_str());
            }
        }
        else
        {
            // If not, deactivate the leg command timer if active
            if (_timer)
            {
                // Cancel the timer
                _timer->cancel();

                // Reset the shared pointer to the timer
                _timer.reset();

                // Give feedback to the user
                RCLCPP_INFO(_node->get_logger(), "Stride Generation node deactivated gait: %s", msg->data.c_str());
            }
        }
    }

    /*
     * Velocity callback function
     * This function is called when a new velocity command is received
     */
    void _velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Check if the timer is active
        if (!_timer)
        {
            // If not, return without processing the velocity command
            // Note that this means the gait will need to be set some time before the velocity command
            // is sent to ensure the timer is active
            return;
        }

        // Update the velocity of the stride generation model
        _model->update_velocity(*msg);
    }

    /*
     * Timer callback function
     * Handles the timing and execution of leg commands and odometry estimates
     */
    void _timer_callback()
    {
        static double last_time = 0.0; // Last time the timer was called

        if (_current_index >= _model->get_trajectory_size())
        {
            // If the current index is greater than the trajectory size, reset the index
            _current_index = 0;
        }

        // Get the timing difference between the current and last leg command on the trajectory
        const double delta = _current_index == 0
                                 ? 0.0
                                 : _model->get_execution_time(_current_index) - _model->get_execution_time(_current_index - 1);

        // Get the timing difference between the current and last leg command execution times
        const double diff = _node->now().seconds() - last_time;

        // Check if the timing difference is greater than the execution difference
        if (delta > diff)
        {
            // If so, return without executing the leg command
            // Essentially waits until the next leg command is ready to be executed
            return;
        }

        // Publish the leg commands for each leg
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            // Get the leg command for the current index
            const auto leg_command = _model->get_leg_command(leg, _current_index);

            // Publish the leg command
            _leg_command_pubs[leg]->publish(leg_command);
        }

        // Publish the odometry estimate
        auto odometry_estimate = _model->get_odometry_estimate(_current_index);
        odometry_estimate.header.stamp = _node->now();
        odometry_estimate.header.frame_id = "base_link";
        _odometry_pub->publish(odometry_estimate);

        // Increment the current index
        ++_current_index;

        // Update the last time
        last_time = _node->now().seconds();
    }

public:
    StrideGenerationNode(rclcpp::Node *node, StrideGenerationModel *model)
        : _node(node), _model(model), _current_index(0)
    {
        // Get ROS parameters
        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        _node->declare_parameter("leg_names", leg_names);
        _node->get_parameter("leg_names", leg_names);
        assert(leg_names.size() == 4);

        // Create the gait subscription
        _gait_sub = _node->create_subscription<std_msgs::msg::String>(
            "gait", qos_reliable(), std::bind(&StrideGenerationNode::_gait_callback, this, std::placeholders::_1));

        // Create the velocity subscription
        _velocity_sub = _node->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_reliable(), std::bind(&StrideGenerationNode::_velocity_callback, this, std::placeholders::_1));

        // Create the leg command publishers
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            _leg_command_pubs[leg] = _node->create_publisher<leg_control_msgs::msg::LegCommand>(
                "leg" + leg_names[leg] + "/command", qos_reliable());
        }

        // Create the odometry publisher
        _odometry_pub = _node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "gait/vel_estimate", qos_reliable());
    }
};