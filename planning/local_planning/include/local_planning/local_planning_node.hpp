#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
 * Local Planning Model Interface
 * This interface defines the methods that any local planning model must implement
 */
class LocalPlanningModel
{
public:
    /*
     * Get the name of the stride generation model gait
     */
    virtual std::string get_model_name() const = 0;

    /*
     * Solve the local planning problem
     * Given a start pose and a goal pose, return the velocity command to reach the goal
     */
    virtual geometry_msgs::msg::Twist solve(const geometry_msgs::msg::Pose &start_pose,
                                            const geometry_msgs::msg::Pose &goal_pose) = 0;

    /*
     * Check if the goal is reached
     * Given a start pose and a goal pose, return true if the goal is reached
     */
    virtual bool is_goal_reached(const geometry_msgs::msg::Pose &start_pose,
                                 const geometry_msgs::msg::Pose &goal_pose) = 0;
};

class LocalPlanningNode
{
private:
    rclcpp::Node *_node;        // Pointer to the ROS node
    LocalPlanningModel *_model; // Pointer to the local planning model
    double _solve_frequency;    // Frequency at which to solve the local planning problem

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;            // Subscription to current gait
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_transition_sub; // Subscription to transition gait
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;  // Subscription to goal pose
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;          // Subscription to odometry
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _gait_pub;               // Publisher for current gait
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;        // Publisher for velocity command
    rclcpp::TimerBase::SharedPtr _timer;                                         // Timer for publishing velocity command

    bool _active; // Flag to indicate if the node is active

    std_msgs::msg::String::SharedPtr _gait_transition_msg; // Gait transition message
    geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg;  // Goal message
    nav_msgs::msg::Odometry::SharedPtr _odom_msg;          // Odometry message

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
            if (!_active)
            {
                // Set active flag to true
                _active = true;

                // Give feedback to the user
                RCLCPP_INFO(_node->get_logger(), "Local Planning node activated gait: %s", _model->get_model_name().c_str());
            }
        }
        else
        {
            // If not, deactivate the leg command timer if active
            if (_active)
            {
                // Set active flag to false
                _active = false;

                if (_timer)
                {
                    // Cancel the timer
                    _timer->cancel();

                    // Reset the shared pointer to the timer
                    _timer.reset();

                    // Reset the message pointers
                    _goal_msg.reset();
                    _odom_msg.reset();
                }

                // Give feedback to the user
                RCLCPP_INFO(_node->get_logger(), "Local Planning node deactivated gait: %s", _model->get_model_name().c_str());
            }
        }
    }

    /*
     * Gait transition callback function
     * This function is called when a new gait transition command is received
     */
    void _gait_transition_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _gait_transition_msg = msg;
    }

    /*
     * Goal callback function
     * This function is called when a new goal command is received
     */
    void _goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Check if the node is active
        if (!_active)
        {
            // If not, ignore the goal
            return;
        }

        _goal_msg = msg;

        // Check if the timer needs to be activated
        if (!_timer)
        {
            // Create a timer at the solve frequency
            _timer = _node->create_wall_timer(
                std::chrono::milliseconds(static_cast<time_t>(1e3 / _solve_frequency)),
                std::bind(&LocalPlanningNode::_timer_callback, this));
        }
    }

    /*
     * Odometry callback function
     * This function is called when a new odometry command is received
     */
    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom_msg = msg;
    }

    /*
     * Timer callback function
     * This function is called at the specified solve frequency
     */
    void _timer_callback()
    {
        if (!(_gait_transition_msg && _goal_msg && _odom_msg))
        {
            // If either the goal or odometry message is not set, return
            return;
        }

        // Check if the goal is reached
        if (_model->is_goal_reached(_odom_msg->pose.pose, _goal_msg->pose))
        {
            // If so, publish the transition gait message as the current gait
            _gait_pub->publish(*_gait_transition_msg);
            return;
        }
        else
        {
            // Otherwise, solve the local planning problem
            const auto cmd_vel = _model->solve(_odom_msg->pose.pose, _goal_msg->pose);

            // Publish the velocity command
            _cmd_vel_pub->publish(cmd_vel);
        }
    }
};
