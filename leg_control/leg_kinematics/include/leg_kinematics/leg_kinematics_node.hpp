#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

// Headers for custom ROS messages
#include <actuation_msgs/msg/motor_command.hpp>
#include <actuation_msgs/msg/motor_estimate.hpp>
#include <leg_control_msgs/msg/leg_command.hpp>
#include <leg_control_msgs/msg/leg_estimate.hpp>

// Use namespaces to simplify code
using namespace Eigen;
using namespace actuation_msgs::msg;
using namespace leg_control_msgs::msg;

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
 * Convert a Vector3f to a geometry_msgs::msg::Vector3 message
 */
static inline geometry_msgs::msg::Vector3 toVector3(const Vector3f &vec)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = vec(0);
    msg.y = vec(1);
    msg.z = vec(2);
    return msg;
}

/*
 * Convert a geometry_msgs::msg::Vector3 to a Vector3f
 */
static inline Vector3f fromVector3(const geometry_msgs::msg::Vector3 &msg)
{
    return Vector3f(msg.x, msg.y, msg.z);
}

/*
 * Abstract class for a Leg Kinematics Model
 */
class LegKinematicsModel
{
public:
    /*
     * Number of motors in the leg kinematics model.
     */
    virtual std::size_t get_num_motors() const = 0;

    /*
     * Forward kinematic function
     * Maps joint angles (motor positions) to the foot position in 3D space.
     */
    virtual Vector3f compute_forward_kinematics(const Vector3f &joint_angles) const = 0;

    /*
     * Inverse kinematic function
     * Maps a desired foot position in 3D space to the required joint angles (motor positions).
     */
    virtual Vector3f compute_inverse_kinematics(const Vector3f &foot_position) const = 0;

    /*
     * Jacobian matrix function
     * Computes the Jacobian matrix for the leg kinematics model.
     */
    virtual Matrix3f compute_jacobian_matrix(const Vector3f &joint_angles) const = 0;
};

/*
 * Leg Kinematics Node class
 */
class LegKinematicsNode
{
private:
    rclcpp::Node *_node;        // Pointer to the ROS node
    LegKinematicsModel *_model; // Pointer to the leg kinematics model

    rclcpp::Subscription<LegCommand>::SharedPtr _leg_command_sub; // Subscription for leg commands
    rclcpp::Publisher<LegEstimate>::SharedPtr _leg_estimate_pub;  // Publisher for leg estimates

    std::vector<rclcpp::Subscription<MotorEstimate>::SharedPtr> _motor_estimate_subs; // Subscriptions for motor estimates
    std::vector<rclcpp::Publisher<MotorCommand>::SharedPtr> _motor_command_pubs;      // Publishers for motor commands

    rclcpp::TimerBase::SharedPtr _motor_estimate_timer; // Timer for periodic leg estimate updates

    std::vector<MotorEstimate> _latest_motor_estimates; // Stores the latest motor estimates

    /*
     * Get the latest motor positions in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_positions() const
    {
        Vector3f positions;
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            positions(i) = _latest_motor_estimates[i].pos_estimate;
        }
        return positions;
    }

    /*
     * Get the latest motor velocities in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_velocities() const
    {
        Vector3f velocities;
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            velocities(i) = _latest_motor_estimates[i].vel_estimate;
        }
        return velocities;
    }

    /*
     * Get the latest motor torques in vector form from the stored motor estimate messages
     */
    Vector3f _get_latest_motor_torques() const
    {
        Vector3f torques;
        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            torques(i) = _latest_motor_estimates[i].torq_estimate;
        }
        return torques;
    }

    /*
     * Callback function for leg command messages
     */
    void _leg_command_callback(const LegCommand &msg)
    {
        // Convert message setpoints to eigen vectors
        const Vector3f pos_setpoint = fromVector3(msg.pos_setpoint);
        const Vector3f vel_setpoint = fromVector3(msg.vel_setpoint);
        const Vector3f force_setpoint = fromVector3(msg.force_setpoint);

        // Compute the current jacobian matrix
        const Matrix3f jacobian = _model->compute_jacobian_matrix(_get_latest_motor_positions());

        // Make sure the jacobian is invertible
        if (jacobian.determinant() == 0.0)
        {
            RCLCPP_ERROR(_node->get_logger(), "Jacobian determinant is zero");
            return;
        }

        // Compute the motor positions, velocities, and torques
        const Vector3f motor_pos = _model->compute_inverse_kinematics(pos_setpoint);
        const Vector3f motor_vels = jacobian.inverse() * vel_setpoint;
        const Vector3f motor_torqs = jacobian.transpose() * force_setpoint;

        for (std::size_t i = 0; i < _model->get_num_motors(); i++)
        {
            // Create new ROS messages for each motor command
            MotorCommand motor_cmd;
            motor_cmd.control_mode = msg.control_mode; // control mode matches the leg control mode
            motor_cmd.pos_setpoint = motor_pos(i);
            motor_cmd.vel_setpoint = motor_vels(i);
            motor_cmd.torq_setpoint = motor_torqs(i);

            // Publish the motor command to the corresponding motor topic
            _motor_command_pubs[i]->publish(motor_cmd);
        }
    }

    /*
     * Callback function for periodic leg estimate updates
     */
    void _leg_estimate_callback()
    {
        // Get the latest motor estimates
        const Vector3f motor_positions = _get_latest_motor_positions();

        // Compute the current jacobian matrix
        const Matrix3f jacobian = _model->compute_jacobian_matrix(motor_positions);

        // Make sure the jacobian is invertible
        if (jacobian.determinant() == 0.0)
        {
            RCLCPP_ERROR(_node->get_logger(), "Jacobian determinant is zero");
            return;
        }

        // Compute the foot position, velocity, and force estimates
        const Vector3f foot_position = _model->compute_forward_kinematics(motor_positions);
        const Vector3f foot_velocity = jacobian * _get_latest_motor_velocities();
        const Vector3f foot_force = jacobian.transpose().inverse() * _get_latest_motor_torques();

        // Create leg estimate message
        LegEstimate msg;
        msg.pos_estimate = toVector3(foot_position);
        msg.vel_estimate = toVector3(foot_velocity);
        msg.force_estimate = toVector3(foot_force);

        // Publish the leg estimate message
        _leg_estimate_pub->publish(msg);
    }

public:
    LegKinematicsNode(rclcpp::Node *node, LegKinematicsModel *model) : _node(node), _model(model)
    {
        const std::size_t num_motors = _model->get_num_motors();
        assert(num_motors < 1 || num_motors > 3); // Ensure the number of motors is valid (1 to 3)

        // Set the motor estimates vector size to the number of motors
        _latest_motor_estimates.resize(num_motors);

        // Create publisher and subscriber for leg commands and estimates
        _leg_command_sub = node->create_subscription<LegCommand>(
            "leg/command", qos_reliable(), std::bind(&LegKinematicsNode::_leg_command_callback, this, std::placeholders::_1));
        _leg_estimate_pub = node->create_publisher<LegEstimate>("leg/estimate", qos_fast());

        for (std::size_t m = 0; m < num_motors; m++)
        {
            // Create a callback function for each motor estimate subscription
            const auto estimate_callback = [this, m](const MotorEstimate::SharedPtr msg)
            {
                _latest_motor_estimates[m] = *msg;
            };

            // Create the subscibers and publishers for each motor
            _motor_estimate_subs.push_back(
                node->create_subscription<MotorEstimate>("motor" + std::to_string(m) + "/estimate", qos_fast(), estimate_callback));
            _motor_command_pubs.push_back(
                node->create_publisher<MotorCommand>("motor" + std::to_string(m) + "/command", qos_reliable()));
        }

        // Create a timer for periodic leg estimates
        node->declare_parameter("estimate_rate", 50.0);
        double estimate_rate = node->get_parameter("estimate_rate").as_double();
        _motor_estimate_timer = node->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&LegKinematicsNode::_leg_estimate_callback, this));

        RCLCPP_INFO(_node->get_logger(), "Leg Kinematics node Initialized");
    }
};
