#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

#include <robot_msgs/msg/motor_command.hpp>
#include <robot_msgs/msg/motor_estimate.hpp>
#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/leg_trajectory.hpp>

#define MAX_COMMAND_FREQUENCY 500.0

using namespace Eigen;
using namespace robot_msgs::msg;

static inline rclcpp::QoS qos_fast(const int depth = 10)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
}

static inline rclcpp::QoS qos_reliable(const int depth = 10)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
}

// Abstract class for leg kinematics model
class LegKinematicsModel
{
public:
    virtual std::size_t getNumMotors() const = 0;

    virtual Vector3f getForwardKinematics(const Vector3f &joint_angles) const = 0;

    virtual Vector3f getInverseKinematics(const Vector3f &foot_position) const = 0;

    virtual Matrix3f getJacobian(const Vector3f &joint_angles) const = 0;
};

// Leg Kinematics Node
class LegKinematicsNode
{
private:
    rclcpp::Node *_node;
    LegKinematicsModel *_model;

    std::vector<MotorEstimate> _latest_motor_positions;

    rclcpp::Subscription<LegCommand>::SharedPtr _leg_command_sub;
    rclcpp::Publisher<LegEstimate>::SharedPtr _leg_estimate_pub;
    rclcpp::Subscription<LegTrajectory>::SharedPtr _leg_trajectory_sub;

    std::vector<rclcpp::Subscription<MotorEstimate>::SharedPtr> _motor_estimate_subs;
    std::vector<rclcpp::Publisher<MotorCommand>::SharedPtr> _motor_command_pubs;

    rclcpp::TimerBase::SharedPtr _motor_estimate_timer;

    Vector3f latestMotorPositions() const
    {
        Vector3f positions;
        for (std::size_t i = 0; i < _model->getNumMotors(); i++)
        {
            positions(i) = _latest_motor_positions[i].pos_estimate;
        }
        return positions;
    }

    Vector3f latestMotorVelocities() const
    {
        Vector3f velocities;
        for (std::size_t i = 0; i < _model->getNumMotors(); i++)
        {
            velocities(i) = _latest_motor_positions[i].vel_estimate;
        }
        return velocities;
    }

    Vector3f latestMotorTorques() const
    {
        Vector3f torques;
        for (std::size_t i = 0; i < _model->getNumMotors(); i++)
        {
            torques(i) = _latest_motor_positions[i].torq_estimate;
        }
        return torques;
    }

    geometry_msgs::msg::Vector3 toVector3(const Vector3f &vec) const
    {
        geometry_msgs::msg::Vector3 msg;
        msg.x = vec(0);
        msg.y = vec(1);
        msg.z = vec(2);
        return msg;
    }

public:
    LegKinematicsNode(rclcpp::Node *node, LegKinematicsModel *model) : _node(node), _model(model)
    {
        const std::size_t num_motors = _model->getNumMotors();
        if (num_motors < 1 || num_motors > 3)
        {
            RCLCPP_ERROR(_node->get_logger(), "Invalid number of motors: %lu (1-3 allowed)", num_motors);
            return;
        }

        _latest_motor_positions.resize(num_motors);

        _leg_command_sub = node->create_subscription<LegCommand>(
            "leg/command", qos_fast(), std::bind(&LegKinematicsNode::legCommand, this, std::placeholders::_1));
        _leg_estimate_pub = node->create_publisher<LegEstimate>("leg/estimate", qos_fast());
        _leg_trajectory_sub = node->create_subscription<LegTrajectory>(
            "leg/trajectory", qos_reliable(), std::bind(&LegKinematicsNode::legTrajectory, this, std::placeholders::_1));

        for (std::size_t m = 0; m < num_motors; m++)
        {
            const auto estimate_callback = [this, m](const MotorEstimate::SharedPtr msg)
            {
                _latest_motor_positions[m] = *msg;
            };

            _motor_estimate_subs.push_back(
                node->create_subscription<MotorEstimate>("motor" + std::to_string(m) + "/estimate", qos_fast(), estimate_callback));
            _motor_command_pubs.push_back(
                node->create_publisher<MotorCommand>("motor" + std::to_string(m) + "/command", qos_fast()));
        }

        double estimate_rate = 50.0;
        _motor_estimate_timer = node->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&LegKinematicsNode::legEstimate, this));

        RCLCPP_INFO(_node->get_logger(), "Leg Kinematics node initialized");
    }

    void legCommand(const LegCommand::UniquePtr msg)
    {
        std::vector<MotorCommand::UniquePtr> motor_cmds(_model->getNumMotors());
        std::generate(motor_cmds.begin(), motor_cmds.end(), []()
                      { return std::make_unique<MotorCommand>(); });

        for (auto &motor_cmd : motor_cmds)
        {
            motor_cmd->control_mode = msg->control_mode;
        }

        const Vector3f pos_setpoint(msg->pos_setpoint.x, msg->pos_setpoint.y, msg->pos_setpoint.z);
        const Vector3f vel_setpoint(msg->vel_setpoint.x, msg->vel_setpoint.y, msg->vel_setpoint.z);
        const Vector3f force_setpoint(msg->force_setpoint.x, msg->force_setpoint.y, msg->force_setpoint.z);

        const Matrix3f jacobian = _model->getJacobian(latestMotorPositions());

        if (jacobian.determinant() == 0.0)
        {
            RCLCPP_WARN_ONCE(_node->get_logger(), "Jacobian determinant is zero");
        }

        const Vector3f motor_pos = _model->getInverseKinematics(pos_setpoint);
        const Vector3f motor_vels = jacobian.inverse() * vel_setpoint;
        const Vector3f motor_torqs = jacobian.transpose() * force_setpoint;

        for (std::size_t i = 0; i < _model->getNumMotors(); i++)
        {
            motor_cmds[i]->pos_setpoint = motor_pos(i);
            motor_cmds[i]->vel_setpoint = motor_vels(i);
            motor_cmds[i]->torq_setpoint = motor_torqs(i);

            _motor_command_pubs[i]->publish(std::move(motor_cmds[i]));
        }
    }

    void legEstimate()
    {
        const Vector3f motor_positions = latestMotorPositions();
        const Matrix3f jacobian = _model->getJacobian(motor_positions);
        const Vector3f foot_position = _model->getForwardKinematics(motor_positions);
        const Vector3f foot_velocity = jacobian * latestMotorVelocities();
        const Vector3f foot_torque = jacobian.transpose() * latestMotorTorques();

        auto msg = std::make_unique<LegEstimate>();
        msg->pos_estimate = toVector3(foot_position);
        msg->vel_estimate = toVector3(foot_velocity);
        msg->force_estimate = toVector3(foot_torque);

        _leg_estimate_pub->publish(std::move(msg));
    }

    void legTrajectory(const LegTrajectory::UniquePtr msg)
    {
        if (msg->timing.size() != msg->commands.size())
        {
            RCLCPP_ERROR(_node->get_logger(), "Trajectory size mismatch: %lu timing, %lu commands",
                         msg->timing.size(), msg->commands.size());
            return;
        }

        assert(std::is_sorted(msg->timing.begin(), msg->timing.end()));

        const static std::chrono::nanoseconds limit_dt(time_t(1E9 / MAX_COMMAND_FREQUENCY));

        const auto cstart = _node->now();
        auto climit = cstart + limit_dt;
        for (std::size_t i = 0; i < msg->commands.size(); i++)
        {
            const auto cnow = _node->now();
            const auto cdiff = (cnow - cstart).to_chrono<std::chrono::nanoseconds>();
            const auto delay = std::chrono::nanoseconds(time_t(msg->timing[i] * 1E9));

            if (cnow + delay < climit)
            {
                continue;
            }

            if (delay > cdiff)
            {
                rclcpp::sleep_for(delay - cdiff);
            }

            legCommand(std::make_unique<LegCommand>(msg->commands[i]));
            climit += limit_dt;
        }
    }
};
