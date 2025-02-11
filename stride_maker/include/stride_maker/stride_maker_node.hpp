#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <robot_msgs/msg/leg_command.hpp>

using namespace std::chrono_literals;

class StrideMakerNode : public rclcpp::Node
{
protected:
    std::string _gait_name;
    double _default_height;
    std::vector<double> _covariances;

    std::vector<double> _timing;
    std::vector<std::vector<robot_msgs::msg::LegCommand>> _leg_commands;
    std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> _gait_odometry;

    void _make_default_stride()
    {
        _timing = {0.0, 0.1};
        robot_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
        leg_command.pos_setpoint.x = 0.0;
        leg_command.pos_setpoint.z = -_default_height;
        std::vector<robot_msgs::msg::LegCommand> commands = {leg_command, leg_command};
        _leg_commands = {commands, commands, commands, commands};
    }

    void _stamp_twist(geometry_msgs::msg::TwistWithCovarianceStamped &msg)
    {
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.twist.covariance[0] = _covariances[0];
        msg.twist.covariance[7] = _covariances[1];
        msg.twist.covariance[14] = _covariances[2];
        msg.twist.covariance[21] = _covariances[3];
        msg.twist.covariance[28] = _covariances[4];
        msg.twist.covariance[35] = _covariances[5];
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_name_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_cmd_pubs;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _gait_odometry_pub;
    rclcpp::TimerBase::SharedPtr _cmd_timer, _odom_timer;

    std::string _current_gait;
    int _idx = 0;
    double _last_time = 0.0;

    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == _gait_name && msg->data == _current_gait)
        {
            return;
        }

        if (msg->data == _gait_name)
        {
            RCLCPP_INFO(this->get_logger(), "Switching to Gait: %s", _gait_name.c_str());
        }

        _timing.clear();
        _leg_commands.clear();
        _gait_odometry.clear();

        _current_gait = msg->data;
        _idx = 0;
    }

    void _cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (_current_gait != _gait_name)
        {
            return;
        }

        _timing.clear();
        _leg_commands.clear();
        _gait_odometry.clear();
        update_stride(msg);
    }

    void _publish_leg_command()
    {
        if (_timing.empty() || _leg_commands.empty())
        {
            _idx = 0;
            return;
        }
        assert(_leg_commands.size() == _leg_cmd_pubs.size());

        if (_idx >= static_cast<int>(_timing.size()))
            _idx = 0;

        const double delta = _idx > 0 ? (_timing[_idx] - _timing[_idx - 1]) : 0.0;
        const double diff = this->now().seconds() - _last_time;
        if (delta <= diff)
        {
            for (size_t i = 0; i < _leg_cmd_pubs.size(); i++)
            {
                assert(_leg_commands[i].size() == _timing.size());
                _leg_cmd_pubs[i]->publish(_leg_commands[i][_idx]);
            }

            if (!_gait_odometry.empty())
            {
                assert(_gait_odometry.size() == _timing.size());
                _stamp_twist(_gait_odometry[_idx]);
                _gait_odometry_pub->publish(_gait_odometry[_idx]);
            }

            ++_idx;
            _last_time = this->now().seconds();
        }
    }

public:
    StrideMakerNode(const std::string &gait_name)
        : Node(gait_name + "_stride_node")
    {
        _gait_name = gait_name;

        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        this->declare_parameter("leg_names", leg_names);
        this->get_parameter("leg_names", leg_names);
        assert(leg_names.size() == 4);

        this->declare_parameter("default_height", 0.175);
        this->get_parameter("default_height", _default_height);

        this->declare_parameter("covariances", std::vector<double>(6, 0.01));
        this->get_parameter("covariances", _covariances);
        assert(_covariances.size() == 6);

        _gait_name_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&StrideMakerNode::_gait_callback, this, std::placeholders::_1));

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&StrideMakerNode::_cmd_vel_callback, this, std::placeholders::_1));

        for (const auto &leg_name : leg_names)
        {
            _leg_cmd_pubs.push_back(this->create_publisher<robot_msgs::msg::LegCommand>(
                "leg" + leg_name + "/command", 10));
        }

        _gait_odometry_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "gait/vel_estimate", 10);

        _cmd_timer = this->create_wall_timer(
            1ms, std::bind(&StrideMakerNode::_publish_leg_command, this));
    }

    virtual void update_stride(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
};