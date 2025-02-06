#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "stride_maker/stride_maker.hpp"

class StrideMakerNode : public rclcpp::Node
{
protected:
    std::string _gait_name = "walk";
    int _stride_resolution = 100;
    double _frequency = 1.0;
    double _default_height = 0.18;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_name_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_cmd_pubs;
    rclcpp::TimerBase::SharedPtr _timer;

    std::string _current_gait;
    int _idx = 0;
    std::vector<robot_msgs::msg::LegTrajectory> _trajectories;
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

        _current_gait = msg->data;
        _trajectories.clear();
        _idx = 0;
    }

    void _cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (_current_gait != _gait_name)
        {
            return;
        }

        _trajectories = get_stride(msg);
    }

    void _publish_leg_command()
    {
        if (_trajectories.empty())
        {
            _idx = 0;
            return;
        }

        const double delta = _idx > 0
                                 ? (_trajectories[0].timing[_idx] - _trajectories[0].timing[_idx - 1])
                                 : 0.0;
        const double diff = this->now().seconds() - _last_time;
        if (delta <= diff)
        {
            for (size_t i = 0; i < _leg_cmd_pubs.size(); i++)
            {
                _leg_cmd_pubs[i]->publish(_trajectories[i].commands[_idx]);
            }

            if (++_idx >= _stride_resolution)
            {
                _idx = 0;
            }

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

        this->declare_parameter("stride_resolution", _stride_resolution);
        this->get_parameter("stride_resolution", _stride_resolution);

        this->declare_parameter("frequency", _frequency);
        this->get_parameter("frequency", _frequency);

        this->declare_parameter("default_height", _default_height);
        this->get_parameter("default_height", _default_height);

        _gait_name_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&StrideMakerNode::_gait_callback, this, std::placeholders::_1));

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&StrideMakerNode::_cmd_vel_callback, this, std::placeholders::_1));

        for (const auto &leg_name : leg_names)
        {
            _leg_cmd_pubs.push_back(this->create_publisher<robot_msgs::msg::LegCommand>(
                "leg" + leg_name + "/command", 10));
        }

        _timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&StrideMakerNode::_publish_leg_command, this));
    }

    virtual std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
};