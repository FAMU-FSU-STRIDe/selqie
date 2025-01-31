#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "stride_maker/stride_maker.hpp"

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

class SwimNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_name_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_cmd_pubs;
    rclcpp::TimerBase::SharedPtr _timer;

    std::string _gait_name = "swim";
    double _robot_width = 1.0;
    double _frequency = 1.0;
    int _stride_resolution = 100;
    double _leg_length = 0.18;
    double _z_amplitude = 0.005;
    double _vel_amplitude_gain = 0.1;

    std::string _current_gait;
    std::size_t _idx = std::numeric_limits<std::size_t>::max();
    std::vector<robot_msgs::msg::LegTrajectory> _trajectories, _next_trajectories;
    rclcpp::Time _start_time;

    void updateGait(const std_msgs::msg::String::SharedPtr msg)
    {
        _current_gait = msg->data;
        _trajectories.clear();
        _next_trajectories.clear();
    }

    void updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (_current_gait != _gait_name)
        {
            return;
        }

        const double vel_x = msg->linear.x;
        const double vel_z = msg->linear.z;
        const double omega_z = msg->angular.z;

        if (vel_x == 0.0 && vel_z == 0.0 && omega_z == 0.0)
        {
            _next_trajectories.clear();
            return;
        }

        const double vel_x_left = vel_x - 0.5 * _robot_width * omega_z;
        const double vel_x_right = vel_x + 0.5 * _robot_width * omega_z;

        const double mag_left = std::sqrt(vel_x_left * vel_x_left + vel_z * vel_z);
        const double mag_right = std::sqrt(vel_x_right * vel_x_right + vel_z * vel_z);

        const double x_amp_left = _vel_amplitude_gain * mag_left;
        const double x_amp_right = _vel_amplitude_gain * mag_right;

        const double phi_left = 2.0 * std::atan2(vel_z - mag_left, vel_x_left);
        const double phi_right = 2.0 * std::atan2(vel_z - mag_right, vel_x_right);

        const auto traj_FL = make_swim_stride(_stride_resolution, _frequency, _leg_length, phi_left, x_amp_left, _z_amplitude);
        const auto traj_RL = make_swim_stride(_stride_resolution, _frequency, _leg_length, phi_left, x_amp_left, _z_amplitude);
        const auto traj_RR = make_swim_stride(_stride_resolution, _frequency, _leg_length, phi_right, x_amp_right, _z_amplitude);
        const auto traj_FR = make_swim_stride(_stride_resolution, _frequency, _leg_length, phi_right, x_amp_right, _z_amplitude);

        _next_trajectories = {traj_FL, traj_RL, traj_RR, traj_FR};
    }

    void publishLegCommand()
    {
        if (_trajectories.empty())
        {
            _trajectories = _next_trajectories;
            return;
        }

        assert(_trajectories.size() == _leg_cmd_pubs.size());

        if (_idx >= _trajectories[0].timing.size())
        {
            _trajectories = _next_trajectories;
            _idx = 0;
            _start_time = this->now();
            return;
        }

        const auto cdiff = (this->now() - _start_time).to_chrono<std::chrono::milliseconds>();
        const auto delay = std::chrono::milliseconds(time_t(_trajectories[0].timing[_idx] * 1E3));
        if (delay <= cdiff)
        {
            for (size_t i = 0; i < _leg_cmd_pubs.size(); i++)
            {
                assert(_idx < _trajectories[i].commands.size());
                _leg_cmd_pubs[i]->publish(_trajectories[i].commands[_idx]);
            }
            _idx++;
        }
    }

public:
    SwimNode() : Node("swim_node")
    {
        this->declare_parameter("gait_name", _gait_name);
        this->get_parameter("gait_name", _gait_name);

        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        this->declare_parameter("leg_names", leg_names);
        this->get_parameter("leg_names", leg_names);

        this->declare_parameter("robot_width", _robot_width);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("frequency", _frequency);
        this->get_parameter("frequency", _frequency);

        this->declare_parameter("stride_resolution", _stride_resolution);
        this->get_parameter("stride_resolution", _stride_resolution);

        this->declare_parameter("leg_length", _leg_length);
        this->get_parameter("leg_length", _leg_length);

        this->declare_parameter("z_amplitude", _z_amplitude);
        this->get_parameter("z_amplitude", _z_amplitude);

        this->declare_parameter("vel_amplitude_gain", _vel_amplitude_gain);
        this->get_parameter("vel_amplitude_gain", _vel_amplitude_gain);

        _gait_name_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", qos_reliable(), std::bind(&SwimNode::updateGait, this, std::placeholders::_1));

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_reliable(), std::bind(&SwimNode::updateCmdVel, this, std::placeholders::_1));

        for (const auto &leg_name : leg_names)
        {
            _leg_cmd_pubs.push_back(this->create_publisher<robot_msgs::msg::LegCommand>(
                "leg" + leg_name + "/command", qos_fast()));
        }

        _timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&SwimNode::publishLegCommand, this));

        RCLCPP_INFO(this->get_logger(), "Swim Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwimNode>());
    rclcpp::shutdown();
    return 0;
}