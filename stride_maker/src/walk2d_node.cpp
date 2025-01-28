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

class Walk2DNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_name_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_cmd_pubs;
    rclcpp::TimerBase::SharedPtr _timer;

    std::string _gait_name = "walk";
    std::vector<double> _hip_positions;
    double _robot_width = 1.0;
    double _body_height = 0.2;
    double _center_shift = 0.0;
    int _stride_resolution = 100;
    double _step_height = 0.05;
    double _duty_factor = 0.5;
    double _max_stance_length = 0.15;

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

    void map_des2cmd(const double des_v, const double des_w, double &cmd_v, double &cmd_w)
    {
        // mapping obtained from walk stride sweep experiment
        const double a = 64 * des_v * des_v - 192 * des_v * des_w - 80 * des_v + 144 * des_w * des_w - 120 * des_w + 25;
        if (a < 0)
        {
            cmd_v = std::numeric_limits<double>::quiet_NaN();
            cmd_w = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        const double sqrta = std::sqrt(a);
        cmd_v = 0.5 * des_v - 0.75 * des_w - 0.0625 * sqrta + 0.3125;
        cmd_w = 4.0 * des_w + (-8.0 * des_v - sqrta + 5.0) / 3.0;
    }

    void updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (_current_gait != _gait_name)
        {
            return;
        }

        if (msg->linear.x == 0.0 && msg->angular.z == 0.0)
        {
            _next_trajectories.clear();
            return;
        }

        double vel_x;
        double omega_z;
        map_des2cmd(msg->linear.x, msg->angular.z, vel_x, omega_z);

        if (std::isnan(vel_x) || std::isnan(omega_z))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid velocity command.");
            return;
        }

        const double vel_left = vel_x - 0.5 * _robot_width * omega_z;
        const double vel_right = vel_x + 0.5 * _robot_width * omega_z;
        const double frequency = std::max(std::abs(vel_left), std::abs(vel_right)) / _max_stance_length * _duty_factor;

        double stance_length_left, stance_length_right;
        if (std::abs(vel_left) > std::abs(vel_right))
        {
            stance_length_left = vel_left > 0 ? _max_stance_length : -_max_stance_length;
            stance_length_right = vel_right * _duty_factor / frequency;
        }
        else
        {
            stance_length_right = vel_right > 0 ? _max_stance_length : -_max_stance_length;
            stance_length_left = vel_left * _duty_factor / frequency;
        }

        const auto traj_FL = make_walk_stride(_stride_resolution, frequency, _duty_factor,
                                              _center_shift, stance_length_left, _body_height,
                                              _step_height, 0.25);

        const auto traj_RL = make_walk_stride(_stride_resolution, frequency, _duty_factor,
                                              _center_shift, stance_length_left, _body_height,
                                              _step_height, 0.75);

        const auto traj_RR = make_walk_stride(_stride_resolution, frequency, _duty_factor,
                                              _center_shift, stance_length_right, _body_height,
                                              _step_height, 0.25);

        const auto traj_FR = make_walk_stride(_stride_resolution, frequency, _duty_factor,
                                              _center_shift, stance_length_right, _body_height,
                                              _step_height, 0.75);

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
    Walk2DNode() : Node("walk2d_node")
    {
        this->declare_parameter("gait_name", _gait_name);
        this->get_parameter("gait_name", _gait_name);

        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        this->declare_parameter("leg_names", leg_names);
        this->get_parameter("leg_names", leg_names);

        this->declare_parameter("hip_positions", _hip_positions);
        this->get_parameter("hip_positions", _hip_positions);
        assert(_hip_positions.size() == 2 * leg_names.size());

        this->declare_parameter("robot_width", _robot_width);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("body_height", _body_height);
        this->get_parameter("body_height", _body_height);

        this->declare_parameter("center_shift", _center_shift);
        this->get_parameter("center_shift", _center_shift);

        this->declare_parameter("stride_resolution", _stride_resolution);
        this->get_parameter("stride_resolution", _stride_resolution);

        this->declare_parameter("step_height", _step_height);
        this->get_parameter("step_height", _step_height);

        this->declare_parameter("duty_factor", _duty_factor);
        this->get_parameter("duty_factor", _duty_factor);

        this->declare_parameter("max_stance_length", _max_stance_length);
        this->get_parameter("max_stance_length", _max_stance_length);

        _gait_name_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", qos_reliable(), std::bind(&Walk2DNode::updateGait, this, std::placeholders::_1));

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_reliable(), std::bind(&Walk2DNode::updateCmdVel, this, std::placeholders::_1));

        for (const auto &leg_name : leg_names)
        {
            _leg_cmd_pubs.push_back(this->create_publisher<robot_msgs::msg::LegCommand>(
                "leg" + leg_name + "/command", qos_fast()));
        }

        _timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Walk2DNode::publishLegCommand, this));

        RCLCPP_INFO(this->get_logger(), "Walk2D Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walk2DNode>());
    rclcpp::shutdown();
    return 0;
}