#include "stride_maker/stride_maker_node.hpp"

// mapping obtained from walk stride sweep experiment
void map_des2cmd(const double des_v, const double des_w, double &cmd_v, double &cmd_w)
{
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

class WalkNode : public StrideMakerNode
{
private:
    double _robot_width = 1.0;
    double _body_height = 0.2;
    double _center_shift = 0.0;
    double _step_height = 0.05;
    double _duty_factor = 0.5;
    double _max_stance_length = 0.15;

    void _make_walk_stride(const std::vector<double> stance_lengths, const std::vector<double> &offsets)
    {
        assert(stance_lengths.size() == 4);
        assert(offsets.size() == 4);

        const int points_per_half = _stride_resolution / 2;
        const double duration = 1.0 / _frequency;
        const double stance_duration = duration * _duty_factor;
        const double stance_dt = stance_duration / points_per_half;
        const double swing_duration = duration * (1.0 - _duty_factor);
        const double swing_dt = swing_duration / points_per_half;

        _timing.reserve(_stride_resolution);
        _leg_commands = {{}, {}, {}, {}};
        for (int leg_idx = 0; leg_idx < 4; leg_idx++)
        {
            _leg_commands[leg_idx].reserve(_stride_resolution);
            const int offset_index = offsets[leg_idx] * _stride_resolution;
            const double stance_length = stance_lengths[leg_idx];
            const double touchdown_x = 0.5 * stance_length * (1.0 + _center_shift);
            const double takeoff_x = touchdown_x - stance_length;
            const double x0 = 0.5 * (takeoff_x + touchdown_x);

            double t = 0.0;
            robot_msgs::msg::LegCommand leg_command;
            leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
            for (int k = 0; k < _stride_resolution; k++)
            {
                if (leg_idx == 0)
                    _timing.push_back(t);

                const int i = (k + offset_index) % _stride_resolution;
                const int p = i % points_per_half;
                const int q = i / points_per_half;
                const double f = static_cast<double>(p) / points_per_half;
                if (q == 0)
                {
                    t += stance_dt;
                    leg_command.pos_setpoint.x = touchdown_x - f * stance_length;
                    leg_command.pos_setpoint.z = -_body_height;
                    _leg_commands[leg_idx].push_back(leg_command);
                }
                else
                {
                    t += swing_dt;
                    leg_command.pos_setpoint.x = x0 - 0.5 * stance_length * std::cos(M_PI * f);
                    leg_command.pos_setpoint.z = -_body_height + _step_height * std::sin(M_PI * f);
                    _leg_commands[leg_idx].push_back(leg_command);
                }
            }
        }
    }

    void update_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->angular.z == 0.0)
        {
            _make_default_stride();
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
        _frequency = std::max(std::abs(vel_left), std::abs(vel_right)) / _max_stance_length * _duty_factor;

        double stance_length_left, stance_length_right;
        if (std::abs(vel_left) > std::abs(vel_right))
        {
            stance_length_left = vel_left > 0 ? _max_stance_length : -_max_stance_length;
            stance_length_right = vel_right * _duty_factor / _frequency;
        }
        else
        {
            stance_length_right = vel_right > 0 ? _max_stance_length : -_max_stance_length;
            stance_length_left = vel_left * _duty_factor / _frequency;
        }

        _make_walk_stride(
            {stance_length_left, stance_length_left, stance_length_right, stance_length_right},
            {0.25, 0.75, 0.25, 0.75});
    }

public:
    WalkNode()
        : StrideMakerNode("walk")
    {
        this->declare_parameter("robot_width", _robot_width);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("body_height", _body_height);
        this->get_parameter("body_height", _body_height);

        this->declare_parameter("center_shift", _center_shift);
        this->get_parameter("center_shift", _center_shift);

        this->declare_parameter("step_height", _step_height);
        this->get_parameter("step_height", _step_height);

        this->declare_parameter("duty_factor", _duty_factor);
        this->get_parameter("duty_factor", _duty_factor);

        this->declare_parameter("max_stance_length", _max_stance_length);
        this->get_parameter("max_stance_length", _max_stance_length);

        RCLCPP_INFO(this->get_logger(), "Walk Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkNode>());
    rclcpp::shutdown();
    return 0;
}