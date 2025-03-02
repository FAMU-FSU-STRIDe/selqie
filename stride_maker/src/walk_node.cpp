#include "stride_maker/stride_maker_node.hpp"

class WalkNode : public StrideMakerNode
{
private:
    double _leg_command_rate;
    double _robot_width;
    double _center_shift;
    double _step_height;
    double _duty_factor;
    double _max_stance_length;
    double _min_velocity;

    double _vel_x_correction_factor;
    double _omega_z_correction_factor;

    double _frequency;

    double _map_A, _map_B, _map_C, _map_D;
    // mapping obtained from walk stride sweep experiment
    void _map_des2cmd(const double des_v, const double des_w, double &cmd_v, double &cmd_w)
    {
        const double abs_v = std::abs(des_v);
        const double abs_w = std::abs(des_w);
        const double sign_v = des_v > 0 ? 1.0 : -1.0;
        const double sign_w = des_w > 0 ? 1.0 : -1.0;

        const double b = _map_A * _map_C + _map_B * abs_w + _map_D * abs_v;
        const double a_v = _map_A * _map_D * sign_v;
        const double a_w = _map_B * _map_C * sign_w;
        const double c_v = _map_B * des_v * abs_w / _map_A;
        // const double c_w = _map_D * des_w * abs_v / _map_C;

        const double root = b * b - 4 * a_v * c_v; // same as b * b - 4 * a_w * c_w (can be proved)
        if (root < 0)
        {
            cmd_v = std::numeric_limits<double>::quiet_NaN();
            cmd_w = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        cmd_v = (-b - std::sqrt(root)) / (2*a_v);
        cmd_w = (-b - std::sqrt(root)) / (2*a_w);
    }

    void _make_walk_stride(const std::vector<double> stance_lengths, const std::vector<double> &offsets)
    {
        assert(stance_lengths.size() == 4);
        assert(offsets.size() == 4);

        const int num_points = int(0.5 * _leg_command_rate / _frequency) * 2;
        const int points_per_half = num_points / 2;
        const double duration = 1.0 / _frequency;
        const double stance_duration = duration * _duty_factor;
        const double stance_dt = stance_duration / points_per_half;
        const double swing_duration = duration * (1.0 - _duty_factor);
        const double swing_dt = swing_duration / points_per_half;

        _timing.reserve(num_points);
        _leg_commands = {{}, {}, {}, {}};
        for (int leg_idx = 0; leg_idx < 4; leg_idx++)
        {
            _leg_commands[leg_idx].reserve(num_points);
            const int offset_index = offsets[leg_idx] * num_points;
            const double stance_length = stance_lengths[leg_idx];
            const double touchdown_x = 0.5 * stance_length * (1.0 + _center_shift);
            const double takeoff_x = touchdown_x - stance_length;
            const double x0 = 0.5 * (takeoff_x + touchdown_x);

            double t = 0.0;
            robot_msgs::msg::LegCommand leg_command;
            leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
            for (int k = 0; k < num_points; k++)
            {
                if (leg_idx == 0)
                    _timing.push_back(t);

                const int i = (k + offset_index) % num_points;
                const int p = i % points_per_half;
                const int q = i / points_per_half;
                const double f = static_cast<double>(p) / points_per_half;
                if (q == 0)
                {
                    t += stance_dt;
                    leg_command.pos_setpoint.x = touchdown_x - f * stance_length;
                    leg_command.pos_setpoint.z = -_default_height;
                    _leg_commands[leg_idx].push_back(leg_command);
                }
                else
                {
                    t += swing_dt;
                    leg_command.pos_setpoint.x = x0 - 0.5 * stance_length * std::cos(M_PI * f);
                    leg_command.pos_setpoint.z = -_default_height + _step_height * std::sin(M_PI * f);
                    _leg_commands[leg_idx].push_back(leg_command);
                }
            }
        }
    }

    void _make_walk_odometry(const double vel_x, const double omega_z)
    {
        for (size_t i = 0; i < _timing.size(); i++)
        {
            geometry_msgs::msg::TwistWithCovarianceStamped odometry;
            odometry.twist.twist.linear.x = vel_x * _vel_x_correction_factor;
            odometry.twist.twist.angular.z = omega_z * _omega_z_correction_factor;
            _gait_odometry.push_back(odometry);
        }
    }

    void update_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->angular.z == 0.0)
        {
            _make_default_stride();
            return;
        }

        double vel_x, omega_z;
        _map_des2cmd(msg->linear.x, msg->angular.z, vel_x, omega_z);

        if (std::isnan(vel_x) || std::isnan(omega_z))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid velocity command.");
            return;
        }

        const double vel_left = vel_x - 0.5 * _robot_width * omega_z;
        const double vel_right = vel_x + 0.5 * _robot_width * omega_z;

        const double vel = std::max(std::abs(vel_left), std::abs(vel_right));
        if (vel < _min_velocity)
        {
            _make_default_stride();
            return;
        }

        _frequency = vel / _max_stance_length * _duty_factor;

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

        _make_walk_odometry(msg->linear.x, msg->angular.z);
    }

public:
    WalkNode()
        : StrideMakerNode("walk")
    {
        this->declare_parameter("leg_command_rate", 250.0);
        this->get_parameter("leg_command_rate", _leg_command_rate);

        this->declare_parameter("robot_width", 0.25);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("center_shift", 0.0);
        this->get_parameter("center_shift", _center_shift);

        this->declare_parameter("step_height", 0.05);
        this->get_parameter("step_height", _step_height);

        this->declare_parameter("duty_factor", 0.5);
        this->get_parameter("duty_factor", _duty_factor);

        this->declare_parameter("max_stance_length", 0.15);
        this->get_parameter("max_stance_length", _max_stance_length);

        this->declare_parameter("min_velocity", 0.05);
        this->get_parameter("min_velocity", _min_velocity);

        this->declare_parameter("vel_x_correction_factor", 1.0);
        this->get_parameter("vel_x_correction_factor", _vel_x_correction_factor);

        this->declare_parameter("omega_z_correction_factor", 1.0);
        this->get_parameter("omega_z_correction_factor", _omega_z_correction_factor);

        this->declare_parameter("map_A", 0.0);
        this->get_parameter("map_A", _map_A);

        this->declare_parameter("map_B", 0.0);
        this->get_parameter("map_B", _map_B);

        this->declare_parameter("map_C", 0.0);
        this->get_parameter("map_C", _map_C);

        this->declare_parameter("map_D", 0.0);
        this->get_parameter("map_D", _map_D);

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