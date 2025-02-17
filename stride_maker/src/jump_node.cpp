#include "stride_maker/stride_maker_node.hpp"

class JumpNode : public StrideMakerNode
{
private:
    double _leg_command_rate;
    double _z_crouch;
    double _z_jump;
    double _time_crouch;
    double _time_hold;

    double _initial_jump_speed;
    double _mass;
    double _gravity;
    double _drag_coeff;

    void _make_jump_stride(const double x0, const double z0, const double x1, const double z1)
    {
        const int num_points = _leg_command_rate * (_time_crouch + _time_hold);

        _timing.reserve(num_points);

        robot_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;

        _leg_commands = {{}, {}, {}, {}};
        for (int leg_idx = 0; leg_idx < 4; leg_idx++)
        {
            _leg_commands[leg_idx].reserve(num_points);

            const int half_points = num_points / 2;
            for (int k = 0; k < half_points; k++)
            {
                const double t = static_cast<double>(k) / half_points;

                if (leg_idx == 0)
                    _timing.push_back(t * _time_crouch);

                leg_command.pos_setpoint.x = x0;
                leg_command.pos_setpoint.z = -_default_height + (z0 + _default_height) * t;
                _leg_commands[leg_idx].push_back(leg_command);
            }

            for (int k = 1; k < half_points - 1; k++)
            {
                const double t = static_cast<double>(k) / half_points;

                if (leg_idx == 0)
                    _timing.push_back(_time_crouch + t * _time_hold);

                leg_command.pos_setpoint.x = x1;
                leg_command.pos_setpoint.z = z1;
                _leg_commands[leg_idx].push_back(leg_command);
            }

            if (leg_idx == 0)
                _timing.push_back(_time_crouch + _time_hold);

            leg_command.pos_setpoint.x = x0;
            leg_command.pos_setpoint.z = -_default_height;
            _leg_commands[leg_idx].push_back(leg_command);
        }
    }

    void _make_jump_odometry(const double n_x, const double n_z)
    {
        const int half_points = _timing.size() / 2;
        const double crouch_speed = (_z_crouch - _default_height) / _time_crouch;
        for (int i = 0; i < half_points; i++)
        {
            geometry_msgs::msg::TwistWithCovarianceStamped odometry;
            odometry.twist.twist.linear.z = crouch_speed;
            _gait_odometry.push_back(odometry);
        }

        const double vel_x0 = _initial_jump_speed * n_x;
        const double vel_z0 = _initial_jump_speed * n_z;
        const double decay_factor = -_drag_coeff / _mass;
        const double offset = _gravity / decay_factor;
        for (int i = half_points; i < int(_timing.size()); i++)
        {
            const double t = _timing[i] - _time_crouch;
            geometry_msgs::msg::TwistWithCovarianceStamped odometry;
            odometry.twist.twist.linear.x = vel_x0 * std::exp(-_drag_coeff * t);
            odometry.twist.twist.linear.z = offset + (vel_z0 - offset) * std::exp(decay_factor * t);
            _gait_odometry.push_back(odometry);
        }
    }

    void update_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            _make_default_stride();
            return;
        }

        const double v_x = msg->linear.x;
        const double v_z = msg->linear.z;
        const double mag_v = std::sqrt(v_x * v_x + v_z * v_z);
        const double n_x = v_x / mag_v;
        const double n_z = v_z / mag_v;

        const double x0 = 0.0;
        const double z0 = _z_crouch;
        const double x1 = _z_jump * n_x;
        const double z1 = _z_jump * n_z;

        _make_jump_stride(x0, z0, x1, z1);
        _make_jump_odometry(n_x, n_z);
    }

public:
    JumpNode() : StrideMakerNode("jump")
    {
        this->declare_parameter("leg_command_rate", 100.0);
        this->get_parameter("leg_command_rate", _leg_command_rate);

        this->declare_parameter("z_crouch", 0.105);
        this->get_parameter("z_crouch", _z_crouch);

        this->declare_parameter("z_jump", 0.195);
        this->get_parameter("z_jump", _z_jump);

        this->declare_parameter("time_crouch", 3.0);
        this->get_parameter("time_crouch", _time_crouch);

        this->declare_parameter("time_hold", 0.5);
        this->get_parameter("time_hold", _time_hold);

        this->declare_parameter("initial_jump_speed", 1.0);
        this->get_parameter("initial_jump_speed", _initial_jump_speed);

        this->declare_parameter("mass", 10.0);
        this->get_parameter("mass", _mass);

        this->declare_parameter("gravity", 9.81);
        this->get_parameter("gravity", _gravity);

        this->declare_parameter("drag_coeff", 1.0);
        this->get_parameter("drag_coeff", _drag_coeff);

        RCLCPP_INFO(this->get_logger(), "Jump Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JumpNode>());
    rclcpp::shutdown();
    return 0;
}