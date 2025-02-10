#include "stride_maker/stride_maker_node.hpp"

class SwimNode : public StrideMakerNode
{
private:
    double _robot_width = 1.0;
    double _leg_length = 0.18;
    double _z_amplitude = 0.005;
    double _vel_amplitude_gain = 0.1;

    void _make_swim_stride(const std::vector<double> phis, const std::vector<double> x_amplitudes)
    {
        assert(phis.size() == 4);
        assert(x_amplitudes.size() == 4);
        
        _timing.reserve(_stride_resolution);

        const double duration = 1.0 / _frequency;
        const double dt = duration / _stride_resolution;

        robot_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;

        _leg_commands = {{}, {}, {}, {}};
        for (int leg_idx = 0; leg_idx < 4; leg_idx++)
        {
            _leg_commands[leg_idx].reserve(_stride_resolution);
            for (int k = 0; k < _stride_resolution; k++)
            {
                const double t = k * dt;

                if (leg_idx == 0)
                    _timing.push_back(t);

                const double phi = phis[leg_idx];
                const double x = x_amplitudes[leg_idx] * std::cos(2 * M_PI * t * _frequency);
                const double z = _z_amplitude * std::sin(2 * M_PI * t * _frequency) - _leg_length;
                leg_command.pos_setpoint.x = x * std::cos(phi) - z * std::sin(phi);
                leg_command.pos_setpoint.z = x * std::sin(phi) + z * std::cos(phi);
                _leg_commands[leg_idx].push_back(leg_command);
            }
        }
    }

    void update_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            _make_default_stride();
            return;
        }

        const double vel_x = msg->linear.x;
        const double vel_z = msg->linear.z;

        const double mag = std::sqrt(vel_x * vel_x + vel_z * vel_z);
        const double x_amp = _vel_amplitude_gain * mag;
        const double phi = 2.0 * std::atan2(vel_z - mag, vel_x);

        _make_swim_stride({phi, phi, phi, phi}, {x_amp, x_amp, x_amp, x_amp});
    }

public:
    SwimNode() : StrideMakerNode("swim")
    {
        this->declare_parameter("robot_width", _robot_width);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("leg_length", _leg_length);
        this->get_parameter("leg_length", _leg_length);

        this->declare_parameter("z_amplitude", _z_amplitude);
        this->get_parameter("z_amplitude", _z_amplitude);

        this->declare_parameter("vel_amplitude_gain", _vel_amplitude_gain);
        this->get_parameter("vel_amplitude_gain", _vel_amplitude_gain);

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