#include "stride_maker/stride_maker_node.hpp"

class SwimNode : public StrideMakerNode
{
private:
    double _robot_width = 1.0;
    double _leg_length = 0.18;
    double _z_amplitude = 0.005;
    double _vel_amplitude_gain = 0.1;

    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            const auto traj = make_default_stride(_default_height);
            return {traj, traj, traj, traj};
        }

        const double vel_x = msg->linear.x;
        const double vel_z = msg->linear.z;

        const double mag = std::sqrt(vel_x * vel_x + vel_z * vel_z);
        const double x_amp = _vel_amplitude_gain * mag;
        const double phi = 2.0 * std::atan2(vel_z - mag, vel_x);

        const auto traj = make_swim_stride(_stride_resolution, _frequency, _leg_length, phi, x_amp, _z_amplitude);
        return {traj, traj, traj, traj};
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