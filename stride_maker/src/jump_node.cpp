#include "stride_maker/stride_maker_node.hpp"

class JumpNode : public StrideMakerNode
{
private:
    double _robot_mass;
    double _z_crouch;
    double _z_jump;
    double _time_crouch;
    double _time_jump;

    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            const auto traj = make_default_stride(_default_height);
            return {traj, traj, traj, traj};
        }

        const double v_x = msg->linear.x;
        const double v_z = msg->linear.z;
        const double v_mag = std::sqrt(v_x * v_x + v_z * v_z);
        const double kin_energy = 0.5 * _robot_mass * v_mag * v_mag;
        const double delta_z = _z_crouch - _z_jump;
        const double thrust = kin_energy / delta_z;
        const double thrust_x = thrust * v_x / v_mag;
        const double thrust_z = thrust * v_z / v_mag;

        const auto traj = make_jump_stride(_stride_resolution, thrust_x, thrust_z, _z_crouch, _time_crouch, _time_jump);

        return {traj, traj, traj, traj};
    }

public:
    JumpNode() : StrideMakerNode("jump")
    {
        this->declare_parameter("robot_mass", 1.0);
        this->get_parameter("robot_mass", _robot_mass);

        this->declare_parameter("z_crouch", 0.105);
        this->get_parameter("z_crouch", _z_crouch);

        this->declare_parameter("z_jump", 0.195);
        this->get_parameter("z_jump", _z_jump);

        this->declare_parameter("time_crouch", 3.0);
        this->get_parameter("time_crouch", _time_crouch);

        this->declare_parameter("time_jump", 0.5);
        this->get_parameter("time_jump", _time_jump);

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