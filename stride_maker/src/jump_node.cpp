#include "stride_maker/stride_maker_node.hpp"

class JumpNode : public StrideMakerNode
{
private:
    double _z_crouch;
    double _z_jump;
    double _time_crouch;
    double _time_hold;

    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->linear.z == 0.0)
        {
            const auto traj = make_default_stride(_default_height);
            return {traj, traj, traj, traj};
        }

        const double v_x = msg->linear.x;
        const double v_z = msg->linear.z;
        const double mag_v = std::sqrt(v_x * v_x + v_z * v_z);

        const double x0 = 0.0;
        const double z0 = _z_crouch;
        const double x1 = x0 + (_z_jump - x0) * v_x / mag_v;
        const double z1 = z0 + (_z_jump - z0) * v_z / mag_v;

        const auto traj = make_jump_stride(x0, z0, x1, z1, _time_crouch, _time_hold);

        return {traj, traj, traj, traj};
    }

public:
    JumpNode() : StrideMakerNode("jump")
    {
        this->declare_parameter("z_crouch", 0.105);
        this->get_parameter("z_crouch", _z_crouch);

        this->declare_parameter("z_jump", 0.195);
        this->get_parameter("z_jump", _z_jump);

        this->declare_parameter("time_crouch", 3.0);
        this->get_parameter("time_crouch", _time_crouch);

        this->declare_parameter("time_hold", 0.5);
        this->get_parameter("time_hold", _time_hold);

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