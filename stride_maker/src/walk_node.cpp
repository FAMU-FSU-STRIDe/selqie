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

class Walk2DNode : public StrideMakerNode
{
private:
    double _robot_width = 1.0;
    double _body_height = 0.2;
    double _center_shift = 0.0;
    double _step_height = 0.05;
    double _duty_factor = 0.5;
    double _max_stance_length = 0.15;

    std::vector<robot_msgs::msg::LegTrajectory> get_stride(const geometry_msgs::msg::Twist::SharedPtr msg) override
    {
        if (msg->linear.x == 0.0 && msg->angular.z == 0.0)
        {
            const auto traj = make_default_stride(_default_height);
            return {traj, traj, traj, traj};
        }

        double vel_x;
        double omega_z;
        map_des2cmd(msg->linear.x, msg->angular.z, vel_x, omega_z);

        if (std::isnan(vel_x) || std::isnan(omega_z))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid velocity command.");
            return {};
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

        const auto traj_FL = make_walk_stride(_stride_resolution, _frequency, _duty_factor,
                                              _center_shift, stance_length_left, _body_height,
                                              _step_height, 0.25);

        const auto traj_RL = make_walk_stride(_stride_resolution, _frequency, _duty_factor,
                                              _center_shift, stance_length_left, _body_height,
                                              _step_height, 0.75);

        const auto traj_RR = make_walk_stride(_stride_resolution, _frequency, _duty_factor,
                                              _center_shift, stance_length_right, _body_height,
                                              _step_height, 0.25);

        const auto traj_FR = make_walk_stride(_stride_resolution, _frequency, _duty_factor,
                                              _center_shift, stance_length_right, _body_height,
                                              _step_height, 0.75);

        return {traj_FL, traj_RL, traj_RR, traj_FR};
    }

public:
    Walk2DNode()
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