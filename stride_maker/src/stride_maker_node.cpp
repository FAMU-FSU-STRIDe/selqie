#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/srv/make_stride.hpp>

class StrideMakerNode : public rclcpp::Node
{
private:
    rclcpp::Service<robot_msgs::srv::MakeStride>::SharedPtr _make_stride_srv;

    void make_stride(const std::shared_ptr<robot_msgs::srv::MakeStride::Request> request,
                     std::shared_ptr<robot_msgs::srv::MakeStride::Response> response)
    {
        const double duration = 1.0 / request->frequency;
        const double stance_duration = duration * request->duty_factor;
        const double swing_duration = duration * (1.0 - request->duty_factor);
        const int points_per_quadrant = request->num_points / 4;
        const double phi0 = std::asin(0.5 * request->stance_length / request->leg_length);
        const double R0 = 0.5 * request->stance_length;
        const double R1 = request->step_height;
        const double body_height = std::sqrt(request->leg_length * request->leg_length - 0.25 * request->stance_length * request->stance_length);
        const double stance_dt = 0.5 * stance_duration / points_per_quadrant;
        const double swing_dt = 0.5 * swing_duration / points_per_quadrant;
        const int offset_index = request->offset * request->num_points;

        robot_msgs::msg::LegTrajectory &leg_trajectory = response->trajectory;
        leg_trajectory.timing.reserve(request->num_points);
        leg_trajectory.commands.reserve(request->num_points);

        robot_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;

        double t = 0.0;
        for (int k = 0; k < request->num_points; k++)
        {
            leg_trajectory.timing.push_back(t);

            const int p = (k + offset_index) % request->num_points;
            const int q = p / points_per_quadrant;
            const double i = p % points_per_quadrant;
            const double f = i / points_per_quadrant;
            switch (q)
            {
            case 0:
            {
                t += stance_dt;
                const double R = request->leg_length;
                const double theta = -(M_PI / 2.0) - phi0 * f;
                const double x = R * cos(theta);
                const double z = R * sin(theta);
                leg_command.pos_setpoint.x = x;
                leg_command.pos_setpoint.z = z;
                leg_trajectory.commands.push_back(leg_command);
                break;
            }
            case 1:
            {
                t += swing_dt;
                const double R = R0 + (R1 - R0) * f;
                const double theta = (M_PI / 2.0) * (2.0 - f);
                const double x = R * cos(theta);
                const double z = R * sin(theta) - body_height;
                leg_command.pos_setpoint.x = x;
                leg_command.pos_setpoint.z = z;
                leg_trajectory.commands.push_back(leg_command);
                break;
            }
            case 2:
            {
                t += swing_dt;
                const double R = R1 + (R0 - R1) * f;
                const double theta = (M_PI / 2.0) * (1.0 - f);
                const double x = R * cos(theta);
                const double z = R * sin(theta) - body_height;
                leg_command.pos_setpoint.x = x;
                leg_command.pos_setpoint.z = z;
                leg_trajectory.commands.push_back(leg_command);
                break;
            }
            case 3:
            {
                t += stance_dt;
                const double R = request->leg_length;
                const double theta = phi0 * (1.0 - f) - (M_PI / 2.0);
                const double x = R * cos(theta);
                const double z = R * sin(theta);
                leg_command.pos_setpoint.x = x;
                leg_command.pos_setpoint.z = z;
                leg_trajectory.commands.push_back(leg_command);
                break;
            }
            }
        }
    }

public:
    StrideMakerNode() : Node("stride_maker_node")
    {
        _make_stride_srv = this->create_service<robot_msgs::srv::MakeStride>(
            "make_stride", std::bind(&StrideMakerNode::make_stride, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Stride Maker Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StrideMakerNode>());
    rclcpp::shutdown();
    return 0;
}