#include "stride_generation/stride_generation_node.hpp"

/*
 * Jump Stride Parameters
 * This struct contains the parameters for the jump stride model
 */
struct JumpStrideParameters
{
    double leg_command_rate = 50.0;  // Rate at which leg commands are sent
    double default_height = 0.175;   // Default height of the robot
    double z_crouch = -0.105;        // Crouch z position
    double z_jump = -0.195;          // Jump z position
    double time_crouch = 3.0;        // Time to crouch
    double time_hold = 0.5;          // Time to hold the jump position
    double initial_jump_speed = 1.0; // Estimate of the initial jump speed
    double robot_mass = 10.0;        // Estimate of the robot body mass
    double gravity = 9.81;           // Gravity
    double drag_coeff = 1.0;         // Estimate of the body drag coefficient
    double variance_vx, variance_vz; // Variance of the velocity in x and z directions
};

/*
 * Jump Stride Model
 * This class implements the jump stride generation model
 */
class JumpStrideModel : public StrideGenerationModel
{
private:
    const rclcpp::Node *_node;          // Pointer to the ROS node
    const JumpStrideParameters _params; // Parameters for the jump stride model

    int _trajectory_size;
    double _nx, _nz;

public:
    JumpStrideModel(const rclcpp::Node *node, const JumpStrideParameters &params)
        : _node(node), _params(params)
    {
        _trajectory_size = 0;
    }

    /*
     * Get the name of the stride generation model gait
     */
    std::string get_model_name() const override
    {
        return "jump";
    }

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    void update_velocity(const geometry_msgs::msg::Twist &velocity) override
    {
        // Get the velocity components and magnitude
        const double vx = velocity.linear.x;
        const double vz = velocity.linear.z;
        const double v = std::sqrt(vx * vx + vz * vz);

        if (v == 0.0)
        {
            // If the magnitude is zero, don't publish any commands
            _trajectory_size = 0;

            // Give feedback to user
            RCLCPP_WARN(_node->get_logger(), "Invalid jump velocity command: (%f, %f)", velocity.linear.x, velocity.angular.z);
        }
        else
        {
            _trajectory_size = _params.leg_command_rate * (_params.time_crouch + _params.time_hold);
        }

        _nx = vx / v;
        _nz = vz / v;
    }

    /*
     * Get the number of points in the stride trajectory
     */
    std::size_t get_trajectory_size() const override
    {
        return _trajectory_size;
    }

    /*
     * Get the time of execution for the leg command at index i along the stride trajectory
     */
    double get_execution_time(const int i) const override
    {
        return i * _params.leg_command_rate;
    }

    /*
     * Get the leg command at index i along the stride trajectory
     */
    leg_control_msgs::msg::LegCommand get_leg_command(const int leg, const int i) const override
    {
        // Get the current execution time
        const double t = get_execution_time(i);

        double x, z;
        // Get the current phase of the stride trajectory
        if (t < _params.time_crouch)
        {
            // Crouch phase
            x = 0.0;
            z = -_params.default_height + (_params.z_crouch + _params.default_height) * (t / _params.time_crouch);
        }
        else
        {
            // Jump & hold phase
            x = _params.z_jump * _nx;
            z = _params.z_jump * _nz;
        }

        // Create and return the leg command message
        leg_control_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = leg_control_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
        leg_command.pos_setpoint.x = x;
        leg_command.pos_setpoint.z = z;
        return leg_command;
    }

    /*
     * Get the odometry estimate at index i along the stride trajectory
     */
    geometry_msgs::msg::TwistWithCovarianceStamped get_odometry_estimate(const int) const override
    {
    }
};

/*
 * Jump Stride Node
 * This class implements the jump stride node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class JumpStrideNode : public rclcpp::Node
{
private:
    std::unique_ptr<JumpStrideModel> _model;                       // Pointer to the jump stride model
    std::unique_ptr<StrideGenerationNode> _stride_generation_node; // Pointer to the stride generation node
public:
    JumpStrideNode()
        : Node("jump_stride_node")
    {
        // Get the parameters for the jump stride model
        JumpStrideParameters params;

        this->declare_parameter("leg_command_rate", params.leg_command_rate);
        this->get_parameter("leg_command_rate", params.leg_command_rate);

        this->declare_parameter("default_height", params.default_height);
        this->get_parameter("default_height", params.default_height);

        this->declare_parameter("z_crouch", params.z_crouch);
        this->get_parameter("z_crouch", params.z_crouch);

        this->declare_parameter("z_jump", params.z_jump);
        this->get_parameter("z_jump", params.z_jump);

        this->declare_parameter("time_crouch", params.time_crouch);
        this->get_parameter("time_crouch", params.time_crouch);

        this->declare_parameter("time_hold", params.time_hold);
        this->get_parameter("time_hold", params.time_hold);

        this->declare_parameter("initial_jump_speed", params.initial_jump_speed);
        this->get_parameter("initial_jump_speed", params.initial_jump_speed);

        this->declare_parameter("robot_mass", params.robot_mass);
        this->get_parameter("robot_mass", params.robot_mass);

        this->declare_parameter("gravity", params.gravity);
        this->get_parameter("gravity", params.gravity);

        this->declare_parameter("drag_coeff", params.drag_coeff);
        this->get_parameter("drag_coeff", params.drag_coeff);

        // Create the jump stride model
        _model = std::make_unique<JumpStrideModel>(this, params);

        // Initialize the stride generation node
        _stride_generation_node = std::make_unique<StrideGenerationNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JumpStrideNode>());
    rclcpp::shutdown();
    return 0;
}
