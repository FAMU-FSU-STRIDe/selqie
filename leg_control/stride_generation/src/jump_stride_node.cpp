#include "stride_generation/stride_generation_node.hpp"

/*
 * Jump Stride Parameters
 * This struct contains the parameters for the jump stride model
 */
struct JumpStrideParameters
{
    double leg_command_rate = 50.0;              // Rate at which leg commands are sent
    double standing_height = 0.175;               // Default height of the robot
    double z_crouch = -0.095;                    // Crouch z position
    double d_jump = -0.210;                      // Jump extension distance from origin
    double time_crouch = 3.0;                    // Time to crouch
    double time_hold = 1.5;                      // Time to hold the jump position
    double initial_jump_speed = 1.0;             // Estimate of the initial jump speed
    double robot_mass = 10.0;                    // Estimate of the robot body mass
    double gravity = 1.15;                       // Gravity
    double drag_coeff = 1.0;                     // Estimate of the body drag coefficient
    double variance_vx = 0.1, variance_vz = 0.1; // Variance of the velocity in x and z directions
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

    int _trajectory_size;                // Size of the trajectory
    double _vx_direction, _vz_direction; // Direction of the velocity in x and z directions

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

        _vx_direction = vx / v;
        _vz_direction = vz / v;
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
        return i / _params.leg_command_rate;
    }

    /*
     * Get the leg command at index i along the stride trajectory
     */
    leg_control_msgs::msg::LegCommand get_leg_command(const int, const int i) const override
    {
        // Get the current execution time
        const double t = get_execution_time(i);

        double x, z;
        // Get the current phase of the stride trajectory
        if (t < _params.time_crouch)
        {
            // Crouch phase
            x = 0.0;
            z = -_params.standing_height + (_params.z_crouch + _params.standing_height) * (t / _params.time_crouch);
        }
        else
        {
            // Jump & hold phase
            x = _params.d_jump * _vx_direction;
            z = _params.d_jump * _vz_direction;
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
    geometry_msgs::msg::TwistWithCovarianceStamped get_vel_estimate(const int i) const override
    {
        // Get the current execution time
        const double t = get_execution_time(i);

        double vx, vz;
        // Get the current phase of the stride trajectory
        if (t < _params.time_crouch)
        {
            // Crouch phase
            vx = 0.0;
            vz = (_params.z_crouch + _params.standing_height) / _params.time_crouch;
        }
        else
        {
            // Jump & hold phase

            // Jump model: (drag and gravity)
            // dvx/dt = -b/m * vx       --> vx(t) = vx0 * exp(-b/m * t)
            // dvz/dt = -b/m * vy - g   --> vz(t) = -mg/b + (vz0 + mg/b) * exp(-b/m * t)

            // Calculate the velocity components based on the jump model
            const double vx0 = _params.initial_jump_speed * _vx_direction;
            const double vz0 = _params.initial_jump_speed * _vz_direction;
            const double beta = -_params.drag_coeff / _params.robot_mass; // -b/m
            const double gamma = _params.gravity / beta;                  // -mg/b

            // Calculate the velocity components at time t after the jump
            const double t_jump = t - _params.time_crouch;
            vx = vx0 * std::exp(beta * t_jump);
            vz = gamma + (vz0 - gamma) * std::exp(beta * t_jump);
        }

        // Create and return the odometry estimate message
        geometry_msgs::msg::TwistWithCovarianceStamped vel_estimate;
        vel_estimate.twist.twist.linear.x = vx;
        vel_estimate.twist.twist.angular.z = vz;
        vel_estimate.twist.covariance[0] = _params.variance_vx;
        vel_estimate.twist.covariance[14] = _params.variance_vz;
        return vel_estimate;
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

        this->declare_parameter("standing_height", params.standing_height);
        this->get_parameter("standing_height", params.standing_height);

        this->declare_parameter("z_crouch", params.z_crouch);
        this->get_parameter("z_crouch", params.z_crouch);

        this->declare_parameter("d_jump", params.d_jump);
        this->get_parameter("d_jump", params.d_jump);

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

        this->declare_parameter("variance_vx", params.variance_vx);
        this->get_parameter("variance_vx", params.variance_vx);

        this->declare_parameter("variance_vz", params.variance_vz);
        this->get_parameter("variance_vz", params.variance_vz);

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
