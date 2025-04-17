#include "stride_generation/stride_generation_node.hpp"

/*
 * Walk Stride Parameters
 * This struct contains the parameters for the walk stride model
 */
struct WalkStrideParameters
{
    double leg_command_rate = 250.0;                              // Rate at which leg commands are sent
    double robot_width = 0.25;                                    // Width of the robot
    double walking_height = 0.175;                                // Default height of the robot
    double center_shift = -0.25;                                  // Center shift of the stride [-1,1]
    double step_height = 0.025;                                   // Height of the step
    double duty_factor = 0.5;                                     // Duty factor of the stride
    double max_stance_length = 0.175;                             // Maximum length of the stance phase
    double min_velocity = 0.025;                                  // Minimum velocity for the stride
    std::array<double, 4> leg_offsets = {0.25, 0.75, 0.25, 0.75}; // Offsets for each leg (FL, RL, RR, FR) [0, 1)
    double mapA = 0.0, mapB = 0.0, mapC = 0.0, mapD = 0.0;        // Parameters for the mapping function
    double variance_vx = 0.1, variance_wz = 0.1;                  // Covariance for the velocity and angular velocity
};

/*
 * Walk Stride Model
 * This class implements the walk stride generation model
 */
class WalkStrideModel : public StrideGenerationModel
{
private:
    const rclcpp::Node *_node;          // Pointer to the ROS node
    const WalkStrideParameters _params; // Parameters for the walk stride model

    int _trajectory_size = 0;          // Size of the trajectory
    double _duration = 0.0;            // Duration of the trajectory
    double _stance_length_right = 0.0; // Length of the stance phase for the right leg
    double _stance_length_left = 0.0;  // Length of the stance phase for the left leg
    double _linear_velocity = 0.0;     // Linear velocity of the robot
    double _angular_velocity = 0.0;    // Angular velocity of the robot

    /*
     * Mapping function
     * This function maps the desired velocity to the commanded velocity
     * Obtained from the walk stride sweep experiment
     */
    void _map_des2cmd(const double des_v, const double des_w, double &cmd_v, double &cmd_w)
    {
        const double abs_v = std::abs(des_v);
        const double abs_w = std::abs(des_w);
        const double sign_v = des_v > 0 ? 1.0 : -1.0;
        const double sign_w = des_w > 0 ? 1.0 : -1.0;
        const double A = _params.mapA, B = _params.mapB, C = _params.mapC, D = _params.mapD;

        const double b = A * C + B * abs_w + D * abs_v;
        const double a_v = A * D * sign_v;
        const double a_w = B * C * sign_w;
        const double c_v = B * des_v * abs_w / A;
        // const double c_w = D * des_w * abs_v / C;

        const double root = b * b - 4 * a_v * c_v; // same as b * b - 4 * a_w * c_w (can be proved)
        if (root < 0)
        {
            // Return NaN if the root is negative (out of domain)
            cmd_v = std::numeric_limits<double>::quiet_NaN();
            cmd_w = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        cmd_v = (-b - std::sqrt(root)) / (2 * a_v);
        cmd_w = (-b - std::sqrt(root)) / (2 * a_w);
    }

public:
    WalkStrideModel(const rclcpp::Node *node, const WalkStrideParameters &params)
        : _node(node), _params(params)
    {
    }

    /*
     * Get the name of the stride generation model gait
     */
    std::string get_model_name() const override
    {
        return "walk";
    }

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    void update_velocity(const geometry_msgs::msg::Twist &velocity) override
    {
        // Update the desired velocity of the robot
        _linear_velocity = velocity.linear.x;
        _angular_velocity = velocity.angular.z;

        // Map the desired velocity to the commanded velocity
        double vel_x, vel_w;
        _map_des2cmd(_linear_velocity, _angular_velocity, vel_x, vel_w);

        // Check if the commanded velocity is valid
        if (std::isnan(vel_x) || std::isnan(vel_w))
        {
            // If not, set the velocity to zero
            vel_x = 0.0;
            vel_w = 0.0;

            // Also update the desired velocity to zero
            _linear_velocity = 0.0;
            _angular_velocity = 0.0;

            // Give feedback to the user
            RCLCPP_WARN(_node->get_logger(), "Invalid walk velocity command: (%f, %f)", velocity.linear.x, velocity.angular.z);
        }

        // Calculate the velocities on the left and right sides of the robot
        double v_left = vel_x - 0.5 * _params.robot_width * vel_w;
        double v_right = vel_x + 0.5 * _params.robot_width * vel_w;
        double v = std::max(std::abs(v_left), std::abs(v_right));

        // Check if the velocity is below the minimum velocity
        if (v < _params.min_velocity)
        {
            // If so, update the left and right velocities to zero
            v_left = 0.0;
            v_right = 0.0;

            // Update the body velocity to the minimum velocity
            v = _params.min_velocity;

            // Also update the desired velocity to zero
            _linear_velocity = 0.0;
            _angular_velocity = 0.0;
        }

        // Calculate the duration of the trajectory based on the commanded velocity
        assert(v > 0); // Ensure that the velocity is positive and non-zero
        _duration = _params.max_stance_length * _params.duty_factor / v;

        // Calculate the length of the stance phase for the left and right legs
        if (std::abs(v_left) > std::abs(v_right))
        {
            _stance_length_left = v_left > 0 ? _params.max_stance_length : -_params.max_stance_length;
            _stance_length_right = v_right * _params.duty_factor * _duration;
        }
        else
        {
            _stance_length_right = v_right > 0 ? _params.max_stance_length : -_params.max_stance_length;
            _stance_length_left = v_left * _params.duty_factor * _duration;
        }

        // Calculate the size of the trajectory based on the duration and leg command rate
        _trajectory_size = static_cast<int>(0.5 * _params.leg_command_rate * _duration) * 2;
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
        // Calculate the time of execution for the leg command at index i
        return i * _duration / _trajectory_size;
    }

    /*
     * Get the leg command at index i along the stride trajectory
     */
    leg_control_msgs::msg::LegCommand get_leg_command(const int leg, const int i) const override
    {
        // Get the stance length for the leg
        const double stance_length = leg < 2 ? _stance_length_left : _stance_length_right;

        // Get the index along the stride trajectory for the leg
        const int index = i + _params.leg_offsets[leg] * _trajectory_size;

        // Calculate the size of a single phase of the stride trajectory
        const int phase_size = _trajectory_size / 2;

        double x, z;
        // Get the current phase of the stride trajectory
        if (index < phase_size)
        {
            // Stance phase

            // Get the stance start point
            const double touchdown = 0.5 * stance_length * (1.0 + _params.center_shift);

            // Get the progression of the leg in the stance phase
            const double f = static_cast<double>(index) / phase_size;

            // Get the position of the leg in the stance phase
            x = touchdown - stance_length * f;
            z = -_params.walking_height;
        }
        else
        {
            // Swing phase

            // Get the progression of the leg in the swing phase
            const double f = static_cast<double>(index - phase_size) / phase_size;

            // Get the center of the leg in the swing phase
            const double x0 = 0.5 * stance_length * _params.center_shift;
            const double z0 = _params.walking_height;

            // Get the position of the leg in the swing phase
            x = x0 - 0.5 * stance_length * std::cos(M_PI * f);
            z = z0 + _params.step_height * std::sin(M_PI * f);
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
    geometry_msgs::msg::TwistWithCovarianceStamped get_vel_estimate(const int) const override
    {
        // Create and return the odometry estimate message
        geometry_msgs::msg::TwistWithCovarianceStamped vel_estimate;
        vel_estimate.twist.twist.linear.x = _linear_velocity;
        vel_estimate.twist.twist.angular.z = _angular_velocity;
        vel_estimate.twist.covariance[0] = _params.variance_vx;
        vel_estimate.twist.covariance[35] = _params.variance_wz;
        return vel_estimate;
    }
};

/*
 * Walk Stride Node
 * This class implements the walk stride node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class WalkStrideNode : public rclcpp::Node
{
private:
    std::unique_ptr<WalkStrideModel> _model;                       // Pointer to the walk stride model
    std::unique_ptr<StrideGenerationNode> _stride_generation_node; // Pointer to the stride generation node
public:
    WalkStrideNode()
        : Node("walk_stride_node")
    {
        // Get the parameters for the walk stride model
        WalkStrideParameters params;

        this->declare_parameter("leg_command_rate", params.leg_command_rate);
        this->get_parameter("leg_command_rate", params.leg_command_rate);

        this->declare_parameter("robot_width", params.robot_width);
        this->get_parameter("robot_width", params.robot_width);

        this->declare_parameter("walking_height", params.walking_height);
        this->get_parameter("walking_height", params.walking_height);

        this->declare_parameter("center_shift", params.center_shift);
        this->get_parameter("center_shift", params.center_shift);

        this->declare_parameter("step_height", params.step_height);
        this->get_parameter("step_height", params.step_height);

        this->declare_parameter("duty_factor", params.duty_factor);
        this->get_parameter("duty_factor", params.duty_factor);

        this->declare_parameter("max_stance_length", params.max_stance_length);
        this->get_parameter("max_stance_length", params.max_stance_length);

        this->declare_parameter("min_velocity", params.min_velocity);
        this->get_parameter("min_velocity", params.min_velocity);

        std::vector<double> leg_offsets(params.leg_offsets.begin(), params.leg_offsets.end());
        this->declare_parameter("leg_offsets", leg_offsets);
        this->get_parameter("leg_offsets", leg_offsets);
        assert(leg_offsets.size() == 4);
        params.leg_offsets = {leg_offsets[0], leg_offsets[1], leg_offsets[2], leg_offsets[3]};

        this->declare_parameter("mapA", params.mapA);
        this->get_parameter("mapA", params.mapA);

        this->declare_parameter("mapB", params.mapB);
        this->get_parameter("mapB", params.mapB);

        this->declare_parameter("mapC", params.mapC);
        this->get_parameter("mapC", params.mapC);

        this->declare_parameter("mapD", params.mapD);
        this->get_parameter("mapD", params.mapD);

        this->declare_parameter("variance_vx", params.variance_vx);
        this->get_parameter("variance_vx", params.variance_vx);

        this->declare_parameter("variance_wz", params.variance_wz);
        this->get_parameter("variance_wz", params.variance_wz);

        // Create the walk stride model
        _model = std::make_unique<WalkStrideModel>(this, params);

        // Initialize the stride generation node
        _stride_generation_node = std::make_unique<StrideGenerationNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkStrideNode>());
    rclcpp::shutdown();
    return 0;
}
