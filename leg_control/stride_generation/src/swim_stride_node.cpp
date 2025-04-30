#include "stride_generation/stride_generation_node.hpp"

/*
 * Swim Stride Parameters
 * This struct contains the parameters for the swim stride model
 */
struct SwimStrideParameters
{
    double leg_command_rate = 250.0;             // Rate at which leg commands are sent
    double body_com_hip_radius = 0.27;           // Radius of the body center of mass to hip joint
    double swim_frequency = 3.5;                 // Frequency of the swim stride
    double nominal_leg_length = 0.18;            // Nominal leg length
    double swim_amplitude_z = 0.005;             // Amplitude of the swim stride in the z direction
    double vel_amplitude_gain = 5.0;             // Gain for the velocity amplitude
    double variance_vx = 0.1, variance_vz = 0.1; // Variance of the velocity in x and z directions
    double variance_wy = 0.1;                     // Variance of the angular velocity
};

/*
 * Swim Stride Model
 * This class implements the swim stride generation model
 */
class SwimStrideModel : public StrideGenerationModel
{
private:
    const rclcpp::Node *_node;          // Pointer to the ROS node
    const SwimStrideParameters _params; // Parameters for the swim stride model

    int _trajectory_size;                          // Size of the trajectory
    double _linear_velocity_x, _linear_velocity_z; // Linear velocity of the robot in x and z directions
    double _angular_velocity_y;                    // Angular velocity of the robot in y direction (pitch)

public:
    SwimStrideModel(const rclcpp::Node *node, const SwimStrideParameters &params)
        : _node(node), _params(params)
    {
        _trajectory_size = _params.leg_command_rate / _params.swim_frequency;
    }

    /*
     * Get the name of the stride generation model gait
     */
    std::string get_model_name() const override
    {
        return "swim";
    }

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    void update_velocity(const geometry_msgs::msg::Twist &velocity) override
    {
        _linear_velocity_x = velocity.linear.x;
        _linear_velocity_z = velocity.linear.z;
        _angular_velocity_y = velocity.angular.y;
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
    leg_control_msgs::msg::LegCommand get_leg_command(const int leg, const int i) const override
    {
        // Calculate the linear velocities based on leg position
        double vx, vz;
        if (leg == 0 || leg == 3)
        {
            // Front leg
            vx = _linear_velocity_x;
            vz = _linear_velocity_z + _angular_velocity_y * _params.body_com_hip_radius;
        }
        else
        {
            // Back leg
            vx = _linear_velocity_x;
            vz = _linear_velocity_z - _angular_velocity_y * _params.body_com_hip_radius;
        }

        // Calculate the amplitude and angle of the swim stride based on the linear velocity
        const double v = std::sqrt(vx * vx + vz * vz);
        const double amplitude_x = v / _params.vel_amplitude_gain;
        const double phi = -2 * std::atan2(vz - v, vx);

        // Calculate the leg position based on the index and stride trajectory
        const double phase = 2 * M_PI * i / _trajectory_size;
        const double xp = amplitude_x * std::cos(phase);
        const double zp = -_params.nominal_leg_length + _params.swim_amplitude_z * std::sin(phase);
        const double x = xp * std::cos(phi) + zp * std::sin(phi);
        const double z = -xp * std::sin(phi) + zp * std::cos(phi);

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
        vel_estimate.twist.twist.linear.x = _linear_velocity_x;
        vel_estimate.twist.twist.linear.z = _linear_velocity_z;
        vel_estimate.twist.twist.angular.y = _angular_velocity_y;
        vel_estimate.twist.covariance[0] = _params.variance_vx;
        vel_estimate.twist.covariance[14] = _params.variance_vz;
        vel_estimate.twist.covariance[28] = _params.variance_wy;
        return vel_estimate;
    }
};

/*
 * Swim Stride Node
 * This class implements the swim stride node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class SwimStrideNode : public rclcpp::Node
{
private:
    std::unique_ptr<SwimStrideModel> _model;                       // Pointer to the swim stride model
    std::unique_ptr<StrideGenerationNode> _stride_generation_node; // Pointer to the stride generation node
public:
    SwimStrideNode()
        : Node("swim_stride_node")
    {
        // Get the parameters for the swim stride model
        SwimStrideParameters params;

        this->declare_parameter("leg_command_rate", params.leg_command_rate);
        this->get_parameter("leg_command_rate", params.leg_command_rate);

        this->declare_parameter("body_com_hip_radius", params.body_com_hip_radius);
        this->get_parameter("body_com_hip_radius", params.body_com_hip_radius);

        this->declare_parameter("swim_frequency", params.swim_frequency);
        this->get_parameter("swim_frequency", params.swim_frequency);

        this->declare_parameter("nominal_leg_length", params.nominal_leg_length);
        this->get_parameter("nominal_leg_length", params.nominal_leg_length);

        this->declare_parameter("swim_amplitude_z", params.swim_amplitude_z);
        this->get_parameter("swim_amplitude_z", params.swim_amplitude_z);

        this->declare_parameter("vel_amplitude_gain", params.vel_amplitude_gain);
        this->get_parameter("vel_amplitude_gain", params.vel_amplitude_gain);

        this->declare_parameter("variance_vx", params.variance_vx);
        this->get_parameter("variance_vx", params.variance_vx);

        this->declare_parameter("variance_vz", params.variance_vz);
        this->get_parameter("variance_vz", params.variance_vz);

        this->declare_parameter("variance_wy", params.variance_wy);
        this->get_parameter("variance_wy", params.variance_wy);

        // Create the swim stride model
        _model = std::make_unique<SwimStrideModel>(this, params);

        // Initialize the stride generation node
        _stride_generation_node = std::make_unique<StrideGenerationNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwimStrideNode>());
    rclcpp::shutdown();
    return 0;
}
