#include "stride_generation/stride_generation_node.hpp"

/*
 * Sink Stride Parameters
 * This struct contains the parameters for the sink stride model
 */
struct SinkStrideParameters
{
    double leg_command_rate = 50.0; // Rate at which leg commands are sent
    double stride_frequency = 1.0;  // Frequency of the sink stride
    double standing_height = 0.175;  // Default height of the robot
    double sink_speed = 0.1;        // Sinking speed
    double variance_vz = 0.1;       // Variance of the sink speed
};

/*
 * Sink Stride Model
 * This class implements the sink stride generation model
 */
class SinkStrideModel : public StrideGenerationModel
{
private:
    const rclcpp::Node *_node;          // Pointer to the ROS node
    const SinkStrideParameters _params; // Parameters for the sink stride model

    int _trajectory_size; // Size of the trajectory

public:
    SinkStrideModel(const rclcpp::Node *node, const SinkStrideParameters &params)
        : _node(node), _params(params)
    {
        _trajectory_size = _params.leg_command_rate / _params.stride_frequency;
    }

    /*
     * Get the name of the stride generation model gait
     */
    std::string get_model_name() const override
    {
        return "sink";
    }

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    void update_velocity(const geometry_msgs::msg::Twist &) override
    {
        // Do nothing as the sink stride does not depend on the velocity command
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
    leg_control_msgs::msg::LegCommand get_leg_command(const int, const int) const override
    {
        // Create and return the leg command message
        leg_control_msgs::msg::LegCommand leg_command;
        leg_command.control_mode = leg_control_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
        leg_command.pos_setpoint.z = -_params.standing_height;
        return leg_command;
    }

    /*
     * Get the odometry estimate at index i along the stride trajectory
     */
    geometry_msgs::msg::TwistWithCovarianceStamped get_vel_estimate(const int) const override
    {
        // Create and return the odometry estimate message
        geometry_msgs::msg::TwistWithCovarianceStamped vel_estimate;
        vel_estimate.twist.twist.linear.z = -_params.sink_speed;
        vel_estimate.twist.covariance[14] = _params.variance_vz;
        return vel_estimate;
    }
};

/*
 * Sink Stride Node
 * This class implements the sink stride node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class SinkStrideNode : public rclcpp::Node
{
private:
    std::unique_ptr<SinkStrideModel> _model;                       // Pointer to the sink stride model
    std::unique_ptr<StrideGenerationNode> _stride_generation_node; // Pointer to the stride generation node
public:
    SinkStrideNode()
        : Node("sink_stride_node")
    {
        // Get the parameters for the sink stride model
        SinkStrideParameters params;

        this->declare_parameter("leg_command_rate", params.leg_command_rate);
        this->get_parameter("leg_command_rate", params.leg_command_rate);

        this->declare_parameter("stride_frequency", params.stride_frequency);
        this->get_parameter("stride_frequency", params.stride_frequency);

        this->declare_parameter("standing_height", params.standing_height);
        this->get_parameter("standing_height", params.standing_height);

        this->declare_parameter("sink_speed", params.sink_speed);
        this->get_parameter("sink_speed", params.sink_speed);

        this->declare_parameter("variance_vz", params.variance_vz);
        this->get_parameter("variance_vz", params.variance_vz);

        // Create the sink stride model
        _model = std::make_unique<SinkStrideModel>(this, params);

        // Initialize the stride generation node
        _stride_generation_node = std::make_unique<StrideGenerationNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinkStrideNode>());
    rclcpp::shutdown();
    return 0;
}
