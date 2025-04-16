#include "stride_generation/stride_generation_node.hpp"

/*
 * Stand Stride Parameters
 * This struct contains the parameters for the stand stride model
 */
struct StandStrideParameters
{
    double leg_command_rate = 250.0; // Rate at which leg commands are sent
};

/*
 * Stand Stride Model
 * This class implements the stand stride generation model
 */
class StandStrideModel : public StrideGenerationModel
{
private:
    const rclcpp::Node *_node;          // Pointer to the ROS node
    const StandStrideParameters _params; // Parameters for the stand stride model

public:
    StandStrideModel(const rclcpp::Node *node, const StandStrideParameters &params)
        : _node(node), _params(params)
    {
    }

    /*
     * Get the name of the stride generation model gait
     */
    std::string get_model_name() const override
    {
        return "stand";
    }

    /*
     * Update the velocity of the stride generation model
     * Called when a new velocity command is received
     */
    void update_velocity(const geometry_msgs::msg::Twist &velocity) override
    {
    }

    /*
     * Get the number of points in the stride trajectory
     */
    std::size_t get_trajectory_size() const override
    {
    }

    /*
     * Get the time of execution for the leg command at index i along the stride trajectory
     */
    double get_execution_time(const int i) const override
    {
    }

    /*
     * Get the leg command at index i along the stride trajectory
     */
    leg_control_msgs::msg::LegCommand get_leg_command(const int leg, const int i) const override
    {
    }

    /*
     * Get the odometry estimate at index i along the stride trajectory
     */
    geometry_msgs::msg::TwistWithCovarianceStamped get_odometry_estimate(const int) const override
    {
    }
};

/*
 * Stand Stride Node
 * This class implements the stand stride node that generates leg commands and odometry estimates
 * based on the velocity command received from the user
 */
class StandStrideNode : public rclcpp::Node
{
private:
    std::unique_ptr<StandStrideModel> _model;                       // Pointer to the stand stride model
    std::unique_ptr<StrideGenerationNode> _stride_generation_node; // Pointer to the stride generation node
public:
    StandStrideNode()
        : Node("stand_stride_node")
    {
        // Get the parameters for the stand stride model
        StandStrideParameters params;

        this->declare_parameter("leg_command_rate", params.leg_command_rate);
        this->get_parameter("leg_command_rate", params.leg_command_rate);

        // Create the stand stride model
        _model = std::make_unique<StandStrideModel>(this, params);

        // Initialize the stride generation node
        _stride_generation_node = std::make_unique<StrideGenerationNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandStrideNode>());
    rclcpp::shutdown();
    return 0;
}
