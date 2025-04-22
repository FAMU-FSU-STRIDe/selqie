#include "local_planning/local_planning_node.hpp"

class SinkPlanningModel : public LocalPlanningModel
{
private:
    rclcpp::Node *_node; // Pointer to the ROS node

    double _goal_threshold = 0.01; // Threshold for goal reached in m/s

public:
    SinkPlanningModel(rclcpp::Node *node)
        : _node(node)
    {
        _node->declare_parameter("goal_threshold", _goal_threshold);
        _node->get_parameter("goal_threshold", _goal_threshold);
    }

    std::string get_model_name() const override
    {
        return "sink";
    }

    geometry_msgs::msg::Twist solve(const nav_msgs::msg::Odometry &,
                                    const geometry_msgs::msg::Pose &) override
    {
        // No velocity command for the sink gait
        // Assumes the robot is negatively bouyant
        return geometry_msgs::msg::Twist();
    }

    bool is_goal_reached(const nav_msgs::msg::Odometry &current_odom,
                         const geometry_msgs::msg::Pose &) override
    {
        // Get velocity of the robot
        const double vx = current_odom.twist.twist.linear.x;
        const double vy = current_odom.twist.twist.linear.y;
        const double vz = current_odom.twist.twist.linear.z;
        const double velocity = std::sqrt(vx * vx + vy * vy + vz * vz);

        // At goal when the robot is on the floor
        // Determined when the velocity of the robot is about zero
        return velocity < _goal_threshold;
    }
};

class SinkPlanningNode : public rclcpp::Node
{
private:
    std::unique_ptr<SinkPlanningModel> _model;               // Pointer to the planning model
    std::unique_ptr<LocalPlanningNode> _local_planning_node; // Pointer to the local planning node
public:
    SinkPlanningNode()
        : Node("sink_planning_node")
    {
        // Create the planning model
        _model = std::make_unique<SinkPlanningModel>(this);

        // Initialize the local planning node
        _local_planning_node = std::make_unique<LocalPlanningNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SinkPlanningNode>());
    rclcpp::shutdown();
    return 0;
}