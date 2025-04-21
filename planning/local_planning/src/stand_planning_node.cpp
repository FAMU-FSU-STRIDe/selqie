#include "local_planning/local_planning_node.hpp"

class StandPlanningModel : public LocalPlanningModel
{
public:
    StandPlanningModel() = default;

    std::string get_model_name() const override
    {
        return "stand";
    }

    geometry_msgs::msg::Twist solve(const geometry_msgs::msg::Pose &start_pose,
                                    const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Should never be called since the goal is always reached
        return geometry_msgs::msg::Twist();
    }

    bool is_goal_reached(const geometry_msgs::msg::Pose &start_pose,
                         const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Always return true for the stand model
        // Will forward the gait transition message to the gait publisher
        return true;
    }
};

class StandPlanningNode : public rclcpp::Node
{
private:
    std::unique_ptr<StandPlanningModel> _model;              // Pointer to the stand planning model
    std::unique_ptr<LocalPlanningNode> _local_planning_node; // Pointer to the local planning node
public:
    StandPlanningNode()
        : Node("stand_planning_node")
    {
        // Create the stand planning model
        _model = std::make_unique<StandPlanningModel>();

        // Initialize the local planning node
        _local_planning_node = std::make_unique<LocalPlanningNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandPlanningNode>());
    rclcpp::shutdown();
    return 0;
}