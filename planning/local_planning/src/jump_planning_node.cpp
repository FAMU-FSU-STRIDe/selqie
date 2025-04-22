#include "local_planning/local_planning_node.hpp"

class JumpPlanningModel : public LocalPlanningModel
{
public:
    JumpPlanningModel() = default;

    std::string get_model_name() const override
    {
        return "jump";
    }

    geometry_msgs::msg::Twist solve(const nav_msgs::msg::Odometry &,
                                    const geometry_msgs::msg::Pose &) override
    {
        /// TODO: UPDATE THIS
        return geometry_msgs::msg::Twist();
    }

    bool is_goal_reached(const nav_msgs::msg::Odometry &,
                         const geometry_msgs::msg::Pose &) override
    {
        /// TODO: UPDATE THIS
        return true;
    }
};

class JumpPlanningNode : public rclcpp::Node
{
private:
    std::unique_ptr<JumpPlanningModel> _model;               // Pointer to the planning model
    std::unique_ptr<LocalPlanningNode> _local_planning_node; // Pointer to the local planning node
public:
    JumpPlanningNode()
        : Node("jump_planning_node")
    {
        // Create the planning model
        _model = std::make_unique<JumpPlanningModel>();

        // Initialize the local planning node
        _local_planning_node = std::make_unique<LocalPlanningNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JumpPlanningNode>());
    rclcpp::shutdown();
    return 0;
}