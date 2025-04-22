#include "local_planning/local_planning_node.hpp"

class SwimPlanningModel : public LocalPlanningModel
{
public:
    SwimPlanningModel() = default;

    std::string get_model_name() const override
    {
        return "swim";
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

class SwimPlanningNode : public rclcpp::Node
{
private:
    std::unique_ptr<SwimPlanningModel> _model;               // Pointer to the planning model
    std::unique_ptr<LocalPlanningNode> _local_planning_node; // Pointer to the local planning node
public:
    SwimPlanningNode()
        : Node("swim_planning_node")
    {
        // Create the planning model
        _model = std::make_unique<SwimPlanningModel>();

        // Initialize the local planning node
        _local_planning_node = std::make_unique<LocalPlanningNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwimPlanningNode>());
    rclcpp::shutdown();
    return 0;
}