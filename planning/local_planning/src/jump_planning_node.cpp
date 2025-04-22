#include "local_planning/local_planning_node.hpp"

class JumpPlanningModel : public LocalPlanningModel
{
private:
    rclcpp::Node *_node; // Pointer to the ROS node

    double _goal_threshold = 0.15; // Goal threshold

public:
    JumpPlanningModel(rclcpp::Node *node)
        : _node(node)
    {
        _node->declare_parameter("goal_threshold", _goal_threshold);
        _node->get_parameter("goal_threshold", _goal_threshold);
    }

    std::string get_model_name() const override
    {
        return "jump";
    }

    geometry_msgs::msg::Twist solve(const nav_msgs::msg::Odometry &current_odom,
                                    const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Get the current state of the robot
        const double state_x = current_odom.pose.pose.position.x;
        const double state_z = current_odom.pose.pose.position.z;

        // Get the goal pose
        const double goal_x = goal_pose.position.x;
        const double goal_z = goal_pose.position.z;

        // Get error to goal
        const double dx = goal_x - state_x;
        const double dz = goal_z - state_z;
        const double distance = std::sqrt(dx * dx + dz * dz);

        // Return proportional velocity command (normalized)
        geometry_msgs::msg::Twist twist;
        twist.linear.x = dx / distance;
        twist.linear.z = dz / distance;
        return twist;
    }

    bool is_goal_reached(const nav_msgs::msg::Odometry &current_odom,
                         const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Get the current state of the robot
        const double state_x = current_odom.pose.pose.position.x;
        const double state_z = current_odom.pose.pose.position.z;

        // Get the goal pose
        const double goal_x = goal_pose.position.x;
        const double goal_z = goal_pose.position.z;

        // Get error to goal
        const double dx = goal_x - state_x;
        const double dz = goal_z - state_z;
        const double distance = std::sqrt(dx * dx + dz * dz);

        // At goal if within threshold
        return distance < _goal_threshold;
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
        _model = std::make_unique<JumpPlanningModel>(this);

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