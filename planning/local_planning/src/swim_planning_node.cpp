#include "local_planning/local_planning_node.hpp"

/*
 * Wrap angle to the range [-pi, pi]
 */
float wrap_angle(float angle)
{
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

/*
 * Convert quaternion to pitch angle
 */
float quaternion_to_pitch(const geometry_msgs::msg::Quaternion &q)
{
    const double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    const double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    return 2 * std::atan2(sinp, cosp) - M_PI / 2;
}

class SwimPlanningModel : public LocalPlanningModel
{
private:
    rclcpp::Node *_node;                 // Pointer to ROS node
    double _max_linear_velocity = 0.25; // Maximum linear velocity
    double _max_angular_velocity = 0.1; // Maximum angular velocity
    double _approach_distance = 0.25;   // Distance to start slowing down
    double _approach_angle = 0.1;       // Angle to start slowing down
    double _goal_threshold = 0.15;      // Threshold to consider the goal reached

public:
    SwimPlanningModel(rclcpp::Node *node)
        : _node(node)
    {
        _node->declare_parameter("max_linear_velocity", _max_linear_velocity);
        _node->get_parameter("max_linear_velocity", _max_linear_velocity);

        _node->declare_parameter("max_angular_velocity", _max_angular_velocity);
        _node->get_parameter("max_angular_velocity", _max_angular_velocity);

        _node->declare_parameter("approach_distance", _approach_distance);
        _node->get_parameter("approach_distance", _approach_distance);

        _node->declare_parameter("approach_angle", _approach_angle);
        _node->get_parameter("approach_angle", _approach_angle);

        _node->declare_parameter("goal_threshold", _goal_threshold);
        _node->get_parameter("goal_threshold", _goal_threshold);
    }

    std::string get_model_name() const override
    {
        return "swim";
    }

    geometry_msgs::msg::Twist solve(const nav_msgs::msg::Odometry &current_odom,
                                    const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Get the current state of the robot
        const double state_x = current_odom.pose.pose.position.x;
        const double state_z = current_odom.pose.pose.position.z;
        const double state_pitch = quaternion_to_pitch(current_odom.pose.pose.orientation);

        // Get the goal pose
        const double goal_x = goal_pose.position.x;
        const double goal_z = goal_pose.position.z;
        const double goal_pitch = quaternion_to_pitch(goal_pose.orientation);

        // Get error to goal
        const double dx = goal_x - state_x;
        const double dz = goal_z - state_z;
        const double dpitch = wrap_angle(goal_pitch - state_pitch);

        // Apply proportional control based on error
        const double vel_x = -_max_linear_velocity * dx / _approach_distance;
        const double vel_z = -_max_linear_velocity * dz / _approach_distance;
        const double omega_y = -_max_angular_velocity * dpitch / _approach_angle;

        // Saturate control to be within velocity bounds
        const double cmd_vx = std::clamp(vel_x, -_max_linear_velocity, _max_linear_velocity);
        const double cmd_vz = std::clamp(vel_z, -_max_linear_velocity, _max_linear_velocity);
        const double cmd_wy = std::clamp(omega_y, -_max_angular_velocity, _max_angular_velocity);

        // Return commanded velocity
        geometry_msgs::msg::Twist twist;
        twist.linear.x = cmd_vx;
        twist.linear.z = cmd_vz;
        twist.angular.y = cmd_wy;
        return twist;
    }

    bool is_goal_reached(const nav_msgs::msg::Odometry &current_odom,
                         const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Get the current state of the robot
        const double state_x = current_odom.pose.pose.position.x;
        const double state_z = current_odom.pose.pose.position.z;
        const double state_pitch = quaternion_to_pitch(current_odom.pose.pose.orientation);

        // Get the goal pose
        const double goal_x = goal_pose.position.x;
        const double goal_z = goal_pose.position.z;
        const double goal_pitch = quaternion_to_pitch(goal_pose.orientation);

        // Get error to goal
        const double dx = goal_x - state_x;
        const double dz = goal_z - state_z;
        const double dpitch = wrap_angle(goal_pitch - state_pitch);

        // Get magnitude of error
        const double distance = std::sqrt(dx * dx + dz * dz);
        const double angle = std::abs(dpitch);

        // At goal if error magnitude is less than threshold
        return distance + angle < _goal_threshold;
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
        _model = std::make_unique<SwimPlanningModel>(this);

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