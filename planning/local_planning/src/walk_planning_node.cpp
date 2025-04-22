#include "local_planning/local_planning_node.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>

// Headers for SBMPO API
#include "sbmpo/types/Model.hpp"
#include "sbmpo/SBMPO.hpp"
#include "sbmpo/tools/PrintTool.hpp"
#include "sbmpo/tools/CSVTool.hpp"
using namespace sbmpo;

/*
 * Wrap angle to the range [-pi, pi]
 */
float wrap_angle(float angle)
{
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

/*
 * Convert quaternion to yaw angle
 */
float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/*
 * Convert yaw angle to quaternion
 */
geometry_msgs::msg::Quaternion yaw_to_quaternion(float yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.z = std::sin(yaw / 2.0);
    return q;
}

/*
 * Convert walking planner model state to ROS pose
 */
geometry_msgs::msg::Pose state_to_pose(const sbmpo::State &state)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = state[WalkingPlannerModel::X];
    pose.position.y = state[WalkingPlannerModel::Y];
    pose.orientation = yaw_to_quaternion(state[WalkingPlannerModel::THETA]);
    return pose;
}

/*
 * Convert SBMPO exit code to string
 */
std::string exit_code_to_string(const sbmpo::ExitCode exit_code)
{
    switch (exit_code)
    {
    case sbmpo::SOLUTION_FOUND:
        return "SOLUTION_FOUND";
    case sbmpo::ITERATION_LIMIT:
        return "ITERATION_LIMIT";
    case sbmpo::NO_NODES_IN_QUEUE:
        return "NO_NODES_IN_QUEUE";
    case sbmpo::GENERATION_LIMIT:
        return "GENERATION_LIMIT";
    case sbmpo::RUNNING:
        return "RUNNING";
    case sbmpo::QUIT_SEARCH:
        return "QUIT_SEARCH";
    case sbmpo::TIME_LIMIT:
        return "TIME_LIMIT";
    case sbmpo::INVALID_START_STATE:
        return "INVALID_START_STATE";
    case sbmpo::NEGATIVE_COST:
        return "NEGATIVE_COST";
    default:
        return "UNKNOWN";
    }
}

/*
 * Walking planner parameters
 */
struct WalkingPlannerParams
{
    float horizon_time = 0.5;
    int integration_steps = 5;
    float goal_threshold = 0.25;
    float heuristic_vel_factor = 2.0;
    float heuristic_omega_factor = 1.0;
    float reverse_cost_factor = 2.0;
};

/*
 * Walking planner model
 * This model defines the dynamics of the system, the cost function, and the heuristic function
 */
class WalkingPlannerModel : public sbmpo::Model
{
private:
    WalkingPlannerParams params; // Parameters for the walking planner

public:
    // States of the Model (Enumerated)
    enum States
    {
        X,
        Y,
        THETA
    };

    // Controls of the Model (Enumerated)
    enum Controls
    {
        VEL,
        OMEGA
    };

    WalkingPlannerModel(const WalkingPlannerParams &params) : params(params) {}

    /*
     * Dynamics of the system
     */
    State next_state(const State &state, const Control &control) override
    {
        // Start at current state
        State next_state = state;

        // Euler integration
        const float time_increment = params.horizon_time / static_cast<double>(params.integration_steps);
        for (int i = 0; i < params.integration_steps; i++)
        {
            // Unicycle steering model control
            next_state[THETA] += control[OMEGA] * time_increment;
            next_state[X] += control[VEL] * std::cos(next_state[THETA]) * time_increment;
            next_state[Y] += control[VEL] * std::sin(next_state[THETA]) * time_increment;

            // Verify the state is valid over its path
            if (!is_valid(next_state))
                return state;
        }

        // Keep angle within [-pi, pi]
        next_state[THETA] = wrap_angle(next_state[THETA]);

        // Return the next state
        return next_state;
    }

    /*
     * Cost of a state and control
     */
    float cost(const State &, const State &, const Control &control) override
    {
        // Additional cost for reverse velocity
        const float reverse_cost_factor = control[VEL] < 0 ? params.reverse_cost_factor : 1.0;

        // Time-based cost function
        return params.horizon_time * reverse_cost_factor;
    }

    /*
     * Heuristic of a state with respect to the goal
     */
    float heuristic(const State &state, const State &goal) override
    {
        // Get distance to goal
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dtheta = wrap_angle(goal[THETA] - state[THETA]);

        // Compute heuristic cost based on distance to goal
        // Heuristic cost is a combination of linear and angular velocity
        const float heur_vel = std::sqrt(dx * dx + dy * dy) * params.heuristic_vel_factor;
        const float heur_omega = std::abs(dtheta) * params.heuristic_omega_factor;

        // Return the heuristic cost
        return heur_vel + heur_omega;
    }

    /*
     * Is this state close enough to the goal to end the plan?
     */
    bool is_goal(const State &state, const State &goal) override
    {
        // Use the heuristic function to determine if the state is close enough to the goal
        return heuristic(state, goal) < params.goal_threshold;
    }

    /*
     * Model constraints
     */
    bool is_valid(const State &) override
    {
        // No constraints on the state
        // If obstacles are added, they would be checked here
        return true;
    }

    /*
     * Get control samples based on the current state (Optional)
     */
    std::vector<Control> get_dynamic_samples(const State &) override
    {
        // Not enabled for this model
        return {};
    }
};

/*
 * Walking planning model
 * This model wraps the walking planner model and provides an interface for local planning
 */
class WalkPlanningModel : public LocalPlanningModel
{
private:
    rclcpp::Node *_node; // Pointer to the ROS node

    WalkingPlannerParams _model_params;          // Parameters for the walking planner
    std::shared_ptr<WalkingPlannerModel> _model; // Pointer to the walking planner model

    SearchParameters _sbmpo_params; // Parameters for SBMPO
    std::unique_ptr<SBMPO> _sbmpo;  // Pointer to the SBMPO planner

    bool _publish_all = false;                                                   // Flag to publish all states
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;                 // Publisher for the path
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _pose_array_pub; // Publisher for the pose array

    /*
     * Publish the resulting path from the SBMPO run
     */
    void _publish_path(const sbmpo::SearchResults &results, const double z)
    {
        // Create the path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = _node->now();
        path_msg.header.frame_id = "odom";

        for (const auto &state : results.state_path)
        {
            // Convert all path states to poses
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = state_to_pose(state);
            pose_stamped.pose.position.z = z;

            // Add the pose to the path
            path_msg.poses.push_back(pose_stamped);
        }

        // Publish the path message
        _path_pub->publish(path_msg);
    }

    /*
     * Publish all samples in the SBMPO run
     * Only gets called if publish_all is set to true
     */
    void _publish_states(const sbmpo::SearchResults &results, const double z)
    {
        // Create the pose array message
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = _node->now();
        pose_array_msg.header.frame_id = "odom";

        for (const auto &node : results.nodes)
        {
            // Convert all nodes to poses
            geometry_msgs::msg::Pose pose = state_to_pose(node->state);
            pose.position.z = z;

            // Add the pose to the pose array
            pose_array_msg.poses.push_back(pose);
        }

        // Publish the pose array message
        _pose_array_pub->publish(pose_array_msg);
    }

public:
    WalkPlanningModel(rclcpp::Node *node)
        : _node(node)
    {
        // Get ROS parameters
        _node->declare_parameter("publish_all", false);
        _node->get_parameter("publish_all", _publish_all);

        _node->declare_parameter("horizon_time", 0.5);
        _node->get_parameter("horizon_time", _model_params.horizon_time);

        _node->declare_parameter("integration_steps", 5);
        _node->get_parameter("integration_steps", _model_params.integration_steps);

        _node->declare_parameter("goal_threshold", 0.25);
        _node->get_parameter("goal_threshold", _model_params.goal_threshold);

        _node->declare_parameter("heuristic_vel_factor", 2.0);
        _node->get_parameter("heuristic_vel_factor", _model_params.heuristic_vel_factor);

        _node->declare_parameter("heuristic_omega_factor", 1.0);
        _node->get_parameter("heuristic_omega_factor", _model_params.heuristic_omega_factor);

        _node->declare_parameter("reverse_cost_factor", 2.0);
        _node->get_parameter("reverse_cost_factor", _model_params.reverse_cost_factor);

        _node->declare_parameter("max_iterations", 500000);
        _node->get_parameter("max_iterations", _sbmpo_params.max_iterations);

        _node->declare_parameter("max_generations", 1000);
        _node->get_parameter("max_generations", _sbmpo_params.max_generations);

        _node->declare_parameter("time_limit_us", 1000000);
        _node->get_parameter("time_limit_us", _sbmpo_params.time_limit_us);

        std::vector<double> grid_resolution;
        _node->declare_parameter("grid_resolution", std::vector<double>{0.05, 0.05, 0.15});
        _node->get_parameter("grid_resolution", grid_resolution);
        assert(grid_resolution.size() == 3);
        _sbmpo_params.grid_resolution = std::vector<float>(grid_resolution.begin(), grid_resolution.end());

        // Fixed samples are hard coded at the moment (I'm lazy)
        _sbmpo_params.fixed_samples = {
            {-0.25, +0.00}, {+0.25, +0.00}, {-0.15, -0.05}, {-0.15, +0.05}, {+0.15, -0.05}, {+0.15, +0.05}, {-0.10, -0.10}, {-0.10, +0.10}, {+0.10, -0.10}, {+0.10, +0.10}, {-0.05, -0.15}, {-0.05, +0.15}, {+0.05, -0.15}, {+0.05, +0.15}, {+0.00, -0.25}, {+0.00, +0.25}};

        // Create the model and planner
        _model = std::make_shared<WalkingPlannerModel>(_model_params);
        _sbmpo = std::make_unique<SBMPO>(_model);

        // Create publishers for path and pose array (if enabled)
        _path_pub = _node->create_publisher<nav_msgs::msg::Path>("walk_planner/path", 10);
        if (_publish_all)
        {
            _pose_array_pub = _node->create_publisher<geometry_msgs::msg::PoseArray>("walk_planner/states", 10);
        }
    }

    std::string get_model_name() const override
    {
        return "walk";
    }

    geometry_msgs::msg::Twist solve(const nav_msgs::msg::Odometry &current_odom,
                                    const geometry_msgs::msg::Pose &) override
    {
        // Current state and goal should be updated in the params from the is_goal_reached function
        // In the implementation, that functions is always called before this one

        // Run the SBMPO planner
        _sbmpo->run(_sbmpo_params);

        // Get the exit code
        const sbmpo::ExitCode exit_code = _sbmpo->results()->exit_code;
        if (exit_code != sbmpo::SOLUTION_FOUND)
        {
            // If no solution is found, warn the user and return zero velocity
            RCLCPP_WARN(_node->get_logger(), "Walking Planner Failed with Exit Code %d: %s",
                        exit_code, exit_code_to_string(exit_code).c_str());
            return geometry_msgs::msg::Twist();
        }
        else if (_sbmpo->results()->control_path.empty())
        {
            // If the path is empty, return zero velocity
            // This happens if the start state is at the goal, however the is_goal_reached function should catch
            // this beforehand. So, this code should never run.
            return geometry_msgs::msg::Twist();
        }
        else
        {
            // If a solution is found, return the first control in the results path
            geometry_msgs::msg::Twist twist;
            twist.linear.x = _sbmpo->results()->control_path[0][WalkingPlannerModel::VEL];
            twist.angular.z = _sbmpo->results()->control_path[0][WalkingPlannerModel::OMEGA];
            return twist;
        }

        // Publish the path and pose array to the ROS network (visualization purposes)
        const float state_z = current_odom.pose.pose.position.z;
        _publish_path(*_sbmpo->results(), state_z);
        if (_publish_all)
        {
            _publish_states(*_sbmpo->results(), state_z);
        }
    }

    bool is_goal_reached(const nav_msgs::msg::Odometry &current_odom,
                         const geometry_msgs::msg::Pose &goal_pose) override
    {
        // Get the current state of the robot
        const float state_x = current_odom.pose.pose.position.x;
        const float state_y = current_odom.pose.pose.position.y;
        const float state_z = current_odom.pose.pose.position.z;
        const float state_theta = quaternion_to_yaw(current_odom.pose.pose.orientation);

        // Get the goal state
        const float goal_x = goal_pose.position.x;
        const float goal_y = goal_pose.position.y;
        const float goal_theta = quaternion_to_yaw(goal_pose.orientation);

        // Update the parameters for the planner
        _sbmpo_params.start_state = {state_x, state_y, state_theta};
        _sbmpo_params.goal_state = {goal_x, goal_y, goal_theta};

        // At goal based on the model is_goal function
        return _model->is_goal(_sbmpo_params.start_state, _sbmpo_params.goal_state);
    }
};

class WalkPlanningNode : public rclcpp::Node
{
private:
    std::unique_ptr<WalkPlanningModel> _model;               // Pointer to the planning model
    std::unique_ptr<LocalPlanningNode> _local_planning_node; // Pointer to the local planning node
public:
    WalkPlanningNode()
        : Node("walk_planning_node")
    {
        // Create the planning model
        _model = std::make_unique<WalkPlanningModel>();

        // Initialize the local planning node
        _local_planning_node = std::make_unique<LocalPlanningNode>(this, _model.get());
    }
};

// Entry point for the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkPlanningNode>());
    rclcpp::shutdown();
    return 0;
}