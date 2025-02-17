#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "local_planning/walking_planner_model.hpp"
#include "sbmpo/SBMPO.hpp"
#include "sbmpo/tools/PrintTool.hpp"
#include "sbmpo/tools/CSVTool.hpp"

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(float yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.z = std::sin(yaw / 2.0);
    return q;
}

geometry_msgs::msg::Pose state_to_pose(const sbmpo::State &state)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = state[WalkingPlannerModel::X];
    pose.position.y = state[WalkingPlannerModel::Y];
    pose.orientation = yaw_to_quaternion(state[WalkingPlannerModel::THETA]);
    return pose;
}

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

class WalkingPlanner : public rclcpp::Node
{
private:
    std::string _gait_name = "walk";
    WalkingPlannerParams _model_params;
    std::shared_ptr<WalkingPlannerModel> _model;
    SearchParameters _sbmpo_params;
    std::unique_ptr<SBMPO> _sbmpo;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_transition_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _gait_pub;

    float _solve_frequency = 1.0;
    rclcpp::TimerBase::SharedPtr _solve_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

    bool _publish_all = false;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _pose_array_pub;

    std_msgs::msg::String::SharedPtr _gait_msg;
    std_msgs::msg::String::SharedPtr _gait_transition_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg;
    nav_msgs::msg::Odometry::SharedPtr _odom_msg;

    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _gait_msg = msg;
    }

    void _gait_transition_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _gait_transition_msg = msg;
    }

    void _goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        _goal_msg = msg->pose.orientation.w == 0.0 ? nullptr : msg;
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom_msg = msg;
    }

    void _publish_cmd(const float vel, const float omega)
    {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = vel;
        cmd_msg.angular.z = omega;
        _cmd_pub->publish(cmd_msg);
    }

    void _publish_transition_gait()
    {
        _gait_pub->publish(*_gait_transition_msg);
    }

    void _publish_path(const sbmpo::SearchResults &results, const double z)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        for (const auto &state : results.state_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = state_to_pose(state);
            pose_stamped.pose.position.z = z;
            path_msg.poses.push_back(pose_stamped);
        }
        _path_pub->publish(path_msg);
    }

    void _publish_states(const sbmpo::SearchResults &results, const double z)
    {
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = this->now();
        pose_array_msg.header.frame_id = "odom";
        for (const auto &node : results.nodes)
        {
            geometry_msgs::msg::Pose pose = state_to_pose(node->state);
            pose.position.z = z;
            pose_array_msg.poses.push_back(pose);
        }
        _pose_array_pub->publish(pose_array_msg);
    }

public:
    WalkingPlanner() : Node("walking_planner_node")
    {
        this->declare_parameter("solve_frequency", 1.0);
        this->get_parameter("solve_frequency", _solve_frequency);

        this->declare_parameter("publish_all", false);
        this->get_parameter("publish_all", _publish_all);

        this->declare_parameter("horizon_time", 0.5);
        this->get_parameter("horizon_time", _model_params.horizon_time);

        this->declare_parameter("integration_steps", 5);
        this->get_parameter("integration_steps", _model_params.integration_steps);

        this->declare_parameter("goal_threshold", 0.25);
        this->get_parameter("goal_threshold", _model_params.goal_threshold);

        this->declare_parameter("heuristic_vel_factor", 2.0);
        this->get_parameter("heuristic_vel_factor", _model_params.heuristic_vel_factor);

        this->declare_parameter("heuristic_omega_factor", 1.0);
        this->get_parameter("heuristic_omega_factor", _model_params.heuristic_omega_factor);

        this->declare_parameter("max_iterations", 500000);
        this->get_parameter("max_iterations", _sbmpo_params.max_iterations);

        this->declare_parameter("max_generations", 1000);
        this->get_parameter("max_generations", _sbmpo_params.max_generations);

        this->declare_parameter("time_limit_us", 1000000);
        this->get_parameter("time_limit_us", _sbmpo_params.time_limit_us);

        std::vector<double> grid_resolution;
        this->declare_parameter("grid_resolution", std::vector<double>{0.05, 0.05, 0.15});
        this->get_parameter("grid_resolution", grid_resolution);
        assert(grid_resolution.size() == 3);
        _sbmpo_params.grid_resolution = std::vector<float>(grid_resolution.begin(), grid_resolution.end());

        _sbmpo_params.fixed_samples = {
            {-0.25, +0.00}, {+0.25, +0.00}, {-0.10, -0.10}, {-0.10, +0.10}, {+0.10, -0.10}, {+0.10, +0.10}, {+0.00, -0.30}, {+0.00, +0.30}};

        _model = std::make_shared<WalkingPlannerModel>(_model_params);
        _sbmpo = std::make_unique<SBMPO>(_model);

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&WalkingPlanner::_gait_callback, this, std::placeholders::_1));

        _gait_transition_sub = this->create_subscription<std_msgs::msg::String>(
            "gait/transition", 10, std::bind(&WalkingPlanner::_gait_transition_callback, this, std::placeholders::_1));

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose/local", 10, std::bind(&WalkingPlanner::_goal_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WalkingPlanner::_odom_callback, this, std::placeholders::_1));

        _gait_pub = this->create_publisher<std_msgs::msg::String>("gait", 10);

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel/raw", 10);

        _path_pub = this->create_publisher<nav_msgs::msg::Path>("walk_planner/path", 10);

        if (_publish_all)
            _pose_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("walk_planner/states", 10);

        _solve_timer = this->create_wall_timer(std::chrono::milliseconds(time_t(1000.0 / _solve_frequency)),
                                               std::bind(&WalkingPlanner::solve, this));

        RCLCPP_INFO(this->get_logger(), "Walking Planner Node Initialized");
    }

    void solve()
    {
        if (!_goal_msg || !_odom_msg || !_gait_msg)
            return;

        if (_gait_msg->data != _gait_name)
            return;

        const float goal_x = _goal_msg->pose.position.x;
        const float goal_y = _goal_msg->pose.position.y;
        const float goal_theta = quaternion_to_yaw(_goal_msg->pose.orientation);

        const float state_x = _odom_msg->pose.pose.position.x;
        const float state_y = _odom_msg->pose.pose.position.y;
        const float state_z = _odom_msg->pose.pose.position.z;
        const float state_theta = quaternion_to_yaw(_odom_msg->pose.pose.orientation);

        _sbmpo_params.start_state = {state_x, state_y, state_theta};
        _sbmpo_params.goal_state = {goal_x, goal_y, goal_theta};

        _sbmpo->run(_sbmpo_params);
        const sbmpo::ExitCode exit_code = _sbmpo->results()->exit_code;

        if (exit_code != sbmpo::SOLUTION_FOUND)
        {
            _publish_cmd(0.0, 0.0);
            RCLCPP_WARN(this->get_logger(), "Walking Planner Failed with Exit Code %d: %s",
                        exit_code, exit_code_to_string(exit_code).c_str());
        }
        else if (_sbmpo->results()->control_path.empty())
        {
            _publish_transition_gait();
            _publish_cmd(0.0, 0.0);
        }
        else
        {
            auto cmd_vel = _sbmpo->results()->control_path[0][WalkingPlannerModel::VEL];
            auto cmd_omega = _sbmpo->results()->control_path[0][WalkingPlannerModel::OMEGA];
            _publish_cmd(cmd_vel, cmd_omega);
        }

        _publish_path(*_sbmpo->results(), state_z);
        if (_publish_all)
        {
            _publish_states(*_sbmpo->results(), state_z);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkingPlanner>());
    rclcpp::shutdown();
    return 0;
}