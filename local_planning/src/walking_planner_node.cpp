#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

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

class WalkingPlanner : public rclcpp::Node
{
private:
    WalkingPlannerParams _model_params;
    std::shared_ptr<WalkingPlannerModel> _model;
    SearchParameters _sbmpo_params;
    std::unique_ptr<SBMPO> _sbmpo;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    bool _publish_all = false;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _pose_array_pub;

    geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg;
    nav_msgs::msg::Odometry::SharedPtr _odom_msg;

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

    void _publish_path(const sbmpo::SearchResults &results)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        for (const auto &state : results.state_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose = state_to_pose(state);
            path_msg.poses.push_back(pose_stamped);
        }
        _path_pub->publish(path_msg);
    }

    void _publish_states(const sbmpo::SearchResults &results)
    {
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = this->now();
        pose_array_msg.header.frame_id = "odom";
        for (const auto &node : results.nodes)
        {
            pose_array_msg.poses.push_back(state_to_pose(node->state));
        }
        _pose_array_pub->publish(pose_array_msg);
    }

public:
    WalkingPlanner() : Node("walking_planner_node")
    {
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

        this->declare_parameter("heuristic_omega_factor", 10.0);
        this->get_parameter("heuristic_omega_factor", _model_params.heuristic_omega_factor);

        this->declare_parameter("max_iterations", 100000);
        this->get_parameter("max_iterations", _sbmpo_params.max_iterations);

        this->declare_parameter("max_generations", 1000);
        this->get_parameter("max_generations", _sbmpo_params.max_generations);

        this->declare_parameter("time_limit_us", 1000000);
        this->get_parameter("time_limit_us", _sbmpo_params.time_limit_us);

        std::vector<double> grid_resolution;
        this->declare_parameter("grid_resolution", std::vector<double>{0.05, 0.05, 0.025});
        this->get_parameter("grid_resolution", grid_resolution);
        assert(grid_resolution.size() == 3);
        _sbmpo_params.grid_resolution = std::vector<float>(grid_resolution.begin(), grid_resolution.end());

        _sbmpo_params.fixed_samples = {
            {-0.50, +0.00}, {+0.50, +0.00}, {-0.25, -0.05}, {-0.25, +0.05}, {+0.25, -0.05}, {+0.25, +0.05}, {-0.10, -0.10}, {-0.10, +0.10}, {+0.10, -0.10}, {+0.10, +0.10}, {+0.00, -0.30}, {+0.00, +0.30}};

        _model = std::make_shared<WalkingPlannerModel>(_model_params);
        _sbmpo = std::make_unique<SBMPO>(_model);

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&WalkingPlanner::_goal_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WalkingPlanner::_odom_callback, this, std::placeholders::_1));

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        _path_pub = this->create_publisher<nav_msgs::msg::Path>("walk/path", 10);

        if (_publish_all)
            _pose_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("walk/states", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WalkingPlanner::solve, this));

        RCLCPP_INFO(this->get_logger(), "Walking Planner Node Initialized");
    }

    void solve()
    {
        if (!_goal_msg || !_odom_msg)
            return;

        const float goal_x = _goal_msg->pose.position.x;
        const float goal_y = _goal_msg->pose.position.y;
        const float goal_theta = quaternion_to_yaw(_goal_msg->pose.orientation);

        const float state_x = _odom_msg->pose.pose.position.x;
        const float state_y = _odom_msg->pose.pose.position.y;
        const float state_theta = quaternion_to_yaw(_odom_msg->pose.pose.orientation);

        _sbmpo_params.start_state = {state_x, state_y, state_theta};
        _sbmpo_params.goal_state = {goal_x, goal_y, goal_theta};

        _sbmpo->run(_sbmpo_params);

        if (_sbmpo->results()->exit_code == ExitCode::SOLUTION_FOUND &&
            !_sbmpo->results()->control_path.empty())
        {
            const auto cmd_vel = _sbmpo->results()->control_path[0][WalkingPlannerModel::VEL];
            const auto cmd_omega = _sbmpo->results()->control_path[0][WalkingPlannerModel::OMEGA];
            _publish_cmd(cmd_vel, cmd_omega);
        }
        else
        {
            _publish_cmd(0.0, 0.0);
            RCLCPP_WARN(this->get_logger(), "Walking Planner Failed to Solve. (Exit Code: %d)", _sbmpo->results()->exit_code);
        }

        _publish_path(*_sbmpo->results());

        if (_publish_all)
        {
            _publish_states(*_sbmpo->results());
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