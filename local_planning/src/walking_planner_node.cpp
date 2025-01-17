#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "local_planning/walking_planner_model.hpp"
#include "sbmpo/SBMPO.hpp"
#include "sbmpo/tools/PrintTool.hpp"
#include "sbmpo/tools/CSVTool.hpp"

float quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class WalkingPlanner : public rclcpp::Node
{
private:
    WalkingPlannerParams _model_params;
    std::shared_ptr<WalkingPlannerModel> _model;
    SearchParameters _sbmpo_params;
    std::unique_ptr<SBMPO> _sbmpo;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    geometry_msgs::msg::Pose::SharedPtr _goal_msg;
    nav_msgs::msg::Odometry::SharedPtr _state_msg;

    void _goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        _goal_msg = msg->orientation.w == 0.0 ? nullptr : msg;
    }

    void _state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _state_msg = msg;
    }

public:
    WalkingPlanner() : Node("walking_planner")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Walking Planner Node");

        _model = std::make_shared<WalkingPlannerModel>(_model_params);
        _sbmpo = std::make_unique<SBMPO>(_model);

        _sbmpo_params.max_iterations = 500000;
        _sbmpo_params.grid_resolution = {0.05, 0.05, 0.05, 0.0, 0.0};
        _sbmpo_params.fixed_samples = {
            {-1.0, -1.0}, {-1.0, 0.0}, {-1.0, +1.0}, {0.0, -1.0}, {0.0, 0.0}, {0.0, +1.0}, {+1.0, -1.0}, {+1.0, 0.0}, {+1.0, +1.0}};

        _goal_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "goal", 10, std::bind(&WalkingPlanner::_goal_callback, this, std::placeholders::_1));

        _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "state", 10, std::bind(&WalkingPlanner::_state_callback, this, std::placeholders::_1));

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WalkingPlanner::solve, this));

        RCLCPP_INFO(this->get_logger(), "Walking Planner Node Initialized");
    }

    void solve()
    {
        if (!_goal_msg || !_state_msg)
            return;

        const float goal_x = _goal_msg->position.x;
        const float goal_y = _goal_msg->position.y;
        const float goal_theta = quaternion_to_yaw(_goal_msg->orientation);

        const float state_x = _state_msg->pose.pose.position.x;
        const float state_y = _state_msg->pose.pose.position.y;
        const float state_theta = quaternion_to_yaw(_state_msg->pose.pose.orientation);
        const float state_vel = _state_msg->twist.twist.linear.x;
        const float state_omega = _state_msg->twist.twist.angular.z;

        _sbmpo_params.start_state = {state_x, state_y, state_theta, state_vel, state_omega};
        _sbmpo_params.goal_state = {goal_x, goal_y, goal_theta, 0.0, 0.0};

        _sbmpo->run(_sbmpo_params);

        if (_sbmpo->results()->exit_code == ExitCode::SOLUTION_FOUND && _sbmpo->results()->state_path.size() > 1)
        {
            const auto cmd_vel = _sbmpo->results()->state_path[1][WalkingPlannerModel::VEL];
            const auto cmd_omega = _sbmpo->results()->state_path[1][WalkingPlannerModel::OMEGA];

            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x = cmd_vel;
            cmd_msg.angular.z = cmd_omega;
            _cmd_pub->publish(cmd_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing Command: VEL=%.2f, OMEGA=%.2f", cmd_vel, cmd_omega);
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