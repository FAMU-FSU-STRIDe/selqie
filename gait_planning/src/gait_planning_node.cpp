#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "sbmpo/SBMPO.hpp"
#include "gait_planning/gait_planning_model.hpp"

class GaitPlanningNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_goal;
    rclcpp::TimerBase::SharedPtr _solve_timer;

    nav_msgs::msg::Odometry::SharedPtr _odom;
    geometry_msgs::msg::PoseStamped::SharedPtr _goal;

    GaitPlanningParams _gait_params;
    sbmpo::SearchParameters _sbmpo_params;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom = msg;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        _goal = rclcpp::Time(msg->header.stamp).nanoseconds() == 0 ? nullptr : msg;
    }

    void solve()
    {
        if (!_odom || !_goal)
        {
            return;
        }

        const float x0 = _odom->pose.pose.position.x;
        const float y0 = _odom->pose.pose.position.y;
        const float z0 = _odom->pose.pose.position.z;
        const auto quat0 = _odom->pose.pose.orientation;
        const float theta0 = std::atan2(2 * (quat0.w * quat0.z + quat0.x * quat0.y), 1 - 2 * (quat0.y * quat0.y + quat0.z * quat0.z));
        _sbmpo_params.start_state = {0.0, theta0, x0, y0, z0, 0}; // fix gait

        const float xf = _goal->pose.position.x;
        const float yf = _goal->pose.position.y;
        const float zf = _goal->pose.position.z;
        const auto quatf = _goal->pose.orientation;
        const float thetaf = std::atan2(2 * (quatf.w * quatf.z + quatf.x * quatf.y), 1 - 2 * (quatf.y * quatf.y + quatf.z * quatf.z));
        _sbmpo_params.goal_state = {0.0, thetaf, xf, yf, zf, 0}; // fix gait

        auto model = std::make_shared<GaitPlanningModel>(_gait_params);

        sbmpo::SBMPO planner(model);
        planner.run(_sbmpo_params);

        switch (planner.results()->exit_code)
        {
            case sbmpo::SOLUTION_FOUND:
                /// TODO
                break;
            case sbmpo::ITERATION_LIMIT:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Iteration limit reached");
                break;
            case sbmpo::NO_NODES_IN_QUEUE:
                RCLCPP_WARN(this->get_logger(), "SBMPO: No nodes in queue (Check grid resolution)");
                break;
            case sbmpo::GENERATION_LIMIT:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Generation limit reached");
                break;
            case sbmpo::RUNNING:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Unknown error! (Very bad)");
                break;
            case sbmpo::QUIT_SEARCH:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Manual quit search");
                break;
            case sbmpo::TIME_LIMIT:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Time limit reached");
                break;
            case sbmpo::INVALID_START_STATE:
                RCLCPP_WARN(this->get_logger(), "SBMPO: Invalid start state");
                break;
        };
    }

public:
    GaitPlanningNode() : rclcpp::Node("gait_planning_node")
    {
        this->declare_parameter("max_iterations", int(_sbmpo_params.max_iterations));
        this->get_parameter("max_iterations", _sbmpo_params.max_iterations);

        this->declare_parameter("max_generations", int(_sbmpo_params.max_generations));
        this->get_parameter("max_generations", _sbmpo_params.max_generations);

        this->declare_parameter("time_limit_us", int(_sbmpo_params.time_limit_us));
        this->get_parameter("time_limit_us", _sbmpo_params.time_limit_us);

        std::vector<double> grid_resolution;
        this->declare_parameter("grid_resolution", grid_resolution);
        this->get_parameter("grid_resolution", grid_resolution);
        assert(grid_resolution.size() == 6);
        _sbmpo_params.grid_resolution = std::vector<float>(grid_resolution.begin(), grid_resolution.end());

        _sbmpo_params.sample_type = sbmpo::DYNAMIC;

        this->declare_parameter("goal_threshold", _gait_params.goal_threshold);
        this->get_parameter("goal_threshold", _gait_params.goal_threshold);

        _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(10), std::bind(&GaitPlanningNode::odom_callback, this, std::placeholders::_1));

        _sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", rclcpp::QoS(10), std::bind(&GaitPlanningNode::goal_callback, this, std::placeholders::_1));

        this->declare_parameter("solve_frequency", 1.0);
        double solve_frequency = this->get_parameter("solve_frequency").as_double();
        _solve_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / solve_frequency), std::bind(&GaitPlanningNode::solve, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitPlanningNode>());
    rclcpp::shutdown();
    return 0;
}