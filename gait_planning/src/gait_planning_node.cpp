#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include "sbmpo/SBMPO.hpp"
#include "gait_planning/gait_planning_model.hpp"

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
    pose.position.x = state[StateIndex::X];
    pose.position.y = state[StateIndex::Y];
    pose.position.z = state[StateIndex::Z];
    pose.orientation = yaw_to_quaternion(state[StateIndex::Q]);
    return pose;
}

GaitType string_to_gait(const std::string &gait)
{
    if (gait == "walk")
        return WALK;
    if (gait == "swim")
        return SWIM;
    if (gait == "jump")
        return JUMP;
    if (gait == "sink")
        return SINK;
    return NONE;
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

class GaitPlanningNode : public rclcpp::Node
{
private:
    GaitDynamicsOptions _dynamics_options;
    GaitPlanningParams _gait_params;
    std::shared_ptr<GaitPlanningModel> _model;
    sbmpo::SearchParameters _sbmpo_params;
    std::unique_ptr<sbmpo::SBMPO> _sbmpo;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::TimerBase::SharedPtr _solve_timer;

    bool _publish_all = false;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _pose_array_pub;

    geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg;
    nav_msgs::msg::Odometry::SharedPtr _odom_msg;
    std_msgs::msg::String::SharedPtr _gait_msg;

    void _goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        _goal_msg = msg->pose.orientation.w == 0.0 ? nullptr : msg;
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom_msg = msg;
    }

    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _gait_msg = msg;
    }

    void _publish_path(const sbmpo::SearchResults &results)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
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
        pose_array_msg.header.frame_id = "map";
        for (const auto &node : results.nodes)
        {
            pose_array_msg.poses.push_back(state_to_pose(node->state));
        }
        _pose_array_pub->publish(pose_array_msg);
    }

public:
    GaitPlanningNode() : rclcpp::Node("gait_planning_node")
    {
        this->declare_parameter("solve_frequency", 1.0);
        const double solve_frequency = this->get_parameter("solve_frequency").as_double();

        this->declare_parameter("publish_all", false);
        this->get_parameter("publish_all", _publish_all);

        this->declare_parameter("horizon_time", 5.0);
        this->get_parameter("horizon_time", _dynamics_options.horizon_time);

        this->declare_parameter("integration_steps", 5);
        this->get_parameter("integration_steps", _dynamics_options.integration_steps);

        this->declare_parameter("cost_of_transport", 1.0);
        this->get_parameter("cost_of_transport", _dynamics_options.cost_of_transport);

        this->declare_parameter("jumping_loadup_time", 0.5);
        this->get_parameter("jumping_loadup_time", _dynamics_options.jumping_loadup_time);

        this->declare_parameter("sinking_speed", 0.25);
        this->get_parameter("sinking_speed", _dynamics_options.sinking_speed);

        this->declare_parameter("goal_threshold", _gait_params.goal_threshold);
        this->get_parameter("goal_threshold", _gait_params.goal_threshold);

        this->declare_parameter("max_iterations", 1000000);
        this->get_parameter("max_iterations", _sbmpo_params.max_iterations);

        this->declare_parameter("max_generations", 1000);
        this->get_parameter("max_generations", _sbmpo_params.max_generations);

        this->declare_parameter("time_limit_us", 5000000);
        this->get_parameter("time_limit_us", _sbmpo_params.time_limit_us);

        std::vector<double> grid_resolution;
        this->declare_parameter("grid_resolution", std::vector<double>{0.0, 0.25, 0.25, 0.50, 0.25, 1.0});
        this->get_parameter("grid_resolution", grid_resolution);
        assert(grid_resolution.size() == 6);
        _sbmpo_params.grid_resolution = std::vector<float>(grid_resolution.begin(), grid_resolution.end());

        _sbmpo_params.sample_type = sbmpo::DYNAMIC;

        _model = std::make_shared<GaitPlanningModel>(_gait_params, _dynamics_options);
        _sbmpo = std::make_unique<SBMPO>(_model);

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&GaitPlanningNode::_goal_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GaitPlanningNode::_odom_callback, this, std::placeholders::_1));

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&GaitPlanningNode::_gait_callback, this, std::placeholders::_1));

        _path_pub = this->create_publisher<nav_msgs::msg::Path>("gait/path", 10);

        if (_publish_all)
            _pose_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("gait/states", 10);

        _solve_timer = this->create_wall_timer(std::chrono::milliseconds(time_t(1000.0 / solve_frequency)),
                                               std::bind(&GaitPlanningNode::solve, this));

        RCLCPP_INFO(this->get_logger(), "Gait Planning Node Initialized");
    }

    void solve()
    {
        if (!_goal_msg || !_odom_msg || !_gait_msg)
            return;

        const float goal_q = quaternion_to_yaw(_goal_msg->pose.orientation);
        const float goal_x = _goal_msg->pose.position.x;
        const float goal_y = _goal_msg->pose.position.y;
        const float goal_z = _goal_msg->pose.position.z;
        _sbmpo_params.goal_state = {0.0, goal_q, goal_x, goal_y, goal_z, GaitType::NONE};

        const float state_q = quaternion_to_yaw(_odom_msg->pose.pose.orientation);
        const float state_x = _odom_msg->pose.pose.position.x;
        const float state_y = _odom_msg->pose.pose.position.y;
        const float state_z = _odom_msg->pose.pose.position.z;
        const GaitType state_gait = string_to_gait(_gait_msg->data);
        if (state_gait == NONE)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Gait Type: %s", _gait_msg->data.c_str());
            return;
        }
        _sbmpo_params.start_state = {0.0, state_q, state_x, state_y, state_z, static_cast<float>(state_gait)};

        _sbmpo->run(_sbmpo_params);
        const sbmpo::ExitCode exit_code = _sbmpo->results()->exit_code;

        if (exit_code != sbmpo::SOLUTION_FOUND)
        {
            RCLCPP_WARN(this->get_logger(), "Gait Planner Failed with Exit Code %d: %s",
                        exit_code, exit_code_to_string(exit_code).c_str());
        }
        else if (_sbmpo->results()->control_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Gait Planner Planner Reached the Goal");
        }
        else
        {
            /// TODO: Publish results
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
    rclcpp::spin(std::make_shared<GaitPlanningNode>());
    rclcpp::shutdown();
    return 0;
}