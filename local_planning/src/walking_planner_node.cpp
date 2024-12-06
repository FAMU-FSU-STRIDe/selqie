#include <rclcpp/rclcpp.hpp>

#include <sbmpo/SBMPO.hpp>
#include <sbmpo/tools/CSVTool.hpp>

#include "local_planning/walking_model.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

class WalkingPlannerNode : public rclcpp::Node
{
private:
    std::unique_ptr<SBMPO> _sbmpo;
    std::shared_ptr<WalkingModel> _model;
    SearchParameters _search_params;
    WalkingModelParams _model_params;
    double _stand_height = 0.25;
    int _res_v = 10;
    int _res_w = 5;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_goal;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr _sub_map;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pub_cmd_pose;

    rclcpp::TimerBase::SharedPtr _solve_timer;

    nav_msgs::msg::Odometry::SharedPtr _odom;
    geometry_msgs::msg::PoseStamped::SharedPtr _goal;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom = msg;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        _goal = msg->pose.position.z < 0.0 ? nullptr : msg;
    }

    void map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(*msg, map);
        _model->set_map(map);
    }

    void generateActions()
    {
        const float dth = 2 * M_PI / static_cast<float>(_res_v);
        const int w_off = _res_w / 2;
        _search_params.fixed_samples.resize(_res_v * _res_w);
        for (int i = 0; i < _res_v; i++)
        {
            for (int j = 0; j < _res_w; j++)
            {
                const float vx = _model_params.max_velocities[0] * std::cos(i * dth);
                const float vy = _model_params.max_velocities[1] * std::sin(i * dth);
                const float wz = _model_params.max_velocities[2] * (j - w_off);
                _search_params.fixed_samples[_res_w * i + j] = {vx, vy, wz};
            }
        }
    }

public:
    WalkingPlannerNode() : rclcpp::Node("walking_planner")
    {
        this->declare_parameter("max_iterations", int(_search_params.max_iterations));
        this->get_parameter("max_iterations", _search_params.max_iterations);

        this->declare_parameter("max_generations", int(_search_params.max_generations));
        this->get_parameter("max_generations", _search_params.max_generations);

        this->declare_parameter("time_limit_us", int(_search_params.time_limit_us));
        this->get_parameter("time_limit_us", _search_params.time_limit_us);

        this->declare_parameter("sample_time", _model_params.dt);
        this->get_parameter("sample_time", _model_params.dt);

        std::vector<double> grid_res;
        this->declare_parameter("grid_resolution", grid_res);
        this->get_parameter("grid_resolution", grid_res);
        assert(grid_res.size() == 3);
        _search_params.grid_resolution = {static_cast<float>(grid_res[0]),
                                          static_cast<float>(grid_res[1]),
                                          static_cast<float>(grid_res[2])};

        std::vector<double> max_vels;
        this->declare_parameter("max_velocities", max_vels);
        this->get_parameter("max_velocities", max_vels);
        assert(max_vels.size() == 3);
        _model_params.max_velocities = {static_cast<float>(max_vels[0]),
                                        static_cast<float>(max_vels[1]),
                                        static_cast<float>(max_vels[2])};

        this->declare_parameter("goal_threshold", _model_params.goal_threshold);
        this->get_parameter("goal_threshold", _model_params.goal_threshold);

        this->declare_parameter("stand_height", _stand_height);
        this->get_parameter("stand_height", _stand_height);

        double solve_frequency = 10.0;
        this->declare_parameter("solve_frequency", solve_frequency);
        this->get_parameter("solve_frequency", solve_frequency);

        _model = std::make_shared<WalkingModel>(_model_params);
        _sbmpo = std::make_unique<SBMPO>(_model);
        generateActions();

        _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos_reliable(), std::bind(&WalkingPlannerNode::odom_callback, this, std::placeholders::_1));

        _sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", qos_reliable(), std::bind(&WalkingPlannerNode::goal_callback, this, std::placeholders::_1));

        _sub_map = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "map", qos_reliable(), std::bind(&WalkingPlannerNode::map_callback, this, std::placeholders::_1));

        _pub_cmd_pose = this->create_publisher<geometry_msgs::msg::Pose>("cmd_pose", qos_reliable());

        _solve_timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / solve_frequency),
            std::bind(&WalkingPlannerNode::solve, this));
    }

    void solve()
    {
        if (!_odom || !_goal)
        {
            return;
        }

        const float x0 = _odom->pose.pose.position.x;
        const float y0 = _odom->pose.pose.position.y;
        const auto quat0 = _odom->pose.pose.orientation;
        const float theta0 = std::atan2(2 * (quat0.w * quat0.z + quat0.x * quat0.y), 1 - 2 * (quat0.y * quat0.y + quat0.z * quat0.z));
        _search_params.start_state = {x0, y0, theta0};

        const float xf = _goal->pose.position.x;
        const float yf = _goal->pose.position.y;
        const auto quatf = _goal->pose.orientation;
        const float thetaf = std::atan2(2 * (quatf.w * quatf.z + quatf.x * quatf.y), 1 - 2 * (quatf.y * quatf.y + quatf.z * quatf.z));
        _search_params.goal_state = {xf, yf, thetaf};

        _sbmpo->run(_search_params);

        switch (_sbmpo->results()->exit_code)
        {
        case SOLUTION_FOUND:
        {
            const State cmd_state_pose = _sbmpo->results()->state_path.size() < 2
                                             ? _search_params.goal_state
                                             : _sbmpo->results()->state_path[1];

            geometry_msgs::msg::Pose cmd_pose;
            cmd_pose.position.x = cmd_state_pose[0];
            cmd_pose.position.y = cmd_state_pose[1];
            cmd_pose.position.z = _stand_height;
            cmd_pose.orientation.w = std::cos(cmd_state_pose[2] / 2);
            cmd_pose.orientation.x = 0.0;
            cmd_pose.orientation.y = 0.0;
            cmd_pose.orientation.z = std::sin(cmd_state_pose[2] / 2);
            _pub_cmd_pose->publish(cmd_pose);
            break;
        }
        case ITERATION_LIMIT:
            RCLCPP_WARN(this->get_logger(), "Iteration limit reached.");
            break;
        case NO_NODES_IN_QUEUE:
            RCLCPP_WARN(this->get_logger(), "No nodes in queue.");
            break;
        case GENERATION_LIMIT:
            RCLCPP_WARN(this->get_logger(), "Generation limit reached.");
            break;
        case TIME_LIMIT:
            RCLCPP_WARN(this->get_logger(), "Time limit reached.");
            break;
        case INVALID_START_STATE:
            RCLCPP_WARN(this->get_logger(), "Invalid start state.");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Failed to solve.");
            break;
        }

        static bool first_run = true;
        if (first_run)
        {
            first_run = false;
            sbmpo_csv::clear_file("/tmp/csv/stats.csv");
            sbmpo_csv::clear_file("/tmp/csv/nodes.csv");
            sbmpo_csv::append_stats("/tmp/csv/stats.csv", *_sbmpo->results());
            sbmpo_csv::append_node_path("/tmp/csv/nodes.csv", _sbmpo->results()->node_path);
            sbmpo_csv::append_nodes("/tmp/csv/nodes.csv", _sbmpo->results()->nodes);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkingPlannerNode>());
    rclcpp::shutdown();
    return 0;
}