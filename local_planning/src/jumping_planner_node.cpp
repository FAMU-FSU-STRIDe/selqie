#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class JumpingPlannerNode : public rclcpp::Node
{
private:
    const std::string _gait_name = "jump";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_transition_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _gait_pub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;

    double _solve_frequency;
    rclcpp::TimerBase::SharedPtr _solve_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

    std_msgs::msg::String::SharedPtr _gait_msg;
    std_msgs::msg::String::SharedPtr _gait_transition_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg;
    nav_msgs::msg::Odometry::SharedPtr _odom_msg;
    bool _send_cmd_vel = false;

    double _goal_threshold;

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
        _goal_msg = msg;
        _send_cmd_vel = true;
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom_msg = msg;
    }

    void _publish_cmd(const double vel_x, const double vel_z)
    {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = vel_x;
        cmd_msg.linear.z = vel_z;
        _cmd_pub->publish(cmd_msg);
    }

public:
    JumpingPlannerNode() : Node("swimming_planner")
    {
        this->declare_parameter("solve_frequency", 20.0);
        this->get_parameter("solve_frequency", _solve_frequency);

        this->declare_parameter("goal_threshold", 0.15);
        this->get_parameter("goal_threshold", _goal_threshold);

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&JumpingPlannerNode::_gait_callback, this, std::placeholders::_1));

        _gait_transition_sub = this->create_subscription<std_msgs::msg::String>(
            "gait/transition", 10, std::bind(&JumpingPlannerNode::_gait_transition_callback, this, std::placeholders::_1));

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose/local", 10, std::bind(&JumpingPlannerNode::_goal_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&JumpingPlannerNode::_odom_callback, this, std::placeholders::_1));

        _gait_pub = this->create_publisher<std_msgs::msg::String>("gait", 10);

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        _solve_timer = this->create_wall_timer(std::chrono::milliseconds(time_t(1000.0 / _solve_frequency)),
                                               std::bind(&JumpingPlannerNode::solve, this));

        RCLCPP_INFO(this->get_logger(), "Jumping Planner Node Initialized");
    }

    void solve()
    {
        if (!_goal_msg || !_odom_msg || !_gait_msg)
            return;

        if (_gait_msg->data != _gait_name)
            return;

        const double goal_x = _goal_msg->pose.position.x;
        const double goal_z = _goal_msg->pose.position.z;

        const double state_x = _odom_msg->pose.pose.position.x;
        const double state_z = _odom_msg->pose.pose.position.z;

        const double dx = goal_x - state_x;
        const double dz = goal_z - state_z;
        const double distance = std::sqrt(dx * dx + dz * dz);

        if (distance < _goal_threshold)
        {
            _gait_pub->publish(*_gait_transition_msg);
            _publish_cmd(0.0, 0.0);
            return;
        }

        if (_send_cmd_vel)
        {
            _publish_cmd(dx / distance, dz / distance);
            _send_cmd_vel = false;
            return;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JumpingPlannerNode>());
    rclcpp::shutdown();
    return 0;
}