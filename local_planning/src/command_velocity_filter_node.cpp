#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CommandVelocityFilterNode : public rclcpp::Node
{
private:
    double _linear_acceleration;
    double _angular_acceleration;
    double _frequency;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    geometry_msgs::msg::Twist _current_cmd_vel, _next_cmd_vel;

    void _cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _next_cmd_vel = *msg;
    }

    void _timer_callback()
    {
        const double deltaVx = _next_cmd_vel.linear.x - _current_cmd_vel.linear.x;
        const double deltaVy = _next_cmd_vel.linear.y - _current_cmd_vel.linear.y;
        const double deltaVz = _next_cmd_vel.linear.z - _current_cmd_vel.linear.z;
        const double deltaWx = _next_cmd_vel.angular.x - _current_cmd_vel.angular.x;
        const double deltaWy = _next_cmd_vel.angular.y - _current_cmd_vel.angular.y;
        const double deltaWz = _next_cmd_vel.angular.z - _current_cmd_vel.angular.z;

        const double deltaV2 = deltaVx * deltaVx + deltaVy * deltaVy + deltaVz * deltaVz;
        const double deltaW2 = deltaWx * deltaWx + deltaWy * deltaWy + deltaWz * deltaWz;

        if (deltaV2 == 0.0 && deltaW2 == 0.0)
        {
            return;
        }

        const double dt = 1.0 / _frequency;
        const double maxDeltaV = _linear_acceleration * dt;
        const double maxDeltaW = _angular_acceleration * dt;

        _current_cmd_vel.linear.x += std::clamp(deltaVx, -maxDeltaV, maxDeltaV);
        _current_cmd_vel.linear.y += std::clamp(deltaVy, -maxDeltaV, maxDeltaV);
        _current_cmd_vel.linear.z += std::clamp(deltaVz, -maxDeltaV, maxDeltaV);
        _current_cmd_vel.angular.x += std::clamp(deltaWx, -maxDeltaW, maxDeltaW);
        _current_cmd_vel.angular.y += std::clamp(deltaWy, -maxDeltaW, maxDeltaW);
        _current_cmd_vel.angular.z += std::clamp(deltaWz, -maxDeltaW, maxDeltaW);

        _cmd_vel_pub->publish(_current_cmd_vel);
    }

public:
    CommandVelocityFilterNode() : Node("command_velocity_filter_node")
    {
        this->declare_parameter("linear_acceleration", 1.0);
        this->get_parameter("linear_acceleration", _linear_acceleration);

        this->declare_parameter("angular_acceleration", 1.0);
        this->get_parameter("angular_acceleration", _angular_acceleration);

        this->declare_parameter("frequency", 50.0);
        this->get_parameter("frequency", _frequency);

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel/raw", 10, std::bind(&CommandVelocityFilterNode::_cmd_vel_callback, this, std::placeholders::_1));
        _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(time_t(1E3 / _frequency)),
                                         std::bind(&CommandVelocityFilterNode::_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Command Velocity Filter Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandVelocityFilterNode>());
    rclcpp::shutdown();
    return 0;
}