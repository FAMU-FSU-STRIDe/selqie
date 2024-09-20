#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_msgs/msg/body_trajectory.hpp>

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

static geometry_msgs::msg::Vector3 toVector3(const geometry_msgs::msg::Point &point)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = point.x;
    vec.y = point.y;
    vec.z = point.z;
    return vec;
}

static geometry_msgs::msg::Vector3 toVector3(const geometry_msgs::msg::Quaternion &quat)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = std::atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x * quat.x + quat.y * quat.y));
    vec.y = std::asin(2 * (quat.w * quat.y - quat.z * quat.x));
    vec.z = std::atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
    return vec;
}

using namespace robot_msgs::msg;

class BodyTrajectoryNode : public rclcpp::Node
{
private:
    int _N = 11;
    double _dt = 0.1;
    double _body_z = 0.3;

    rclcpp::Publisher<BodyTrajectory>::SharedPtr _body_traj_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;

    geometry_msgs::msg::Twist::SharedPtr _twist;

    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!_twist)
        {
            return;
        }

        BodyTrajectory traj;
        traj.header.stamp = this->now();
        traj.time_step = _dt;

        traj.positions.resize(_N);
        traj.orientations.resize(_N);
        traj.linear_velocities.resize(_N);
        traj.angular_velocities.resize(_N);

        traj.positions[0] = toVector3(msg->pose.pose.position);
        traj.orientations[0] = toVector3(msg->pose.pose.orientation);
        traj.linear_velocities[0] = msg->twist.twist.linear;
        traj.angular_velocities[0] = msg->twist.twist.angular;

        for (int k = 1; k < _N; k++)
        {
            traj.positions[k].x = traj.positions[k - 1].x +
                                  _twist->linear.x * _dt * std::cos(traj.orientations[k - 1].z) -
                                  _twist->linear.y * _dt * std::sin(traj.orientations[k - 1].z);
            traj.positions[k].y = traj.positions[k - 1].y +
                                  _twist->linear.y * _dt * std::cos(traj.orientations[k - 1].z) +
                                  _twist->linear.x * _dt * std::sin(traj.orientations[k - 1].z);
            traj.positions[k].z = _body_z;

            traj.orientations[k].x = 0.0;
            traj.orientations[k].y = 0.0;
            traj.orientations[k].z = traj.orientations[k - 1].z + _twist->angular.z * _dt;

            traj.linear_velocities[k].x = _twist->linear.x;
            traj.linear_velocities[k].y = _twist->linear.y;
            traj.linear_velocities[k].z = 0.0;

            traj.angular_velocities[k].x = 0.0;
            traj.angular_velocities[k].y = 0.0;
            traj.angular_velocities[k].z = _twist->angular.z;
        }

        _body_traj_pub->publish(traj);
    }

    void updateTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _twist = msg;
    }

public:
    BodyTrajectoryNode() : Node("body_trajectory_node")
    {
        this->declare_parameter("window_size", _N);
        this->get_parameter("window_size", _N);

        this->declare_parameter("time_step", _dt);
        this->get_parameter("time_step", _dt);

        this->declare_parameter("body_height", _body_z);
        this->get_parameter("body_height", _body_z);

        _body_traj_pub = this->create_publisher<BodyTrajectory>("body/trajectory", qos_reliable());
        _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos_fast(),
            std::bind(&BodyTrajectoryNode::updateOdometry, this, std::placeholders::_1));
        _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_reliable(),
            std::bind(&BodyTrajectoryNode::updateTwist, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Body Trajectory Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}