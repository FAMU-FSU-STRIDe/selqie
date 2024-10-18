#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>

class ImuDeadReckoning : public rclcpp::Node
{
private:
    std::string _frame_id, _child_frame_id;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
    rclcpp::Time _last_time;
    Eigen::Vector3d _position, _velocity, _gravity_vector;

    void _imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (_last_time.nanoseconds() == 0)
        {
            _last_time = msg->header.stamp;
            return;
        }

        const rclcpp::Time current_time = msg->header.stamp;
        const double dt = (current_time - _last_time).seconds();

        const Eigen::Quaterniond orientation(msg->orientation.w,
                                             msg->orientation.x,
                                             msg->orientation.y,
                                             msg->orientation.z);

        const Eigen::Vector3d angular_velocity(msg->angular_velocity.x,
                                               msg->angular_velocity.y,
                                               msg->angular_velocity.z);

        const Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x,
                                                  msg->linear_acceleration.y,
                                                  msg->linear_acceleration.z);

        const Eigen::Vector3d world_acceleration = orientation * linear_acceleration; // - _gravity_vector;

        const auto rot = orientation.toRotationMatrix();
        RCLCPP_INFO_STREAM(this->get_logger(), "Rot: " << rot);

        RCLCPP_INFO(this->get_logger(), "Body Acceleration: [%f, %f, %f]", linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z());
        RCLCPP_INFO(this->get_logger(), "World Acceleration: [%f, %f, %f]", world_acceleration.x(), world_acceleration.y(), world_acceleration.z());

        _velocity += world_acceleration * dt;
        _position += _velocity * dt;

        const Eigen::Vector3d body_velocity = orientation.inverse() * _velocity;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = _frame_id;
        odom.child_frame_id = _child_frame_id;
        odom.pose.pose.position.x = _position.x();
        odom.pose.pose.position.y = _position.y();
        odom.pose.pose.position.z = _position.z();
        odom.pose.pose.orientation = msg->orientation;
        odom.twist.twist.linear.x = body_velocity.x();
        odom.twist.twist.linear.y = body_velocity.y();
        odom.twist.twist.linear.z = body_velocity.z();
        odom.twist.twist.angular = msg->angular_velocity;
        _odom_pub->publish(odom);

        // RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", _position.x(), _position.y(), _position.z());

        if (_tf_broadcaster)
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = current_time;
            transform.header.frame_id = _frame_id;
            transform.child_frame_id = _child_frame_id;
            transform.transform.translation.x = _position.x();
            transform.transform.translation.y = _position.y();
            transform.transform.translation.z = _position.z();
            transform.transform.rotation = msg->orientation;
            _tf_broadcaster->sendTransform(transform);
        }
    }

public:
    ImuDeadReckoning() : Node("imu_dead_reckoning")
    {
        this->declare_parameter("frame_id", "odom");
        this->get_parameter("frame_id", _frame_id);

        this->declare_parameter("child_frame_id", "base_link");
        this->get_parameter("child_frame_id", _child_frame_id);

        this->declare_parameter("publish_tf", true);
        if (this->get_parameter("publish_tf").as_bool())
        {
            _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&ImuDeadReckoning::_imu_callback, this, std::placeholders::_1));

        _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        _position = Eigen::Vector3d::Zero();
        _velocity = Eigen::Vector3d::Zero();
        _gravity_vector = Eigen::Vector3d(0.0, 0.0, 9.81);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuDeadReckoning>());
    rclcpp::shutdown();
    return 0;
}