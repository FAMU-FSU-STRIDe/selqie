#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuCalibrationNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

    tf2::Vector3 _bias = tf2::Vector3(0, 0, 0);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert orientation to tf2 quaternion
        tf2::Quaternion q;
        tf2::convert(msg->orientation, q);

        // Rotate acceleration vector by orientation
        tf2::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        acc = tf2::Matrix3x3(q) * acc;

        // Apply bias correction
        acc -= _bias;

        // Rotate acceleration vector back
        acc = tf2::Matrix3x3(q.inverse()) * acc;

        // Publish corrected IMU message
        auto msg_corrected = std::make_shared<sensor_msgs::msg::Imu>(*msg);
        msg_corrected->linear_acceleration.x = acc.x();
        msg_corrected->linear_acceleration.y = acc.y();
        msg_corrected->linear_acceleration.z = acc.z();
        _imu_pub->publish(*msg_corrected);
    }

public:
    ImuCalibrationNode() : Node("imu_bias_correction_node")
    {
        std::vector<double> bias = {0, 0, 0};
        this->declare_parameter("bias", bias);
        this->get_parameter("bias", bias);
        assert(bias.size() == 3);
        _bias = tf2::Vector3(bias[0], bias[1], bias[2]);

        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&ImuCalibrationNode::imu_callback, this, std::placeholders::_1));

        _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_corrected", 10);

        RCLCPP_INFO(this->get_logger(), "IMU Bias Correction Node initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}