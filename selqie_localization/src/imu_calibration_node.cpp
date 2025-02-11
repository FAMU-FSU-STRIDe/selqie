#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuCalibrationNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _calibrate_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

    int _sample_size = 100;

    tf2::Vector3 _bias = tf2::Vector3(0, 0, 0);
    bool _calibrating = false;
    int _sample_count = 0;

    void calibrate_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        _bias = tf2::Vector3(0, 0, 0);
        _calibrating = true;
        _sample_count = 0;
        RCLCPP_INFO(this->get_logger(), "Calibrating IMU...");
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert orientation to tf2 quaternion
        tf2::Quaternion q;
        tf2::convert(msg->orientation, q);

        // Rotate acceleration vector by orientation
        tf2::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        acc = tf2::Matrix3x3(q) * acc;

        if (_calibrating)
        {
            // Accumulate acceleration samples
            _bias += acc;
            _sample_count++;

            if (_sample_count >= _sample_size)
            {
                // Compute average bias
                _bias /= _sample_size;
                _calibrating = false;
                RCLCPP_INFO(this->get_logger(), "IMU calibration complete.");
            }

            _imu_pub->publish(*msg);
        }
        else
        {
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
    }

public:
    ImuCalibrationNode() : Node("imu_calibration_node")
    {
        this->declare_parameter("sample_size", 100);
        this->get_parameter("sample_size", _sample_size);

        _bias = tf2::Vector3(0, 0, 0);

        _calibrate_sub = this->create_subscription<std_msgs::msg::Empty>(
            "imu/calibrate", 10, std::bind(&ImuCalibrationNode::calibrate_callback, this, std::placeholders::_1));

        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&ImuCalibrationNode::imu_callback, this, std::placeholders::_1));

        _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data/calibrated", 10);

        RCLCPP_INFO(this->get_logger(), "IMU Calibration Node initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}