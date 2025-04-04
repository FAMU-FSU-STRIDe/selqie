#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*
 * IMU Calibration Node
 */
class ImuCalibrationNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _calibrate_sub; // Subscription for calibration trigger
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;      // Subscription for raw IMU data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;         // Publisher for calibrated IMU data

    int _sample_size = 100; // Number of samples to take for calibration

    tf2::Vector3 _bias = tf2::Vector3(0, 0, 0); // Bias vector for acceleration
    bool _calibrating = false;                  // Flag to indicate if we are in calibration mode
    int _sample_count = 0;                      // Counter for the number of samples taken during calibration
    std::string _calibration_file_path = "";    // Path to the calibration file

    /*
     * Callback function for calibration trigger
     */
    void calibrate_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        // Reset bias vector and sample count
        _bias = tf2::Vector3(0, 0, 0);
        _sample_count = 0;

        // Set calibrating flag to true
        _calibrating = true;

        // Feedback to the user
        RCLCPP_INFO(this->get_logger(), "Calibrating IMU...");
    }

    /*
     * Callback function for IMU data
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert orientation to tf2 quaternion
        tf2::Quaternion q;
        tf2::convert(msg->orientation, q);

        // Rotate acceleration vector by orientation
        tf2::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        acc = tf2::Matrix3x3(q) * acc;

        // Check if we are calibrating
        if (_calibrating)
        {
            // Accumulate acceleration samples
            _bias += acc;
            _sample_count++;

            // Check if we have enough samples
            if (_sample_count >= _sample_size)
            {
                // Compute average bias
                _bias /= _sample_size;

                // Set calibrating flag to false
                _calibrating = false;
                
                // Feedback to the user
                RCLCPP_INFO(this->get_logger(), "IMU calibration complete.");

                // Check if a calibration file path is set
                // If so, save the calibration data to the specified file
                if (!_calibration_file_path.empty())
                {
                    // Save calibration to file
                    std::ofstream file(_calibration_file_path);

                    // Values are space-separated
                    file << _bias[0] << " " << _bias[1] << " " << _bias[2];

                    // Feedback to the user
                    RCLCPP_INFO(this->get_logger(), "Saved IMU calibration to file: %s", _calibration_file_path.c_str());
                }
            }

            // If calibrating, we do not publish the corrected IMU message yet
            _imu_pub->publish(*msg);
        }
        else
        {
            // If not calibrating, we apply the bias correction to the acceleration data

            // Apply bias correction
            acc -= _bias;

            // Rotate acceleration vector back into the IMU frame
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
        // Get ROS parameters
        this->declare_parameter("sample_size", 100);
        this->get_parameter("sample_size", _sample_size);

        this->declare_parameter("calibration_file", "");
        this->get_parameter("calibration_file", _calibration_file_path);

        if (_calibration_file_path.empty())
        {
            // If no calibration file path is provided, use default bias
            _bias = tf2::Vector3(0, 0, 0);
            RCLCPP_INFO(this->get_logger(), "Using default IMU calibration.");
        }
        else
        {
            // Otherwise, load the calibration from the specified file

            // Open the calibration file
            std::ifstream file(_calibration_file_path);
            if (file)
            {
                // Read calibration values from the file
                file >> _bias[0] >> _bias[1] >> _bias[2];
                RCLCPP_INFO(this->get_logger(), "Loaded IMU calibration from file: %s", _calibration_file_path.c_str());
            }
            else
            {
                // If the file cannot be opened, log an error
                RCLCPP_ERROR(this->get_logger(), "Failed to load IMU calibration from file: %s", _calibration_file_path.c_str());
            }
        }

        // Create publishers and subscribers
        _calibrate_sub = this->create_subscription<std_msgs::msg::Empty>(
            "imu/calibrate", 10, std::bind(&ImuCalibrationNode::calibrate_callback, this, std::placeholders::_1));

        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&ImuCalibrationNode::imu_callback, this, std::placeholders::_1));

        _imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data/calibrated", 10);

        RCLCPP_INFO(this->get_logger(), "IMU Calibration Node Initialized.");
    }
};

// Entry point for the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}