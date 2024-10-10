#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

class StereoUSBCamNode : public rclcpp::Node
{
private:
    cv::VideoCapture _left_capture, _right_capture;
    std_msgs::msg::Header _left_header, _right_header;
    sensor_msgs::msg::CameraInfo _left_camera_info, _right_camera_info;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _left_image_pub, _right_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _left_camera_info_pub, _right_camera_info_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    void timer_callback()
    {
        const rclcpp::Time timestamp = this->now();
        _left_capture.grab();
        _right_capture.grab();

        cv::Mat left_frame;
        if (!_left_capture.retrieve(left_frame))
        {
            RCLCPP_ERROR(get_logger(), "Failed to capture left frame");
            return;
        }

        cv::Mat right_frame;
        if (!_right_capture.retrieve(right_frame))
        {
            RCLCPP_ERROR(get_logger(), "Failed to capture right frame");
            return;
        }

        const auto left_image = cv_bridge::CvImage(_left_header, "bgr8", left_frame).toImageMsg();
        left_image->header.stamp = timestamp;
        _left_image_pub->publish(*left_image);

        const auto right_image = cv_bridge::CvImage(_right_header, "bgr8", right_frame).toImageMsg();
        right_image->header.stamp = timestamp;
        _right_image_pub->publish(*right_image);
    }

public:
    StereoUSBCamNode() : Node("stereo_usb_cam_node")
    {
        this->declare_parameter("width", 640);
        const int width = this->get_parameter("width").as_int();

        this->declare_parameter("height", 480);
        const int height = this->get_parameter("height").as_int();

        this->declare_parameter("framerate", 30);
        const int framerate = this->get_parameter("framerate").as_int();

        this->declare_parameter("left_video_device", "/dev/video0");
        const std::string left_video_device = this->get_parameter("left_video_device").as_string();
        _left_capture.open(left_video_device);
        if (!_left_capture.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open left camera");
            rclcpp::shutdown();
        }
        else
        {
            _left_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
            _left_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            _left_capture.set(cv::CAP_PROP_FPS, framerate);
        }

        this->declare_parameter("right_video_device", "/dev/video1");
        const std::string right_video_device = this->get_parameter("right_video_device").as_string();
        _right_capture.open(right_video_device);
        if (!_right_capture.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open right camera");
            rclcpp::shutdown();
        }
        else
        {
            _right_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
            _right_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            _right_capture.set(cv::CAP_PROP_FPS, framerate);
        }

        this->declare_parameter("left_frame_id", "left_camera");
        this->get_parameter("left_frame_id", _left_header.frame_id);

        this->declare_parameter("right_frame_id", "right_camera");
        this->get_parameter("right_frame_id", _right_header.frame_id);

        this->declare_parameter("left_camera_info_url", "");
        const std::string left_camera_info_url = this->get_parameter("left_camera_info_url").as_string();
        if (left_camera_info_url.empty())
        {
            RCLCPP_ERROR(get_logger(), "Left camera calibration file not provided");
            rclcpp::shutdown();
        }

        this->declare_parameter("right_camera_info_url", "");
        const std::string right_camera_info_url = this->get_parameter("right_camera_info_url").as_string();
        if (right_camera_info_url.empty())
        {
            RCLCPP_ERROR(get_logger(), "Right camera calibration file not provided");
            rclcpp::shutdown();
        }

        _left_image_pub = create_publisher<sensor_msgs::msg::Image>("stereo/left/image", 10);
        _right_image_pub = create_publisher<sensor_msgs::msg::Image>("stereo/right/image", 10);

        _left_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("stereo/left/camera_info", 10);
        _right_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("stereo/right/camera_info", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(1000 / framerate),
                                         std::bind(&StereoUSBCamNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Stereo USB Cam Node Initialized");
    }

    ~StereoUSBCamNode()
    {
        _left_capture.release();
        _right_capture.release();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoUSBCamNode>());
    rclcpp::shutdown();
    return 0;
}