#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

class ExploreHDStereoNode : public rclcpp::Node
{
private:
    cv::VideoCapture _left_capture;
    cv::VideoCapture _right_capture;

    sensor_msgs::msg::CameraInfo _left_camera_info;
    sensor_msgs::msg::CameraInfo _right_camera_info;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _left_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _right_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _left_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _right_camera_info_pub;

    rclcpp::TimerBase::SharedPtr _timer;

    void timer_callback()
    {
        cv::Mat left_frame, right_frame;
        _left_capture >> left_frame;
        _right_capture >> right_frame;

        if (left_frame.empty() || right_frame.empty())
        {
            RCLCPP_ERROR(get_logger(), "Failed to capture frame");
            return;
        }

        sensor_msgs::msg::Image left_image;
        sensor_msgs::msg::Image right_image;

        left_image.header.stamp = this->now();
        left_image.header.frame_id = "left_camera";
        left_image.height = left_frame.rows;
        left_image.width = left_frame.cols;
        left_image.encoding = "bgr8";
        left_image.is_bigendian = false;
        left_image.step = left_frame.cols * left_frame.elemSize();
        left_image.data = std::vector<uint8_t>(left_frame.data, left_frame.data + left_frame.total() * left_frame.elemSize());

        right_image.header.stamp = this->now();
        right_image.header.frame_id = "right_camera";
        right_image.height = right_frame.rows;
        right_image.width = right_frame.cols;
        right_image.encoding = "bgr8";
        right_image.is_bigendian = false;
        right_image.step = right_frame.cols * right_frame.elemSize();
        right_image.data = std::vector<uint8_t>(right_frame.data, right_frame.data + right_frame.total() * right_frame.elemSize());

        _left_image_pub->publish(left_image);
        _right_image_pub->publish(right_image);

        _left_camera_info.header.stamp = this->now();
        _left_camera_info.header.frame_id = "left_camera";
        _left_camera_info_pub->publish(_left_camera_info);

        _right_camera_info.header.stamp = this->now();
        _right_camera_info.header.frame_id = "right_camera";
        _right_camera_info_pub->publish(_right_camera_info);
    }

public:
    ExploreHDStereoNode() : Node("explorehd_stereo_node")
    {
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("framerate", 30);
        this->declare_parameter("left_camera_id", 0);
        this->declare_parameter("right_camera_id", 1);
        this->declare_parameter("left_camera_calibration", "");
        this->declare_parameter("right_camera_calibration", "");

        const int width = this->get_parameter("width").as_int();
        const int height = this->get_parameter("height").as_int();
        const int framerate = this->get_parameter("framerate").as_int();
        const int left_camera_id = this->get_parameter("left_camera_id").as_int();
        const int right_camera_id = this->get_parameter("right_camera_id").as_int();
        const std::string left_camera_calibration = this->get_parameter("left_camera_calibration").as_string();
        const std::string right_camera_calibration = this->get_parameter("right_camera_calibration").as_string();

        _left_capture.open(left_camera_id);
        _right_capture.open(right_camera_id);

        if (!_left_capture.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open left camera");
            rclcpp::shutdown();
        }

        if (!_right_capture.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open right camera");
            rclcpp::shutdown();
        }

        _left_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        _left_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        _left_capture.set(cv::CAP_PROP_FPS, framerate);

        _right_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        _right_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        _right_capture.set(cv::CAP_PROP_FPS, framerate);

        _left_image_pub = create_publisher<sensor_msgs::msg::Image>("stereo/left/image", 10);
        _right_image_pub = create_publisher<sensor_msgs::msg::Image>("stereo/right/image", 10);
        _left_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("stereo/left/camera_info", 10);
        _right_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("stereo/right/camera_info", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(1000 / framerate),
                                         std::bind(&ExploreHDStereoNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "ExploreHD Stereo Node Initialized");
    }

    ~ExploreHDStereoNode()
    {
        _left_capture.release();
        _right_capture.release();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExploreHDStereoNode>());
    rclcpp::shutdown();
    return 0;
}