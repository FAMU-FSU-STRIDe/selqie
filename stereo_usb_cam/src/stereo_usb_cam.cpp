#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

namespace stereo_usb_cam
{

    class StereoUsbCam : public rclcpp::Node
    {
    private:
        cv::VideoCapture _left_capture, _right_capture;
        std_msgs::msg::Header _left_header, _right_header;
        sensor_msgs::msg::CameraInfo _left_camera_info, _right_camera_info;
        image_transport::CameraPublisher _left_camera_pub, _right_camera_pub;
        rclcpp::TimerBase::SharedPtr _timer;
        std::string _encoding;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _left_image_playback_sub, _right_image_playback_sub;

        void timer_callback()
        {
            const rclcpp::Time left_timestamp = this->now();

            if (!_left_capture.grab())
            {
                RCLCPP_ERROR(get_logger(), "Failed to grab left frame");
                return;
            }

            if (!_right_capture.grab())
            {
                RCLCPP_ERROR(get_logger(), "Failed to grab right frame");
                return;
            }

            cv::Mat left_frame, right_frame;
            if (!_left_capture.retrieve(left_frame))
            {
                RCLCPP_ERROR(get_logger(), "Failed to capture left frame");
                return;
            }

            if (!_right_capture.retrieve(right_frame))
            {
                RCLCPP_ERROR(get_logger(), "Failed to capture right frame");
                return;
            }

            if (_encoding == "rgb8")
            {
                cv::cvtColor(left_frame, left_frame, cv::COLOR_BGR2RGB);
                cv::cvtColor(right_frame, right_frame, cv::COLOR_BGR2RGB);
            }

            const auto delta_frame_time = _right_capture.get(cv::CAP_PROP_POS_MSEC) - _left_capture.get(cv::CAP_PROP_POS_MSEC);
            const auto right_timestamp = left_timestamp + rclcpp::Duration(0, delta_frame_time * 1e6);

            const auto left_image = cv_bridge::CvImage(_left_header, _encoding, left_frame).toImageMsg();
            left_image->header.stamp = left_timestamp;
            _left_camera_info.header.stamp = left_timestamp;
            _left_camera_pub.publish(left_image, std::make_shared<sensor_msgs::msg::CameraInfo>(_left_camera_info));

            const auto right_image = cv_bridge::CvImage(_right_header, _encoding, right_frame).toImageMsg();
            right_image->header.stamp = right_timestamp;
            _right_camera_info.header.stamp = right_timestamp;
            _right_camera_pub.publish(right_image, std::make_shared<sensor_msgs::msg::CameraInfo>(_right_camera_info));
        }

    public:
        StereoUsbCam(const rclcpp::NodeOptions &options)
            : Node("stereo_usb_cam_node", options)
        {

            this->declare_parameter("left_frame_id", "camera_left");
            this->get_parameter("left_frame_id", _left_header.frame_id);

            this->declare_parameter("right_frame_id", "camera_right");
            this->get_parameter("right_frame_id", _right_header.frame_id);

            this->declare_parameter("left_camera_name", "left_camera");
            const std::string left_camera_name = this->get_parameter("left_camera_name").as_string();

            this->declare_parameter("right_camera_name", "right_camera");
            const std::string right_camera_name = this->get_parameter("right_camera_name").as_string();

            this->declare_parameter("left_camera_info_url", "");
            const std::string left_camera_info_url = this->get_parameter("left_camera_info_url").as_string();
            if (!left_camera_info_url.empty())
            {
                camera_info_manager::CameraInfoManager left_camera_info_manager(this, left_camera_name, left_camera_info_url);
                _left_camera_info = left_camera_info_manager.getCameraInfo();
                _left_camera_info.header.frame_id = _left_header.frame_id;
            }

            this->declare_parameter("right_camera_info_url", "");
            const std::string right_camera_info_url = this->get_parameter("right_camera_info_url").as_string();
            if (!right_camera_info_url.empty())
            {
                camera_info_manager::CameraInfoManager right_camera_info_manager(this, right_camera_name, right_camera_info_url);
                _right_camera_info = right_camera_info_manager.getCameraInfo();
                _right_camera_info.header.frame_id = _right_header.frame_id;
            }

            _left_camera_pub = image_transport::create_camera_publisher(this, "left/image_raw", rclcpp::QoS{100}.get_rmw_qos_profile());
            _right_camera_pub = image_transport::create_camera_publisher(this, "right/image_raw", rclcpp::QoS{100}.get_rmw_qos_profile());

            this->declare_parameter("playback", false);
            const bool playback = this->get_parameter("playback").as_bool();

            if (!playback)
            {

                this->declare_parameter("width", 640);
                const int width = this->get_parameter("width").as_int();

                this->declare_parameter("height", 480);
                const int height = this->get_parameter("height").as_int();

                this->declare_parameter("framerate", 30.0);
                const double framerate = this->get_parameter("framerate").as_double();

                this->declare_parameter("encoding", "bgr8");
                _encoding = this->get_parameter("encoding").as_string();

                this->declare_parameter("left_video_device", "/dev/video0");
                const std::string left_video_device = this->get_parameter("left_video_device").as_string();

                _left_capture.open(left_video_device, cv::CAP_V4L2);
                _left_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                _left_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
                _left_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                _left_capture.set(cv::CAP_PROP_FPS, framerate);

                if (!_left_capture.isOpened())
                {
                    RCLCPP_ERROR(get_logger(), "Failed to open left camera");
                    rclcpp::shutdown();
                }

                this->declare_parameter("right_video_device", "/dev/video1");
                const std::string right_video_device = this->get_parameter("right_video_device").as_string();

                _right_capture.open(right_video_device, cv::CAP_V4L2);
                _right_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                _right_capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
                _right_capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                _right_capture.set(cv::CAP_PROP_FPS, framerate);

                if (!_right_capture.isOpened())
                {
                    RCLCPP_ERROR(get_logger(), "Failed to open right camera");
                    rclcpp::shutdown();
                }

                _timer = this->create_wall_timer(std::chrono::milliseconds(time_t(1000 / framerate)),
                                                 std::bind(&StereoUsbCam::timer_callback, this));
            }
            else
            {
                _left_image_playback_sub = this->create_subscription<sensor_msgs::msg::Image>(
                    "left/image_raw/playback", 10,
                    [this](const sensor_msgs::msg::Image::SharedPtr msg)
                    {
                        _left_camera_info.header.stamp = msg->header.stamp;
                        _left_camera_pub.publish(msg, std::make_shared<sensor_msgs::msg::CameraInfo>(_left_camera_info));
                    });

                _right_image_playback_sub = this->create_subscription<sensor_msgs::msg::Image>(
                    "right/image_raw/playback", 10,
                    [this](const sensor_msgs::msg::Image::SharedPtr msg)
                    {
                        _right_camera_info.header.stamp = msg->header.stamp;
                        _right_camera_pub.publish(msg, std::make_shared<sensor_msgs::msg::CameraInfo>(_right_camera_info));
                    });
            }

            RCLCPP_INFO(get_logger(), "Stereo USB Cam Node Initialized");
        }

        ~StereoUsbCam()
        {
            _left_capture.release();
            _right_capture.release();
        }
    };

} // namespace stereo_usb_cam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_usb_cam::StereoUsbCam)