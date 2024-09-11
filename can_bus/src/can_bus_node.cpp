#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/can_frame.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define CAN_RECEIVE_RATE 2000.0

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

namespace can_bus
{

    using namespace robot_msgs::msg;

    // CAN Transmitter Node
    class CanBusNode : public rclcpp::Node
    {
    private:
        std::string _interface = "can0";
        int _socket = -1;

        rclcpp::Subscription<CanFrame>::SharedPtr _can_tx_sub;
        rclcpp::Publisher<CanFrame>::SharedPtr _can_rx_pub;

        rclcpp::TimerBase::SharedPtr _receive_timer;

    public:
        CanBusNode(const rclcpp::NodeOptions &options)
            : Node("can_bus_node", options)
        {
            this->declare_parameter("interface", _interface);
            this->get_parameter("interface", _interface);

            _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (_socket < 0)
            {
                throw std::runtime_error("Failed to create CAN socket");
            }

            struct ifreq ifr;
            std::strcpy(ifr.ifr_name, _interface.c_str());
            ioctl(_socket, SIOCGIFINDEX, &ifr);

            struct sockaddr_can addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                throw std::runtime_error("Failed to bind socket to interface");
            }

            _can_tx_sub = this->create_subscription<CanFrame>(
                "can/tx", qos_fast(), std::bind(&CanBusNode::send, this, std::placeholders::_1));

            _can_rx_pub = this->create_publisher<CanFrame>("can/rx", qos_fast());

            _receive_timer = this->create_wall_timer(
                std::chrono::microseconds(time_t(1E6 / CAN_RECEIVE_RATE)),
                std::bind(&CanBusNode::receive, this));

            RCLCPP_INFO(this->get_logger(), "CAN bus node initialized on interface %s", _interface.c_str());
        }

        void send(const CanFrame::UniquePtr msg)
        {
            if (msg->size > CAN_MAX_DLC)
            {
                RCLCPP_ERROR(this->get_logger(), "CAN message too large (Max: %d)", CAN_MAX_DLC);
                return;
            }

            struct can_frame frame;
            frame.can_id = msg->id;
            frame.can_dlc = msg->size;
            std::memcpy(frame.data, msg->data.data(), msg->size);

            if (write(_socket, &frame, sizeof(frame)) < ssize_t(sizeof(frame)))
            {
                RCLCPP_ERROR(this->get_logger(), "CAN buffer full, failed to send frame");
            }
        }

        void receive()
        {
            struct can_frame frame;

            if (read(_socket, &frame, sizeof(frame)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive CAN frame");
                return;
            }

            auto msg = std::make_unique<CanFrame>();
            msg->id = frame.can_id;
            msg->size = frame.can_dlc;
            std::copy(std::begin(frame.data), std::end(frame.data), std::begin(msg->data));

            static int count = 0;
            RCLCPP_INFO(this->get_logger(), "CAN MSG COUNT: %d", ++count);

            _can_rx_pub->publish(std::move(msg));
        }
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(can_bus::CanBusNode)