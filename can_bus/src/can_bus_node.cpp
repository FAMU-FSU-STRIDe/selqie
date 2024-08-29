#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/can_frame.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <thread>

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

using namespace robot_msgs::msg;

// CAN Transmitter Node
class CanBusNode : public rclcpp::Node
{
private:
    std::string _interface = "can0";
    int _socket = -1;

    rclcpp::Subscription<CanFrame>::SharedPtr _can_tx_sub;
    rclcpp::Publisher<CanFrame>::SharedPtr _can_rx_pub;

public:
    CanBusNode() : Node("can_bus_node")
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

        std::thread([this]()
                    { while (rclcpp::ok()) receive(); })
            .detach();

        RCLCPP_INFO(this->get_logger(), "CAN bus node initialized on interface %s", _interface.c_str());
    }

    void send(const CanFrame::SharedPtr msg)
    {
        if (msg->id > CAN_MAX_DLEN)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN ID is out of range (0x%03X)", msg->id);
            return;
        }

        struct can_frame frame;
        frame.can_id = msg->id;
        frame.can_dlc = msg->size;
        std::memcpy(frame.data, msg->data.data(), msg->size);

        if (write(_socket, &frame, sizeof(frame)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
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

        auto msg = CanFrame();
        msg.id = frame.can_id;
        msg.size = frame.can_dlc;
        msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);

        _can_rx_pub->publish(msg);
    }
};

// Entry point
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanBusNode>());
    rclcpp::shutdown();
    return 0;
}