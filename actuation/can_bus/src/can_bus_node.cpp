#include <rclcpp/rclcpp.hpp>
#include <actuation_msgs/msg/can_frame.hpp>

// Headers for CAN bus communication on Linux
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

// Maximum polling rate of the CAN bus
// Each CAN frame is 144 bits, and the bus supports a maximum of 1 Mbps
// Therefore, the maximum receive rate is 1Mbps / 144bpf = 6945 frames per second
// We set a conservative rate of 5000 frames per second to avoid overloading the CPU
#define CAN_RECEIVE_RATE 5000.0

/*
 * Fast Quality of Service Protocol
 * Messages can be dropped if neccessary to maintain performance
 * Used for high-frequency data like sensor readings
 */
static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

/*
 * Reliable Quality of Service Protocol
 * Ensures that messages are delivered reliably, even if it means delaying them
 * Used for critical commands or control messages where data loss is unacceptable
 */
static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

// Use namespaces to simplify code
using namespace actuation_msgs::msg;

/*
 * Controller Area Network (CAN) Bus Node
 * This node handles sending and receiving CAN frames over a specified CAN interfaces
 */
class CanBusNode : public rclcpp::Node
{
private:
    std::string _interface = "can0"; // CAN interface name, default is "can0"
    int _socket = -1;                // CAN socket file descriptor

    rclcpp::Subscription<CanFrame>::SharedPtr _can_tx_sub; // Subscription for transmitting CAN frames
    rclcpp::Publisher<CanFrame>::SharedPtr _can_rx_pub;    // Publisher for received CAN frames

    rclcpp::TimerBase::SharedPtr _poll_timer; // Timer for polling the CAN bus

public:
    CanBusNode() : Node("can_bus_node")
    {
        // Get ROS parameters
        this->declare_parameter("interface", _interface);
        this->get_parameter("interface", _interface);

        // Create the CAN socket
        _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        // Check if the socket was created successfully
        if (_socket < 0)
        {
            throw std::runtime_error("Failed to create CAN socket");
        }

        // Set the interface for the CAN socket
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, _interface.c_str());
        ioctl(_socket, SIOCGIFINDEX, &ifr);

        // Set the CAN interface index in the sockaddr_can structure
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        // Bind the socket to the CAN interface
        if (bind(_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            throw std::runtime_error("Failed to bind socket to interface");
        }

        // Create the subscription for transmitting CAN frames
        _can_tx_sub = this->create_subscription<CanFrame>(
            "can/tx", qos_reliable(), std::bind(&CanBusNode::send, this, std::placeholders::_1));

        // Create the publisher for receiving CAN frames
        _can_rx_pub = this->create_publisher<CanFrame>("can/rx", qos_fast());

        // Create the timer for polling the CAN bus
        _poll_timer = this->create_wall_timer(
            std::chrono::microseconds(time_t(1E6 / CAN_RECEIVE_RATE)),
            std::bind(&CanBusNode::receive, this));

        RCLCPP_INFO(this->get_logger(), "CAN Bus Node Initialized on Interface %s", _interface.c_str());
    }

    /*
     * Function to send a CAN frame on the CAN bus.
     * Callback for the CAN transmit subscription.
     */
    void send(const CanFrame::UniquePtr msg)
    {
        // Check if the message is too large
        if (msg->size > CAN_MAX_DLC)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN message too large (Max: %d)", CAN_MAX_DLC);
            return;
        }

        // Create the CAN frame to send
        struct can_frame frame;
        frame.can_id = msg->id;
        frame.can_dlc = msg->size;
        std::memcpy(frame.data, msg->data.data(), msg->size);

        // Send the CAN frame
        if (write(_socket, &frame, sizeof(frame)) < ssize_t(sizeof(frame)))
        {
            RCLCPP_ERROR(this->get_logger(), "CAN buffer full, failed to send frame");
        }
    }

    /*
     * Function to receive CAN frames from the CAN bus.
     * Called by the poll timer.
     */
    void receive()
    {
        // Read a CAN frame from the socket
        // This is a blocking call, it will wait until a frame is available
        struct can_frame frame;
        if (read(_socket, &frame, sizeof(frame)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive CAN frame");
            return;
        }

        // Convert frame into ROS message
        auto msg = std::make_unique<CanFrame>();
        msg->id = frame.can_id;
        msg->size = frame.can_dlc;
        std::copy(std::begin(frame.data), std::end(frame.data), std::begin(msg->data));

        // Publish the received CAN frame
        _can_rx_pub->publish(std::move(msg));
    }
};

// Entry point for the CAN Bus Node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanBusNode>());
    rclcpp::shutdown();
    return 0;
}