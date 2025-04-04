#include <rclcpp/rclcpp.hpp>

// Headers for custom actuation ROS messages
#include <actuation_msgs/msg/can_frame.hpp>
#include <actuation_msgs/msg/motor_command.hpp>
#include <actuation_msgs/msg/motor_estimate.hpp>
#include <actuation_msgs/msg/motor_config.hpp>
#include <actuation_msgs/msg/motor_info.hpp>

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

// CAN Command IDs for ODrive
// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol
enum CanCommandID
{
    HEARTBEAT = 0x001,
    GET_ENCODER_ESTIMATES = 0x009,
    GET_IQ = 0x014,
    GET_TEMPERATURE = 0x015,
    GET_BUS_VOLTAGE_CURRENT = 0x017,
    GET_TORQUES = 0x01C,
    SET_AXIS_STATE = 0x007,
    SET_CONTROL_MODE = 0x00b,
    SET_LIMITS = 0x00f,
    SET_POS_GAIN = 0x01a,
    SET_VEL_GAINS = 0x01b,
    SET_POSITION = 0x00c,
    SET_VELOCITY = 0x00d,
    SET_TORQUE = 0x00e,
    CLEAR_ERRORS = 0x018
};

// Use namespaces to simplify code
using namespace actuation_msgs::msg;

/*
 * ODrive Controller Area Network (CAN) Node
 * This node handles converting Motor messages to CAN frames and vice versa
 */
class ODriveCanNode : public rclcpp::Node
{
private:
    uint8_t _id = 0; // ODrive device ID, default is 0

    float _gear_ratio = 1.0; // Gear ratio for the motor, default is 1.0 (no gearing)

    rclcpp::Subscription<CanFrame>::SharedPtr _can_rx_sub; // Subscription for receiving CAN frames
    rclcpp::Publisher<CanFrame>::SharedPtr _can_tx_pub;    // Publisher for transmitting CAN frames

    rclcpp::Subscription<MotorCommand>::SharedPtr _command_sub; // Subscription for motor commands
    rclcpp::Publisher<MotorEstimate>::SharedPtr _estimate_pub;  // Publisher for motor estimates
    rclcpp::Subscription<MotorConfig>::SharedPtr _config_sub;   // Subscription for motor configuration
    rclcpp::Publisher<MotorInfo>::SharedPtr _info_pub;          // Publisher for motor info

    rclcpp::TimerBase::SharedPtr _estimate_timer; // Timer for publishing motor estimates
    rclcpp::TimerBase::SharedPtr _info_timer;     // Timer for publishing motor info

    MotorEstimate _estimate_msg; // Current motor estimates to be published
    MotorInfo _info_msg;         // Current motor info to be published
    MotorCommand _command_msg;   // Last received motor command message

    /*
     * Function to encode the Arbitration ID from the CAN ID and Command ID
     */
    inline uint32_t getArbitrationID(const uint8_t can_id, const uint8_t cmd_id) const
    {
        return static_cast<uint32_t>(can_id << 5 | cmd_id);
    }

    /*
     * Function to decode the CAN ID from the Arbitration ID
     */
    inline uint8_t getCanID(const uint32_t arb_id) const
    {
        return static_cast<uint8_t>(arb_id >> 5 & 0b111111);
    }

    /*
     * Function to decode the Command ID from the Arbitration ID
     */
    inline uint8_t getCommandID(const uint32_t arb_id) const
    {
        return static_cast<uint8_t>(arb_id & 0b11111);
    }

    /*
     * Function to create a CAN frame from the given arbitration ID, data, and length
     */
    inline CanFrame::UniquePtr createFrame(const uint32_t arb_id, const uint8_t *data, const size_t len) const
    {
        auto frame = std::make_unique<CanFrame>();
        frame->id = arb_id;
        frame->size = len;
        std::copy(data, data + len, frame->data.begin());
        return frame;
    }

public:
    ODriveCanNode() : Node("odrive_node")
    {
        // Get ROS parameters
        this->declare_parameter("id", _id);
        this->get_parameter("id", _id);

        this->declare_parameter("gear_ratio", _gear_ratio);
        this->get_parameter("gear_ratio", _gear_ratio);

        this->declare_parameter("estimate_rate", 50.0);
        double estimate_rate = this->get_parameter("estimate_rate").as_double();

        this->declare_parameter("info_rate", 20.0);
        double info_rate = this->get_parameter("info_rate").as_double();

        // Create CAN RX subscription and TX publisher
        _can_rx_sub = this->create_subscription<CanFrame>(
            "can/rx", qos_fast(), std::bind(&ODriveCanNode::receive, this, std::placeholders::_1));

        _can_tx_pub = this->create_publisher<CanFrame>("can/tx", qos_reliable());

        // Create motor publishers and subscriptions
        _command_sub = this->create_subscription<MotorCommand>(
            "odrive/command", qos_reliable(), std::bind(&ODriveCanNode::command, this, std::placeholders::_1));

        _estimate_pub = this->create_publisher<MotorEstimate>("odrive/estimate", qos_fast());

        _config_sub = this->create_subscription<MotorConfig>(
            "odrive/config", qos_reliable(), std::bind(&ODriveCanNode::config, this, std::placeholders::_1));

        _info_pub = this->create_publisher<MotorInfo>("odrive/info", qos_fast());

        // Create estimate and info timers
        _estimate_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&ODriveCanNode::estimate, this));

        _info_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / info_rate)),
            std::bind(&ODriveCanNode::info, this));

        RCLCPP_INFO(this->get_logger(), "ODrive CAN Node Initialized with ID %d", _id);
    }

    /*
     * Function to receive CAN frames from the CAN bus.
     * Callback for the CAN RX subscription.
     */
    void receive(const CanFrame::UniquePtr msg)
    {
        // Convert frame arbitration id to can id and command id
        const uint8_t can_id = getCanID(msg->id);
        const uint8_t cmd_id = getCommandID(msg->id);

        // Check if the message is for this ODrive node
        if (can_id != _id)
        {
            return;
        }

        // Parse the frame data based on the command id and update the appropriate message
        switch (cmd_id)
        {
        case CanCommandID::HEARTBEAT:
            // Heartbeat
            uint32_t axis_error;
            uint8_t axis_state;
            std::memcpy(&axis_error, msg->data.data(), sizeof(axis_error));
            std::memcpy(&axis_state, msg->data.data() + 4, sizeof(axis_state));
            {
                _info_msg.axis_error = axis_error;
                _info_msg.axis_state = axis_state;
            }
            break;
        case CanCommandID::GET_IQ:
            // Get_Iq
            float iq_setpoint, iq_measured;
            std::memcpy(&iq_setpoint, msg->data.data(), sizeof(iq_setpoint));
            std::memcpy(&iq_measured, msg->data.data() + 4, sizeof(iq_measured));
            {
                _info_msg.iq_setpoint = iq_setpoint;
                _info_msg.iq_measured = iq_measured;
            }
            break;
        case CanCommandID::GET_TEMPERATURE:
            // Get_Temperature
            float fet_temperature, motor_temperature;
            std::memcpy(&fet_temperature, msg->data.data(), sizeof(fet_temperature));
            std::memcpy(&motor_temperature, msg->data.data() + 4, sizeof(motor_temperature));
            {
                _info_msg.fet_temperature = fet_temperature;
                _info_msg.motor_temperature = motor_temperature;
            }
            break;
        case CanCommandID::GET_BUS_VOLTAGE_CURRENT:
            // Get_Bus_Voltage_Current
            float bus_voltage, bus_current;
            std::memcpy(&bus_voltage, msg->data.data(), sizeof(bus_voltage));
            std::memcpy(&bus_current, msg->data.data() + 4, sizeof(bus_current));
            {
                _info_msg.bus_voltage = bus_voltage;
                _info_msg.bus_current = bus_current;
            }
            break;
        case CanCommandID::GET_ENCODER_ESTIMATES:
            // Get_Encoder_Estimates
            float pos_estimate, vel_estimate;
            std::memcpy(&pos_estimate, msg->data.data(), sizeof(pos_estimate));
            std::memcpy(&vel_estimate, msg->data.data() + 4, sizeof(vel_estimate));
            {
                _estimate_msg.pos_estimate = pos_estimate * (2.0f * M_PI) / _gear_ratio;
                _estimate_msg.vel_estimate = vel_estimate * (2.0f * M_PI) / _gear_ratio;
            }
            break;
        case CanCommandID::GET_TORQUES:
            // Get_Torques
            float torq_estimate;
            std::memcpy(&torq_estimate, msg->data.data(), sizeof(torq_estimate));
            {
                _estimate_msg.torq_estimate = torq_estimate * _gear_ratio;
            }
            break;
        }
    }

    /*
     * Function to handle motor command messages.
     * Callback for the motor command subscription.
     */
    void command(const MotorCommand::UniquePtr msg)
    {
        // Check for an update to the control mode
        if (msg->control_mode != _command_msg.control_mode)
        {
            // Get the arbitration id
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_CONTROL_MODE);

            // Copy the bytes into the message data
            uint8_t data[8];
            std::memcpy(data, &msg->control_mode, sizeof(msg->control_mode));

            // Check if the input mode is set
            if (msg->input_mode != 0)
            {
                std::memcpy(data + 4, &msg->input_mode, sizeof(msg->input_mode));
            }
            else
            {
                // input mode defaults to 1 (passthrough)
                std::memset(data + 4, uint32_t(1), sizeof(uint32_t));
            }

            // Publish update to control and input mode
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
        }

        // Convert the setpoints from radians to revolutions and apply gear ratio
        const float pos_rev = msg->pos_setpoint * _gear_ratio / (2.0f * M_PI);
        const float vel_rev = msg->vel_setpoint * _gear_ratio / (2.0f * M_PI);
        const float torq_N = msg->torq_setpoint / _gear_ratio;

        // Motor command is based on the current control mode
        switch (msg->control_mode)
        {
        case MotorCommand::CONTROL_MODE_POSITION:
        {
            // Check for valid position setpoint
            if (!std::isfinite(pos_rev))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid position setpoint (%.3f)", msg->pos_setpoint);
                return;
            }

            // Get the arbitration id for position command
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_POSITION);

            // Convert position to int16_t for CAN frame
            // Based on ODrive CAN protocol documentation
            const int16_t vel_ff_int = (int16_t)(vel_rev * 1E3F);
            const int16_t torq_ff_int = (int16_t)(torq_N * 1E3F);

            // Copy command into the data array
            uint8_t data[8];
            std::memcpy(data, &pos_rev, sizeof(pos_rev));
            std::memcpy(data + 4, &vel_ff_int, sizeof(vel_ff_int));
            std::memcpy(data + 6, &torq_ff_int, sizeof(torq_ff_int));

            // Publish position command frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            break;
        }
        case MotorCommand::CONTROL_MODE_VELOCITY:
        {
            // Check for valid velocity setpoint
            if (!std::isfinite(vel_rev))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid velocity setpoint (%.3f)", msg->vel_setpoint);
                return;
            }

            // Get the arbitration id for velocity command
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_VELOCITY);

            // Copy command into the data array
            uint8_t data[8];
            std::memcpy(data, &vel_rev, sizeof(vel_rev));
            std::memcpy(data + 4, &torq_N, sizeof(torq_N));

            // Publish velocity command frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            break;
        }
        case MotorCommand::CONTROL_MODE_TORQUE:
        {
            // Check for valid torque setpoint
            if (!std::isfinite(torq_N))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid torque setpoint (%.3f)", msg->torq_setpoint);
                return;
            }

            // Get the arbitration id for torque command
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_TORQUE);

            // Copy command into the data array
            uint8_t data[4];
            std::memcpy(data, &torq_N, sizeof(torq_N));

            // Publish torque command frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            break;
        }
        }

        // Update the last command message
        // Used to track changes to control/input mode
        _command_msg = *msg;
    }

    /*
     * Function to handle motor configuration messages.
     * Callback for the motor configuration subscription.
     */
    void config(const MotorConfig::UniquePtr msg)
    {
        // Check if the axis state is set
        // Only update if so
        if (msg->axis_state != 0)
        {
            // Get the arbitration ID for setting axis state
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_AXIS_STATE);

            // Copy state into the data array
            uint8_t data[4];
            std::memcpy(data, &msg->axis_state, sizeof(msg->axis_state));

            // Publish the axis state frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));

            // Give feedback to the user
            RCLCPP_INFO(this->get_logger(), "Set axis state to %d", int(msg->axis_state));
        }

        // Check if the gear ratio is set
        if (msg->gear_ratio != 0.0)
        {
            // Update the internal gear ratio
            _gear_ratio = msg->gear_ratio;

            // Give feedback to the user
            RCLCPP_INFO(this->get_logger(), "Set gear ratio to %.3f", msg->gear_ratio);
        }

        // Check if the velocity and current limits are set
        // Both must be non-zero to send the limits
        if (msg->vel_limit != 0.0 && msg->curr_limit != 0.0)
        {
            // Get the arbitration ID for setting limits
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_LIMITS);

            // Copy the limits into the data array
            uint8_t data[8];
            std::memcpy(data, &msg->vel_limit, sizeof(msg->vel_limit));
            std::memcpy(data + 4, &msg->curr_limit, sizeof(msg->curr_limit));

            // Publish the limits frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
        }
        else if (msg->vel_limit + msg->curr_limit != 0.0)
        {
            // Feedback if either limit is zero
            RCLCPP_ERROR(this->get_logger(), "Both velocity and current limits must be set (Skipping)");
        }

        // Check if the position gain is set
        if (msg->pos_gain != 0.0)
        {
            // Get the arbitration ID for setting position gain
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_POS_GAIN);

            // Copy the position gain into the data array
            uint8_t data[4];
            std::memcpy(data, &msg->pos_gain, sizeof(msg->pos_gain));

            // Publish the position gain frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
        }

        // Check if the velocity gains are set
        if (msg->vel_gain != 0.0 && msg->vel_int_gain != 0.0)
        {
            // Get the arbitration ID for setting velocity gains
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_VEL_GAINS);

            // Copy the velocity gains into the data array
            uint8_t data[8];
            std::memcpy(data, &msg->vel_gain, sizeof(msg->vel_gain));
            std::memcpy(data + 4, &msg->vel_int_gain, sizeof(msg->vel_int_gain));

            // Publish the velocity gains frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
        }
        else if (msg->vel_gain + msg->vel_int_gain != 0.0)
        {
            // Feedback if either velocity gain is zero
            RCLCPP_ERROR(this->get_logger(), "Both velocity gains must be set (Skipping)");
        }

        // Check if the clear errors flag is set
        if (msg->clear_errors)
        {
            // Get the arbitration ID for clearing errors
            const uint32_t arb_id = getArbitrationID(_id, CanCommandID::CLEAR_ERRORS);

            // Clear errors by sending a frame with zero data
            uint8_t data[4];
            std::memset(data, 0, 4);

            // Publish the clear errors frame
            _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));

            // Give feedback to the user
            RCLCPP_INFO(this->get_logger(), "Cleared errors");
        }
    }

    /*
     * Function to publish the motor estimates at a regular interval
     * This is called by the estimate timer.
     */
    void estimate()
    {
        // Publish the current motor estimates
        _estimate_pub->publish(_estimate_msg);
    }

    /*
     * Function to publish the motor info at a regular interval
     * This is called by the info timer.
     */
    void info()
    {
        // Publish the current motor info
        _info_pub->publish(_info_msg);
    }
};

// Entry point for the ODrive CAN Node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveCanNode>());
    rclcpp::shutdown();
    return 0;
}