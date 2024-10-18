#include <rclcpp/rclcpp.hpp>

#include <robot_msgs/msg/can_frame.hpp>
#include <robot_msgs/msg/motor_command.hpp>
#include <robot_msgs/msg/motor_estimate.hpp>
#include <robot_msgs/msg/o_drive_config.hpp>
#include <robot_msgs/msg/o_drive_info.hpp>

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

namespace odrive_ros2
{

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

    using namespace robot_msgs::msg;

    // ODrive Node
    class ODriveCanNode : public rclcpp::Node
    {
    private:
        uint8_t _id = 0;

        float _gear_ratio = 1.0;

        rclcpp::Subscription<CanFrame>::SharedPtr _can_rx_sub;
        rclcpp::Publisher<CanFrame>::SharedPtr _can_tx_pub;

        rclcpp::Subscription<MotorCommand>::SharedPtr _command_sub;
        rclcpp::Publisher<MotorEstimate>::SharedPtr _estimate_pub;
        rclcpp::Subscription<ODriveConfig>::SharedPtr _config_sub;
        rclcpp::Publisher<ODriveInfo>::SharedPtr _info_pub;

        rclcpp::TimerBase::SharedPtr _estimate_timer;
        rclcpp::TimerBase::SharedPtr _info_timer;

        MotorEstimate _estimate_msg;
        ODriveInfo _info_msg;
        MotorCommand _command_msg;

        inline uint32_t getArbitrationID(const uint8_t can_id, const uint8_t cmd_id) const
        {
            return static_cast<uint32_t>(can_id << 5 | cmd_id);
        }

        inline uint8_t getCanID(const uint32_t arb_id) const
        {
            return static_cast<uint8_t>(arb_id >> 5 & 0b111111);
        }

        inline uint8_t getCommandID(const uint32_t arb_id) const
        {
            return static_cast<uint8_t>(arb_id & 0b11111);
        }

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
            this->declare_parameter("id", _id);
            this->get_parameter("id", _id);

            this->declare_parameter("gear_ratio", _gear_ratio);
            this->get_parameter("gear_ratio", _gear_ratio);

            _can_rx_sub = this->create_subscription<CanFrame>(
                "can/rx", qos_fast(), std::bind(&ODriveCanNode::receive, this, std::placeholders::_1));

            _can_tx_pub = this->create_publisher<CanFrame>("can/tx", qos_reliable());

            _command_sub = this->create_subscription<MotorCommand>(
                "odrive/command", qos_fast(), std::bind(&ODriveCanNode::command, this, std::placeholders::_1));

            _config_sub = this->create_subscription<ODriveConfig>(
                "odrive/config", qos_reliable(), std::bind(&ODriveCanNode::config, this, std::placeholders::_1));

            _estimate_pub = this->create_publisher<MotorEstimate>("odrive/estimate", qos_fast());

            _info_pub = this->create_publisher<ODriveInfo>("odrive/info", qos_fast());

            double estimate_rate = 50.0;
            _estimate_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
                std::bind(&ODriveCanNode::estimate, this));

            double info_rate = 20.0;
            _info_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / info_rate)),
                std::bind(&ODriveCanNode::info, this));

            RCLCPP_INFO(this->get_logger(), "ODrive node initialized with ID %d", _id);
        }

        void receive(const CanFrame::UniquePtr msg)
        {
            const uint8_t can_id = getCanID(msg->id);
            const uint8_t cmd_id = getCommandID(msg->id);

            if (can_id != _id)
            {
                return;
            }

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

        void command(const MotorCommand::UniquePtr msg)
        {
            if (msg->control_mode != _command_msg.control_mode)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_CONTROL_MODE);

                uint8_t data[8];
                std::memcpy(data, &msg->control_mode, sizeof(msg->control_mode));

                if (msg->input_mode != 0)
                {
                    std::memcpy(data + 4, &msg->input_mode, sizeof(msg->input_mode));
                }
                else
                {
                    // input mode defaults to 1 (passthrough)
                    std::memset(data + 4, uint32_t(1), sizeof(uint32_t));
                }

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            }

            const float pos_rev = msg->pos_setpoint * _gear_ratio / (2.0f * M_PI);
            const float vel_rev = msg->vel_setpoint * _gear_ratio / (2.0f * M_PI);
            const float torq_N = msg->torq_setpoint / _gear_ratio;

            switch (msg->control_mode)
            {
            case MotorCommand::CONTROL_MODE_POSITION:
            {
                if (!std::isfinite(pos_rev))
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid position setpoint (%.3f)", msg->pos_setpoint);
                    return;
                }

                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_POSITION);

                const int16_t vel_ff_int = (int16_t)(vel_rev * 1E3F);
                const int16_t torq_ff_int = (int16_t)(torq_N * 1E3F);

                uint8_t data[8];
                std::memcpy(data, &pos_rev, sizeof(pos_rev));
                std::memcpy(data + 4, &vel_ff_int, sizeof(vel_ff_int));
                std::memcpy(data + 6, &torq_ff_int, sizeof(torq_ff_int));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
                break;
            }
            case MotorCommand::CONTROL_MODE_VELOCITY:
            {
                if (!std::isfinite(vel_rev))
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid velocity setpoint (%.3f)", msg->vel_setpoint);
                    return;
                }

                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_VELOCITY);

                uint8_t data[8];
                std::memcpy(data, &vel_rev, sizeof(vel_rev));
                std::memcpy(data + 4, &torq_N, sizeof(torq_N));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
                break;
            }
            case MotorCommand::CONTROL_MODE_TORQUE:
            {
                if (!std::isfinite(torq_N))
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid torque setpoint (%.3f)", msg->torq_setpoint);
                    return;
                }

                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_TORQUE);

                uint8_t data[4];
                std::memcpy(data, &torq_N, sizeof(torq_N));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
                break;
            }
            }

            _command_msg = *msg;
        }

        void config(const ODriveConfig::UniquePtr msg)
        {
            if (msg->axis_state != 0)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_AXIS_STATE);

                uint8_t data[4];
                std::memcpy(data, &msg->axis_state, sizeof(msg->axis_state));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));

                RCLCPP_INFO(this->get_logger(), "Set axis state to %d", int(msg->axis_state));
            }

            if (msg->gear_ratio != 0.0)
            {
                _gear_ratio = msg->gear_ratio;

                RCLCPP_INFO(this->get_logger(), "Set gear ratio to %.3f", msg->gear_ratio);
            }

            if (msg->vel_limit != 0.0 && msg->curr_limit != 0.0)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_LIMITS);

                uint8_t data[8];
                std::memcpy(data, &msg->vel_limit, sizeof(msg->vel_limit));
                std::memcpy(data + 4, &msg->curr_limit, sizeof(msg->curr_limit));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            }
            else if (msg->vel_limit + msg->curr_limit != 0.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Both velocity and current limits must be set (Skipping)");
            }

            if (msg->pos_gain != 0.0)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_POS_GAIN);

                uint8_t data[4];
                std::memcpy(data, &msg->pos_gain, sizeof(msg->pos_gain));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            }

            if (msg->vel_gain != 0.0 && msg->vel_int_gain != 0.0)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::SET_VEL_GAINS);

                uint8_t data[8];
                std::memcpy(data, &msg->vel_gain, sizeof(msg->vel_gain));
                std::memcpy(data + 4, &msg->vel_int_gain, sizeof(msg->vel_int_gain));

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));
            }
            else if (msg->vel_gain + msg->vel_int_gain != 0.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Both velocity gains must be set (Skipping)");
            }

            if (msg->clear_errors)
            {
                const uint32_t arb_id = getArbitrationID(_id, CanCommandID::CLEAR_ERRORS);

                uint8_t data[4];
                std::memset(data, 0, 4);

                _can_tx_pub->publish(createFrame(arb_id, data, sizeof(data)));

                RCLCPP_INFO(this->get_logger(), "Cleared errors");
            }
        }

        void estimate()
        {
            _estimate_pub->publish(std::make_unique<MotorEstimate>(_estimate_msg));
        }

        void info()
        {
            _info_pub->publish(_info_msg);
        }
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odrive_ros2::ODriveCanNode>());
    rclcpp::shutdown();
    return 0;
}