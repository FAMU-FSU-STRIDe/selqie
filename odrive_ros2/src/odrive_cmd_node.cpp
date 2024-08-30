#include <rclcpp/rclcpp.hpp>

#include <robot_msgs/msg/motor_command.hpp>
#include <robot_msgs/msg/motor_config.hpp>

using namespace robot_msgs::msg;

class ODriveCommandNode
{
private:
    rclcpp::Node *_node;
    std::string _cmd = "";
    std::string _args = "";
    rclcpp::Publisher<MotorConfig>::SharedPtr _config_pub;
    rclcpp::Publisher<MotorCommand>::SharedPtr _command_pub;

    void idle()
    {
        MotorConfig cfg;
        cfg.axis_state = MotorConfig::AXIS_STATE_IDLE;
        _config_pub->publish(cfg);
        RCLCPP_INFO(_node->get_logger(), "Motor state set to IDLE");
    }

    void ready()
    {
        MotorConfig cfg;
        cfg.axis_state = MotorConfig::AXIS_STATE_CLOSED_LOOP_CONTROL;
        _config_pub->publish(cfg);
        RCLCPP_INFO(_node->get_logger(), "Motor state set to CLOSED_LOOP_CONTROL");
    }

    void zero()
    {
        MotorCommand cmd;
        cmd.control_mode = MotorCommand::CONTROL_MODE_POSITION;
        cmd.pos_setpoint = 0.0;
        _command_pub->publish(cmd);
        RCLCPP_INFO(_node->get_logger(), "Motor position set to 0.0");
    }

    void clear()
    {
        MotorConfig cfg;
        cfg.clear_errors = true;
        _config_pub->publish(cfg);
        RCLCPP_INFO(_node->get_logger(), "Motor error CLEARED");
    }

public:
    ODriveCommandNode(rclcpp::Node *node)
        : _node(node)
    {
        node->declare_parameter("cmd", _cmd);
        node->get_parameter("cmd", _cmd);

        node->declare_parameter("args", _args);
        node->get_parameter("args", _args);

        _config_pub = _node->create_publisher<MotorConfig>("config", 10);
        _command_pub = _node->create_publisher<MotorCommand>("command", 10);
    }

    void parse()
    {
        if (_cmd == "idle")
        {
            idle();
        }
        else if (_cmd == "ready")
        {
            ready();
        }
        else if (_cmd == "zero")
        {
            zero();
        }
        else if (_cmd == "clear")
        {
            clear();
        }
        else
        {
            RCLCPP_ERROR(_node->get_logger(), "Command '%s' not found", _cmd.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("odrive_cmd_node");
    ODriveCommandNode odrive_cmd(node.get());
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    odrive_cmd.parse();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}