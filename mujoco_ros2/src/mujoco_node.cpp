#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <robot_msgs/msg/motor_command.hpp>
#include <robot_msgs/msg/motor_estimate.hpp>
#include <robot_msgs/msg/o_drive_config.hpp>
#include <robot_msgs/msg/o_drive_info.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mujoco_ros2/mujoco.hpp"

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

using namespace robot_msgs::msg;

// Motor Node
class MuJoCoMotorNode
{
private:
    rclcpp::Node *_node;
    uint8_t _id = 0;

    struct
    {
        uint32_t state = ODriveConfig::AXIS_STATE_IDLE;
        uint32_t control_mode = MotorCommand::CONTROL_MODE_TORQUE;
        uint32_t input_mode = MotorCommand::INPUT_MODE_PASSTHROUGH;
        float pos_cmd = 0.f;
        float vel_cmd = 0.f;
        float torq_cmd = 0.f;
        float kp = 100.0f;
        float kd = 5.0f;
        float ki = 0.0f;
        float torq_integral = 0.f;
        float pos_est = 0.0;
        float vel_est = 0.0;
        float torq_est = 0.0;
    } _state;

    float _gear_ratio = 1.0;

    rclcpp::Subscription<MotorCommand>::SharedPtr _command_sub;
    rclcpp::Subscription<ODriveConfig>::SharedPtr _config_sub;
    rclcpp::Publisher<MotorEstimate>::SharedPtr _estimate_pub;

    rclcpp::TimerBase::SharedPtr _estimate_timer;

public:
    MuJoCoMotorNode(rclcpp::Node *node, const uint8_t id) : _node(node), _id(id)
    {
        _command_sub = node->create_subscription<MotorCommand>(
            "motor" + std::to_string(id) + "/command", qos_reliable(), std::bind(&MuJoCoMotorNode::command, this, std::placeholders::_1));

        _config_sub = node->create_subscription<ODriveConfig>(
            "motor" + std::to_string(id) + "/config", qos_reliable(), std::bind(&MuJoCoMotorNode::config, this, std::placeholders::_1));

        _estimate_pub = node->create_publisher<MotorEstimate>("motor" + std::to_string(id) + "/estimate", qos_fast());

        double estimate_rate = 50.0;
        _estimate_timer = node->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&MuJoCoMotorNode::estimate, this));

        MuJoCoData.control_functions.push_back(
            std::bind(&MuJoCoMotorNode::controlMotor, this, std::placeholders::_1, std::placeholders::_2));
    }

    void command(const MotorCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(MuJoCoData.mutex);

        if (msg->control_mode != 0)
        {
            _state.control_mode = msg->control_mode;

            if (msg->input_mode != 0)
            {
                _state.input_mode = msg->input_mode;
            }
        }

        _state.pos_cmd = msg->pos_setpoint * _gear_ratio;
        _state.vel_cmd = msg->vel_setpoint * _gear_ratio;
        _state.torq_cmd = msg->torq_setpoint / _gear_ratio;
    }

    void config(const ODriveConfig::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(MuJoCoData.mutex);

        if (msg->axis_state != 0)
        {
            _state.state = msg->axis_state;
        }

        if (msg->gear_ratio != 0.0)
        {
            _gear_ratio = msg->gear_ratio;
        }

        if (msg->pos_gain != 0.0)
        {
            _state.kp = msg->pos_gain;
        }

        if (msg->vel_gain != 0.0)
        {
            _state.kd = msg->vel_gain;
        }

        if (msg->vel_int_gain != 0.0)
        {
            _state.ki = msg->vel_int_gain;
        }
    }

    void estimate()
    {
        MotorEstimate estimate_msg;

        {
            std::lock_guard<std::mutex> lock(MuJoCoData.mutex);
            estimate_msg.pos_estimate = _state.pos_est;
            estimate_msg.vel_estimate = _state.vel_est;
            estimate_msg.torq_estimate = _state.torq_est;
        }

        _estimate_pub->publish(estimate_msg);
    }

    void controlMotor(const mjModel *model, mjData *data)
    {
        std::lock_guard<std::mutex> lock(MuJoCoData.mutex);

        const int joint_id = model->actuator_trnid[_id * 2];
        const int qpos_adr = model->jnt_qposadr[joint_id];
        const int qvel_adr = model->jnt_dofadr[joint_id];
        const mjtNum pos_est = data->qpos[qpos_adr];
        const mjtNum vel_est = data->qvel[qvel_adr];
        const mjtNum torq_est = data->ctrl[_id];

        switch (_state.control_mode)
        {
        case MotorCommand::CONTROL_MODE_POSITION:
        {
            if (std::isnan(_state.pos_cmd) || std::isnan(_state.vel_cmd) || std::isnan(_state.torq_cmd))
            {
                break;
            }

            const mjtNum pos_err = _state.pos_cmd - pos_est;
            const mjtNum vel_err = _state.vel_cmd - vel_est;

            const mjtNum kp = _state.kp;
            const mjtNum kd = _state.kd;
            const mjtNum ki = _state.ki;
            _state.torq_integral += pos_err * ki;
            const mjtNum torq_cmd = _state.torq_cmd + pos_err * kp + vel_err * kd + _state.torq_integral;
            data->ctrl[_id] = torq_cmd;
            break;
        }
        case MotorCommand::CONTROL_MODE_VELOCITY:
        {
            if (std::isnan(_state.vel_cmd) || std::isnan(_state.torq_cmd))
            {
                break;
            }

            const mjtNum vel_err = _state.vel_cmd - vel_est;

            const mjtNum kp = _state.kp;
            const mjtNum torq_cmd = _state.torq_cmd + vel_err * kp;
            data->ctrl[_id] = torq_cmd;
            break;
        }
        case MotorCommand::CONTROL_MODE_TORQUE:
        {
            if (std::isnan(_state.torq_cmd))
            {
                break;
            }

            data->ctrl[_id] = _state.torq_cmd;
            break;
        }
        }

        _state.pos_est = pos_est;
        _state.vel_est = vel_est;
        _state.torq_est = torq_est;
    }
};

// MuJoCo Node
class MuJoCoNode : public rclcpp::Node
{
private:
    std::string _model_path = "/home/ubuntu/ros2_ws/src/mujoco_ros2/models/robot.xml";
    float _frame_rate = 60.0;
    std::string _odom_frame_id = "odom";
    std::string _base_frame_id = "base_link";

    std::vector<std::shared_ptr<MuJoCoMotorNode>> _motor_nodes;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr _clock_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

public:
    MuJoCoNode() : Node("mujoco_node")
    {
        this->declare_parameter("model_path", _model_path);
        this->get_parameter("model_path", _model_path);

        this->declare_parameter("frame_rate", _frame_rate);
        this->get_parameter("frame_rate", _frame_rate);

        this->declare_parameter("odom_frame_id", _odom_frame_id);
        this->get_parameter("odom_frame_id", _odom_frame_id);

        this->declare_parameter("base_frame_id", _base_frame_id);
        this->get_parameter("base_frame_id", _base_frame_id);

        _clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", qos_reliable());
        _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos_reliable());

        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::thread([this]()
                    { runMuJoCo(); })
            .detach();

        RCLCPP_INFO(this->get_logger(), "MuJoCo node initialized.");
    }

    void runMuJoCo()
    {
        initMuJoCo(_model_path);

        MuJoCoData.control_functions.push_back(
            std::bind(&MuJoCoNode::update, this, std::placeholders::_1, std::placeholders::_2));

        const int num_motors = MuJoCoData.model->nu;
        for (int i = 0; i < num_motors; i++)
        {
            _motor_nodes.push_back(std::make_shared<MuJoCoMotorNode>(this, i));
        }

        openMuJoCo(_frame_rate);

        rclcpp::shutdown();
    }

    void update(const mjModel *, mjData *data)
    {
        publishClock(data);
        publishOdometry(data);
    }

    void publishClock(mjData *data)
    {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(int64_t(data->time * 1E9));
        _clock_pub->publish(clock_msg);
    }

    void publishOdometry(mjData *data)
    {

        const double qw = data->qpos[3];
        const double qx = data->qpos[4];
        const double qy = data->qpos[5];
        const double qz = data->qpos[6];

        const double vx_world = data->qvel[0];
        const double vy_world = data->qvel[1];
        const double vz_world = data->qvel[2];

        double lw = qx * vx_world + qy * vy_world + qz * vz_world;
        double lx = qw * vx_world - qy * vz_world + qz * vy_world;
        double ly = qw * vy_world - qz * vx_world + qx * vz_world;
        double lz = qw * vz_world - qx * vy_world + qy * vx_world;

        double vx_body = lw * qx + lx * qw + ly * qz - lz * qy;
        double vy_body = lw * qy - lx * qz + ly * qw + lz * qx;
        double vz_body = lw * qz + lx * qy - ly * qx + lz * qw;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = _odom_frame_id;
        odom_msg.header.stamp = rclcpp::Time(int64_t(data->time * 1E9));

        odom_msg.pose.pose.position.x = data->qpos[0];
        odom_msg.pose.pose.position.y = data->qpos[1];
        odom_msg.pose.pose.position.z = data->qpos[2];

        odom_msg.pose.pose.orientation.w = qw;
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;

        odom_msg.twist.twist.linear.x = vx_body;
        odom_msg.twist.twist.linear.y = vy_body;
        odom_msg.twist.twist.linear.z = vz_body;

        odom_msg.twist.twist.angular.x = data->qvel[3];
        odom_msg.twist.twist.angular.y = data->qvel[4];
        odom_msg.twist.twist.angular.z = data->qvel[5];

        _odom_pub->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom_msg.header.stamp;
        tf_msg.header.frame_id = _odom_frame_id;
        tf_msg.child_frame_id = _base_frame_id;
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

        _tf_broadcaster->sendTransform(tf_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MuJoCoNode>());
    rclcpp::shutdown();
    return 0;
}