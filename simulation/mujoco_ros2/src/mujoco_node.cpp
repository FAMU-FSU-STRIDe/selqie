#include "mujoco_ros2/mujoco.hpp"

#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
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

/*
 * MuJoCo Motor Control Model
 * This model handles the control of a motor for different control modes
 * including position, velocity, and torque control.
 */
struct MotorControlModel
{
    mjtNum pos_cmd = 0.f;       // Position command
    mjtNum vel_cmd = 0.f;       // Velocity command
    mjtNum torq_cmd = 0.f;      // Torque command
    mjtNum kp = 100.0f;         // Position gain
    mjtNum kd = 5.0f;           // Velocity gain
    mjtNum ki = 0.0f;           // Integral gain
    mjtNum torq_integral = 0.f; // Torque integral
    mjtNum pos_est = 0.0;       // Estimated position
    mjtNum vel_est = 0.0;       // Estimated velocity
    mjtNum torq_est = 0.0;      // Estimated torque

    /*
     * Calculate position control command using PID control
     */
    mjtNum calculate_position_control()
    {
        // Calculate errors
        const mjtNum pos_err = pos_cmd - pos_est;
        const mjtNum vel_err = vel_cmd - vel_est;
        torq_integral += pos_err * ki;

        // Calculate control command
        return torq_cmd + pos_err * kp + vel_err * kd + torq_integral;
    }

    /*
     * Calculate velocity control command using PD control
     */
    mjtNum calculate_velocity_control()
    {
        // Calculate error
        const mjtNum vel_err = vel_cmd - vel_est;

        // Calculate control command
        return torq_cmd + kp * vel_err;
    }

    /*
     * Calculate torque control command
     */
    mjtNum calculate_torque_control()
    {
        // Return torque command
        return torq_cmd;
    }
};

// Use namespaces to simplify code
using namespace actuation_msgs::msg;

/*
 * MuJoCo Motor Node
 * This node handles converting Motor messages to MuJoCo control commands
 * for individual motors.
 */
class MuJoCoMotorNode
{
private:
    const uint8_t _id = 0; // Motor ID

    uint32_t _state = MotorConfig::AXIS_STATE_IDLE;              // Motor state
    uint32_t _control_mode = MotorCommand::CONTROL_MODE_TORQUE;  // Control mode
    uint32_t _input_mode = MotorCommand::INPUT_MODE_PASSTHROUGH; // Input mode

    float _gear_ratio = 1.0; // Gear ratio for the motor, default is 1.0 (no gearing)

    MotorControlModel _model; // Motor control model

    rclcpp::Subscription<MotorCommand>::SharedPtr _command_sub; // Subscription for motor commands
    rclcpp::Subscription<MotorConfig>::SharedPtr _config_sub;   // Subscription for motor configuration
    rclcpp::Publisher<MotorEstimate>::SharedPtr _estimate_pub;  // Publisher for motor estimates
    rclcpp::TimerBase::SharedPtr _estimate_timer;               // Timer for publishing motor estimates

    std::mutex _mutex; // Mutex for thread safety

    /*
     * Callbacks for handling motor command messages
     */
    void _command_callback(const MotorCommand::SharedPtr msg)
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(_mutex);

        if (msg->control_mode != 0)
        {
            // Update control mode if it is set
            _control_mode = msg->control_mode;

            if (msg->input_mode != 0)
            {
                // Update input mode if it is set
                _input_mode = msg->input_mode;
            }
        }

        // Apply gear ratio to setpoints and update model commands
        _model.pos_cmd = msg->pos_setpoint * _gear_ratio;
        _model.vel_cmd = msg->vel_setpoint * _gear_ratio;
        _model.torq_cmd = msg->torq_setpoint / _gear_ratio;
    }

    /*
     * Callbacks for handling motor configuration messages
     */
    void _config_callback(const MotorConfig::SharedPtr msg)
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(_mutex);

        // Update motor configuration parameters if they are set
        _state = msg->axis_state == 0 ? _state : msg->axis_state;
        _gear_ratio = msg->gear_ratio == 0.0 ? _gear_ratio : msg->gear_ratio;
        _model.kp = msg->pos_gain == 0.0 ? _model.kp : msg->pos_gain;
        _model.kd = msg->vel_gain == 0.0 ? _model.kd : msg->vel_gain;
        _model.ki = msg->vel_int_gain == 0.0 ? _model.ki : msg->vel_int_gain;
    }

    /*
     * Callback for publishing motor estimates
     */
    void _estimate_callback()
    {
        // Lock the mutex to ensure thread safety
        std::unique_lock<std::mutex> lock(_mutex);

        // Create the motor estimate message
        MotorEstimate estimate_msg;
        estimate_msg.pos_estimate = _model.pos_est;
        estimate_msg.vel_estimate = _model.vel_est;
        estimate_msg.torq_estimate = _model.torq_est;

        // Unlock the mutex before publishing
        lock.unlock();

        // Publish the motor estimate message
        _estimate_pub->publish(estimate_msg);
    }

    /*
     * Callback for controlling the motor
     * This function is called during the MuJoCo simulation loop
     */
    void _motor_control_callback(const mjModel *model, mjData *data)
    {
        // Get indexing values for the motor
        const int joint_id = model->actuator_trnid[_id * 2];
        const int qpos_adr = model->jnt_qposadr[joint_id];
        const int qvel_adr = model->jnt_dofadr[joint_id];

        // Update the current position, velocity, and torque estimates
        _model.pos_est = data->qpos[qpos_adr];
        _model.vel_est = data->qvel[qvel_adr];
        _model.torq_est = data->ctrl[_id];

        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(_mutex);

        switch (_control_mode)
        {
        case MotorCommand::CONTROL_MODE_POSITION:
        {
            // Make sure all commands are valid
            if (std::isnan(_model.pos_cmd) || std::isnan(_model.vel_cmd) || std::isnan(_model.torq_cmd))
            {
                break;
            }

            // Calculate the torque command using PID control
            data->ctrl[_id] = _model.calculate_position_control();
            break;
        }
        case MotorCommand::CONTROL_MODE_VELOCITY:
        {
            // Make sure all commands are valid
            if (std::isnan(_model.vel_cmd) || std::isnan(_model.torq_cmd))
            {
                break;
            }

            // Calculate the torque command using PD control
            data->ctrl[_id] = _model.calculate_velocity_control();
            break;
        }
        case MotorCommand::CONTROL_MODE_TORQUE:
        {
            // Make sure all commands are valid
            if (std::isnan(_model.torq_cmd))
            {
                break;
            }

            // Set the torque command directly
            data->ctrl[_id] = _model.calculate_torque_control();
            break;
        }
        }
    }

public:
    MuJoCoMotorNode(rclcpp::Node *node, const uint8_t id) : _id(id)
    {
        // Create a command subscription for the motor
        _command_sub = node->create_subscription<MotorCommand>(
            "motor" + std::to_string(id) + "/command", qos_reliable(), std::bind(&MuJoCoMotorNode::_command_callback, this, std::placeholders::_1));

        // Create a configuration subscription for the motor
        _config_sub = node->create_subscription<MotorConfig>(
            "motor" + std::to_string(id) + "/config", qos_reliable(), std::bind(&MuJoCoMotorNode::_config_callback, this, std::placeholders::_1));

        // Create a publisher for the motor estimate
        _estimate_pub = node->create_publisher<MotorEstimate>("motor" + std::to_string(id) + "/estimate", qos_fast());

        // Create a timer for publishing motor estimates
        double estimate_rate = 50.0;
        _estimate_timer = node->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / estimate_rate)),
            std::bind(&MuJoCoMotorNode::_estimate_callback, this));

        // Add the motor control function to the MuJoCo data structure
        // This function will be called during the MuJoCo simulation loop
        MuJoCoData.control_functions.push_back(
            std::bind(&MuJoCoMotorNode::_motor_control_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
};

/*
 * MuJoCo Node
 * This node handles the initialization and running of the MuJoCo simulation
 * and creates motor nodes for controlling individual motors.
 */
class MuJoCoNode : public rclcpp::Node
{
private:
    std::string _model_path;                  // Path to the MuJoCo model file
    float _frame_rate = 60.0;                 // Frame rate for the simulation
    std::string _odom_frame_id = "odom";      // Frame ID for odometry
    std::string _base_frame_id = "base_link"; // Frame ID for the base link

    std::vector<std::unique_ptr<MuJoCoMotorNode>> _motor_nodes; // Vector of motor nodes

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr _clock_pub; // Publisher for simulation clock messages
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;    // Publisher for odometry messages

    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster; // Transform broadcaster for sending TF messages

    /*
     * Function to run the MuJoCo simulation
     * This function initializes MuJoCo, creates motor nodes,
     * and starts the simulation loop.
     */
    void _run_mujoco()
    {
        // Initialize MuJoCo
        initMuJoCo(_model_path);

        // Add functions to publish clock and odometry information to control functions structure
        MuJoCoData.control_functions.push_back(
            std::bind(&MuJoCoNode::_publish_sim_clock, this, std::placeholders::_1, std::placeholders::_2));
        MuJoCoData.control_functions.push_back(
            std::bind(&MuJoCoNode::_publish_odometry, this, std::placeholders::_1, std::placeholders::_2));

        // Create motor nodes for each motor in the MuJoCo model
        const int num_motors = MuJoCoData.model->nu;
        for (int i = 0; i < num_motors; i++)
        {
            _motor_nodes.push_back(std::make_unique<MuJoCoMotorNode>(this, i));
        }

        // Open the MuJoCo simulation
        // Blocks until the window is closed
        openMuJoCo(_frame_rate);

        // Shutdown when the simulation is closed
        rclcpp::shutdown();
    }

    /*
     * Publish simulation clock information to ROS
     */
    void _publish_sim_clock(const mjModel *, mjData *data)
    {
        // Convert MuJoCo time to ROS time
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(int64_t(data->time * 1E9)); // seconds to nanoseconds

        // Publish the clock message
        _clock_pub->publish(clock_msg);
    }

    /*
     * Publish odometry information to ROS
     */
    void _publish_odometry(const mjModel *, mjData *data)
    {
        // Get orientation as quaternion
        const double qw = data->qpos[3];
        const double qx = data->qpos[4];
        const double qy = data->qpos[5];
        const double qz = data->qpos[6];

        // Get linear velocity in world coordinates
        const double vx_world = data->qvel[0];
        const double vy_world = data->qvel[1];
        const double vz_world = data->qvel[2];

        // Convert linear velocity to body coordinates using quaternion
        double lw = qx * vx_world + qy * vy_world + qz * vz_world;
        double lx = qw * vx_world - qy * vz_world + qz * vy_world;
        double ly = qw * vy_world - qz * vx_world + qx * vz_world;
        double lz = qw * vz_world - qx * vy_world + qy * vx_world;

        double vx_body = lw * qx + lx * qw + ly * qz - lz * qy;
        double vy_body = lw * qy - lx * qz + ly * qw + lz * qx;
        double vz_body = lw * qz + lx * qy - ly * qx + lz * qw;

        // Create the odometry message and fill in the data
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

        // Publish the odometry message
        _odom_pub->publish(odom_msg);

        // Create the transform message and fill in the data
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom_msg.header.stamp;
        tf_msg.header.frame_id = _odom_frame_id;
        tf_msg.child_frame_id = _base_frame_id;
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

        // Send the transform message
        _tf_broadcaster->sendTransform(tf_msg);
    }

public:
    MuJoCoNode() : Node("mujoco_node")
    {
        // Get ROS parameters
        this->declare_parameter("model_path", _model_path);
        this->get_parameter("model_path", _model_path);

        this->declare_parameter("frame_rate", _frame_rate);
        this->get_parameter("frame_rate", _frame_rate);

        this->declare_parameter("odom_frame_id", _odom_frame_id);
        this->get_parameter("odom_frame_id", _odom_frame_id);

        this->declare_parameter("base_frame_id", _base_frame_id);
        this->get_parameter("base_frame_id", _base_frame_id);

        // Create publishers for clock and odometry messages
        _clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", qos_reliable());
        _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos_reliable());

        // Create a transform broadcaster for sending TF messages
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Start the MuJoCo simulation in a separate thread
        // This allows the simulation to run concurrently with ROS2
        std::thread([this]()
                    { _run_mujoco(); })
            .detach();

        RCLCPP_INFO(this->get_logger(), "MuJoCo Node Initialized");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MuJoCoNode>());
    rclcpp::shutdown();
    return 0;
}