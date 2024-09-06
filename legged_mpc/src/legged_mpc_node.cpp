#include <rclcpp/rclcpp.hpp>

#include <legged_mpc/legged_mpc.hpp>

#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/stance_pattern.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace robot_msgs::msg;

static inline SequencePattern getSequencePattern(const std::size_t num_legs, const StancePattern &pattern)
{
  SequencePattern sequence_pattern;
  sequence_pattern.duration = milliseconds(time_t(1E3 / pattern.frequency));
  for (std::size_t i = 0; i < pattern.timing.size(); i++)
  {
    const milliseconds time(time_t(pattern.timing[i] / StancePattern::GAIT_RESOLUTION * (1E3 / pattern.frequency)));
    const uint32_t bitset = pattern.stance[i];
    sequence_pattern.stance_timing[time] = std::vector<bool>(num_legs);
    for (std::size_t j = 0; j < num_legs; j++)
    {
      if (sequence_pattern.stance_timing[time][j] = (bitset & (1 << j)) != 0)
      {
        sequence_pattern.num_stance++;
      }
    }
  }
  return sequence_pattern;
}

class LeggedMPCNode : public rclcpp::Node
{
private:
  LeggedMPCConfig _config;

  std::vector<robot_msgs::msg::LegEstimate> _leg_estimates;

  std::vector<rclcpp::Subscription<robot_msgs::msg::LegEstimate>::SharedPtr> _leg_estimate_subs;
  std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_command_pubs;

  rclcpp::Subscription<robot_msgs::msg::StancePattern>::SharedPtr _stance_pattern_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;

  rclcpp::TimerBase::SharedPtr _mpc_timer;

  bool sync()
  {
    const auto next_pattern = _config.stance_sequence.next_pattern;
    if (next_pattern.duration.count() == 0)
    {
      return false;
    }

    const auto current_time = duration_cast<milliseconds>(nanoseconds(this->get_clock()->now().nanoseconds()));

    auto &start_time = _config.stance_sequence.start_time;
    auto &current_pattern = _config.stance_sequence.current_pattern;
    if (current_time >= start_time + current_pattern.duration)
    {
      start_time = current_time;
      current_pattern = next_pattern;
    }
  }

  void solve()
  {
    /// TODO: Solve
  }

public:
  LeggedMPCNode() : Node("legged_mpc_node")
  {
    int window_size = 10;
    this->declare_parameter("window_size", window_size);
    this->get_parameter("window_size", window_size);
    _config.window_size = window_size;

    int time_step = 100;
    this->declare_parameter("time_step_ms", time_step);
    this->get_parameter("time_step_ms", time_step);
    _config.time_step = milliseconds(time_step);

    std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
    this->declare_parameter("leg_names", leg_names);
    this->get_parameter("leg_names", leg_names);
    _config.num_legs = leg_names.size();

    std::vector<double> gravity_vector = {0.0, 0.0, -9.81};
    this->declare_parameter("gravity_vector", gravity_vector);
    this->get_parameter("gravity_vector", gravity_vector);
    _config.gravity_vector = Vector3d(Map<Vector3d>(gravity_vector.data()));

    _config.body_mass = 10.0;
    this->declare_parameter("body_mass", _config.body_mass);
    this->get_parameter("body_mass", _config.body_mass);

    std::vector<double> body_inertia = {10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0};
    this->declare_parameter("body_inertia", body_inertia);
    this->get_parameter("body_inertia", body_inertia);
    _config.body_inertia = Matrix3d(Map<Matrix3d>(body_inertia.data()));

    /// TODO: Hip positions and default leg positions
    _config.hip_positions.resize(_config.num_legs);
    _config.default_leg_positions.resize(_config.num_legs);

    _config.friction_coefficient_x = 0.5;
    this->declare_parameter("friction_coefficient_x", _config.friction_coefficient_x);
    this->get_parameter("friction_coefficient_x", _config.friction_coefficient_x);

    _config.friction_coefficient_y = 0.5;
    this->declare_parameter("friction_coefficient_y", _config.friction_coefficient_y);
    this->get_parameter("friction_coefficient_y", _config.friction_coefficient_y);

    _config.force_z_min = 0.0;
    this->declare_parameter("force_z_min", _config.force_z_min);
    this->get_parameter("force_z_min", _config.force_z_min);

    _config.force_z_max = 1000.0;
    this->declare_parameter("force_z_max", _config.force_z_max);
    this->get_parameter("force_z_max", _config.force_z_max);

    std::vector<double> linear_velocity_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("linear_velocity_weights", linear_velocity_weights);
    this->get_parameter("linear_velocity_weights", linear_velocity_weights);
    _config.linear_velocity_weights = Vector3d(Map<Vector3d>(linear_velocity_weights.data()));

    std::vector<double> angular_velocity_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("angular_velocity_weights", angular_velocity_weights);
    this->get_parameter("angular_velocity_weights", angular_velocity_weights);
    _config.angular_velocity_weights = Vector3d(Map<Vector3d>(angular_velocity_weights.data()));

    std::vector<double> force_weights = {0.01, 0.01, 0.01};
    this->declare_parameter("force_weights", force_weights);
    this->get_parameter("force_weights", force_weights);
    _config.force_weights = Vector3d(Map<Vector3d>(force_weights.data()));

    _config.current_linear_velocity = Vector3d::Zero();
    _config.current_angular_velocity = Vector3d::Zero();
    _config.desired_linear_velocity = Vector3d::Zero();
    _config.desired_angular_velocity = Vector3d::Zero();

    _leg_estimate_subs.resize(_config.num_legs);
    _leg_command_pubs.resize(_config.num_legs);
    for (std::size_t i = 0; i < _config.num_legs; i++)
    {
      _leg_estimate_subs[i] = this->create_subscription<robot_msgs::msg::LegEstimate>(
          "leg" + leg_names[i] + "/estimate", 10,
          [this, i](const LegEstimate &msg)
          {
            _leg_estimates[i] = msg;
          });

      _leg_command_pubs[i] = this->create_publisher<robot_msgs::msg::LegCommand>("leg" + leg_names[i] + "/command", 10);
    }

    _stance_pattern_sub = this->create_subscription<StancePattern>(
        "stance_pattern", 10,
        [this](const robot_msgs::msg::StancePattern &msg)
        {
          _config.stance_sequence.next_pattern = getSequencePattern(_config.num_legs, msg);
        });

    _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        [this](const nav_msgs::msg::Odometry &msg)
        {
          _config.current_linear_velocity = Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
          _config.current_angular_velocity = Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
        });

    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist", 10,
        [this](const geometry_msgs::msg::Twist &msg)
        {
          _config.desired_linear_velocity = Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
          _config.desired_angular_velocity = Vector3d(msg.angular.x, msg.angular.y, msg.angular.z);
        });

    double mpc_rate = 1000.0;
    _mpc_timer = this->create_wall_timer(
        microseconds(static_cast<int>(1E6 / mpc_rate)),
        std::bind(&LeggedMPCNode::solve, this));

    RCLCPP_INFO(this->get_logger(), "Legged MPC node initialized.");
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeggedMPCNode>());
  rclcpp::shutdown();
  return 0;
}