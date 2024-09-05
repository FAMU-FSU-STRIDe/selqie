#include <rclcpp/rclcpp.hpp>

#include <legged_mpc/legged_mpc.hpp>

#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/stance_pattern.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class LeggedMPCNode : public rclcpp::Node
{
private:
  LeggedMPCConfig _config;

  std::vector<robot_msgs::msg::LegEstimate> _leg_estimates;
  robot_msgs::msg::StancePattern _stance_pattern;
  nav_msgs::msg::Odometry _odometry;
  geometry_msgs::msg::Twist _twist;

  std::vector<rclcpp::Subscription<robot_msgs::msg::LegEstimate>::SharedPtr> _leg_estimate_subs;
  std::vector<rclcpp::Publisher<robot_msgs::msg::LegCommand>::SharedPtr> _leg_command_pubs;

  rclcpp::Subscription<robot_msgs::msg::StancePattern>::SharedPtr _stance_pattern_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;

  rclcpp::TimerBase::SharedPtr _mpc_timer;

  void solve()
  {
    /// TODO: Implement MPC solver
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
    _config.time_step = std::chrono::milliseconds(time_step);

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

    _config.linear_velocity = Vector3d::Zero();
    _config.angular_velocity = Vector3d::Zero();

    _config.stance_trajectory.resize(_config.window_size);
    for (std::size_t k = 0; k < _config.window_size; k++)
    {
      StanceState &stance_state = _config.stance_trajectory[k];
      stance_state.num_stance = _config.num_legs;
      stance_state.in_stance.resize(_config.num_legs, true);
      stance_state.leg_positions.resize(_config.num_legs, Vector3d::Zero());
    }

    _leg_estimate_subs.resize(_config.num_legs);
    _leg_command_pubs.resize(_config.num_legs);
    for (std::size_t i = 0; i < _config.num_legs; i++)
    {
      _leg_estimate_subs[i] = this->create_subscription<robot_msgs::msg::LegEstimate>(
          "leg" + leg_names[i] + "/estimate", 10,
          [this, i](const robot_msgs::msg::LegEstimate &msg)
          {
            _leg_estimates[i] = msg;
          });

      _leg_command_pubs[i] = this->create_publisher<robot_msgs::msg::LegCommand>("leg" + leg_names[i] + "/command", 10);
    }

    _stance_pattern_sub = this->create_subscription<robot_msgs::msg::StancePattern>(
        "stance_pattern", 10,
        [this](const robot_msgs::msg::StancePattern &msg)
        {
          _stance_pattern = msg;
        });

    _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        [this](const nav_msgs::msg::Odometry &msg)
        {
          _odometry = msg;
        });

    _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist", 10,
        [this](const geometry_msgs::msg::Twist &msg)
        {
          _twist = msg;
        });

    double mpc_rate = 1000.0;
    _mpc_timer = this->create_wall_timer(
        std::chrono::microseconds(static_cast<int>(1E6 / mpc_rate)),
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