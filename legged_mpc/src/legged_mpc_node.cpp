#include <rclcpp/rclcpp.hpp>

#include <legged_mpc/legged_mpc.hpp>

#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/body_trajectory.hpp>
#include <robot_msgs/msg/foothold_trajectory.hpp>

#include <nav_msgs/msg/odometry.hpp>

static inline rclcpp::QoS qos_fast()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

static inline Eigen::Vector3d toVector3(const geometry_msgs::msg::Vector3 &v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

static inline geometry_msgs::msg::Vector3 toVectorMsg(const Eigen::Vector3d &v)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

using namespace robot_msgs::msg;

class LeggedMPCNode : public rclcpp::Node
{
private:
  LeggedMPCConfig _config;
  OSQPSettings _osqp_settings;

  std::vector<rclcpp::Publisher<LegCommand>::SharedPtr> _leg_command_pubs;
  rclcpp::Subscription<BodyTrajectory>::SharedPtr _body_traj_sub;
  rclcpp::Subscription<FootholdTrajectory>::SharedPtr _foothold_traj_sub;

  rclcpp::Time _ref_stamp;
  std::vector<bool> _current_stance;

  void updateReference(const BodyTrajectory &msg)
  {
    _ref_stamp = msg.header.stamp;

    _config.N = msg.positions.size();
    _config.time_step = msg.time_step;
    if (_config.N < 2)
    {
      RCLCPP_WARN(this->get_logger(), "Body trajectory must have at least 2 waypoints.");
      return;
    }

    _config.position.resize(_config.N);
    _config.orientation.resize(_config.N);
    _config.linear_velocity.resize(_config.N);
    _config.angular_velocity.resize(_config.N);
    for (std::size_t k = 0; k < _config.N; k++)
    {
      _config.position[k] = toVector3(msg.positions[k]);
      _config.orientation[k] = toVector3(msg.orientations[k]);
      _config.linear_velocity[k] = toVector3(msg.linear_velocities[k]);
      _config.angular_velocity[k] = toVector3(msg.angular_velocities[k]);
    }
  }

  void updateFootholds(const FootholdTrajectory &msg)
  {
    if (msg.header.stamp != _ref_stamp)
    {
      RCLCPP_WARN(this->get_logger(), "Foothold trajectory timestamp does not match reference timestamp.");
      return;
    }

    _current_stance.resize(_config.num_legs);
    _current_stance = msg.foothold_states[0].stance;

    _config.num_stance.resize(_config.N);
    _config.in_stance.resize(_config.N);
    _config.foothold_positions.resize(_config.N);
    for (std::size_t k = 0; k < _config.N; k++)
    {
      _config.num_stance[k] = 0;
      _config.in_stance[k].resize(_config.num_legs);
      _config.foothold_positions[k].resize(_config.num_legs);
      for (std::size_t i = 0; i < _config.num_legs; i++)
      {
        _config.in_stance[k][i] = msg.foothold_states[k].stance[i];
        _config.foothold_positions[k][i] = toVector3(msg.foothold_states[k].footholds[i]);
        if (_config.in_stance[k][i])
        {
          _config.num_stance[k]++;
        }
      }
    }

    solve();
  }

public:
  LeggedMPCNode() : Node("legged_mpc_node")
  {
    osqp_set_default_settings(&_osqp_settings);
    _osqp_settings.verbose = false;

    std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
    this->declare_parameter("leg_names", leg_names);
    this->get_parameter("leg_names", leg_names);
    _config.num_legs = leg_names.size();

    std::vector<double> gravity_vector = {0.0, 0.0, -9.81};
    this->declare_parameter("gravity_vector", gravity_vector);
    this->get_parameter("gravity_vector", gravity_vector);
    _config.gravity_vector = Eigen::Map<OSQPVector3>(gravity_vector.data());

    _config.body_mass = 10.0;
    this->declare_parameter("body_mass", _config.body_mass);
    this->get_parameter("body_mass", _config.body_mass);

    std::vector<double> body_inertia = {10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0};
    this->declare_parameter("body_inertia", body_inertia);
    this->get_parameter("body_inertia", body_inertia);
    _config.body_inertia = Eigen::Map<Eigen::Matrix3d>(body_inertia.data(), 3, 3);

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

    std::vector<double> position_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("position_weights", position_weights);
    this->get_parameter("position_weights", position_weights);
    _config.position_weights = Eigen::Map<OSQPVector3>(position_weights.data());

    std::vector<double> orientation_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("orientation_weights", orientation_weights);
    this->get_parameter("orientation_weights", orientation_weights);
    _config.orientation_weights = Eigen::Map<OSQPVector3>(orientation_weights.data());

    std::vector<double> linear_velocity_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("linear_velocity_weights", linear_velocity_weights);
    this->get_parameter("linear_velocity_weights", linear_velocity_weights);
    _config.linear_velocity_weights = Eigen::Map<OSQPVector3>(linear_velocity_weights.data());

    std::vector<double> angular_velocity_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("angular_velocity_weights", angular_velocity_weights);
    this->get_parameter("angular_velocity_weights", angular_velocity_weights);
    _config.angular_velocity_weights = Eigen::Map<OSQPVector3>(angular_velocity_weights.data());

    std::vector<double> force_weights = {0.01, 0.01, 0.01};
    this->declare_parameter("force_weights", force_weights);
    this->get_parameter("force_weights", force_weights);
    _config.force_weights = Eigen::Map<OSQPVector3>(force_weights.data());

    _leg_command_pubs.resize(_config.num_legs);
    for (std::size_t i = 0; i < _config.num_legs; i++)
    {
      _leg_command_pubs[i] = this->create_publisher<LegCommand>("leg" + leg_names[i] + "/command", qos_fast());
    }

    _body_traj_sub = this->create_subscription<BodyTrajectory>(
        "body/trajectory", qos_reliable(), std::bind(&LeggedMPCNode::updateReference, this, std::placeholders::_1));

    _foothold_traj_sub = this->create_subscription<FootholdTrajectory>(
        "foothold/trajectory", qos_reliable(), std::bind(&LeggedMPCNode::updateFootholds, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Legged MPC Node Initialized.");
  }

  void solve()
  {
    const MPCProblem mpc = getMPCProblem(_config);
    const MPCSolution sol = solveMPC(mpc, &_osqp_settings);

    if (sol.exit_flag != OSQP_SOLVED)
    {
      RCLCPP_ERROR(this->get_logger(), "OSQP failed to solve the problem.");
      return;
    }

    for (std::size_t i = 0, j = 0; i < _config.num_legs; i++)
    {
      if (_current_stance[i])
      {
        OSQPVector3 ctrl = -sol.ustar[0].block<3, 1>(3 * j++, 0);

        LegCommand cmd;
        cmd.control_mode = LegCommand::CONTROL_MODE_FORCE;
        cmd.force_setpoint = toVectorMsg(ctrl);
        _leg_command_pubs[i]->publish(cmd);
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeggedMPCNode>());
  rclcpp::shutdown();
  return 0;
}