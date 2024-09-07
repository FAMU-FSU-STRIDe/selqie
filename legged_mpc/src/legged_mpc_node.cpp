#include <rclcpp/rclcpp.hpp>

#include <legged_mpc/legged_mpc.hpp>

#include <robot_msgs/msg/leg_command.hpp>
#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/body_trajectory.hpp>
#include <robot_msgs/msg/foothold_trajectory.hpp>

using namespace robot_msgs::msg;

static Vector3d quat2eul(const double w, const double x, const double y, const double z)
{
  const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const double pitch = std::asin(2 * (w * y - z * x));
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return Vector3d(roll, pitch, yaw);
}

class LeggedMPCNode : public rclcpp::Node
{
private:
  LeggedMPCConfig _config;
  std::unique_ptr<OSQPSettings> _osqp_settings;

  std::vector<rclcpp::Publisher<LegCommand>::SharedPtr> _leg_command_pubs;
  rclcpp::Subscription<BodyTrajectory>::SharedPtr _body_traj_sub;
  rclcpp::Subscription<FootholdTrajectory>::SharedPtr _foothold_traj_sub;
  rclcpp::TimerBase::SharedPtr _mpc_timer;

  void solve()
  {
    const MPCProblem mpc = getMPCProblem(_config);
    const QPProblem qp = getQPProblem(mpc);
    const QPSolution sol = solveOSQP(qp, _osqp_settings.get());

    if (sol.exit_flag != OSQP_SOLVED)
    {
      RCLCPP_ERROR(this->get_logger(), "OSQP failed to solve the problem.");
      return;
    }

    /// TODO: Parse solution and publish leg commands
  }

public:
  LeggedMPCNode() : Node("legged_mpc_node")
  {
    _osqp_settings = std::make_unique<OSQPSettings>();
    osqp_set_default_settings(_osqp_settings.get());

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
    _config.position_weights = Vector3d(Map<Vector3d>(position_weights.data()));

    std::vector<double> orientation_weights = {1.0, 1.0, 1.0};
    this->declare_parameter("orientation_weights", orientation_weights);
    this->get_parameter("orientation_weights", orientation_weights);
    _config.orientation_weights = Vector3d(Map<Vector3d>(orientation_weights.data()));

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

    _config.position.resize(_config.window_size);
    _config.orientation.resize(_config.window_size);
    _config.linear_velocity.resize(_config.window_size);
    _config.angular_velocity.resize(_config.window_size);

    _leg_command_pubs.resize(_config.num_legs);
    for (std::size_t i = 0; i < _config.num_legs; i++)
    {
      _leg_command_pubs[i] = this->create_publisher<LegCommand>("leg" + leg_names[i] + "/command", 10);
    }

    _body_traj_sub = this->create_subscription<BodyTrajectory>(
        "body/trajectory", 10,
        [this](const BodyTrajectory &msg)
        {
          if (msg.body_states.size() != _config.window_size)
          {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory size.");
            return;
          }

          for (std::size_t k = 0; k < _config.window_size; k++)
          {
            const auto &pose = msg.body_states[k].pose.pose;
            const auto &twist = msg.body_states[k].twist.twist;
            _config.position[k] = Vector3d(pose.position.x, pose.position.y, pose.position.z);
            _config.orientation[k] = quat2eul(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            _config.linear_velocity[k] = Vector3d(twist.linear.x, twist.linear.y, twist.linear.z);
            _config.angular_velocity[k] = Vector3d(twist.angular.x, twist.angular.y, twist.angular.z);
          }
        });

    _foothold_traj_sub = this->create_subscription<FootholdTrajectory>(
        "twist", 10,
        [this](const FootholdTrajectory &msg)
        {
          if (msg.foothold_states.size() != _config.window_size)
          {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory size.");
            return;
          }

          for (std::size_t k = 0; k < _config.window_size; k++)
          {
            const std::size_t num_stance = msg.foothold_states[k].num_in_stance;
            _config.num_stance[k] = num_stance;
            for (std::size_t i = 0; i < _config.num_legs; i++)
            {
              const auto &in_stance = msg.foothold_states[k].stance[i];
              const auto &foothold = msg.foothold_states[k].footholds[i];
              _config.in_stance[k][i] = in_stance;
              _config.foothold_positions[k][i] = Vector3d(foothold.x, foothold.y, foothold.z);
            }
          }
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