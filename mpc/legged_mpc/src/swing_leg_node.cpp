#include <rclcpp/rclcpp.hpp>
#include <leg_control_msgs/msg/leg_trajectory.hpp>
#include <mpc_msgs/msg/body_trajectory.hpp>
#include <mpc_msgs/msg/foothold_trajectory.hpp>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

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

using namespace mpc_msgs::msg;
using namespace leg_control_msgs::msg;

class SwingLegNode : public rclcpp::Node
{
private:
    std::size_t _num_legs;
    Eigen::Matrix3Xd _hip_locations;
    Eigen::Matrix3Xd _default_leg_positions;
    double _dt;
    int _swing_resolution = 100;
    double _step_height = 0.075;
    double _swing_duration_factor = 0.5;

    rclcpp::Subscription<BodyTrajectory>::SharedPtr _body_traj_sub;
    rclcpp::Subscription<FootholdTrajectory>::SharedPtr _foothold_traj_sub;
    std::vector<rclcpp::Publisher<LegTrajectory>::SharedPtr> _leg_traj_pubs;

    BodyTrajectory::SharedPtr _body_trajectory;
    std::vector<bool> _current_stance;

    void updateBodyTrajectory(const BodyTrajectory::SharedPtr msg)
    {
        _body_trajectory = msg;
    }

    void updateFootholds(const FootholdTrajectory::SharedPtr msg)
    {
        assert(_body_trajectory);

        if (msg->header.stamp != _body_trajectory->header.stamp)
        {
            RCLCPP_WARN(this->get_logger(), "Foothold trajectory timestamp does not match Body trajectory timestamp.");
            return;
        }

        const std::size_t N = msg->foothold_states.size();
        assert(N > 0);

        const FootholdState &foothold_state = msg->foothold_states[0];
        assert(foothold_state.stance.size() == _leg_traj_pubs.size());

        if (_current_stance.size() != _num_legs)
        {
            _current_stance = foothold_state.stance;
            return;
        }

        const double stance_duration = foothold_state.duty_factor * foothold_state.duration;
        const double swing_duration = foothold_state.duration - stance_duration;

        const std::size_t node_span = std::min(static_cast<std::size_t>(std::round(swing_duration / _dt)), N);

        assert(foothold_state.stance.size() == _num_legs);
        assert(foothold_state.footholds.size() == _num_legs);
        for (std::size_t i = 0; i < _leg_traj_pubs.size(); i++)
        {
            if (_current_stance[i] && !foothold_state.stance[i]) // lifting
            {
                const Vector3d b_pos_foot_start = toVector3(foothold_state.footholds[i]) - _hip_locations.col(i);

                const Vector3d b_vel_body = toVector3(_body_trajectory->linear_velocities[node_span]);
                const Vector3d b_omega_body = toVector3(_body_trajectory->angular_velocities[node_span]);
                const Vector3d b_vel_hip = b_vel_body + b_omega_body.cross(_hip_locations.col(i));
                const Vector3d b_pos_foot_end = _default_leg_positions.col(i) + 0.5 * b_vel_hip * stance_duration;

                const Vector3d delta = b_pos_foot_end - b_pos_foot_start;
                const double radius = 0.5 * delta.norm();

                const double angle = std::atan2(delta.y(), delta.x());
                const Matrix3d rotation = AngleAxis<double>(angle, Vector3d::UnitZ()).toRotationMatrix();

                LegTrajectory trajectory;
                trajectory.timing.resize(_swing_resolution + 1);
                trajectory.commands.resize(_swing_resolution + 1);
                for (int j = 0; j <= _swing_resolution; j++)
                {
                    const double ratio = static_cast<double>(j) / static_cast<double>(_swing_resolution);
                    const double s = M_PI * ratio;
                    const double px = radius * (1 - std::cos(s));
                    const double pz = _step_height * std::sin(s);
                    const Vector3d arc_position = Vector3d(px, 0.0f, pz);
                    const Vector3d position = rotation * arc_position + b_pos_foot_start;

                    trajectory.timing[j] = swing_duration * ratio * _swing_duration_factor;
                    trajectory.commands[j].control_mode = LegCommand::CONTROL_MODE_POSITION;
                    trajectory.commands[j].pos_setpoint = toVectorMsg(position);
                }

                _leg_traj_pubs[i]->publish(trajectory);
            }
        }

        _current_stance = foothold_state.stance;
    }

public:
    SwingLegNode() : Node("swing_leg_node")
    {
        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        this->declare_parameter("leg_names", leg_names);
        this->get_parameter("leg_names", leg_names);
        _num_legs = leg_names.size();

        std::vector<double> hip_locations = {};
        this->declare_parameter("hip_locations", hip_locations);
        this->get_parameter("hip_locations", hip_locations);
        assert(hip_locations.size() == 3 * _num_legs);
        _hip_locations = Eigen::Map<Eigen::MatrixXd>(hip_locations.data(), 3, _num_legs);

        std::vector<double> default_leg_positions = {};
        this->declare_parameter("default_leg_positions", default_leg_positions);
        this->get_parameter("default_leg_positions", default_leg_positions);
        assert(default_leg_positions.size() == 3 * _num_legs);
        _default_leg_positions = Eigen::Map<Eigen::MatrixXd>(default_leg_positions.data(), 3, _num_legs);

        this->declare_parameter("time_step", _dt);
        this->get_parameter("time_step", _dt);

        this->declare_parameter("swing_resolution", _swing_resolution);
        this->get_parameter("swing_resolution", _swing_resolution);

        this->declare_parameter("step_height", _step_height);
        this->get_parameter("step_height", _step_height);

        this->declare_parameter("swing_duration_factor", _swing_duration_factor);
        this->get_parameter("swing_duration_factor", _swing_duration_factor);

        _body_traj_sub = this->create_subscription<BodyTrajectory>(
            "/body/trajectory", qos_reliable(),
            std::bind(&SwingLegNode::updateBodyTrajectory, this, std::placeholders::_1));

        _foothold_traj_sub = this->create_subscription<FootholdTrajectory>(
            "/foothold/trajectory", qos_reliable(),
            std::bind(&SwingLegNode::updateFootholds, this, std::placeholders::_1));

        for (const auto &leg_name : leg_names)
        {
            _leg_traj_pubs.push_back(this->create_publisher<LegTrajectory>("leg" + leg_name + "/trajectory", qos_reliable()));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingLegNode>());
    rclcpp::shutdown();
    return 0;
}