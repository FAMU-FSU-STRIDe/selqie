#include <assert.h>
#include <cmath>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <leg_control_msgs/msg/leg_estimate.hpp>
#include <mpc_msgs/msg/body_trajectory.hpp>
#include <mpc_msgs/msg/foothold_trajectory.hpp>
#include <mpc_msgs/msg/stance_pattern.hpp>

#include <eigen3/Eigen/Dense>

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

using namespace mpc_msgs::msg;
using namespace leg_control_msgs::msg;

class FootholdPlannerNode : public rclcpp::Node
{
private:
    std::size_t _num_legs;
    Eigen::Matrix3Xd _hip_locations;
    Eigen::Matrix3Xd _default_leg_positions;

    std::vector<rclcpp::Subscription<LegEstimate>::SharedPtr> _leg_estimate_subs;
    rclcpp::Publisher<FootholdTrajectory>::SharedPtr _foothold_traj_pub;
    rclcpp::Subscription<BodyTrajectory>::SharedPtr _body_traj_sub;
    rclcpp::Subscription<StancePattern>::SharedPtr _stance_pattern_sub;

    Eigen::Matrix3Xd _leg_positions;

    struct
    {
        double duration = 0.0;
        double start_time = 0.0;
        double stance_duration = 0.0;
        std::map<double, std::vector<bool>> stance_timing;
    } _current_pattern, _next_pattern;

    void updateNextStancePattern(const StancePattern &msg)
    {
        auto &pattern = _next_pattern;
        pattern.duration = msg.frequency == 0.0 ? 0.0 : 1.0 / msg.frequency;
        pattern.start_time = _current_pattern.start_time + _current_pattern.duration;
        pattern.stance_duration = 0.0;
        pattern.stance_timing.clear();
        double last_time = 0.0;
        for (std::size_t i = 0; i < msg.timing.size(); i++)
        {
            const double time = msg.timing[i] * pattern.duration;
            const uint32_t bitset = msg.stance[i];
            pattern.stance_timing[time] = std::vector<bool>(_num_legs);
            for (std::size_t j = 0; j < _num_legs; j++)
            {
                const bool stance = (bitset & (1 << j)) != 0;
                pattern.stance_timing[time][j] = stance;

                if (j == 0 && stance)
                {
                    pattern.stance_duration += time - last_time;
                }
            }

            last_time = time;
        }
    }

    void updateTrajectory(const BodyTrajectory &msg)
    {
        using namespace Eigen;

        if (_next_pattern.duration == 0.0)
        {
            return;
        }

        const double current_time = rclcpp::Time(msg.header.stamp).seconds();
        if (current_time > _next_pattern.start_time)
        {
            _current_pattern = _next_pattern;
        }

        const std::size_t N = msg.positions.size();
        const double dt = msg.time_step;

        FootholdTrajectory foothold_traj;
        foothold_traj.header.stamp = msg.header.stamp;
        foothold_traj.foothold_states.resize(N);

        const Matrix3Xd b_pos_feet_def = _hip_locations + _default_leg_positions;
        Matrix3Xd b_pos_feet = _hip_locations + _leg_positions;

        const double start_time = _current_pattern.start_time;
        const double end_time = _next_pattern.start_time;
        for (std::size_t k = 0; k < N; k++)
        {
            const double time = current_time + k * dt;
            const double rel_time = time < end_time ? time - start_time : std::fmod(time - end_time, _next_pattern.duration);
            const auto &pattern = time < end_time ? _current_pattern : _next_pattern;

            const auto stance_it = pattern.stance_timing.lower_bound(rel_time);
            assert(stance_it != pattern.stance_timing.end());

            const std::vector<bool> &in_stance = stance_it->second;

            const Vector3d b_vel_body = toVector3(msg.linear_velocities[k]);
            const Vector3d b_omega_body = toVector3(msg.angular_velocities[k]);

            foothold_traj.foothold_states[k].duration = pattern.duration;
            foothold_traj.foothold_states[k].duty_factor = pattern.stance_duration / pattern.duration;
            foothold_traj.foothold_states[k].stance = in_stance;
            foothold_traj.foothold_states[k].footholds.resize(_num_legs);
            for (std::size_t i = 0; i < _num_legs; i++)
            {
                if (k == 0) // current
                {
                    // nothing to do
                }
                else if (in_stance[i]) // standing or landing
                {
                    const Vector3d delta = -b_vel_body * dt;
                    b_pos_feet.col(i) += delta;
                }
                else // swinging
                {
                    const Vector3d b_vel_hip = b_vel_body + b_omega_body.cross(_hip_locations.col(i));
                    b_pos_feet.col(i) = b_pos_feet_def.col(i) + 0.5 * b_vel_hip * pattern.stance_duration;
                }

                foothold_traj.foothold_states[k].footholds[i] = toVectorMsg(b_pos_feet.col(i));
            }
        }

        _foothold_traj_pub->publish(foothold_traj);
    }

public:
    FootholdPlannerNode() : Node("foothold_planner_node")
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

        _leg_estimate_subs.resize(leg_names.size());
        _leg_positions.resize(3, leg_names.size());
        for (std::size_t i = 0; i < leg_names.size(); i++)
        {
            _leg_estimate_subs[i] = this->create_subscription<LegEstimate>(
                "leg" + leg_names[i] + "/estimate", qos_fast(),
                [this, i](const LegEstimate::SharedPtr msg)
                {
                    _leg_positions.col(i) = toVector3(msg->pos_estimate);
                });
        }

        _foothold_traj_pub = this->create_publisher<FootholdTrajectory>("foothold/trajectory", qos_reliable());

        _body_traj_sub = this->create_subscription<BodyTrajectory>(
            "body/trajectory", qos_reliable(),
            std::bind(&FootholdPlannerNode::updateTrajectory, this, std::placeholders::_1));

        _stance_pattern_sub = this->create_subscription<StancePattern>(
            "stance_pattern", qos_reliable(),
            std::bind(&FootholdPlannerNode::updateNextStancePattern, this, std::placeholders::_1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootholdPlannerNode>());
    rclcpp::shutdown();
    return 0;
}