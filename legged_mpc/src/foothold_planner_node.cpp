#include <rclcpp/rclcpp.hpp>

#include <assert.h>
#include <cmath>
#include <vector>
#include <map>

#include <robot_msgs/msg/leg_estimate.hpp>
#include <robot_msgs/msg/body_trajectory.hpp>
#include <robot_msgs/msg/foothold_trajectory.hpp>
#include <robot_msgs/msg/stance_pattern.hpp>

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

static inline Eigen::Matrix3d toRotationMatrix(const geometry_msgs::msg::Vector3 &orientation)
{
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(orientation.z, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(orientation.y, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(orientation.x, Eigen::Vector3d::UnitX());
    return rotation;
}

using namespace robot_msgs::msg;

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
        std::map<double, std::vector<bool>> stance_timing;
    } _current_pattern, _next_pattern;

    void updateNextStancePattern(const StancePattern &msg)
    {
        auto &pattern = _next_pattern;
        pattern.duration = 1.0 / msg.frequency;
        pattern.start_time = _current_pattern.start_time + _current_pattern.duration;
        for (std::size_t i = 0; i < msg.timing.size(); i++)
        {
            const double time = msg.timing[i] * pattern.duration;
            const uint32_t bitset = msg.stance[i];
            pattern.stance_timing[time] = std::vector<bool>(_num_legs);
            for (std::size_t j = 0; j < _num_legs; j++)
            {
                pattern.stance_timing[time][j] = (bitset & (1 << j)) != 0;
            }
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

        const Matrix3Xd b_pos_hips = _hip_locations;
        const Matrix3Xd b_pos_feet = b_pos_hips + _leg_positions;
        const Matrix3Xd b_pos_feet_def = b_pos_hips + _default_leg_positions;

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

            const Vector3d w_pos_body = toVector3(msg.positions[k]);
            const Matrix3d w_rot_b = toRotationMatrix(msg.orientations[k]);
            const Vector3d w_vel_body = toVector3(msg.linear_velocities[k]);
            const Vector3d w_omega_body = toVector3(msg.angular_velocities[k]);

            const Matrix3Xd w_pos_hips_b = w_rot_b * b_pos_hips;

            foothold_traj.foothold_states[k].stance = in_stance;
            foothold_traj.foothold_states[k].footholds.resize(_num_legs);
            for (std::size_t i = 0; i < _num_legs; i++)
            {
                if (k == 0) // current
                {
                    const Vector3d w_pos_feet = w_pos_body + w_rot_b * b_pos_feet.col(i);
                    foothold_traj.foothold_states[0].footholds[i] = toVectorMsg(w_pos_feet);
                }
                else if (in_stance[i]) // standing or landing
                {
                    foothold_traj.foothold_states[k].footholds[i] = foothold_traj.foothold_states[k - 1].footholds[i];
                }
                else // swinging
                {
                    const Vector3d w_vel_hip = w_vel_body + w_omega_body.cross(w_pos_hips_b.col(i));
                    const double stance_duration = 0.5 * pattern.duration;
                    const Vector3d w_pos_foot = w_pos_body + w_rot_b * b_pos_feet_def.col(i) + 0.5 * w_vel_hip * stance_duration;

                    foothold_traj.foothold_states[k].footholds[i] = toVectorMsg(w_pos_foot);
                }
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

        std::vector<double> hip_locations = {/* TODO */};
        this->declare_parameter("hip_locations", hip_locations);
        this->get_parameter("hip_locations", hip_locations);
        assert(hip_locations.size() == 3 * _num_legs);
        _hip_locations = Eigen::Map<Eigen::MatrixXd>(hip_locations.data(), 3, _num_legs);

        std::vector<double> default_leg_positions = {/* TODO */};
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