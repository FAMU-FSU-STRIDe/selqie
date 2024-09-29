#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_msgs/msg/body_trajectory.hpp>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

static inline rclcpp::QoS qos_fast()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

static inline Vector3d toVector3(const geometry_msgs::msg::Vector3 &v)
{
    return Vector3d(v.x, v.y, v.z);
}

static inline Vector3d toVector3(const geometry_msgs::msg::Point &p)
{
    return Vector3d(p.x, p.y, p.z);
}

static Vector3d toVector3(const geometry_msgs::msg::Quaternion &quat)
{
    Vector3d vec;
    vec.x() = std::atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x * quat.x + quat.y * quat.y));
    vec.y() = std::asin(2 * (quat.w * quat.y - quat.z * quat.x));
    vec.z() = std::atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
    return vec;
}

static inline geometry_msgs::msg::Vector3 toVectorMsg(const Vector3d &v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

static geometry_msgs::msg::Vector3 toVectorMsg(const geometry_msgs::msg::Point &point)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = point.x;
    vec.y = point.y;
    vec.z = point.z;
    return vec;
}

static geometry_msgs::msg::Vector3 toVectorMsg(const geometry_msgs::msg::Quaternion &quat)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = std::atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x * quat.x + quat.y * quat.y));
    vec.y = std::asin(2 * (quat.w * quat.y - quat.z * quat.x));
    vec.z = std::atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
    return vec;
}

static inline Matrix3d toRotationMatrix(const geometry_msgs::msg::Vector3 &orientation)
{
    Matrix3d rotation;
    rotation = AngleAxisd(orientation.z, Vector3d::UnitZ()) *
               AngleAxisd(orientation.y, Vector3d::UnitY()) *
               AngleAxisd(orientation.x, Vector3d::UnitX());
    return rotation;
}

using namespace robot_msgs::msg;

class BodyTrajectoryNode : public rclcpp::Node
{
private:
    int _N = 11;
    double _dt = 0.1;
    double _body_z = 0.3;
    Vector3d _max_linear_velocity;
    Vector3d _max_angular_velocity;

    rclcpp::Publisher<BodyTrajectory>::SharedPtr _body_traj_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _pose_sub;

    rclcpp::TimerBase::SharedPtr _timer;

    nav_msgs::msg::Odometry::SharedPtr _odometry;
    geometry_msgs::msg::Pose::SharedPtr _cmd_pose;
    geometry_msgs::msg::Twist::SharedPtr _cmd_vel;

    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odometry = msg;
    }

    void updatePose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        _cmd_pose = msg->position.z <= 0.0 ? nullptr : msg;
        _cmd_vel = nullptr;
    }

    void updateTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _cmd_pose = nullptr;
        _cmd_vel = msg;
    }

public:
    BodyTrajectoryNode() : Node("body_trajectory_node")
    {
        this->declare_parameter("window_size", _N);
        this->get_parameter("window_size", _N);

        this->declare_parameter("time_step", _dt);
        this->get_parameter("time_step", _dt);

        this->declare_parameter("body_height", _body_z);
        this->get_parameter("body_height", _body_z);

        std::vector<double> max_linear_velocity = {0.5, 0.5, 0.5};
        this->declare_parameter("max_linear_velocity", max_linear_velocity);
        this->get_parameter("max_linear_velocity", max_linear_velocity);
        _max_linear_velocity = Eigen::Map<Vector3d>(max_linear_velocity.data());

        std::vector<double> max_angular_velocity = {0.5, 0.5, 0.5};
        this->declare_parameter("max_angular_velocity", max_angular_velocity);
        this->get_parameter("max_angular_velocity", max_angular_velocity);
        _max_angular_velocity = Eigen::Map<Vector3d>(max_angular_velocity.data());

        _body_traj_pub = this->create_publisher<BodyTrajectory>("body/trajectory", qos_reliable());
        _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos_reliable(),
            std::bind(&BodyTrajectoryNode::updateOdometry, this, std::placeholders::_1));
        _pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "cmd_pose", qos_reliable(),
            std::bind(&BodyTrajectoryNode::updatePose, this, std::placeholders::_1));
        _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_reliable(),
            std::bind(&BodyTrajectoryNode::updateTwist, this, std::placeholders::_1));

        double solve_frequency = 100.0;
        this->declare_parameter("solve_frequency", solve_frequency);
        this->get_parameter("solve_frequency", solve_frequency);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / solve_frequency)),
            std::bind(&BodyTrajectoryNode::solve, this));

        RCLCPP_INFO(this->get_logger(), "Body Trajectory Node Initialized.");
    }

    void solve()
    {
        if (!_odometry || (!_cmd_pose && !_cmd_vel))
        {
            return;
        }

        BodyTrajectory traj;
        traj.header.stamp = _odometry->header.stamp;
        traj.time_step = _dt;

        traj.positions.resize(_N);
        traj.orientations.resize(_N);
        traj.linear_velocities.resize(_N);
        traj.angular_velocities.resize(_N);

        traj.positions[0] = toVectorMsg(_odometry->pose.pose.position);
        traj.orientations[0] = toVectorMsg(_odometry->pose.pose.orientation);
        traj.linear_velocities[0] = _odometry->twist.twist.linear;
        traj.angular_velocities[0] = _odometry->twist.twist.angular;

        Vector3d last_position = toVector3(traj.positions[0]);
        Vector3d last_orientation = toVector3(traj.orientations[0]);
        for (int k = 1; k < _N; k++)
        {
            const Matrix3d R = toRotationMatrix(traj.orientations[k - 1]);

            Vector3d linear_velocity;
            Vector3d angular_velocity;
            if (_cmd_pose)
            {
                const Vector3d cmd_position = toVector3(_cmd_pose->position);
                const Vector3d delta_p = R.transpose() * (cmd_position - last_position);
                const Vector3d v = delta_p / _dt;
                linear_velocity = v.cwiseMin(_max_linear_velocity).cwiseMax(-_max_linear_velocity);

                const Vector3d cmd_orientation = toVector3(_cmd_pose->orientation);
                Vector3d delta_o = cmd_orientation - last_orientation;
                for (int j = 0; j < 3; j++)
                {
                    delta_o[j] = delta_o[j] - 2 * M_PI * floor((delta_o[j] + M_PI) / (2 * M_PI));
                }
                const Vector3d w = delta_o / _dt;
                angular_velocity = w.cwiseMin(_max_angular_velocity).cwiseMax(-_max_angular_velocity);
            }
            else if (_cmd_vel)
            {
                linear_velocity = toVector3(_cmd_vel->linear).cwiseMin(_max_linear_velocity).cwiseMax(-_max_linear_velocity);
                const double delta_z = _body_z - last_position.z();
                const double vz = delta_z / _dt;
                linear_velocity.z() = std::max(std::min(vz, _max_linear_velocity.z()), -_max_linear_velocity.z());

                angular_velocity = toVector3(_cmd_vel->angular).cwiseMin(_max_angular_velocity).cwiseMax(-_max_angular_velocity);
                const Eigen::Vector2d delta_o = -last_orientation.head(2);
                const Eigen::Vector2d vo = delta_o / _dt;
                angular_velocity.head(2) = vo.cwiseMin(_max_angular_velocity.head(2)).cwiseMax(-_max_angular_velocity.head(2));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid control mode.");
                return;
            }

            last_position += R * linear_velocity * _dt;
            last_orientation += angular_velocity * _dt;

            traj.positions[k] = toVectorMsg(last_position);
            traj.orientations[k] = toVectorMsg(last_orientation);
            traj.linear_velocities[k] = toVectorMsg(linear_velocity);
            traj.angular_velocities[k] = toVectorMsg(angular_velocity);
        }

        _body_traj_pub->publish(traj);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}