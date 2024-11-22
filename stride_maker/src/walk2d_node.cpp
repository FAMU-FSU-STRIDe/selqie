#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_msgs/srv/make_stride.hpp>

#include <thread>
#include <mutex>

static inline rclcpp::QoS qos_reliable()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

class Walk2DNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    rclcpp::Client<robot_msgs::srv::MakeStride>::SharedPtr _stride_client;
    std::vector<rclcpp::Publisher<robot_msgs::msg::LegTrajectory>::SharedPtr> _leg_traj_pubs;

    std::vector<double> _hip_positions;
    double _min_velocity = 0.1;
    double _min_radius = 1.0;
    double _robot_width = 1.0;
    double _leg_length = 0.2;
    int _stride_resolution = 100;
    double _step_height = 0.05;
    double _duty_factor = 0.5;
    double _max_stance_length = 0.15;

    std::mutex _mutex;
    robot_msgs::msg::LegTrajectory _traj_pos, _traj_neg;
    double _frequency;

    void get_stride(robot_msgs::srv::MakeStride::Request request, const bool is_pos)
    {
        auto future = _stride_client->async_send_request(
            std::make_shared<robot_msgs::srv::MakeStride::Request>(request),
            [this, is_pos](rclcpp::Client<robot_msgs::srv::MakeStride>::SharedFuture future)
            {
                robot_msgs::msg::LegTrajectory traj;
                try
                {
                    traj = future.get()->trajectory;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
                if (is_pos)
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _traj_pos = traj;
                }
                else
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _traj_neg = traj;
                }
            });
    }

    void cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double vel_x = msg->linear.x;
        const double omega_z = msg->angular.z;

        if (vel_x < _min_velocity)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _traj_pos = robot_msgs::msg::LegTrajectory();
            _traj_neg = robot_msgs::msg::LegTrajectory();
            _frequency = 10.0;
            return;
        }

        if (vel_x / omega_z < _min_radius)
        {
            RCLCPP_WARN(this->get_logger(), "The turning radius is smaller than the minimum radius.");
            return;
        }

        const double vel_pos = vel_x - 0.5 * _robot_width * omega_z;
        const double vel_neg = vel_x + 0.5 * _robot_width * omega_z;

        double llen_pos, llen_neg;
        if (vel_pos > vel_neg)
        {
            llen_pos = _leg_length;
            llen_neg = _leg_length * vel_neg / vel_pos;
        }
        else
        {
            llen_pos = _leg_length * vel_pos / vel_neg;
            llen_neg = _leg_length;
        }

        robot_msgs::srv::MakeStride::Request req_pos;
        req_pos.num_points = _stride_resolution;
        req_pos.leg_length = llen_pos;
        req_pos.step_height = _step_height;
        req_pos.duty_factor = _duty_factor;
        req_pos.offset = 0.0;

        robot_msgs::srv::MakeStride::Request req_neg;
        req_neg.num_points = _stride_resolution;
        req_neg.leg_length = llen_neg;
        req_neg.step_height = _step_height;
        req_neg.duty_factor = _duty_factor;
        req_neg.offset = 0.5;

        if (vel_pos > vel_neg)
        {
            req_pos.stance_length = _max_stance_length;
            req_pos.frequency = vel_pos * _duty_factor / req_pos.stance_length;
            req_neg.frequency = req_pos.frequency;
            req_neg.stance_length = vel_neg * _duty_factor / req_neg.frequency;
        }
        else
        {
            req_neg.stance_length = _max_stance_length;
            req_neg.frequency = vel_neg * _duty_factor / req_neg.stance_length;
            req_pos.frequency = req_neg.frequency;
            req_pos.stance_length = vel_pos * _duty_factor / req_pos.frequency;
        }

        {
            std::lock_guard<std::mutex> lock(_mutex);
            _traj_pos = robot_msgs::msg::LegTrajectory();
            _traj_neg = robot_msgs::msg::LegTrajectory();
            _frequency = req_pos.frequency;
        }

        get_stride(req_pos, true);
        get_stride(req_neg, false);
    }

    void run_walk()
    {
        while (rclcpp::ok())
        {
            std::chrono::milliseconds duration;
            robot_msgs::msg::LegTrajectory traj_pos, traj_neg;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                traj_pos = _traj_pos;
                traj_neg = _traj_neg;
                duration = std::chrono::milliseconds(static_cast<int>(1000.0 / _frequency));
            }

            if (traj_pos.commands.empty() || traj_neg.commands.empty())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            for (size_t i = 0; i < _leg_traj_pubs.size(); i++)
            {
                if (_hip_positions[2 * i] * _hip_positions[2 * i + 1] > 0)
                {
                    _leg_traj_pubs[i]->publish(traj_pos);
                }
                else
                {
                    _leg_traj_pubs[i]->publish(traj_neg);
                }
            }

            rclcpp::sleep_for(duration);
        }
    }

public:
    Walk2DNode() : Node("walk2d_node")
    {
        std::vector<std::string> leg_names = {"FL", "RL", "RR", "FR"};
        this->declare_parameter("leg_names", leg_names);
        this->get_parameter("leg_names", leg_names);

        this->declare_parameter("hip_positions", _hip_positions);
        this->get_parameter("hip_positions", _hip_positions);
        assert(_hip_positions.size() == 2 * leg_names.size());

        this->declare_parameter("min_velocity", _min_velocity);
        this->get_parameter("min_velocity", _min_velocity);

        this->declare_parameter("min_radius", _min_radius);
        this->get_parameter("min_radius", _min_radius);

        this->declare_parameter("robot_width", _robot_width);
        this->get_parameter("robot_width", _robot_width);

        this->declare_parameter("leg_length", _leg_length);
        this->get_parameter("leg_length", _leg_length);

        this->declare_parameter("stride_resolution", _stride_resolution);
        this->get_parameter("stride_resolution", _stride_resolution);

        this->declare_parameter("step_height", _step_height);
        this->get_parameter("step_height", _step_height);

        this->declare_parameter("duty_factor", _duty_factor);
        this->get_parameter("duty_factor", _duty_factor);

        this->declare_parameter("max_stance_length", _max_stance_length);
        this->get_parameter("max_stance_length", _max_stance_length);

        _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos_reliable(), std::bind(&Walk2DNode::cmd_vel, this, std::placeholders::_1));

        _stride_client = this->create_client<robot_msgs::srv::MakeStride>("make_stride");

        if (!_stride_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service not available.");
        }

        for (const auto &leg_name : leg_names)
        {
            _leg_traj_pubs.push_back(this->create_publisher<robot_msgs::msg::LegTrajectory>("leg" + leg_name + "/trajectory", qos_reliable()));
        }

        std::thread thread(&Walk2DNode::run_walk, this);
        thread.detach();

        RCLCPP_INFO(this->get_logger(), "Walk2D Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walk2DNode>());
    rclcpp::shutdown();
    return 0;
}