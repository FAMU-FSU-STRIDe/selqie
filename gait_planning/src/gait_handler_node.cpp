#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_msgs/msg/gait_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GaitHandlerNode : public rclcpp::Node
{
    private:
        int _lookahead = 1;

        rclcpp::Subscription<robot_msgs::msg::GaitTrajectory>::SharedPtr _gait_traj_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _local_goal_pose_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _gait_pub;
    public:
        GaitHandlerNode() : rclcpp::Node("gait_handler_node")
        {
            this->declare_parameter("lookahead", 1);
            this->get_parameter("lookahead", _lookahead);

            _gait_traj_sub = this->create_subscription<robot_msgs::msg::GaitTrajectory>(
                "gait/trajectory", 10, std::bind(&GaitHandlerNode::_gait_traj_callback, this, std::placeholders::_1));

            _local_goal_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose/local", 10);

            _gait_pub = this->create_publisher<std_msgs::msg::String>("gait", 10);

            RCLCPP_INFO(this->get_logger(), "Gait Handler Node Initialized");
        }

        void _gait_traj_callback(const robot_msgs::msg::GaitTrajectory::SharedPtr msg)
        {
            if (msg->gait.size() == 0)
            {
                std_msgs::msg::String gait_msg;
                gait_msg.data = "none";
                _gait_pub->publish(gait_msg);
                return;
            }

            std::string start_gait = msg->gait[0];
            int n = std::min(_lookahead, int(msg->gait.size()));
            for (int i = 0; i < n; i++)
            {
                if (msg->gait[i] != start_gait)
                {
                    n = i;
                    break;
                }
            }

            geometry_msgs::msg::PoseStamped local_goal_pose = msg->path.poses[n];
            local_goal_pose.header.frame_id = "map";
            local_goal_pose.header.stamp = this->now();
            _local_goal_pose_pub->publish(local_goal_pose);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitHandlerNode>());
    rclcpp::shutdown();
    return 0;
}