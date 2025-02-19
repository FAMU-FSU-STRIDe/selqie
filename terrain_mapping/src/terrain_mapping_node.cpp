#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

class TerrainMappingNode : public rclcpp::Node
{
private:
    grid_map::GridMap _grid_map;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _reset_sub;
    rclcpp::TimerBase::SharedPtr _timer;

    const std::string _ground_gait = "stand";
    double _robot_height;
    bool _on_ground = false;
    double _ground_level = 0.0;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

    int _point_thickness;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::vector<std::string> _pointcloud_layers;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> _cloud_subs;

    void _publish_map()
    {
        const auto map_msg = grid_map::GridMapRosConverter::toMessage(_grid_map);
        map_msg->header.stamp = this->now();
        _map_pub->publish(*map_msg);
    }

    void _reset_map_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        _grid_map.clearAll();
        _publish_map();
    }

    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _on_ground = (msg->data == _ground_gait);
    }

    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (_on_ground)
        {
            _ground_level = msg->pose.pose.position.z - _robot_height;
        }
    }

    void _update_ground()
    {
        for (Eigen::Index i = 0; i < _grid_map.getSize().x(); i++)
        {
            for (Eigen::Index j = 0; j < _grid_map.getSize().y(); j++)
            {
                grid_map::Index index(i, j);
                if (_grid_map.at("ground", index) == 1.0)
                {
                    _grid_map.at("elevation", index) = _ground_level;
                }
            }
        }
    }

    void _update_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &layer_name)
    {
        if (!_grid_map.exists(layer_name))
        {
            _grid_map.add(layer_name, 0.0);
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = _tf_buffer->lookupTransform(
                _grid_map.getFrameId(), msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::fromROSMsg(*msg, cloud_in);

        for (const auto &cloud_point : cloud_in.points)
        {
            geometry_msgs::msg::Point point_cam;
            point_cam.x = cloud_point.x;
            point_cam.y = cloud_point.y;
            point_cam.z = cloud_point.z;

            geometry_msgs::msg::Point point_map;
            tf2::doTransform(point_cam, point_map, transform);

            grid_map::Index index;
            if (_grid_map.getIndex(grid_map::Position(point_map.x, point_map.y), index))
            {
                for (int i = -_point_thickness; i <= _point_thickness; i++)
                {
                    for (int j = -_point_thickness; j <= _point_thickness; j++)
                    {
                        grid_map::Index index_offset = index + grid_map::Index(i, j);
                        if (_grid_map.isValid(index_offset))
                        {
                            _grid_map.at("ground", index_offset) = 0.0;
                            _grid_map.at(layer_name, index_offset) = 1.0;

                            auto &elevation = _grid_map.at("elevation", index_offset);
                            if (elevation < point_map.z)
                            {
                                elevation = point_map.z;
                            }
                        }
                    }
                }
            }
        }
    }

    void _update_map()
    {
        _update_ground();
        _publish_map();
    }

public:
    TerrainMappingNode() : Node("terrain_mapping_node")
    {
        this->declare_parameter<std::string>("map_frame_id", "map");
        const std::string frame_id = this->get_parameter("map_frame_id").as_string();

        this->declare_parameter<double>("map_resolution", 0.1);
        const double resolution = this->get_parameter("map_resolution").as_double();

        this->declare_parameter<double>("map_length", 10.0);
        const double length = this->get_parameter("map_length").as_double();

        this->declare_parameter("frequency", 2.0);
        const double frequency = this->get_parameter("frequency").as_double();

        this->declare_parameter("robot_height", 0.2);
        this->get_parameter("robot_height", _robot_height);

        this->declare_parameter("point_thickness", 1);
        this->get_parameter("point_thickness", _point_thickness);

        this->declare_parameter("pointcloud_layers", std::vector<std::string>{});
        this->get_parameter("pointcloud_layers", _pointcloud_layers);

        _grid_map.setFrameId(frame_id);
        _grid_map.setGeometry(grid_map::Length(length, length), resolution);
        _grid_map.setBasicLayers({"elevation", "ground"});
        _grid_map.add("elevation");
        _grid_map.add("ground", 1.0);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        _map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        _reset_sub = this->create_subscription<std_msgs::msg::Empty>(
            "map/reset", 10, std::bind(&TerrainMappingNode::_reset_map_callback, this, std::placeholders::_1));

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&TerrainMappingNode::_gait_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TerrainMappingNode::_odom_callback, this, std::placeholders::_1));

        for (const auto &layer_name : _pointcloud_layers)
        {
            _cloud_subs.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points/" + layer_name, 10, [this, layer_name](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                { this->_update_cloud(msg, layer_name); }));
        }

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(time_t(1E3 / frequency)),
            std::bind(&TerrainMappingNode::_update_map, this));

        RCLCPP_INFO(this->get_logger(), "Terrain Mapping Node Initialized.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainMappingNode>());
    rclcpp::shutdown();
    return 0;
}