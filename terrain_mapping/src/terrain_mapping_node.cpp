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
    std::string _map_frame_id;
    double _map_resolution;
    double _map_length;
    grid_map::GridMap _grid_map;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _reset_sub;
    rclcpp::TimerBase::SharedPtr _timer;

    const std::string _ground_gait = "stand";
    double _robot_height;
    bool _on_ground = false;
    double _ground_level = 0.0;
    double _robot_x = 0.0, _robot_y = 0.0;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

    int _point_thickness;
    double _cast_radius = 1.0;
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
        _robot_x = msg->pose.pose.position.x;
        _robot_y = msg->pose.pose.position.y;
        if (_on_ground)
        {
            _ground_level = msg->pose.pose.position.z - _robot_height;
        }
    }

    void _update_center()
    {
        _grid_map.move(grid_map::Position(_robot_x, _robot_y));
    }

    void _update_ground()
    {
        for (Eigen::Index i = 0; i < _grid_map.getSize().x(); i++)
        {
            for (Eigen::Index j = 0; j < _grid_map.getSize().y(); j++)
            {
                grid_map::Index index(i, j);
                auto &ground = _grid_map.at("ground", index);
                if (std::isnan(ground))
                {
                    ground = 1.0;
                }
                if (ground == 1.0)
                {
                    _grid_map.at("elevation", index) = _ground_level;
                }
            }
        }
    }

    std::vector<geometry_msgs::msg::Point> _tranform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud, const geometry_msgs::msg::TransformStamped &transform)
    {
        std::vector<geometry_msgs::msg::Point> cloud_out;
        for (const auto &cloud_point : cloud.points)
        {
            geometry_msgs::msg::Point point_cam;
            point_cam.x = cloud_point.x;
            point_cam.y = cloud_point.y;
            point_cam.z = cloud_point.z;

            geometry_msgs::msg::Point point_map;
            tf2::doTransform(point_cam, point_map, transform);
            cloud_out.push_back(point_map);
        }

        return cloud_out;
    }

    std::map<std::pair<float, float>, float> _ray_cast(const std::vector<geometry_msgs::msg::Point> &points)
    {
        std::map<std::pair<float, float>, float> ray_cast;
        for (const auto &point : points)
        {
            const float dx = point.x - _robot_x;
            const float dy = point.y - _robot_y;
            const float distance = std::sqrt(dx * dx + dy * dy);

            const float step = _map_resolution;
            for (float r = 0.0; r < distance; r += step)
            {
                const float factor = r / distance;
                const float x_r = factor * dx + _robot_x;
                const float y_r = factor * dy + _robot_y;
                const std::pair<float, float> key(x_r, y_r);
                if (ray_cast.find(key) == ray_cast.end())
                {
                    ray_cast[key] = std::numeric_limits<float>::quiet_NaN();
                }
            }

            const std::pair<float, float> key(point.x, point.y);
            if (ray_cast.find(key) == ray_cast.end())
            {
                ray_cast[key] = point.z;
            }
            else
            {
                ray_cast[key] = std::max(ray_cast[key], static_cast<float>(point.z));
            }
        }

        for (float rx = -_cast_radius; rx <= +_cast_radius; rx += _map_resolution)
        {
            for (float ry = -_cast_radius; ry <= +_cast_radius; ry += _map_resolution)
            {
                if (rx * rx + ry * ry > _cast_radius * _cast_radius)
                {
                    continue;
                }

                const std::pair<float, float> key(rx + _robot_x, ry + _robot_y);
                if (ray_cast.find(key) == ray_cast.end())
                {
                    ray_cast[key] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        return ray_cast;
    }

    void _update_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &layer_name)
    {
        if (!_grid_map.exists(layer_name))
        {
            _grid_map.add(layer_name, 0.0);
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

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

        const auto points = _tranform_pointcloud(cloud, transform);
        const auto ray_cast = _ray_cast(points);

        for (const auto &point : ray_cast)
        {
            const float x = point.first.first;
            const float y = point.first.second;
            const float z = point.second;
            grid_map::Index index;
            if (_grid_map.getIndex(grid_map::Position(x, y), index))
            {
                if (std::isnan(z))
                {
                    _grid_map.at("ground", index) = 1.0;
                    _grid_map.at(layer_name, index) = 0.0;
                    _grid_map.at("elevation", index) = _ground_level;
                    continue;
                }

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
                            if (elevation < z)
                            {
                                elevation = z;
                            }
                        }
                    }
                }
            }
        }
    }

    void _update_map()
    {
        _update_center();
        _update_ground();
        _publish_map();
    }

public:
    TerrainMappingNode() : Node("terrain_mapping_node")
    {
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->get_parameter("map_frame_id", _map_frame_id);

        this->declare_parameter<double>("map_resolution", 0.1);
        this->get_parameter("map_resolution", _map_resolution);

        this->declare_parameter<double>("map_length", 10.0);
        this->get_parameter("map_length", _map_length);

        this->declare_parameter("frequency", 2.0);
        const double frequency = this->get_parameter("frequency").as_double();

        this->declare_parameter("robot_height", 0.2);
        this->get_parameter("robot_height", _robot_height);

        this->declare_parameter("point_thickness", 1);
        this->get_parameter("point_thickness", _point_thickness);

        this->declare_parameter("cast_radius", 1.0);
        this->get_parameter("cast_radius", _cast_radius);

        this->declare_parameter("pointcloud_layers", std::vector<std::string>{});
        this->get_parameter("pointcloud_layers", _pointcloud_layers);

        _grid_map.setFrameId(_map_frame_id);
        _grid_map.setGeometry(grid_map::Length(_map_length, _map_length), _map_resolution);
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