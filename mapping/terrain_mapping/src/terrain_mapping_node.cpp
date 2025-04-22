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

// Gait name that indicates the robot is on the ground
// When the robot is in this gait, the ground level is updated
// and the grid map is updated accordingly.
#define GROUND_GAIT_NAME "stand"

/*
 * Terrain Mapping Node
 * This node subscribes to point cloud data and creates a grid map of the terrain.
 */
class TerrainMappingNode : public rclcpp::Node
{
private:
    grid_map::GridMap _grid_map;                 // Grid map object
    std::vector<std::string> _pointcloud_layers; // Layers for point clouds
    double _robot_height = 0.0;                  // Height of the robot
    int _point_thickness = 1;                    // Thickness to place points in the grid map

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> _cloud_subs; // Subscriptions for point clouds
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _gait_sub;                        // Subscription for gait command
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;                      // Subscription for odometry data
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _reset_sub;                        // Subscription for reset command
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _map_pub;                      // Publisher for the grid map
    rclcpp::TimerBase::SharedPtr _timer;                                                     // Timer for periodic updates

    std::unique_ptr<tf2_ros::TransformListener> _tf_listener; // Transform listener for TF2
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;              // Buffer for TF2 transforms

    bool _on_ground = false;               // Flag to check if the robot is on the ground
    double _ground_level = 0.0;            // Ground level of the robot
    double _robot_x = 0.0, _robot_y = 0.0; // Robot position in the map

    /*
     * Update the grid map with point cloud data
     * Called when a new point cloud message is received.
     */
    void _update_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &layer_name)
    {
        // Check if the layer exists in the grid map
        if (!_grid_map.exists(layer_name))
        {
            // If not, add the layer with default values
            _grid_map.add(layer_name, 0.0);
        }

        // Convert the point cloud message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Get the transform from the point cloud frame to the map frame at the time of the message
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            // Lookup the transform using TF2
            transform = _tf_buffer->lookupTransform(
                _grid_map.getFrameId(), msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        // Transform the point cloud to the map frame
        const auto points = _tranform_pointcloud(cloud, transform);

        // Add the points to the grid map
        for (const auto &point : points)
        {
            // Get the corresponding index in the grid map
            grid_map::Index index;
            if (_grid_map.getIndex(grid_map::Position(point.x, point.y), index))
            {
                // Add thickness to the point to inflate the obstacles
                for (int i = -_point_thickness; i <= _point_thickness; i++)
                {
                    for (int j = -_point_thickness; j <= _point_thickness; j++)
                    {
                        // Circular area around the point
                        if (i * i + j * j > _point_thickness * _point_thickness)
                        {
                            continue;
                        }

                        // Get the index offset for the point
                        grid_map::Index index_offset = index + grid_map::Index(i, j);
                        if (_grid_map.isValid(index_offset))
                        {
                            // Reclassify the grid cell as *not* ground
                            _grid_map.at("ground", index_offset) = 0.0;
                            _grid_map.at(layer_name, index_offset) = 1.0;

                            // Update the elevation layer if the point is higher
                            auto &elevation = _grid_map.at("elevation", index_offset);
                            elevation = std::max(elevation, static_cast<float>(point.z));
                        }
                    }
                }
            }
        }
    }

    /*
     * Transform the point cloud to the map frame
     * This function transforms each point in the point cloud to the map frame using the provided transform.
     */
    std::vector<geometry_msgs::msg::Point> _tranform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud, const geometry_msgs::msg::TransformStamped &transform)
    {
        std::vector<geometry_msgs::msg::Point> cloud_out;
        for (const auto &cloud_point : cloud.points)
        {
            // Convert the PCL point to a geometry_msgs::msg::Point
            geometry_msgs::msg::Point point_cam;
            point_cam.x = cloud_point.x;
            point_cam.y = cloud_point.y;
            point_cam.z = cloud_point.z;

            // Perform the transformation
            geometry_msgs::msg::Point point_map;
            tf2::doTransform(point_cam, point_map, transform);

            // Add the transformed point to the output vector
            cloud_out.push_back(point_map);
        }

        // Return the transformed point cloud
        return cloud_out;
    }

    /*
     * Reset the grid map
     * This function clears all layers of the grid map and repopulates it.
     */
    void _reset_map_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        _grid_map.clearAll();
        _publish_map();
    }

    /*
     * Gait callback
     * This function checks if the robot is on the ground based on the gait command.
     */
    void _gait_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        _on_ground = (msg->data == GROUND_GAIT_NAME);
    }

    /*
     * Odometry callback
     * This function updates the robot's position and ground level based on the odometry data.
     */
    void _odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _robot_x = msg->pose.pose.position.x;
        _robot_y = msg->pose.pose.position.y;
        if (_on_ground)
        {
            // Update ground level if the robot is on the ground (standing)
            _ground_level = msg->pose.pose.position.z - _robot_height;
        }
    }

    /*
     * Update the grid map
     * This function is called periodically to update the grid map.
     */
    void _update_map()
    {
        _update_center();
        _update_ground();
        _publish_map();
    }

    /*
     * Update the center of the grid map
     * This function moves the grid map to the robot's position.
     * The robot's position is updated based on the odometry data.
     */
    void _update_center()
    {
        _grid_map.move(grid_map::Position(_robot_x, _robot_y));
    }

    /*
     * Update the ground layer
     * This function updates all grid cells classified as ground to the current ground level.
     */
    void _update_ground()
    {
        for (Eigen::Index i = 0; i < _grid_map.getSize().x(); i++)
        {
            for (Eigen::Index j = 0; j < _grid_map.getSize().y(); j++)
            {
                // Get the ground classification for the cell
                grid_map::Index index(i, j);
                auto &ground = _grid_map.at("ground", index);

                if (std::isnan(ground))
                {
                    // If the ground value is NaN, set it to 1.0
                    // It will be NaN when the map is initialized or reset
                    // Essentially makes the cell a ground cell by default
                    ground = 1.0;
                }

                if (ground == 1.0)
                {
                    // If the cell is classified as ground, set its elevation to the current ground level
                    _grid_map.at("elevation", index) = _ground_level;
                }
            }
        }
    }

    /*
     * Publish the grid map
     * This function converts the grid map to a ROS message and publishes it.
     */
    void _publish_map()
    {
        // Use grid map converter tool
        const auto map_msg = grid_map::GridMapRosConverter::toMessage(_grid_map);

        // Stamp and publish the map
        map_msg->header.stamp = this->now();
        _map_pub->publish(*map_msg);
    }

public:
    TerrainMappingNode() : Node("terrain_mapping_node")
    {
        // Get ROS parameters
        this->declare_parameter<std::string>("map_frame_id", "map");
        const std::string map_frame_id = this->get_parameter("map_frame_id").as_string();

        this->declare_parameter<double>("map_resolution", 0.1);
        const double map_resolution = this->get_parameter("map_resolution").as_double();

        this->declare_parameter<double>("map_length", 10.0);
        const double map_length = this->get_parameter("map_length").as_double();

        this->declare_parameter("frequency", 2.0);
        const double frequency = this->get_parameter("frequency").as_double();

        this->declare_parameter("robot_height", 0.2);
        this->get_parameter("robot_height", _robot_height);

        this->declare_parameter("point_thickness", 1);
        this->get_parameter("point_thickness", _point_thickness);

        this->declare_parameter("pointcloud_layers", std::vector<std::string>{});
        this->get_parameter("pointcloud_layers", _pointcloud_layers);

        // Initialize the grid map
        _grid_map.setFrameId(map_frame_id);
        _grid_map.setGeometry(grid_map::Length(map_length, map_length), map_resolution);
        _grid_map.setBasicLayers({"elevation", "ground"});
        _grid_map.add("elevation");
        _grid_map.add("ground", 1.0);

        // Create the TF2 buffer and listener
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        // Create the publisher and subscribers
        _map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);

        _reset_sub = this->create_subscription<std_msgs::msg::Empty>(
            "map/reset", 10, std::bind(&TerrainMappingNode::_reset_map_callback, this, std::placeholders::_1));

        _gait_sub = this->create_subscription<std_msgs::msg::String>(
            "gait", 10, std::bind(&TerrainMappingNode::_gait_callback, this, std::placeholders::_1));

        _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TerrainMappingNode::_odom_callback, this, std::placeholders::_1));

        for (const auto &layer_name : _pointcloud_layers)
        {
            // Subscribe to each point cloud layer
            _cloud_subs.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points/" + layer_name, 10, [this, layer_name](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                { this->_update_cloud(msg, layer_name); }));
        }

        // Create a timer to periodically update the grid map
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(time_t(1E3 / frequency)),
            std::bind(&TerrainMappingNode::_update_map, this));

        RCLCPP_INFO(this->get_logger(), "Terrain Mapping Node initialized with parameters:");
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", map_frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  Resolution: %fm", map_resolution);
        RCLCPP_INFO(this->get_logger(), "  Length: %fm", map_length);
        RCLCPP_INFO(this->get_logger(), "  Update Rate: %fHz", frequency);
        RCLCPP_INFO(this->get_logger(), "  Point Thickness: %d", _point_thickness);
        RCLCPP_INFO(this->get_logger(), "  Layers: %s", _pointcloud_layers.size() > 0 ? _pointcloud_layers[0].c_str() : "none");
        for (const auto &layer_name : _pointcloud_layers)
        {
            RCLCPP_INFO(this->get_logger(), "  - %s", layer_name.c_str());
        }
    }
};

// Entry point of the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainMappingNode>());
    rclcpp::shutdown();
    return 0;
}