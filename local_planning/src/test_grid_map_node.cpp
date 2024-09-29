#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

class TestGridMapNode : public rclcpp::Node
{
private:
    std::vector<std::array<double, 5>> _boxes;     // x, y, len, wid, height
    std::vector<std::array<double, 4>> _cylinders; // x, y, rad, height

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;

public:
    TestGridMapNode() : Node("test_grid_map_node")
    {
        _boxes = {
            {0.5, 0.5, 0.25, 0.5, 0.05},
        };

        _cylinders = {
            {-0.5, 0.2, 0.25, 0.05},
            {0.0, -0.5, 0.25, 0.1},
        };

        grid_map::GridMap map({"elevation", "cost"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(5.0, 5.0), 0.01);
        map.add("elevation", 0.0);
        map.add("cost", 0.0);

        for (const auto &box : _boxes)
        {
            grid_map::Position center(box[0], box[1]);
            grid_map::Length length(box[2], box[3]);
            bool success;
            grid_map::SubmapGeometry submap(map, center, length, success);
            for (auto iter = grid_map::SubmapIterator(submap); !iter.isPastEnd(); ++iter)
            {
                map.at("elevation", *iter) = box[4];
                map.at("cost", *iter) = std::numeric_limits<float>::infinity();
            }
        }

        for (const auto &cylinder : _cylinders)
        {
            grid_map::Position center(cylinder[0], cylinder[1]);
            for (auto iter = grid_map::CircleIterator(map, center, cylinder[2]); !iter.isPastEnd(); ++iter)
            {
                map.at("elevation", *iter) = cylinder[3];
                map.at("cost", *iter) = std::numeric_limits<float>::infinity();
            }
        }

        auto msg = *grid_map::GridMapRosConverter::toMessage(map).get();

        _pub = this->create_publisher<grid_map_msgs::msg::GridMap>("map", 10);
        _timer = this->create_wall_timer(std::chrono::seconds(1), [this, msg]()
                                         { _pub->publish(msg); });

        RCLCPP_INFO(this->get_logger(), "TestGridMapNode has been started.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestGridMapNode>());
    rclcpp::shutdown();
    return 0;
}