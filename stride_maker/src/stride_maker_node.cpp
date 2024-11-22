#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/leg_trajectory.hpp>

class StrideMakerNode : public rclcpp::Node
{
private:
    int _resolution = 200;
    double _step_height = 0.05;
    double _leg_length = 0.2;

    void make_stride(const double stride_frequency, const double stance_length, const double duty_factor, const double offset = 0.0)
    {
        const double duration = 1.0 / stride_frequency;
        const double stance_duration = duration * duty_factor;
        const double swing_duration = duration * (1.0 - duty_factor);
        const int points_per_quadrant = _resolution / 4;
        const double phi0 = std::asin(0.5 * stance_length / _leg_length);
        const double R0 = 0.5 * stance_length;
        const double R1 = _step_height;
        const double body_height = std::sqrt(_leg_length * _leg_length - 0.25 * stance_length * stance_length);
        const double stance_dt = 0.5 * stance_duration / points_per_quadrant;
        const double swing_dt = 0.5 * swing_duration / points_per_quadrant;
        const int offset_index = offset * _resolution;

        double t = 0.0;
        std::vector<double> pt, px, pz;
        for (int k = 0; k < _resolution; k++)
        {
            pt.push_back(t);

            const int p = (k + offset_index) % _resolution;
            const int q = p / points_per_quadrant;
            const double i = p % points_per_quadrant;
            const double f = i / points_per_quadrant;
            switch (q)
            {
            case 0:
            {
                const double R = _leg_length;
                const double theta = phi0 * (1.0 - f) - (M_PI / 2.0);
                const double x = R * cos(theta);
                const double z = R * sin(theta);
                px.push_back(x);
                pz.push_back(z);
                t += stance_dt;
                break;
            }
            case 1:
            {
                const double R = _leg_length;
                const double theta = -(M_PI / 2.0) - phi0 * f;
                const double x = R * cos(theta);
                const double z = R * sin(theta);
                px.push_back(x);
                pz.push_back(z);
                t += stance_dt;
                break;
            }
            case 2:
            {
                const double R = R0 + (R1 - R0) * f;
                const double theta = (M_PI / 2.0) * (2.0 - f);
                const double x = R * cos(theta);
                const double z = R * sin(theta) - body_height;
                px.push_back(x);
                pz.push_back(z);
                t += swing_dt;
                break;
            }
            case 3:
            {
                const double R = R1 + (R0 - R1) * f;
                const double theta = (M_PI / 2.0) * (1.0 - f);
                const double x = R * cos(theta);
                const double z = R * sin(theta) - body_height;
                px.push_back(x);
                pz.push_back(z);
                t += swing_dt;
            }
            }
        }
    }

public:
    StrideMakerNode() : Node("stride_maker_node")
    {

        RCLCPP_INFO(this->get_logger(), "Stride Maker Node Initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}