#include <rclcpp/rclcpp.hpp>

#include "leg_kinematics/leg_kinematics_node.hpp"

class UnitreeA1LegModel : public LegKinematicsModel
{
private:
    const float _D = 0.0;
    const float _Lt = 0.0;
    const float _Lc = 0.0;
    const float _YA = 1.0;

public:
    UnitreeA1LegModel(const float D, const float Lt, const float Lc, const float y_axis)
        : _D(D), _Lt(Lt), _Lc(Lc), _YA(y_axis)
    {
    }

    std::size_t getNumMotors() const override
    {
        return 3;
    }

    Vector3f getForwardKinematics(const Vector3f &joint_angles) const override
    {
        const float q1 = joint_angles(0);
        const float q2 = joint_angles(1);
        const float q3 = joint_angles(2);

        const float s1 = std::sin(q1);
        const float c1 = std::cos(q1);
        const float s2 = std::sin(q2);
        const float c2 = std::cos(q2);
        const float s3 = std::sin(q3);
        const float c3 = std::cos(q3);
        const float s23 = std::sin(q2 + q3);

        const float x = -_Lc * s23 - _Lt * s2;
        const float y = _YA * _D * c1 - _Lc * (s1 * s2 * s3 - c2 * c3 * s1) + _Lt * c2 * q1;
        const float z = _YA * _D * s1 - _Lc * (c1 * c2 * c3 - c1 * s2 * s3) - _Lt * c1 * c2;

        return Vector3f(x, y, z);
    }

    Vector3f getInverseKinematics(const Vector3f &foot_position) const override
    {
        const float x = foot_position(0);
        const float y = _YA * foot_position(1);
        const float z = foot_position(2);

        // Get q1 from Y-Z plane
        const float r1 = std::sqrt(y * y + z * z);
        const float alpha = std::asin(_D / r1);
        const float beta = M_PI_2 - alpha;
        const float gamma = std::atan2(z, y);
        const float q1 = beta + gamma;

        // Transform to the new coordinate system
        const float dy = y - _D * std::cos(q1);
        const float dz = z - _D * std::sin(q1);

        const float zp = -dy * sin(q1) + dz * cos(q1);

        const float r2 = std::sqrt(x * x + zp * zp);
        const float phi = std::atan2(zp, x);
        const float f = phi + M_PI_2;
        const float lambda = std::acos((r2 * r2 + _Lt * _Lt - _Lc * _Lc) / (2 * r2 * _Lt));
        const float del = std::asin(_Lt * std::sin(lambda) / _Lc);
        const float epsilon = M_PI - del - lambda;
        const float q2 = f - lambda;
        const float q3 = M_PI - epsilon;

        return Vector3f(q1 * _YA, -q2, -q3);
    }

    Matrix3f getJacobian(const Vector3f &joint_angles) const override
    {

        const float q1 = joint_angles(0);
        const float q2 = joint_angles(1);
        const float q3 = joint_angles(2);

        const float s1 = std::sin(q1);
        const float c1 = std::cos(q1);
        const float s2 = std::sin(q2);
        const float c2 = std::cos(q2);
        const float s3 = std::sin(q3);
        const float c3 = std::cos(q3);
        const float s23 = std::sin(q2 + q3);
        const float c23 = std::cos(q2 + q3);

        const float dXdq1 = 0.0;
        const float dXdq2 = -_Lc * c23 - _Lt * c2;
        const float dXdq3 = -_Lc * c23;
        const float dYdq1 = _Lc * (c1 * c2 * c3 - c1 * s2 * s3) - _YA * _D * s1 + _Lt * c1 * c2;
        const float dYdq2 = -s1 * (_Lc * s23 + _Lt * s2);
        const float dYdq3 = -_Lc * s1 * s23;
        const float dZdq1 = _YA * _D * c1 - _Lc * (s1 * s2 * s3 - c2 * c3 * s1) + _Lt * c2 * s1;
        const float dZdq2 = c1 * (_Lc * s23 + _Lt * s2);
        const float dZdq3 = _Lc * c1 * s23;

        Matrix3f jacobian;
        jacobian << dXdq1, dXdq2, dXdq3,
            dYdq1, dYdq2, dYdq3,
            dZdq1, dZdq2, dZdq3;

        return jacobian;
    }
};

namespace leg_kinematics
{

    class UnitreeA1LegNode : public rclcpp::Node
    {
    private:
        std::unique_ptr<LegKinematicsModel> _model;
        std::unique_ptr<LegKinematicsNode> _leg_kinematics_node;

    public:
        UnitreeA1LegNode(const rclcpp::NodeOptions &options)
            : Node("unitree_a1_leg_node", options)
        {
            float D = 0.08505;
            this->declare_parameter("D", D);
            this->get_parameter("D", D);

            float Lt = 0.2;
            this->declare_parameter("Lt", Lt);
            this->get_parameter("Lt", Lt);

            float Lc = 0.2;
            this->declare_parameter("Lc", Lc);
            this->get_parameter("Lc", Lc);

            bool flip_y = false;
            this->declare_parameter("flip_y", flip_y);
            this->get_parameter("flip_y", flip_y);
            const float YA = flip_y ? -1.0 : 1.0;

            _model = std::make_unique<UnitreeA1LegModel>(D, Lt, Lc, YA);
            _leg_kinematics_node = std::make_unique<LegKinematicsNode>(this, _model.get());
        }
    };

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(leg_kinematics::UnitreeA1LegNode)