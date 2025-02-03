#include <rclcpp/rclcpp.hpp>

#include "leg_kinematics/leg_kinematics_node.hpp"

class FiveBar2DModel : public LegKinematicsModel
{
private:
    const float _L1 = 0.0;
    const float _L2 = 0.0;
    const float _YA = 1.0;

public:
    FiveBar2DModel(const float L1, const float L2, const float y_axis)
        : _L1(L1), _L2(L2), _YA(y_axis)
    {
    }

    std::size_t getNumMotors() const override
    {
        return 2;
    }

    Vector3f getForwardKinematics(const Vector3f &joint_angles) const override
    {
        const float thetaA = _YA * joint_angles(0);
        const float thetaB = _YA * joint_angles(1);

        const float alpha = 0.5f * (M_PI - thetaA - thetaB);
        const float gamma = std::asin(_L1 * std::sin(alpha) / _L2);
        const float phi = M_PI - alpha - gamma;

        const float theta = -(thetaA + alpha);
        const float R = _L2 * std::sin(phi) / std::sin(alpha);

        const float X = R * std::cos(theta);
        const float Z = R * std::sin(theta);

        return Vector3f(X, 0, Z);
    }

    Vector3f getInverseKinematics(const Vector3f &foot_position) const override
    {
        const float X = foot_position.x();
        const float Z = foot_position.z();

        static int n = 0;
        static bool signZ_last = Z > 0.0f;
        bool signZ = Z > 0.0f;
        if ((X < 0.0f) && (signZ != signZ_last))
        {
            n += signZ ? 1 : -1;
        }
        signZ_last = signZ;

        const float theta0 = 2.0 * M_PI * n - std::atan2(Z, X);
        const float theta1 = M_PI - theta0;
        const float R = std::sqrt(X * X + Z * Z);
        const float alpha = std::acos((R * R + _L1 * _L1 - _L2 * _L2) / (2.0f * R * _L1));

        const float thetaA = theta0 - alpha;
        const float thetaB = theta1 - alpha;

        return Vector3f(_YA * thetaA, _YA * thetaB, 0);
    }

    Matrix3f getJacobian(const Vector3f &joint_angles) const override
    {
        using namespace std;

        const float A = joint_angles(0);
        const float B = joint_angles(1);

        const float Y_2 = _YA / 2.0;
        const float Y_2A = Y_2 * A;
        const float Y_2B = Y_2 * B;
        const float s1 = sin(Y_2A + Y_2B);
        const float c1 = cos(Y_2A + Y_2B);
        const float s2 = sin(Y_2A - Y_2B);
        const float c2 = cos(Y_2A - Y_2B);

        const float as1 = Y_2A + Y_2B - asin((_L1 * c1) / _L2);

        const float t1 = sin(as1) * (Y_2 + (_L1 * Y_2 * s1) / (_L2 * sqrt(-(_L1 * _L1 * c1 * c1 - _L2 * _L2) / (_L2 * _L2))));

        const float t2 = _L2 * t1 / c1;
        const float t3 = _L2 * Y_2 * cos(as1) / c1;
        const float t4 = t3 * s1 / c1;

        const float dXdA = t2 * s2 - t3 * c2 - t4 * s2;
        const float dXdB = t2 * s2 + t3 * c2 - t4 * s2;
        const float dXdC = 0.0;
        const float dYdA = 0.0;
        const float dYdB = 0.0;
        const float dYdC = 1.0;
        const float dZdA = t2 * c2 + t3 * s2 - t4 * c2;
        const float dZdB = t2 * c2 - t3 * s2 - t4 * c2;
        const float dZdC = 0.0;

        Matrix3f jacobian;
        jacobian << dXdA, dXdB, dXdC,
            dYdA, dYdB, dYdC,
            dZdA, dZdB, dZdC;

        return jacobian;
    }
};

namespace leg_kinematics
{

    class FiveBar2DNode : public rclcpp::Node
    {
    private:
        std::unique_ptr<LegKinematicsModel> _model;
        std::unique_ptr<LegKinematicsNode> _leg_kinematics_node;

    public:
        FiveBar2DNode() : Node("fivebar2d_node")
        {
            float L1 = 0.066;
            this->declare_parameter("L1", L1);
            this->get_parameter("L1", L1);

            float L2 = 0.15;
            this->declare_parameter("L2", L2);
            this->get_parameter("L2", L2);

            bool flip_y = false;
            this->declare_parameter("flip_y", flip_y);
            this->get_parameter("flip_y", flip_y);
            const float YA = flip_y ? -1.0 : 1.0;

            _model = std::make_unique<FiveBar2DModel>(L1, L2, YA);
            _leg_kinematics_node = std::make_unique<LegKinematicsNode>(this, _model.get());
        }
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leg_kinematics::FiveBar2DNode>());
    rclcpp::shutdown();
    return 0;
}
