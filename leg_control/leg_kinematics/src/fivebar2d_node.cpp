#include <rclcpp/rclcpp.hpp>

#include "leg_kinematics/leg_kinematics_node.hpp"

/*
 * FiveBar2DModel class based on LegKinematicsModel
 * See kinematic diagram in the media folder for reference.
 */
class FiveBar2DModel : public LegKinematicsModel
{
private:
    const float _L1 = 0.0; // Length of the first link
    const float _L2 = 0.0; // Length of the second link
    const float _YA = 1.0; // Y-axis direction factor (1.0 for normal, -1.0 for flipped)

public:
    FiveBar2DModel(const float L1, const float L2, const float y_axis)
        : _L1(L1), _L2(L2), _YA(y_axis)
    {
    }

    /*
     * Number of motors in the leg kinematics model.
     */
    std::size_t get_num_motors() const override
    {
        return 2;
    }

    /*
     * Forward kinematic function
     * Maps joint angles (motor positions) to the foot position in 3D space.
     */
    Vector3f compute_forward_kinematics(const Vector3f &joint_angles) const override
    {
        const float thetaA = _YA * joint_angles(0);
        const float thetaB = _YA * joint_angles(1);

        const float alpha = 0.5f * (+M_PI - thetaA - thetaB);
        const float gamma = std::asin(_L1 * std::sin(alpha) / _L2);
        const float phi = M_PI - alpha - gamma;

        const float theta = 0.5f * (-M_PI - thetaA + thetaB);
        const float R = _L2 * std::sin(phi) / std::sin(alpha);

        const float X = R * std::cos(theta);
        const float Z = R * std::sin(theta);

        return Vector3f(X, 0, Z);
    }

    /*
     * Inverse kinematic function
     * Maps a desired foot position in 3D space to the required joint angles (motor positions).
     */
    Vector3f compute_inverse_kinematics(const Vector3f &foot_position) const override
    {
        const float X = foot_position.x();
        const float Z = foot_position.z();

        // Loop correction technique
        // This ensures that the angles don't jump discontinuously at the domain border
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

    /*
     * Jacobian matrix function
     * Computes the Jacobian matrix for the leg kinematics model.
     */
    Matrix3f compute_jacobian_matrix(const Vector3f &joint_angles) const override
    {
        // Evaluate forward kinematics
        const float thetaA = _YA * joint_angles(0);
        const float thetaB = _YA * joint_angles(1);

        const float alpha = 0.5f * (+M_PI - thetaA - thetaB);
        const float sin_alpha = std::sin(alpha);
        const float cos_alpha = std::cos(alpha);

        const float gamma = std::asin(_L1 * sin_alpha / _L2);

        const float phi = M_PI - alpha - gamma;
        const float sin_phi = std::sin(phi);
        const float cos_phi = std::cos(phi);

        const float theta = 0.5f * (-M_PI - thetaA + thetaB);
        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);

        const float R = _L2 * sin_phi / sin_alpha;

        // Get partial derivatives
        const float dXdR = cos_theta;
        const float dXdT = -R * sin_theta;
        const float dZdR = sin_theta;
        const float dZdT = R * cos_theta;

        const float dRdA = -_L2 * sin_phi * cos_alpha / (sin_alpha * sin_alpha);
        const float dRdP = _L2 * cos_phi / sin_alpha;

        const float dPdA = -1;
        const float dPdG = -1;

        const float L1L2 = _L1 / _L2;
        const float dGdA = L1L2 * cos_alpha / std::sqrt(1.0f - L1L2 * L1L2 * sin_alpha * sin_alpha);

        const float dAd0 = -0.5f * _YA;
        const float dAd1 = -0.5f * _YA;
        const float dTd0 = -0.5f * _YA;
        const float dTd1 = +0.5f * _YA;

        // Evaluate jacobian
        const float dXd0 = dXdR * (dRdA * dAd0 + dRdP * (dPdA * dAd0 + dPdG * dGdA * dAd0)) + dXdT * dTd0;
        const float dXd1 = dXdR * (dRdA * dAd1 + dRdP * (dPdA * dAd1 + dPdG * dGdA * dAd1)) + dXdT * dTd1;
        const float dXd2 = 0.0;
        const float dYd0 = 0.0;
        const float dYd1 = 0.0;
        const float dYd2 = 1.0;
        const float dZd0 = dZdR * (dRdA * dAd0 + dRdP * (dPdA * dAd0 + dPdG * dGdA * dAd0)) + dZdT * dTd0;
        const float dZd1 = dZdR * (dRdA * dAd1 + dRdP * (dPdA * dAd1 + dPdG * dGdA * dAd1)) + dZdT * dTd1;
        const float dZd2 = 0.0;

        Matrix3f jacobian;
        jacobian << dXd0, dXd1, dXd2,
            dYd0, dYd1, dYd2,
            dZd0, dZd1, dZd2;

        return jacobian;
    }
};

/*
 * FiveBar2DNode class
 */
class FiveBar2DNode : public rclcpp::Node
{
private:
    std::unique_ptr<LegKinematicsModel> _model;              // Pointer to the leg kinematics model
    std::unique_ptr<LegKinematicsNode> _leg_kinematics_node; // Pointer to the leg kinematics node

public:
    FiveBar2DNode() : Node("fivebar2d_node")
    {
        // Get the parameters for the leg model
        this->declare_parameter("L1", 0.066);
        const float L1 = this->get_parameter("L1").as_double();

        this->declare_parameter("L2", 0.15);
        const float L2 = this->get_parameter("L2").as_double();

        this->declare_parameter("flip_y", false);
        const bool flip_y = this->get_parameter("flip_y").as_bool();
        const float YA = flip_y ? -1.0 : 1.0;

        // Create the leg model
        _model = std::make_unique<FiveBar2DModel>(L1, L2, YA);

        // Initialize the leg kinematics node
        _leg_kinematics_node = std::make_unique<LegKinematicsNode>(this, _model.get());
    }
};

// Entry point for the FiveBar2DNode
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FiveBar2DNode>());
    rclcpp::shutdown();
    return 0;
}
