#pragma once

#include "legged_mpc/mpc.hpp"

using namespace std::chrono;

struct LeggedMPCConfig
{
    std::size_t window_size;
    milliseconds time_step;

    std::size_t num_legs;
    Vector3d gravity_vector;
    double body_mass;
    Matrix3d body_inertia;
    double friction_coefficient_x, friction_coefficient_y;
    double force_z_min, force_z_max;

    Vector3d position_weights, orientation_weights;
    Vector3d linear_velocity_weights, angular_velocity_weights;
    Vector3d force_weights;

    std::vector<Vector3d> position, orientation;
    std::vector<Vector3d> linear_velocity, angular_velocity;

    std::vector<std::size_t> num_stance;
    std::vector<std::vector<bool>> in_stance;
    std::vector<std::vector<Vector3d>> foothold_positions;
};

static inline Matrix3d getSkewSymmetricMatrix(const Vector3d &v)
{
    Matrix3d S;
    S << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return S;
}

MPCProblem getMPCProblem(const LeggedMPCConfig &config)
{
    assert(config.window_size > 1);

    MPCProblem mpc;
    mpc.window_size = config.window_size;
    mpc.xref.resize(mpc.window_size);
    mpc.Q.resize(mpc.window_size);
    mpc.R.resize(mpc.window_size - 1);
    mpc.A.resize(mpc.window_size - 1);
    mpc.B.resize(mpc.window_size - 1);
    mpc.C.resize(mpc.window_size - 1);
    mpc.lb.resize(mpc.window_size - 1);
    mpc.ub.resize(mpc.window_size - 1);

    const double invm = 1.0 / config.body_mass;
    const Matrix3d invI = config.body_inertia.inverse();

    assert(config.linear_velocity.size() == mpc.window_size);
    assert(config.angular_velocity.size() == mpc.window_size);
    assert(config.num_stance.size() == mpc.window_size);
    assert(config.in_stance.size() == mpc.window_size);
    assert(config.foothold_positions.size() == mpc.window_size);
    for (std::size_t k = 0; k < mpc.window_size; k++)
    {
        // X reference
        mpc.xref[k] = Vector<OSQPFloat, 13>::Zero();

        mpc.xref[k].block<3, 1>(0, 0) = config.orientation[k];
        mpc.xref[k].block<3, 1>(3, 0) = config.position[k];
        mpc.xref[k].block<3, 1>(6, 0) = config.angular_velocity[k];
        mpc.xref[k].block<3, 1>(9, 0) = config.linear_velocity[k];
        mpc.xref[k](12) = 1.0;

        // Q
        mpc.Q[k] = Matrix<OSQPFloat, 13, 13>::Zero();
        mpc.Q[k].block<3, 3>(0, 0) = config.orientation_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(3, 3) = config.position_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(6, 6) = config.angular_velocity_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(9, 9) = config.linear_velocity_weights.asDiagonal();
        mpc.Q[k](12, 12) = 0.0;

        if (k < config.window_size - 1)
        {
            const std::size_t Ns = config.num_stance[k];
            assert(Ns <= config.num_legs);

            // R
            mpc.R[k] = MatrixX<OSQPFloat>::Zero(3 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.R[k].block<3, 3>(3 * i, 3 * i) = config.force_weights.asDiagonal();
            }

            // A
            mpc.A[k] = Matrix<OSQPFloat, 13, 13>::Zero();
            mpc.A[k].block<3, 3>(0, 6) = AngleAxisd(config.orientation[k].z(), Vector3d::UnitZ()).toRotationMatrix();
            mpc.A[k].block<3, 3>(3, 9) = Matrix3d::Identity();
            mpc.A[k].block<3, 1>(9, 12) = config.gravity_vector;
            mpc.A[k] = mpc.A[k] * config.time_step.count() + Matrix<OSQPFloat, 13, 13>::Identity();

            const auto &in_stance = config.in_stance[k];
            assert(in_stance.size() == config.num_legs);

            const auto &leg_positions = config.foothold_positions[k];
            assert(leg_positions.size() == config.num_legs);

            // B
            mpc.B[k] = MatrixX<OSQPFloat>::Zero(13, 3 * Ns);
            for (std::size_t i = 0, j = 0; i < config.num_legs; i++)
            {
                if (in_stance[i])
                {
                    const Matrix3d invIskewr = invI * getSkewSymmetricMatrix(leg_positions[i]);
                    mpc.B[k].block<3, 3>(6, 3 * j) = invIskewr;
                    mpc.B[k].block<3, 3>(9, 3 * j) = invm * Matrix3d::Identity();
                    j++;
                }
            }
            mpc.B[k] = mpc.B[k] * config.time_step.count();

            // C
            mpc.C[k] = MatrixX<OSQPFloat>::Zero(5 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                const auto mux = config.friction_coefficient_x;
                const auto muy = config.friction_coefficient_y;
                mpc.C[k].block<3, 3>(5 * i, 3 * i) << 0, 0, 1,
                    -1, 0, -mux,
                    1, 0, -mux,
                    0, -1, -muy,
                    0, 1, -muy;
            }

            // Bounds
            mpc.lb[k] = VectorX<OSQPFloat>::Constant(5 * Ns, -std::numeric_limits<OSQPFloat>::infinity());
            mpc.ub[k] = VectorX<OSQPFloat>::Constant(5 * Ns, 0);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.lb[k](5 * i) = config.force_z_min;
                mpc.ub[k](5 * i) = config.force_z_max;
            }
        }
    }

    return mpc;
}