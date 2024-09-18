#pragma once

#include "legged_mpc/osqp_mpc.hpp"

using OSQPVector3 = Eigen::Vector3<OSQPFloat>;
using OSQPMatrix3 = Eigen::Matrix3<OSQPFloat>;

struct LeggedMPCConfig
{
    std::size_t N;
    std::chrono::milliseconds time_step;

    std::size_t num_legs;
    OSQPVector gravity_vector;
    double body_mass;
    OSQPMatrix3 body_inertia;
    double friction_coefficient_x, friction_coefficient_y;
    double force_z_min, force_z_max;

    OSQPVector position_weights, orientation_weights;
    OSQPVector linear_velocity_weights, angular_velocity_weights;
    OSQPVector force_weights;

    std::vector<OSQPVector> position, orientation;
    std::vector<OSQPVector> linear_velocity, angular_velocity;

    std::vector<std::size_t> num_stance;
    std::vector<std::vector<bool>> in_stance;
    std::vector<std::vector<OSQPVector>> foothold_positions;
};

static inline OSQPMatrix3 getSkewSymmetricMatrix(const OSQPVector &v)
{
    OSQPMatrix3 S;
    S << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return S;
}

MPCProblem getMPCProblem(const LeggedMPCConfig &config)
{
    using namespace Eigen;
    
    assert(config.N > 1);

    MPCProblem mpc;
    mpc.N = config.N;
    mpc.xref.resize(mpc.N);
    mpc.Q.resize(mpc.N);
    mpc.R.resize(mpc.N - 1);
    mpc.A.resize(mpc.N - 1);
    mpc.B.resize(mpc.N - 1);
    mpc.C.resize(mpc.N - 1);
    mpc.lb.resize(mpc.N - 1);
    mpc.ub.resize(mpc.N - 1);

    const double invm = 1.0 / config.body_mass;
    const OSQPMatrix3 invI = config.body_inertia.inverse();

    assert(config.linear_velocity.size() == mpc.N);
    assert(config.angular_velocity.size() == mpc.N);
    assert(config.num_stance.size() == mpc.N);
    assert(config.in_stance.size() == mpc.N);
    assert(config.foothold_positions.size() == mpc.N);
    for (std::size_t k = 0; k < mpc.N; k++)
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

        if (k < config.N - 1)
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
            mpc.A[k].block<3, 3>(0, 6) = AngleAxisd(config.orientation[k].z(), OSQPVector3::UnitZ()).toRotationMatrix();
            mpc.A[k].block<3, 3>(3, 9) = OSQPMatrix3::Identity();
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
                    const OSQPMatrix3 invIskewr = invI * getSkewSymmetricMatrix(leg_positions[i]);
                    mpc.B[k].block<3, 3>(6, 3 * j) = invIskewr;
                    mpc.B[k].block<3, 3>(9, 3 * j) = invm * OSQPMatrix3::Identity();
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