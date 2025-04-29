#pragma once

#include "legged_mpc/osqp_mpc.hpp"

using OSQPVector3 = Eigen::Vector3<OSQPFloat>;
using OSQPMatrix3 = Eigen::Matrix3<OSQPFloat>;

struct LeggedMPCConfig
{
    std::size_t N;
    double time_step;

    std::size_t num_legs;
    OSQPVector3 gravity_vector;
    double body_mass;
    OSQPMatrix3 body_inertia;
    double friction_coefficient_x, friction_coefficient_y;
    double force_z_min, force_z_max;

    OSQPVector3 linear_velocity_weights, angular_velocity_weights;
    OSQPVector3 force_weights;

    std::vector<OSQPVector3> linear_velocity, angular_velocity;

    std::vector<std::size_t> num_stance;
    std::vector<std::vector<bool>> in_stance;
    std::vector<std::vector<OSQPVector3>> foothold_positions;
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
    assert(config.N > 1);

    MPCProblem mpc(config.N);
    
    using namespace Eigen;

    assert(config.linear_velocity.size() == mpc.N);
    assert(config.angular_velocity.size() == mpc.N);
    mpc.x0 = Vector<OSQPFloat, 7>::Zero();
    mpc.x0.block<3, 1>(0, 0) = config.angular_velocity[0];
    mpc.x0.block<3, 1>(3, 0) = config.linear_velocity[0];
    mpc.x0(6) = 1.0;

    assert(config.body_inertia.determinant() != 0.0);
    const double invm = 1.0 / config.body_mass;
    const OSQPMatrix3 invI = config.body_inertia.inverse();

    assert(config.num_stance.size() == mpc.N);
    assert(config.in_stance.size() == mpc.N);
    assert(config.foothold_positions.size() == mpc.N);
    for (std::size_t k = 0; k < mpc.N; k++)
    {
        // X reference
        mpc.xref[k] = Vector<OSQPFloat, 7>::Zero();
        mpc.xref[k].block<3, 1>(0, 0) = config.angular_velocity[k];
        mpc.xref[k].block<3, 1>(3, 0) = config.linear_velocity[k];
        mpc.xref[k](6) = 1.0;

        // Q
        mpc.Q[k] = Matrix<OSQPFloat, 7, 7>::Zero();
        mpc.Q[k].block<3, 3>(0, 0) = config.angular_velocity_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(3, 3) = config.linear_velocity_weights.asDiagonal();
        mpc.Q[k](6, 6) = 0.0;

        // Cx
        mpc.C[k] = Matrix<OSQPFloat, 0, 7>::Zero();

        // Bounds X
        mpc.lbx[k] = Vector<OSQPFloat, 0>();
        mpc.ubx[k] = Vector<OSQPFloat, 0>();

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
            mpc.A[k] = Matrix<OSQPFloat, 7, 7>::Zero();
            mpc.A[k].block<3, 1>(3, 6) = config.gravity_vector;
            mpc.A[k] = mpc.A[k] * config.time_step + Matrix<OSQPFloat, 7, 7>::Identity();

            const auto &in_stance = config.in_stance[k];
            assert(in_stance.size() == config.num_legs);

            const auto &leg_positions = config.foothold_positions[k];
            assert(leg_positions.size() == config.num_legs);

            // B
            mpc.B[k] = MatrixX<OSQPFloat>::Zero(7, 3 * Ns);
            for (std::size_t i = 0, j = 0; i < config.num_legs; i++)
            {
                if (in_stance[i])
                {
                    const Vector3d r = leg_positions[i];
                    const OSQPMatrix3 invIskewr = invI * getSkewSymmetricMatrix(r);
                    mpc.B[k].block<3, 3>(0, 3 * j) = invIskewr;
                    mpc.B[k].block<3, 3>(3, 3 * j) = invm * OSQPMatrix3::Identity();
                    j++;
                }
            }
            mpc.B[k] = mpc.B[k] * config.time_step;

            // Cu
            mpc.D[k] = MatrixX<OSQPFloat>::Zero(5 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                const auto mux = config.friction_coefficient_x;
                const auto muy = config.friction_coefficient_y;
                mpc.D[k].block<5, 3>(5 * i, 3 * i) << 0, 0, 1,
                    -1, 0, -mux,
                    1, 0, -mux,
                    0, -1, -muy,
                    0, 1, -muy;
            }

            // Bounds U
            mpc.lbu[k] = VectorX<OSQPFloat>::Constant(5 * Ns, -std::numeric_limits<OSQPFloat>::infinity());
            mpc.ubu[k] = VectorX<OSQPFloat>::Constant(5 * Ns, 0);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.lbu[k](5 * i) = config.force_z_min;
                mpc.ubu[k](5 * i) = config.force_z_max;
            }
        }
    }

    return mpc;
}