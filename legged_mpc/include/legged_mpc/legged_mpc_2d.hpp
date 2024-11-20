#pragma once

#include "legged_mpc/osqp_mpc.hpp"

#define NUM_STATES 7
#define NUM_CONTROLS_PER_LEG 2
#define NUM_CONSTRAINTS (2 * NUM_CONTROLS_PER_LEG - 1)

using OSQPVectorNC = Eigen::Vector<OSQPFloat, NUM_CONTROLS_PER_LEG>;
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
    double friction_coefficient_x;
    double force_z_min, force_z_max;

    OSQPVector3 linear_velocity_weights, angular_velocity_weights;
    OSQPVectorNC force_weights;

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
    mpc.x0 = Vector<OSQPFloat, NUM_STATES>::Zero();
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
        mpc.xref[k] = Vector<OSQPFloat, NUM_STATES>::Zero();
        mpc.xref[k].block<3, 1>(0, 0) = config.angular_velocity[k];
        mpc.xref[k].block<3, 1>(3, 0) = config.linear_velocity[k];
        mpc.xref[k](6) = 1.0;

        // Q
        mpc.Q[k] = Matrix<OSQPFloat, NUM_STATES, NUM_STATES>::Zero();
        mpc.Q[k].block<3, 3>(0, 0) = config.angular_velocity_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(3, 3) = config.linear_velocity_weights.asDiagonal();
        mpc.Q[k](6, 6) = 0.0;

        // Cx
        mpc.C[k] = Matrix<OSQPFloat, 0, NUM_STATES>::Zero();

        // Bounds X
        mpc.lbx[k] = Vector<OSQPFloat, 0>();
        mpc.ubx[k] = Vector<OSQPFloat, 0>();

        if (k < config.N - 1)
        {
            const std::size_t Ns = config.num_stance[k];
            assert(Ns <= config.num_legs);

            // R
            mpc.R[k] = MatrixX<OSQPFloat>::Zero(NUM_CONTROLS_PER_LEG * Ns, NUM_CONTROLS_PER_LEG * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.R[k].block<NUM_CONTROLS_PER_LEG, NUM_CONTROLS_PER_LEG>(
                    NUM_CONTROLS_PER_LEG * i, NUM_CONTROLS_PER_LEG * i) = config.force_weights.asDiagonal();
            }

            // A
            mpc.A[k] = Matrix<OSQPFloat, NUM_STATES, NUM_STATES>::Zero();
            mpc.A[k].block<3, 1>(3, 6) = config.gravity_vector;
            mpc.A[k] = mpc.A[k] * config.time_step + Matrix<OSQPFloat, NUM_STATES, NUM_STATES>::Identity();

            const auto &in_stance = config.in_stance[k];
            assert(in_stance.size() == config.num_legs);

            const auto &leg_positions = config.foothold_positions[k];
            assert(leg_positions.size() == config.num_legs);

            // B
            mpc.B[k] = MatrixX<OSQPFloat>::Zero(NUM_STATES, NUM_CONTROLS_PER_LEG * Ns);
            for (std::size_t i = 0, j = 0; i < config.num_legs; i++)
            {
                if (in_stance[i])
                {
                    const Vector3d r = leg_positions[i];
                    const OSQPMatrix3 invIskewr = invI * getSkewSymmetricMatrix(r);

                    Matrix<OSQPFloat, 3, NUM_CONTROLS_PER_LEG> invIskewr_xz;
                    invIskewr_xz.block<3, 1>(0, 0) = invIskewr.col(0);
                    invIskewr_xz.block<3, 1>(0, 1) = invIskewr.col(2);

                    Matrix<OSQPFloat, 3, NUM_CONTROLS_PER_LEG> eye_xz;
                    eye_xz.block<3, 1>(0, 0) = OSQPVector3::UnitX();
                    eye_xz.block<3, 1>(0, 1) = OSQPVector3::UnitZ();

                    mpc.B[k].block<3, NUM_CONTROLS_PER_LEG>(0, NUM_CONTROLS_PER_LEG * j) = invIskewr_xz;
                    mpc.B[k].block<3, NUM_CONTROLS_PER_LEG>(3, NUM_CONTROLS_PER_LEG * j) = invm * eye_xz;
                    j++;
                }
            }
            mpc.B[k] = mpc.B[k] * config.time_step;

            // Cu
            mpc.D[k] = MatrixX<OSQPFloat>::Zero(NUM_CONSTRAINTS * Ns, NUM_CONTROLS_PER_LEG * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                const auto mux = config.friction_coefficient_x;
                mpc.D[k].block<NUM_CONSTRAINTS, NUM_CONTROLS_PER_LEG>(NUM_CONSTRAINTS * i, NUM_CONTROLS_PER_LEG * i) << 0, 1, -1, -mux, 1, -mux;
            }

            // Bounds U
            mpc.lbu[k] = VectorX<OSQPFloat>::Constant(NUM_CONSTRAINTS * Ns, -std::numeric_limits<OSQPFloat>::infinity());
            mpc.ubu[k] = VectorX<OSQPFloat>::Constant(NUM_CONSTRAINTS * Ns, 0);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.lbu[k](NUM_CONSTRAINTS * i) = config.force_z_min;
                mpc.ubu[k](NUM_CONSTRAINTS * i) = config.force_z_max;
            }
        }
    }

    return mpc;
}