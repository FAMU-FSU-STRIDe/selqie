#pragma once

#include "legged_mpc/mpc.hpp"

struct StanceState
{
    std::size_t num_stance;
    std::vector<bool> in_stance;
    std::vector<Vector3d> leg_positions;
};

struct LeggedMPCConfig
{
    std::size_t window_size;
    std::chrono::milliseconds time_step;
    std::size_t num_legs;
    Vector3d gravity_vector;
    double body_mass;
    Matrix3d body_inertia;
    double friction_coefficient_x, friction_coefficient_y;
    double force_z_min, force_z_max;
    Vector3d linear_velocity, angular_velocity;
    Vector3d linear_velocity_weights, angular_velocity_weights, force_weights;
    std::vector<StanceState> stance_trajectory;
};

static inline Matrix3d getSkewSymmetricMatrix(const Vector3d &v)
{
    Matrix3d S;
    S << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return S;
}

MPCProblem getMPCProblem(const LeggedMPCConfig &control)
{
    assert(control.window_size > 1);

    MPCProblem mpc;
    mpc.window_size = control.window_size;
    mpc.xref.resize(mpc.window_size);
    mpc.Q.resize(mpc.window_size);
    mpc.R.resize(mpc.window_size - 1);
    mpc.A.resize(mpc.window_size - 1);
    mpc.B.resize(mpc.window_size - 1);
    mpc.C.resize(mpc.window_size - 1);
    mpc.lb.resize(mpc.window_size - 1);
    mpc.ub.resize(mpc.window_size - 1);

    const double invm = 1.0 / control.body_mass;
    const Matrix3d invI = control.body_inertia.inverse();

    for (std::size_t k = 0; k < mpc.window_size; k++)
    {
        const StanceState &stance_state = control.stance_trajectory[k];
        const auto Ns = stance_state.num_stance;

        // X reference
        mpc.xref[k] = Vector<OSQPFloat, 7>::Zero();
        mpc.xref[k].block<3, 1>(0, 0) = control.angular_velocity;
        mpc.xref[k].block<3, 1>(3, 0) = control.linear_velocity;
        mpc.xref[k](6) = 1.0;

        // Q
        mpc.Q[k] = Matrix<OSQPFloat, 7, 7>::Zero();
        mpc.Q[k].block<3, 3>(0, 0) = control.linear_velocity_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(3, 3) = control.angular_velocity_weights.asDiagonal();
        mpc.Q[k](6, 6) = 0.0;

        if (k < control.window_size - 1)
        {
            // R
            mpc.R[k] = MatrixX<OSQPFloat>::Zero(3 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.R[k].block<3, 3>(3 * i, 3 * i) = control.force_weights.asDiagonal();
            }

            // A
            mpc.A[k] = Matrix<OSQPFloat, 7, 7>::Zero();
            mpc.A[k].block<3, 1>(3, 6) = control.gravity_vector;
            mpc.A[k] = mpc.A[k] * control.time_step.count() + Matrix<OSQPFloat, 7, 7>::Identity();

            // B
            assert(stance_state.in_stance.size() == control.num_legs);
            mpc.B[k] = MatrixX<OSQPFloat>::Zero(7, 3 * Ns);
            for (std::size_t i = 0, j = 0; i < control.num_legs; i++)
            {
                if (stance_state.in_stance[i])
                {
                    const Matrix3d invIskewr = invI * getSkewSymmetricMatrix(stance_state.leg_positions[i]);
                    mpc.B[k].block<3, 3>(0, 3 * j) = invIskewr;
                    mpc.B[k].block<3, 3>(3, 3 * j) = invm * Matrix3d::Identity();
                    j++;
                }
            }
            mpc.B[k] = mpc.B[k] * control.time_step.count();

            // C
            mpc.C[k] = MatrixX<OSQPFloat>::Zero(5 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                const auto mux = control.friction_coefficient_x;
                const auto muy = control.friction_coefficient_y;
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
                mpc.lb[k](5 * i) = control.force_z_min;
                mpc.ub[k](5 * i) = control.force_z_max;
            }
        }
    }

    return mpc;
}