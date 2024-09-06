#pragma once

#include "legged_mpc/mpc.hpp"

using namespace std::chrono;

struct SequencePattern
{
    static constexpr uint32_t GAIT_RESOLUTION = 1000;

    milliseconds duration = milliseconds(0);
    std::size_t num_stance;
    std::map<milliseconds, std::vector<bool>> stance_timing;
};

struct StanceSequence
{
    static constexpr uint32_t GAIT_RESOLUTION = 1000;

    milliseconds start_time = milliseconds(0);
    milliseconds current_time;
    SequencePattern current_pattern, next_pattern;
};

struct StanceState
{
    std::size_t num_stance;
    std::vector<bool> in_stance;
};

struct LeggedMPCConfig
{
    std::size_t window_size;
    milliseconds time_step;
    std::size_t num_legs;
    Vector3d gravity_vector;
    double body_mass;
    Matrix3d body_inertia;
    std::vector<Vector3d> hip_positions;
    std::vector<Vector3d> default_leg_positions;
    double friction_coefficient_x, friction_coefficient_y;
    double force_z_min, force_z_max;
    Vector3d current_linear_velocity, current_angular_velocity;
    Vector3d desired_linear_velocity, desired_angular_velocity;
    Vector3d linear_velocity_weights, angular_velocity_weights, force_weights;
    StanceSequence stance_sequence;
};

static inline std::vector<StanceState> getStanceTrajectory(const LeggedMPCConfig &config)
{
    const auto &start_time = config.stance_sequence.start_time;
    const auto &current_time = config.stance_sequence.current_time;
    const auto &current_pattern = config.stance_sequence.current_pattern;
    const auto &next_pattern = config.stance_sequence.next_pattern;

    const auto end_time = start_time + current_pattern.duration;
    std::vector<StanceState> stance_trajectory(config.window_size);
    for (std::size_t k = 0; k < config.window_size; k++)
    {
        const auto time = current_time + k * config.time_step;
        const auto rel_time = time < end_time ? time - start_time : (time - end_time) % next_pattern.duration;
        const auto &pattern = time < end_time ? current_pattern : next_pattern;

        const auto stance_it = pattern.stance_timing.lower_bound(rel_time);
        assert(stance_it != pattern.stance_timing.end());

        stance_trajectory[k].num_stance = pattern.num_stance;
        stance_trajectory[k].in_stance = stance_it->second;
    }
    return stance_trajectory;
}

static inline std::vector<Vector3d> getFootholdPositions(const StanceState &stance_state, const LeggedMPCConfig &config)
{
    return {};
}

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
    const auto stance_trajectory = getStanceTrajectory(config);

    for (std::size_t k = 0; k < mpc.window_size; k++)
    {
        const StanceState &stance_state = stance_trajectory[k];
        const auto Ns = stance_state.num_stance;
        const auto leg_positions = getFootholdPositions(stance_state, config);

        // X reference
        mpc.xref[k] = Vector<OSQPFloat, 7>::Zero();
        if (k == 0)
        {
            mpc.xref[k].block<3, 1>(0, 0) = config.current_angular_velocity;
            mpc.xref[k].block<3, 1>(3, 0) = config.current_linear_velocity;
        }
        else
        {
            mpc.xref[k].block<3, 1>(0, 0) = config.desired_angular_velocity;
            mpc.xref[k].block<3, 1>(3, 0) = config.desired_linear_velocity;
        }
        mpc.xref[k](6) = 1.0;

        // Q
        mpc.Q[k] = Matrix<OSQPFloat, 7, 7>::Zero();
        mpc.Q[k].block<3, 3>(0, 0) = config.linear_velocity_weights.asDiagonal();
        mpc.Q[k].block<3, 3>(3, 3) = config.angular_velocity_weights.asDiagonal();
        mpc.Q[k](6, 6) = 0.0;

        if (k < config.window_size - 1)
        {
            // R
            mpc.R[k] = MatrixX<OSQPFloat>::Zero(3 * Ns, 3 * Ns);
            for (std::size_t i = 0; i < Ns; i++)
            {
                mpc.R[k].block<3, 3>(3 * i, 3 * i) = config.force_weights.asDiagonal();
            }

            // A
            mpc.A[k] = Matrix<OSQPFloat, 7, 7>::Zero();
            mpc.A[k].block<3, 1>(3, 6) = config.gravity_vector;
            mpc.A[k] = mpc.A[k] * config.time_step.count() + Matrix<OSQPFloat, 7, 7>::Identity();

            // B
            assert(stance_state.in_stance.size() == config.num_legs);
            mpc.B[k] = MatrixX<OSQPFloat>::Zero(7, 3 * Ns);
            for (std::size_t i = 0, j = 0; i < config.num_legs; i++)
            {
                if (stance_state.in_stance[i])
                {
                    const Matrix3d invIskewr = invI * getSkewSymmetricMatrix(leg_positions[i]);
                    mpc.B[k].block<3, 3>(0, 3 * j) = invIskewr;
                    mpc.B[k].block<3, 3>(3, 3 * j) = invm * Matrix3d::Identity();
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