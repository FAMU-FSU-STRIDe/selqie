#pragma once

#include "sbmpo/types/Model.hpp"

using namespace sbmpo;

float wrap_angle(float angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

struct WalkingPlannerParams
{
    float horizon_time = 0.5;
    int integration_steps = 5;
    float goal_threshold = 0.25;
    float heuristic_vel_factor = 2.0;
    float heuristic_omega_factor = 1.0;
};

class WalkingPlannerModel : public sbmpo::Model
{
private:
    WalkingPlannerParams params;

public:
    enum States
    {
        X,
        Y,
        THETA
    };

    enum Controls
    {
        VEL,
        OMEGA
    };

    WalkingPlannerModel(const WalkingPlannerParams &params) : params(params) {}

    /*
        Dynamics of the system
        How does each state change with respect to the controls?
    */
    State next_state(const State &state, const Control &control) override
    {
        State next_state = state;
        const float time_increment = params.horizon_time / static_cast<double>(params.integration_steps);
        for (int i = 0; i < params.integration_steps; i++)
        {
            next_state[THETA] += control[OMEGA] * time_increment;
            next_state[X] += control[VEL] * std::cos(next_state[THETA]) * time_increment;
            next_state[Y] += control[VEL] * std::sin(next_state[THETA]) * time_increment;
            if (!is_valid(next_state))
                return state;
        }
        next_state[THETA] = wrap_angle(next_state[THETA]);
        return next_state;
    }

    /*
        Cost of a state and control
        What am I trying to minimize?
        i.e Distance, Time, Energy
    */
    float cost(const State &, const State &, const Control &control) override
    {
        const float reverse_cost_factor = control[VEL] < 0 ? 2.0 : 1.0;
        return params.horizon_time * reverse_cost_factor;
    }

    /*
        Heuristic of a state with respect to the goal
        Leads the planner to the goal
        What is the lowest cost possible from this state to the goal?
    */
    float heuristic(const State &state, const State &goal) override
    {
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dtheta = wrap_angle(goal[THETA] - state[THETA]);
        const float heur_vel = std::sqrt(dx * dx + dy * dy) * params.heuristic_vel_factor;
        const float heur_omega = std::abs(dtheta) * params.heuristic_omega_factor;
        return heur_vel + heur_omega;
    }

    /*
        Is this state close enough to the goal to end the plan?
    */
    bool is_goal(const State &state, const State &goal) override
    {
        return heuristic(state, goal) < params.goal_threshold;
    }

    /*
        Does this state meet the model constraints?
        i.e Boundary constraints, Obstacles, State limits
    */
    bool is_valid(const State &state) override
    {
        const std::vector<std::vector<float>> obstacles = {
            {1.0, 1.0, 0.2},
            {1.5, 1.5, 0.2},
            {2.0, 2.0, 0.2},
            {1.0, 2.0, 0.2},
            {2.5, 1.5, 0.2},
            {1.5, 2.5, 0.2},
            {1.75, 2.25, 0.2},
            {1.75, 1.0, 0.2}
        };
        for (const auto &obstacle : obstacles)
        {
            const float dx = obstacle[0] - state[X];
            const float dy = obstacle[1] - state[Y];
            const float distance_squared = dx * dx + dy * dy;
            if (distance_squared < obstacle[2] * obstacle[2])
                return false;
        }
        return true;
    }

    /*
        Get control samples based on the current state (Optional)
        Enabled using SearchParameters.sample_type = DYNAMIC
    */
    std::vector<Control> get_dynamic_samples(const State &) override
    {
        return {};
    }
};