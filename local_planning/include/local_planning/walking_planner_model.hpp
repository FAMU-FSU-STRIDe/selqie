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
    float horizon_time = 0.25;
    int integration_steps = 5;
    float goal_threshold = 0.5;
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
        THETA,
        VEL,
        OMEGA
    };

    enum Controls
    {
        ACC,
        ALPHA
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
            next_state[OMEGA] += control[ALPHA] * time_increment;
            next_state[VEL] += control[ACC] * time_increment;
            next_state[THETA] += next_state[OMEGA] * time_increment;
            next_state[X] += next_state[VEL] * std::cos(next_state[THETA]) * time_increment;
            next_state[Y] += next_state[VEL] * std::sin(next_state[THETA]) * time_increment;
        }
        next_state[THETA] = wrap_angle(next_state[THETA]);
        return next_state;
    }

    /*
        Cost of a state and control
        What am I trying to minimize?
        i.e Distance, Time, Energy
    */
    float cost(const State &state1, const State &state2, const Control &) override
    {
        const float dx = state2[X] - state1[X];
        const float dy = state2[Y] - state1[Y];
        return std::sqrt(dx * dx + dy * dy);
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
        return std::sqrt(dx * dx + dy * dy) + std::abs(dtheta);
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
        // velocity domain
        const float v = std::abs(state[VEL]);
        const float w = std::abs(state[OMEGA]);
        const float a = 64.f * v * v - 192.f * v * w - 80.f * v + 144.f * w * w - 120.f * w + 25.f;
        return a >= 0.0;
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