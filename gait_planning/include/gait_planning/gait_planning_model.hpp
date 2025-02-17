#pragma once

#include "sbmpo/types/Model.hpp"
#include "gait_planning/gait_dynamics.hpp"

struct GaitPlanningParams
{
    float goal_threshold = 1.0;
    float heuristic_omega_factor = 0.0;
    float heuristic_vel_factor = 2.0;
};

class GaitPlanningModel : public Model
{
private:
    std::vector<std::unique_ptr<GaitDynamics>> _gait_dynamics;
    GaitPlanningParams _params;

public:
    GaitPlanningModel(const GaitPlanningParams &params,
                      GaitDynamicsOptions &options, grid_map::GridMap &map) : _params(params)
    {
        _gait_dynamics.emplace_back(nullptr);
        _gait_dynamics.emplace_back(std::make_unique<WalkingDynamics>(options, map));
        _gait_dynamics.emplace_back(std::make_unique<SwimmingDynamics>(options, map));
        _gait_dynamics.emplace_back(std::make_unique<JumpingDynamics>(options, map));
        _gait_dynamics.emplace_back(std::make_unique<SinkingDynamics>(options, map));
    }

    /*
        Dynamics of the system
        How does each state change with respect to the controls?
    */
    State next_state(const State &state, const Control &control) override
    {
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->getNextState(state, control);
    }

    /*
        Cost of a state and control
        What am I trying to minimize?
        i.e Distance, Time, Energy
    */
    float cost(const State &state1, const State &state2, const Control &control) override
    {
        const auto gait = static_cast<GaitType>(state1[GAIT]);
        return _gait_dynamics[gait]->getCost(state1, state2, control);
    }

    /*
        Heuristic of a state with respect to the goal
        Leads the planner to the goal
        What is the lowest cost possible from this state to the goal?
    */
    float heuristic(const State &state, const State &goal) override
    {
        const float dq = wrap_angle(goal[Q] - state[Q]);
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dz = goal[Z] - state[Z];
        const float heur_vel = std::sqrt(dx * dx + dy * dy + dz * dz) * _params.heuristic_vel_factor;
        const float heur_omega = std::abs(dq) * _params.heuristic_omega_factor;
        return heur_vel + heur_omega;
    }

    /*
        Is this state close enough to the goal to end the plan?
    */
    bool is_goal(const State &state, const State &goal) override
    {
        return this->heuristic(state, goal) <= _params.goal_threshold;
    }

    /*
        Does this state meet the model constraints?
        i.e Boundary constraints, Obstacles, State limits
    */
    bool is_valid(const State &state) override
    {
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->isValid(state);
    }

    /*
        Get control samples based on the current state (Optional)
        Enabled using SearchParameters.sample_type = DYNAMIC
    */
    std::vector<Control> get_dynamic_samples(const State &state) override
    {
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->getControls(state);
    }
};