#pragma once

#include "sbmpo/types/Model.hpp"
#include "gait_planning/gait_dynamics.hpp"

struct GaitPlanningParams
{
    float goal_threshold = 1.0;
};

class GaitPlanningModel : public Model
{
private:
    std::vector<std::unique_ptr<GaitDynamics>> _gait_dynamics;
    GaitPlanningParams _params;

public:
    enum GaitType : uint8_t
    {
        WALK,
        SWIM,
        JUMP,
        SINK
    };

    GaitPlanningModel()
    {
        _gait_dynamics.emplace_back(std::make_unique<WalkingDynamics>());
        _gait_dynamics.emplace_back(std::make_unique<SwimmingDynamics>());
        _gait_dynamics.emplace_back(std::make_unique<JumpingDynamics>());
        _gait_dynamics.emplace_back(std::make_unique<SinkingDynamics>());
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
        const float dX = goal[X] - state[X];
        const float dY = goal[Y] - state[Y];
        const float dZ = goal[Z] - state[Z];
        return std::sqrt(dX * dX + dY * dY + dZ * dZ);
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
        return state[Z] >= 0.0;
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