#pragma once

#include "sbmpo/types/Model.hpp"
#include "gait_planning/gait_dynamics.hpp"

// Number of gaits used in the planner
#define NUM_GAITS 5

/*
 * Options for the Gait Planning Model
 */
struct GaitPlanningParams
{
    float goal_threshold = 1.0;         // Goal threshold for the planner
    float heuristic_omega_factor = 0.0; // Heuristic factor for angular velocity
    float heuristic_vel_factor = 2.0;   // Heuristic factor for linear velocity
};

/*
 * Gait Planning Model
 * Based on the SBMPO Model, to be used in the planner
 * Incorporates the Gait Dynamics models to be used dynamically based on the sample gait
 */
class GaitPlanningModel : public Model
{
private:
    GaitPlanningParams _params;                                // Parameters for the gait planning model
    std::vector<std::unique_ptr<GaitDynamics>> _gait_dynamics; // Vector of gait dynamics models

public:
    GaitPlanningModel(const GaitPlanningParams &params,
                      GaitDynamicsOptions &options, grid_map::GridMap &map) 
                      : _params(params), _gait_dynamics(NUM_GAITS)
    {
        // Create the gait dynamics models for each gait
        _gait_dynamics[GaitType::NONE] = nullptr; // No dynamics for NONE gait
        _gait_dynamics[GaitType::WALK] = std::make_unique<WalkingDynamics>(options, map);
        _gait_dynamics[GaitType::SWIM] = std::make_unique<SwimmingDynamics>(options, map);
        _gait_dynamics[GaitType::JUMP] = std::make_unique<JumpingDynamics>(options, map);
        _gait_dynamics[GaitType::SINK] = std::make_unique<SinkingDynamics>(options, map);
    }

    /*
        Dynamics of the system
    */
    State next_state(const State &state, const Control &control) override
    {
        // Use the gait model of the sampled state
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->compute_next_state(state, control);
    }

    /*
        Cost of a state and control
    */
    float cost(const State &state1, const State &state2, const Control &control) override
    {
        // Use the gait model of the sampled state
        const auto gait = static_cast<GaitType>(state1[GAIT]);
        return _gait_dynamics[gait]->compute_cost(state1, state2, control);
    }

    /*
        Heuristic of a state with respect to the goal
    */
    float heuristic(const State &state, const State &goal) override
    {
        // Heuristic based on distance to goal
        const float dq = wrap_angle(goal[Q] - state[Q]);
        const float dx = goal[X] - state[X];
        const float dy = goal[Y] - state[Y];
        const float dz = goal[Z] - state[Z];

        // Use heuristic factors to scale the distance
        // Since cost is in time, the heuristic factor would ideally be in units [s/m]
        // For a conservative approach, we can use the inverse of the max velocity of the robot as the heuristic factor
        const float heur_vel = std::sqrt(dx * dx + dy * dy + dz * dz) * _params.heuristic_vel_factor;
        const float heur_omega = std::abs(dq) * _params.heuristic_omega_factor;
        return heur_vel + heur_omega;
    }

    /*
        Is this state close enough to the goal to end the plan?
    */
    bool is_goal(const State &state, const State &goal) override
    {
        // Use the heuristic function to determine if the state is close enough to the goal
        return this->heuristic(state, goal) <= _params.goal_threshold;
    }

    /*
        Does this state meet the model constraints?
    */
    bool is_valid(const State &state) override
    {
        // Use the gait model of the sampled state
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->check_validity(state);
    }

    /*
        Get control samples based on the current state
    */
    std::vector<Control> get_dynamic_samples(const State &state) override
    {
        // Use the gait model of the sampled state
        const auto gait = static_cast<GaitType>(state[GAIT]);
        return _gait_dynamics[gait]->get_dynamic_controls(state);
    }
};