#pragma once

#include "sbmpo/types/types.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

using namespace sbmpo;

// States for the Gait Dynamics
enum StateIndex : uint8_t
{
    TIME,
    Q, // yaw
    X,
    Y,
    Z,
    GAIT
};

// Controls for the Gait Dynamics
enum ControlIndex : uint8_t
{
    Wz, // yaw rate
    Vx,
    Vz,
    NEW_GAIT
};

// Types of gaits
enum GaitType : uint8_t
{
    NONE,
    WALK,
    SWIM,
    JUMP,
    SINK,
    STAND
};

/*
 * Options for the Gait Dynamics
 */
struct GaitDynamicsOptions
{
    float horizon_time = 1.0F;  // Time horizon for the dynamics
    int integration_steps = 5;  // Number of Euler integration steps
    float robot_height = 0.25F; // Height of the robot

    float walk_cost_of_transport = 1.0F; // Cost of walking
    float walk_cost_of_reverse = 2.0F;   // Cost of walking in reverse

    float jump_cost_of_transport = 1.5F; // Cost of jumping
    float jumping_loadup_time = 0.5F;    // Time to load up for a jump
    float jump_height = 0.5F;            // Height of the jump

    float swim_cost_of_transport = 5.0F; // Cost of swimming
    float swim_cost_of_reverse = 3.0F;   // Cost of swimming in reverse

    float sink_cost_of_transport = 0.1F; // Cost of sinking
    float sinking_speed = 0.25F;         // Speed of sinking
};

/*
 * Wrap angle to the range [-pi, pi]
 */
float wrap_angle(float angle)
{
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

/*
 * Gait Dynamics class
 * This class defines the dynamics of the system, the cost function, and the heuristic function
 */
class GaitDynamics
{
protected:
    GaitDynamicsOptions &_options; // Options for the dynamics
    grid_map::GridMap &_map;       // Grid map for the environment

public:
    GaitDynamics(GaitDynamicsOptions &options, grid_map::GridMap &map)
        : _options(options), _map(map) {}

    /*
     * Dynamics of the system
     */
    virtual State compute_next_state(const State &state, const Control &control)
    {
        State next_state = state;

        // Euler integration
        const float dt = _options.horizon_time / _options.integration_steps;
        for (int i = 0; i < _options.integration_steps; i++)
        {
            // By default, use Unicycle Steering Model
            next_state[TIME] += dt;
            next_state[Q] += control[Wz] * dt;
            next_state[X] += control[Vx] * std::cos(state[Q]) * dt;
            next_state[Y] += control[Vx] * std::sin(state[Q]) * dt;
            next_state[Z] += control[Vz] * dt;
            next_state[GAIT] = control[NEW_GAIT];

            // Make sure the state is valid over its path
            if (!check_validity(next_state))
                break;
        }

        // Keep angle within [-pi, pi]
        next_state[Q] = wrap_angle(next_state[Q]);

        // Return the next state
        return next_state;
    }

    /*
     * Cost of a state and control
     */
    virtual float compute_cost(const State &, const State &, const Control &)
    {
        // Time based cost function
        return _options.horizon_time;
    }

    /*
     * Validity check of the state
     */
    virtual bool check_validity(const State &state)
    {
        // Convert position to grid map cell index
        const grid_map::Position position(state[X], state[Y]);
        grid_map::Index index;
        if (_map.getIndex(position, index))
        {
            // By default, valid if the robot is above the elevation
            const float elevation = _map.at("elevation", index);
            return state[Z] > elevation;
        }

        // If the position is outside the map, return true
        return true;
    }

    /*
     * Get control samples based on the current state
     */
    virtual std::vector<Control> get_dynamic_controls(const State &state) = 0;
};

/*
 * Gait Dynamics Models
 * These models define the specific dynamics for each gait type
 */

/*
 * Walking dynamics model
 */
class WalkingDynamics : public GaitDynamics
{
public:
    WalkingDynamics(GaitDynamicsOptions &options, grid_map::GridMap &map) : GaitDynamics(options, map) {}

    float compute_cost(const State &, const State &, const Control &control) override
    {
        // Use the cost of transport for walking
        // Penalize reverse velocity
        const float reverse_cost = control[Vx] < 0 ? _options.walk_cost_of_reverse : 1.0F;
        return _options.horizon_time * _options.walk_cost_of_transport * reverse_cost;
    }

    std::vector<Control> get_dynamic_controls(const State &) override
    {
        // The walking gait can either remain in walking mode or switch to jumping
        return {{+0.30, 0.000, 0.0, WALK},
                {-0.30, 0.000, 0.0, WALK},

                {+0.10, +0.10, 0.0, WALK},
                {+0.10, -0.10, 0.0, WALK},
                {-0.10, +0.10, 0.0, WALK},
                {-0.10, -0.10, 0.0, WALK},

                {0.000, +0.25, 0.0, WALK},
                {0.000, +0.10, 0.0, WALK},
                {0.000, -0.25, 0.0, WALK},
                {0.000, -0.10, 0.0, WALK},

                {0.000, 0.000, 0.0, JUMP}};
    }

    bool check_validity(const State &state)
    {
        // Force jump over rock
        // if (std::abs(state[Y]) > 0.5)
        // {
        //     return false;
        // }

        // Can only walk on the ground
        const grid_map::Position position(state[X], state[Y]);
        grid_map::Index index;
        if (_map.getIndex(position, index) && _map.exists("ground"))
        {
            const bool on_ground = _map.at("ground", index) == 1.0;
            return on_ground;
        }

        // If the position is outside the map, return true
        return true;
    }
};

/*
 * Swimming dynamics model
 */
class SwimmingDynamics : public GaitDynamics
{
public:
    SwimmingDynamics(GaitDynamicsOptions &options, grid_map::GridMap &map) : GaitDynamics(options, map) {}

    float compute_cost(const State &, const State &, const Control &control) override
    {
        // Use the cost of transport for swimming
        // Penalize reverse velocity
        const float reverse_cost = control[Vx] < 0 ? _options.swim_cost_of_reverse : 1.0F;
        return _options.horizon_time * _options.swim_cost_of_transport * reverse_cost;
    }

    bool check_validity(const State &state)
    {
        // Force jump over rock
        // if (std::abs(state[Y]) > 0.5)
        // {
        //     return false;
        // }

        // Must maintain a height above the ground
        const grid_map::Position position(state[X], state[Y]);
        grid_map::Index index;
        if (_map.getIndex(position, index))
        {
            const float elevation = _map.at("elevation", index);
            return state[Z] > elevation + _options.robot_height;
        }

        // If the position is outside the map, return true
        return true;
    }

    std::vector<Control> get_dynamic_controls(const State &) override
    {
        // The swimming gait can either remain in swimming mode or switch to sinking
        return {{0.000, +0.20, 0.000, SWIM},
                {0.000, -0.20, 0.000, SWIM},
                {0.000, +0.10, 0.000, SWIM},
                {0.000, -0.10, 0.000, SWIM},

                {0.000, +0.05, +0.05, SWIM},
                {0.000, -0.05, +0.05, SWIM},
                {0.000, +0.05, -0.05, SWIM},
                {0.000, -0.05, -0.05, SWIM},

                {0.000, 0.000, +0.10, SWIM},
                {0.000, 0.000, -0.05, SWIM},

                {0.000, 0.000, 0.000, SINK}};
    }
};

/*
 * Jumping dynamics model
 */
class JumpingDynamics : public GaitDynamics
{
public:
    JumpingDynamics(GaitDynamicsOptions &options, grid_map::GridMap &map) : GaitDynamics(options, map) {}

    State compute_next_state(const State &state, const Control &control) override
    {
        State next_state = state;

        // Time has added component for the jumping loadup time
        next_state[TIME] += _options.horizon_time + _options.jumping_loadup_time;

        // Jump forwards and upwards by the jump height
        next_state[X] += control[Vx] * std::cos(state[Q]) * _options.jump_height;
        next_state[Y] += control[Vx] * std::sin(state[Q]) * _options.jump_height;
        next_state[Z] += control[Vz] * _options.jump_height;
        next_state[GAIT] = control[NEW_GAIT];
        return next_state;
    }

    float compute_cost(const State &, const State &, const Control &) override
    {
        // Use the cost of transport for jumping
        return _options.horizon_time * _options.jump_cost_of_transport;
    }

    std::vector<Control> get_dynamic_controls(const State &) override
    {
        // The jumping gait can only transition to swimming
        return {
            {0.0, +0.25, +0.75, SWIM},
            {0.0, 0.000, +1.00, SWIM},
        };
    }
};

/*
 * Sinking dynamics model
 */
class SinkingDynamics : public GaitDynamics
{
public:
    SinkingDynamics(GaitDynamicsOptions &options, grid_map::GridMap &map) : GaitDynamics(options, map) {}

    State compute_next_state(const State &state, const Control &control) override
    {
        State next_state = state;

        // Can only sink on the map, because the ground level is required
        const grid_map::Position position(state[X], state[Y]);
        grid_map::Index index;
        if (!_map.getIndex(position, index))
        {
            throw std::runtime_error("Invalid state");
        }

        // Determine the final z position
        const float z_final = _map.at("elevation", index) + _options.robot_height;

        // Determine the time required to sink to the final z position
        // Make sure the change in z is not negative (it shouldn't be)
        next_state[TIME] += std::min((state[Z] - z_final) / _options.sinking_speed, 0.F);
        next_state[Z] = z_final;
        next_state[GAIT] = control[NEW_GAIT];
        return next_state;
    }

    float compute_cost(const State &, const State &, const Control &) override
    {
        // Use the cost of transport for sinking
        return _options.horizon_time * _options.sink_cost_of_transport;
    }

    std::vector<Control> get_dynamic_controls(const State &) override
    {
        // The sinking gait can only transition to walking
        return {
            {0.0, 0.0, 0.0, WALK}};
    }
};