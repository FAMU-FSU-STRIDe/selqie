#pragma once

#include "sbmpo/types/types.hpp"

using namespace sbmpo;

// State vector indices
enum StateIndex : uint8_t
{
    TIME,
    Q,
    X,
    Y,
    Z,
    GAIT
};

// Control vector indices
enum ControlIndex : uint8_t
{
    Wz,
    Vx,
    Vz,
    NEW_GAIT
};

enum GaitType : uint8_t
{
    NONE,
    WALK,
    SWIM,
    JUMP,
    SINK
};

struct GaitDynamicsOptions
{
    float horizon_time = 1.0F;
    int integration_steps = 5;
    float cost_of_transport = 1.0F;
    float jumping_loadup_time = 0.5F;
    float sinking_speed = 0.25F;
    float robot_height = 0.25F;
    float ground_level = 0.0F;
};

float wrap_angle(float angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

// Interface class for gait dynamics models
class GaitDynamics
{
protected:
    GaitDynamicsOptions &_options;

public:
    GaitDynamics(GaitDynamicsOptions &options) : _options(options) {}

    virtual State getNextState(const State &state, const Control &control)
    {
        const float dt = _options.horizon_time / _options.integration_steps;
        State next_state = state;
        for (int i = 0; i < _options.integration_steps; i++)
        {
            next_state[TIME] += dt;
            next_state[Q] += control[Wz] * dt;
            next_state[X] += control[Vx] * std::cos(state[Q]) * dt;
            next_state[Y] += control[Vx] * std::sin(state[Q]) * dt;
            next_state[Z] += control[Vz] * dt;
            next_state[GAIT] = control[NEW_GAIT];
        }
        next_state[Q] = wrap_angle(next_state[Q]);
        return next_state;
    }

    virtual float getCost(const State &, const State &, const Control &)
    {
        return _options.cost_of_transport * _options.horizon_time;
    }

    virtual bool isValid(const State &state)
    {
        return state[Z] > _options.ground_level;
    }

    virtual std::vector<Control> getControls(const State &state) = 0;
};

// Ground walking dynamics model
class WalkingDynamics : public GaitDynamics
{
public:
    WalkingDynamics(GaitDynamicsOptions &options) : GaitDynamics(options) {}

    std::vector<Control> getControls(const State &) override
    {
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
};

// Swimming dynamics model
class SwimmingDynamics : public GaitDynamics
{
public:
    SwimmingDynamics(GaitDynamicsOptions &options) : GaitDynamics(options) {}

    bool isValid(const State &state)
    {
        return GaitDynamics::isValid(state) && state[Z] > _options.robot_height + _options.ground_level;
    }

    std::vector<Control> getControls(const State &) override
    {
        return {{+0.05, +0.10, 0.000, SWIM},
                {+0.05, 0.000, 0.000, SWIM},
                {+0.05, -0.10, 0.000, SWIM},
                {-0.05, +0.10, 0.000, SWIM},
                {-0.05, 0.000, 0.000, SWIM},
                {-0.05, -0.10, 0.000, SWIM},

                {0.000, +0.20, 0.000, SWIM},
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

// Jumping dynamics model
class JumpingDynamics : public GaitDynamics
{
public:
    JumpingDynamics(GaitDynamicsOptions &options) : GaitDynamics(options) {}

    State getNextState(const State &state, const Control &control) override
    {
        State next_state = state;
        next_state[TIME] += _options.horizon_time + _options.jumping_loadup_time;
        next_state[X] += control[Vx] * _options.horizon_time;
        next_state[Z] += control[Vz] * _options.horizon_time;
        next_state[GAIT] = control[NEW_GAIT];
        return next_state;
    }

    std::vector<Control> getControls(const State &) override
    {
        return {
            {0.0, +0.10, +0.40, SWIM},
            {0.0, +0.05, +0.45, SWIM},
            {0.0, 0.000, +0.50, SWIM},
        };
    }
};

// Sinking dynamics model
class SinkingDynamics : public GaitDynamics
{
public:
    SinkingDynamics(GaitDynamicsOptions &options) : GaitDynamics(options) {}

    State getNextState(const State &state, const Control &control) override
    {
        const float z_final = _options.ground_level + _options.robot_height;
        State next_state = state;
        next_state[TIME] += std::min((state[Z] - z_final) / _options.sinking_speed, 0.F);
        next_state[Z] = z_final;
        next_state[GAIT] = control[NEW_GAIT];
        return next_state;
    }

    std::vector<Control> getControls(const State &) override
    {
        return {
            {0.0, 0.0, 0.0, WALK}};
    }
};