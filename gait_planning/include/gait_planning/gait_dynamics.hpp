#pragma once

#include "sbmpo/types/types.hpp"

using namespace sbmpo;

// State vector indices
enum StateIndex : uint8_t
{
    TIME,
    X,
    Y,
    Z,
    GAIT
};

// Control vector indices
enum ControlIndex : uint8_t
{
    dX,
    dY,
    dZ,
    NEW_GAIT
};

struct GaitDynamicsOptions
{
    float cost_of_transport = 1.0F;
};

// Interface class for gait dynamics models
class GaitDynamics
{
public:
    virtual State getNextState(const State &state, const Control &control) = 0;

    virtual float getCost(const State &state1, const State &state2, const Control &control) = 0;

    virtual bool isValid(const State &state) = 0;

    virtual std::vector<Control> getControls(const State &state) = 0;
};

// Ground walking dynamics model
class WalkingDynamics : public GaitDynamics
{
public:
    WalkingDynamics() {}

    State getNextState(const State &state, const Control &control) override
    {
        State next_state = state;
        next_state[X] += control[dX];
        next_state[Y] += control[dY];
        next_state[Z] += control[dZ];
        next_state[GAIT] = control[NEW_GAIT];
        return next_state;
    }

    float getCost(const State &state1, const State &state2, const Control &control) override {}

    bool isValid(const State &state) override {}

    std::vector<Control> getControls(const State &state) override {}
};

// Swimming dynamics model
class SwimmingDynamics : public GaitDynamics
{
public:
    SwimmingDynamics() {}

    State getNextState(const State &state, const Control &control) override {}

    float getCost(const State &state1, const State &state2, const Control &control) override {}

    bool isValid(const State &state) override {}

    std::vector<Control> getControls(const State &state) override {}
};

// Jumping dynamics model
class JumpingDynamics : public GaitDynamics
{
public:
    JumpingDynamics() {}

    State getNextState(const State &state, const Control &control) override {}

    float getCost(const State &state1, const State &state2, const Control &control) override {}

    bool isValid(const State &state) override {}

    std::vector<Control> getControls(const State &state) override {}
};

// Sinking dynamics model
class SinkingDynamics : public GaitDynamics
{
public:
    SinkingDynamics() {}

    State getNextState(const State &state, const Control &control) override {}

    float getCost(const State &state1, const State &state2, const Control &control) override {}

    bool isValid(const State &state) override {}

    std::vector<Control> getControls(const State &state) override {}
};