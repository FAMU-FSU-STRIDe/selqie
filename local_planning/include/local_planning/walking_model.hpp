#pragma once

#include <sbmpo/types/Model.hpp>
#include <grid_map_core/grid_map_core.hpp>

using namespace sbmpo;

inline static float wrap(const float x)
{
    return std::fmod(x + M_PI, 2 * M_PI) - M_PI;
}

struct WalkingModelParams
{
    float dt = 0.1;
    std::vector<float> max_velocities = {1.0, 1.0, 1.0};
    float goal_threshold = 1.0;
};

class WalkingModel : public Model
{
private:
    WalkingModelParams _params;
    grid_map::GridMap _map;
    bool _is_map_set = false;

public:
    WalkingModel(const WalkingModelParams &params)
        : _params(params) {}

    void set_map(const grid_map::GridMap &map)
    {
        if (map.exists("cost"))
        {
            _map = map;
            _is_map_set = true;
        }
        else
        {
            _is_map_set = false;
        }
    }

    State next_state(const State &x, const Control &u) override
    {
        const float x1 = x[0];
        const float y1 = x[1];
        const float theta1 = x[2];

        const float vx = u[0];
        const float vy = u[1];
        const float omega = u[2];

        const float theta2 = wrap(theta1 + omega * _params.dt);
        const float x2 = x1 + vx * std::cos(theta2) * _params.dt - vy * std::sin(theta2) * _params.dt;
        const float y2 = y1 + vx * std::sin(theta2) * _params.dt + vy * std::cos(theta2) * _params.dt;

        return {x2, y2, theta2};
    }

    float cost(const State &state1, const State &, const Control &) override
    {
        double cost = _params.dt;
        if (_is_map_set)
        {
            grid_map::Position pos(state1[0], state1[1]);
            if (_map.isInside(pos))
            {
                cost += _map.atPosition("cost", pos);
            }
        }
        return cost;
    }

    float heuristic(const State &state, const State &goal) override
    {
        const float dx = goal[0] - state[0];
        const float dy = goal[1] - state[1];
        const float dz = wrap(goal[2] - state[2]);

        const float dt_x = std::abs(dx) / _params.max_velocities[0];
        const float dt_y = std::abs(dy) / _params.max_velocities[1];
        const float dt_z = std::abs(dz) / _params.max_velocities[2];

        return std::sqrt(dt_x * dt_x + dt_y * dt_y + dt_z * dt_z);
    }

    bool is_goal(const State &state, const State &goal) override
    {
        return heuristic(state, goal) < _params.goal_threshold;
    }

    bool is_valid(const State &) override
    {
        return true;
    }
};