#pragma once

#include <math.h>
#include <robot_msgs/msg/leg_trajectory.hpp>

robot_msgs::msg::LegTrajectory make_walk_stride(const int num_points, const double frequency, const double duty_factor,
                                                const double center_shift, const double stance_length, const double body_height,
                                                const double step_height, const double offset = 0)
{

    robot_msgs::msg::LegTrajectory leg_trajectory;
    leg_trajectory.timing.reserve(num_points);
    leg_trajectory.commands.reserve(num_points);

    const int points_per_half = num_points / 2;
    const double duration = 1.0 / frequency;
    const int offset_index = offset * num_points;
    const double touchdown_x = 0.5 * stance_length * (1.0 + center_shift);
    const double takeoff_x = touchdown_x - stance_length;
    const double x0 = 0.5 * (takeoff_x + touchdown_x);
    const double stance_duration = duration * duty_factor;
    const double stance_dt = stance_duration / points_per_half;
    const double swing_duration = duration * (1.0 - duty_factor);
    const double swing_dt = swing_duration / points_per_half;

    double t = 0.0;
    robot_msgs::msg::LegCommand leg_command;
    leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
    for (int k = 0; k < num_points; k++)
    {
        leg_trajectory.timing.push_back(t);

        const int i = (k + offset_index) % num_points;
        const int p = i % points_per_half;
        const int q = i / points_per_half;
        const double f = static_cast<double>(p) / points_per_half;
        if (q == 0)
        {
            t += stance_dt;
            leg_command.pos_setpoint.x = touchdown_x - f * stance_length;
            leg_command.pos_setpoint.z = -body_height;
            leg_trajectory.commands.push_back(leg_command);
        }
        else
        {
            t += swing_dt;
            leg_command.pos_setpoint.x = x0 - 0.5 * stance_length * std::cos(M_PI * f);
            leg_command.pos_setpoint.z = -body_height + step_height * std::sin(M_PI * f);
            leg_trajectory.commands.push_back(leg_command);
        }
    }

    return leg_trajectory;
}

robot_msgs::msg::LegTrajectory make_swim_stride(const int num_points, const double frequency, const double leg_length,
                                                const double angle, const double x_amplitude, const double z_amplitude)
{
    robot_msgs::msg::LegTrajectory leg_trajectory;
    leg_trajectory.timing.reserve(num_points);
    leg_trajectory.commands.reserve(num_points);

    const double duration = 1.0 / frequency;
    const double dt = duration / num_points;
    const double theta = angle + M_PI_2;

    double t = 0.0;
    robot_msgs::msg::LegCommand leg_command;
    leg_command.control_mode = robot_msgs::msg::LegCommand::CONTROL_MODE_POSITION;
    for (int k = 0; k < num_points; k++)
    {
        leg_trajectory.timing.push_back(t);
        const double x = x_amplitude * std::cos(2 * M_PI * t * frequency);
        const double z = z_amplitude * std::sin(2 * M_PI * t * frequency) - leg_length;
        leg_command.pos_setpoint.x = x * std::cos(theta) - z * std::sin(theta);
        leg_command.pos_setpoint.z = x * std::sin(theta) + z * std::cos(theta);
        leg_trajectory.commands.push_back(leg_command);
        t += dt;
    }

    return leg_trajectory;
}