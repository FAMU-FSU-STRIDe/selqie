#pragma once

#include "legged_mpc/legged_mpc.hpp"

using namespace std::chrono;

struct SequencePattern
{
  static constexpr uint32_t GAIT_RESOLUTION = 1000;

  milliseconds duration = milliseconds(0);
  std::size_t num_stance;
  std::map<milliseconds, std::vector<bool>> stance_timing;
};

struct StanceSequence
{
  static constexpr uint32_t GAIT_RESOLUTION = 1000;

  milliseconds start_time = milliseconds(0);
  milliseconds current_time;
  SequencePattern current_pattern, next_pattern;
};

struct StanceState
{
  std::size_t num_stance;
  std::vector<bool> in_stance;
};

static inline std::vector<StanceState> getStanceTrajectory(const LeggedMPCConfig &config)
{
  const auto &start_time = config.stance_sequence.start_time;
  const auto &current_time = config.stance_sequence.current_time;
  const auto &current_pattern = config.stance_sequence.current_pattern;
  const auto &next_pattern = config.stance_sequence.next_pattern;

  const auto end_time = start_time + current_pattern.duration;
  std::vector<StanceState> stance_trajectory(config.window_size);
  for (std::size_t k = 0; k < config.window_size; k++)
  {
    const auto time = current_time + k * config.time_step;
    const auto rel_time = time < end_time ? time - start_time : (time - end_time) % next_pattern.duration;
    const auto &pattern = time < end_time ? current_pattern : next_pattern;

    const auto stance_it = pattern.stance_timing.lower_bound(rel_time);
    assert(stance_it != pattern.stance_timing.end());

    stance_trajectory[k].num_stance = pattern.num_stance;
    stance_trajectory[k].in_stance = stance_it->second;
  }
  return stance_trajectory;
}

#include <robot_msgs/msg/stance_pattern.hpp>

using namespace robot_msgs::msg;

static inline SequencePattern getSequencePattern(const std::size_t num_legs, const StancePattern &pattern)
{
  SequencePattern sequence_pattern;
  sequence_pattern.duration = milliseconds(time_t(1E3 / pattern.frequency));
  for (std::size_t i = 0; i < pattern.timing.size(); i++)
  {
    const milliseconds time(time_t(pattern.timing[i] / StancePattern::GAIT_RESOLUTION * (1E3 / pattern.frequency)));
    const uint32_t bitset = pattern.stance[i];
    sequence_pattern.stance_timing[time] = std::vector<bool>(num_legs);
    for (std::size_t j = 0; j < num_legs; j++)
    {
      if (sequence_pattern.stance_timing[time][j] = (bitset & (1 << j)) != 0)
      {
        sequence_pattern.num_stance++;
      }
    }
  }
  return sequence_pattern;
}

bool sync()
{
  const auto next_pattern = _config.stance_sequence.next_pattern;
  if (next_pattern.duration.count() == 0)
  {
    return false;
  }

  const auto current_time = duration_cast<milliseconds>(nanoseconds(this->get_clock()->now().nanoseconds()));

  auto &start_time = _config.stance_sequence.start_time;
  auto &current_pattern = _config.stance_sequence.current_pattern;
  if (current_time >= start_time + current_pattern.duration)
  {
    start_time = current_time;
    current_pattern = next_pattern;
  }
}