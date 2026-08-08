// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef LOCAL_PLANNING__OBSERVATION_GATE_HPP_
#define LOCAL_PLANNING__OBSERVATION_GATE_HPP_

#include <algorithm>
#include <cmath>

namespace local_planning
{

struct ObservationGate
{
  int observation_count{1};
  double minimum_duration_sec{0.0};
};

inline ObservationGate interpolateObservationGate(
  double obstacle_distance_m,
  double near_distance_m,
  double far_distance_m,
  int near_observation_count,
  int far_observation_count,
  double near_minimum_duration_sec,
  double far_minimum_duration_sec)
{
  const double ratio = std::clamp(
    (obstacle_distance_m - near_distance_m) / (far_distance_m - near_distance_m),
    0.0, 1.0);
  ObservationGate gate;
  gate.observation_count = static_cast<int>(std::lround(
      near_observation_count + ratio * (far_observation_count - near_observation_count)));
  gate.minimum_duration_sec = near_minimum_duration_sec +
    ratio * (far_minimum_duration_sec - near_minimum_duration_sec);
  return gate;
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__OBSERVATION_GATE_HPP_
