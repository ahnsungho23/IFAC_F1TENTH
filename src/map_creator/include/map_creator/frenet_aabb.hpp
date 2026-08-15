// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__FRENET_AABB_HPP_
#define MAP_CREATOR__FRENET_AABB_HPP_

#include <optional>

#include "global_planning/clcs_frenet_converter.hpp"

namespace map_creator
{

struct FrenetAabb
{
  double s_center{0.0};
  double d_center{0.0};
  double s_start{0.0};
  double s_end{0.0};
  double d_right{0.0};
  double d_left{0.0};
};

// Projects a map-frame Cartesian AABB onto the P0 CLCS reference.
// Only the centre is globally projected; the four corners are rotated into the
// centre tangent frame, so no corner can land on a spatially close but
// longitudinally distant track branch (same construction as obstacle_detector's
// aabb_frenet_projector, minus the exact near-side refinement that painting
// does not need — the envelope is conservative, never tighter than the AABB).
// Returns nullopt when the AABB is degenerate or non-finite, the centre
// projection fails, or |d_center| exceeds max_center_abs_d (<= 0 disables the
// bound).
std::optional<FrenetAabb> projectCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  double x_min, double x_max, double y_min, double y_max,
  double max_center_abs_d);

}  // namespace map_creator

#endif  // MAP_CREATOR__FRENET_AABB_HPP_
