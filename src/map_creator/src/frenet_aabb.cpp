// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include "map_creator/frenet_aabb.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace map_creator
{
namespace
{

double wrapS(double s, double track_length)
{
  if (!(track_length > 0.0) || !std::isfinite(track_length)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

}  // namespace

std::optional<FrenetAabb> projectCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  const double x_min, const double x_max, const double y_min, const double y_max,
  const double max_center_abs_d)
{
  if (!std::isfinite(x_min) || !std::isfinite(x_max) ||
    !std::isfinite(y_min) || !std::isfinite(y_max) ||
    x_min > x_max || y_min > y_max)
  {
    return std::nullopt;
  }
  const double diagonal = std::hypot(x_max - x_min, y_max - y_min);
  if (!(diagonal > std::numeric_limits<double>::epsilon())) {
    return std::nullopt;
  }

  const double x_center = 0.5 * (x_min + x_max);
  const double y_center = 0.5 * (y_min + y_max);

  global_planning::ClcsConversionInput center_input;
  center_input.x = x_center;
  center_input.y = y_center;
  const auto center = converter.convert(center_input);
  if (!center.valid || !std::isfinite(center.s) || !std::isfinite(center.d) ||
    !std::isfinite(center.reference_yaw))
  {
    return std::nullopt;
  }
  if (max_center_abs_d > 0.0 && std::abs(center.d) > max_center_abs_d) {
    return std::nullopt;
  }

  const double track_length = converter.stats().track_length;
  const double tangent_x = std::cos(center.reference_yaw);
  const double tangent_y = std::sin(center.reference_yaw);
  const double normal_x = -tangent_y;
  const double normal_y = tangent_x;
  const std::array<std::pair<double, double>, 4> corners = {{
    {x_min, y_min},
    {x_min, y_max},
    {x_max, y_min},
    {x_max, y_max},
  }};

  double min_longitudinal = std::numeric_limits<double>::infinity();
  double max_longitudinal = -std::numeric_limits<double>::infinity();
  double min_lateral = std::numeric_limits<double>::infinity();
  double max_lateral = -std::numeric_limits<double>::infinity();
  for (const auto & corner : corners) {
    const double dx = corner.first - x_center;
    const double dy = corner.second - y_center;
    const double longitudinal = dx * tangent_x + dy * tangent_y;
    const double lateral = dx * normal_x + dy * normal_y;
    min_longitudinal = std::min(min_longitudinal, longitudinal);
    max_longitudinal = std::max(max_longitudinal, longitudinal);
    min_lateral = std::min(min_lateral, lateral);
    max_lateral = std::max(max_lateral, lateral);
  }

  FrenetAabb bounds;
  bounds.s_center = wrapS(center.s, track_length);
  bounds.d_center = center.d;
  bounds.s_start = wrapS(bounds.s_center + min_longitudinal, track_length);
  bounds.s_end = wrapS(bounds.s_center + max_longitudinal, track_length);
  bounds.d_right = bounds.d_center + min_lateral;
  bounds.d_left = bounds.d_center + max_lateral;
  return bounds;
}

}  // namespace map_creator
