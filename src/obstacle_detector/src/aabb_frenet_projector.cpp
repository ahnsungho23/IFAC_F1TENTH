// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "obstacle_detector/aabb_frenet_projector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace obstacle_detector
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-12;

double wrapS(double s, double track_length)
{
  if (!(track_length > 0.0) || !std::isfinite(track_length)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

double squaredDistanceToAabb(
  double x, double y, double x_min, double x_max, double y_min, double y_max)
{
  const double dx = x < x_min ? x_min - x : (x > x_max ? x - x_max : 0.0);
  const double dy = y < y_min ? y_min - y : (y > y_max ? y - y_max : 0.0);
  return dx * dx + dy * dy;
}

double squaredDistanceToSegment(
  double x, double y, double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double length_squared = dx * dx + dy * dy;
  if (!(length_squared > kGeometryEpsilon)) {
    const double error_x = x - x0;
    const double error_y = y - y0;
    return error_x * error_x + error_y * error_y;
  }
  const double t = std::clamp(((x - x0) * dx + (y - y0) * dy) / length_squared, 0.0, 1.0);
  const double closest_x = x0 + t * dx;
  const double closest_y = y0 + t * dy;
  const double error_x = x - closest_x;
  const double error_y = y - closest_y;
  return error_x * error_x + error_y * error_y;
}

bool segmentIntersectsAabb(
  double x0, double y0, double x1, double y1,
  double x_min, double x_max, double y_min, double y_max)
{
  double t_min = 0.0;
  double t_max = 1.0;
  const auto clip_axis = [&t_min, &t_max](
    double origin, double delta, double lower, double upper) {
      if (std::abs(delta) <= kGeometryEpsilon) {
        return origin >= lower && origin <= upper;
      }
      double first = (lower - origin) / delta;
      double second = (upper - origin) / delta;
      if (first > second) {
        std::swap(first, second);
      }
      t_min = std::max(t_min, first);
      t_max = std::min(t_max, second);
      return t_min <= t_max;
    };
  return clip_axis(x0, x1 - x0, x_min, x_max) &&
         clip_axis(y0, y1 - y0, y_min, y_max);
}

double squaredDistanceBetweenSegmentAndAabb(
  double x0, double y0, double x1, double y1,
  double x_min, double x_max, double y_min, double y_max)
{
  if (segmentIntersectsAabb(x0, y0, x1, y1, x_min, x_max, y_min, y_max)) {
    return 0.0;
  }

  double best = std::min(
    squaredDistanceToAabb(x0, y0, x_min, x_max, y_min, y_max),
    squaredDistanceToAabb(x1, y1, x_min, x_max, y_min, y_max));
  const std::array<std::pair<double, double>, 4> corners = {{
    {x_min, y_min},
    {x_min, y_max},
    {x_max, y_min},
    {x_max, y_max},
  }};
  for (const auto & corner : corners) {
    best = std::min(
      best,
      squaredDistanceToSegment(
        corner.first, corner.second, x0, y0, x1, y1));
  }
  return best;
}

double distanceToLongitudinalInterval(
  double s, double start, double end, double track_length, bool closed_loop)
{
  const auto linear_distance = [start, end](double value) {
      if (value < start) {
        return start - value;
      }
      if (value > end) {
        return value - end;
      }
      return 0.0;
    };
  double best = linear_distance(s);
  if (closed_loop && track_length > 0.0) {
    best = std::min(best, linear_distance(s - track_length));
    best = std::min(best, linear_distance(s + track_length));
  }
  return best;
}

std::optional<double> closestBranchLockedAabbDistance(
  const global_planning::ClcsFrenetConverter & converter,
  const global_planning::ClcsConversionResult & center,
  double x_min, double x_max, double y_min, double y_max, double diagonal)
{
  const auto & reference = converter.reference_points();
  if (reference.size() < 2U || center.segment_index < 0) {
    return std::nullopt;
  }

  std::vector<double> cumulative_s(reference.size(), 0.0);
  double maximum_segment_length = 0.0;
  for (std::size_t i = 0; i + 1U < reference.size(); ++i) {
    const double segment_length = std::hypot(
      reference[i + 1U].x - reference[i].x,
      reference[i + 1U].y - reference[i].y);
    cumulative_s[i + 1U] = cumulative_s[i] + segment_length;
    maximum_segment_length = std::max(maximum_segment_length, segment_length);
  }
  const double reference_length = cumulative_s.back();
  if (!(reference_length > 0.0)) {
    return std::nullopt;
  }
  const bool closed_loop =
    std::hypot(
    reference.front().x - reference.back().x,
    reference.front().y - reference.back().y) <= 1.0e-6;
  const double center_s = wrapS(center.s, reference_length);
  // Every AABB point is at most one diagonal from its centre. The extra longest segment includes
  // both segments adjacent to a waypoint while excluding a spatially close, distant snake branch.
  const double branch_window = diagonal + maximum_segment_length;

  double best_squared_distance = std::numeric_limits<double>::infinity();
  bool checked_segment = false;
  for (std::size_t i = 0; i + 1U < reference.size(); ++i) {
    const bool center_segment = static_cast<int>(i) == center.segment_index;
    const double longitudinal_distance = distanceToLongitudinalInterval(
      center_s, cumulative_s[i], cumulative_s[i + 1U], reference_length, closed_loop);
    if (!center_segment && longitudinal_distance > branch_window) {
      continue;
    }
    checked_segment = true;
    best_squared_distance = std::min(
      best_squared_distance,
      squaredDistanceBetweenSegmentAndAabb(
        reference[i].x, reference[i].y,
        reference[i + 1U].x, reference[i + 1U].y,
        x_min, x_max, y_min, y_max));
    if (best_squared_distance <= kGeometryEpsilon) {
      return 0.0;
    }
  }

  if (!checked_segment || !std::isfinite(best_squared_distance)) {
    return std::nullopt;
  }
  return std::sqrt(std::max(0.0, best_squared_distance));
}

}  // namespace

std::optional<FrenetAabbBounds> projectCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  double x_min,
  double x_max,
  double y_min,
  double y_max)
{
  if (!std::isfinite(x_min) || !std::isfinite(x_max) ||
    !std::isfinite(y_min) || !std::isfinite(y_max) ||
    x_min > x_max || y_min > y_max)
  {
    return std::nullopt;
  }

  const double width = x_max - x_min;
  const double height = y_max - y_min;
  const double diagonal = std::hypot(width, height);
  if (!(diagonal > std::numeric_limits<double>::epsilon())) {
    return std::nullopt;
  }

  FrenetAabbBounds bounds;
  bounds.x_center = 0.5 * (x_min + x_max);
  bounds.y_center = 0.5 * (y_min + y_max);
  bounds.diagonal = diagonal;
  const double track_length = converter.stats().track_length;

  global_planning::ClcsConversionInput center_input;
  center_input.x = bounds.x_center;
  center_input.y = bounds.y_center;
  const auto center = converter.convert(center_input);
  if (!center.valid || !std::isfinite(center.s) || !std::isfinite(center.d) ||
    !std::isfinite(center.reference_yaw))
  {
    return std::nullopt;
  }

  bounds.s_center = wrapS(center.s, track_length);
  bounds.d_center = center.d;

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
    const double dx = corner.first - bounds.x_center;
    const double dy = corner.second - bounds.y_center;
    const double longitudinal = dx * tangent_x + dy * tangent_y;
    const double lateral = dx * normal_x + dy * normal_y;
    min_longitudinal = std::min(min_longitudinal, longitudinal);
    max_longitudinal = std::max(max_longitudinal, longitudinal);
    min_lateral = std::min(min_lateral, lateral);
    max_lateral = std::max(max_lateral, lateral);
  }

  bounds.s_start = wrapS(bounds.s_center + min_longitudinal, track_length);
  bounds.s_end = wrapS(bounds.s_center + max_longitudinal, track_length);
  bounds.d_right = bounds.d_center + min_lateral;
  bounds.d_left = bounds.d_center + max_lateral;
  bounds.longitudinal_half_extent =
    0.5 * (max_longitudinal - min_longitudinal);

  const auto closest_abs_d = closestBranchLockedAabbDistance(
    converter, center, x_min, x_max, y_min, y_max, diagonal);
  if (!closest_abs_d.has_value()) {
    return std::nullopt;
  }
  bounds.closest_abs_d = closest_abs_d.value();
  // Preserve the far side of the centre-tangent envelope for avoidance target construction, but
  // replace its race-line-facing side with the exact curve-to-AABB distance. Blocking therefore
  // uses the closest face on the centre-locked branch instead of a single-tangent approximation.
  if (bounds.closest_abs_d <= kGeometryEpsilon) {
    bounds.d_right = std::min(bounds.d_right, 0.0);
    bounds.d_left = std::max(bounds.d_left, 0.0);
  } else if (bounds.d_center > 0.0) {
    bounds.d_right = bounds.closest_abs_d;
    bounds.d_left = std::max(bounds.d_left, bounds.d_right);
  } else {
    bounds.d_left = -bounds.closest_abs_d;
    bounds.d_right = std::min(bounds.d_right, bounds.d_left);
  }
  return bounds;
}

}  // namespace obstacle_detector
