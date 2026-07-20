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

#include "local_planning/safe_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kIntervalEpsilon = 1e-6;

std::vector<LateralInterval> mergeIntervals(std::vector<LateralInterval> intervals)
{
  if (intervals.empty()) {
    return intervals;
  }
  std::sort(
    intervals.begin(), intervals.end(),
    [](const LateralInterval & left, const LateralInterval & right) {
      return left.min_d < right.min_d;
    });
  std::vector<LateralInterval> merged;
  merged.reserve(intervals.size());
  merged.push_back(intervals.front());
  for (std::size_t index = 1; index < intervals.size(); ++index) {
    auto & tail = merged.back();
    const auto & current = intervals[index];
    if (current.min_d <= tail.max_d + kIntervalEpsilon) {
      tail.max_d = std::max(tail.max_d, current.max_d);
      tail.contains_unknown = tail.contains_unknown || current.contains_unknown;
    } else {
      merged.push_back(current);
    }
  }
  return merged;
}

std::vector<LateralInterval> subtractIntervals(
  const double minimum, const double maximum,
  const std::vector<LateralInterval> & blocked)
{
  std::vector<LateralInterval> feasible;
  if (maximum <= minimum + kIntervalEpsilon) {
    return feasible;
  }
  double cursor = minimum;
  for (const auto & interval : blocked) {
    const double clipped_min = std::max(minimum, interval.min_d);
    const double clipped_max = std::min(maximum, interval.max_d);
    if (clipped_max <= minimum || clipped_min >= maximum) {
      continue;
    }
    if (clipped_min > cursor + kIntervalEpsilon) {
      feasible.push_back({cursor, clipped_min, false});
    }
    cursor = std::max(cursor, clipped_max);
    if (cursor >= maximum - kIntervalEpsilon) {
      break;
    }
  }
  if (cursor < maximum - kIntervalEpsilon) {
    feasible.push_back({cursor, maximum, false});
  }
  return feasible;
}

std::vector<LateralInterval> sideIntervals(
  const SafeCorridorSample & sample, const bool left_side)
{
  std::vector<LateralInterval> intervals;
  for (const auto & source : sample.feasible_intervals) {
    auto clipped = source;
    if (left_side) {
      clipped.min_d = std::max(0.0, clipped.min_d);
    } else {
      clipped.max_d = std::min(0.0, clipped.max_d);
    }
    if (clipped.max_d > clipped.min_d + kIntervalEpsilon) {
      intervals.push_back(clipped);
    }
  }
  return mergeIntervals(std::move(intervals));
}

double corridorInterpolationRatio(
  const std::vector<SafeCorridorSample> & samples,
  const int begin_index, const int end_index, const int index)
{
  const double begin_s = samples[static_cast<std::size_t>(begin_index)].reference.unwrapped_s;
  const double end_s = samples[static_cast<std::size_t>(end_index)].reference.unwrapped_s;
  const double current_s = samples[static_cast<std::size_t>(index)].reference.unwrapped_s;
  if (std::isfinite(begin_s) && std::isfinite(end_s) && std::isfinite(current_s) &&
    end_s > begin_s + kIntervalEpsilon)
  {
    return std::clamp((current_s - begin_s) / (end_s - begin_s), 0.0, 1.0);
  }
  return std::clamp(
    static_cast<double>(index - begin_index) /
    static_cast<double>(std::max(1, end_index - begin_index)), 0.0, 1.0);
}

bool quinticSegmentInsideCorridor(
  const std::vector<SafeCorridorSample> & samples,
  const int begin_index, const double begin_d,
  const int end_index, const double end_d,
  const bool left_side)
{
  for (int index = begin_index; index <= end_index; ++index) {
    const double ratio = corridorInterpolationRatio(samples, begin_index, end_index, index);
    const double blend = ratio * ratio * ratio *
      (10.0 + ratio * (-15.0 + 6.0 * ratio));
    const double d = begin_d + (end_d - begin_d) * blend;
    if ((left_side && d < -kIntervalEpsilon) ||
      (!left_side && d > kIntervalEpsilon) ||
      !SafeCorridorBuilder::corridorContains(
        samples[static_cast<std::size_t>(index)], d))
    {
      return false;
    }
  }
  return true;
}

}  // namespace

SafeCorridorBuilder::SafeCorridorBuilder(SafeCorridorConfig config)
: config_(std::move(config))
{
}

UnknownCellPolicy SafeCorridorBuilder::parseUnknownCellPolicy(const std::string & value)
{
  if (value == "treat_as_free") {
    return UnknownCellPolicy::kTreatAsFree;
  }
  if (value == "treat_as_occupied") {
    return UnknownCellPolicy::kTreatAsOccupied;
  }
  if (value == "reject_candidate") {
    return UnknownCellPolicy::kRejectCandidate;
  }
  throw std::invalid_argument(
          "unknown_cell_policy must be treat_as_free, treat_as_occupied, or reject_candidate");
}

const char * SafeCorridorBuilder::unknownCellPolicyName(const UnknownCellPolicy policy)
{
  switch (policy) {
    case UnknownCellPolicy::kTreatAsFree:
      return "treat_as_free";
    case UnknownCellPolicy::kTreatAsOccupied:
      return "treat_as_occupied";
    case UnknownCellPolicy::kRejectCandidate:
      return "reject_candidate";
  }
  return "reject_candidate";
}

bool SafeCorridorBuilder::intervalContains(
  const LateralInterval & interval, const double d)
{
  return d >= interval.min_d - kIntervalEpsilon && d <= interval.max_d + kIntervalEpsilon;
}

bool SafeCorridorBuilder::corridorContains(
  const SafeCorridorSample & sample, const double d)
{
  return std::any_of(
    sample.feasible_intervals.begin(), sample.feasible_intervals.end(),
    [d](const LateralInterval & interval) {return intervalContains(interval, d);});
}

std::vector<LateralInterval> SafeCorridorBuilder::intersectFeasibleIntervals(
  const std::vector<SafeCorridorSample> & samples,
  const int begin_index, const int end_index, const bool left_side)
{
  if (samples.empty() || begin_index < 0 || end_index < begin_index ||
    end_index >= static_cast<int>(samples.size()))
  {
    return {};
  }

  std::vector<LateralInterval> common;
  for (int sample_index = begin_index; sample_index <= end_index; ++sample_index) {
    std::vector<LateralInterval> side_intervals;
    for (const auto & interval :
      samples[static_cast<std::size_t>(sample_index)].feasible_intervals)
    {
      LateralInterval clipped = interval;
      if (left_side) {
        clipped.min_d = std::max(0.0, clipped.min_d);
      } else {
        clipped.max_d = std::min(0.0, clipped.max_d);
      }
      if (clipped.max_d > clipped.min_d + kIntervalEpsilon) {
        side_intervals.push_back(clipped);
      }
    }
    side_intervals = mergeIntervals(std::move(side_intervals));
    if (side_intervals.empty()) {
      return {};
    }
    if (sample_index == begin_index) {
      common = std::move(side_intervals);
      continue;
    }

    std::vector<LateralInterval> intersection;
    for (const auto & previous : common) {
      for (const auto & current : side_intervals) {
        const double minimum = std::max(previous.min_d, current.min_d);
        const double maximum = std::min(previous.max_d, current.max_d);
        if (maximum > minimum + kIntervalEpsilon) {
          intersection.push_back(
            {minimum, maximum,
              previous.contains_unknown || current.contains_unknown});
        }
      }
    }
    common = mergeIntervals(std::move(intersection));
    if (common.empty()) {
      return {};
    }
  }
  return common;
}

std::vector<double> SafeCorridorBuilder::sampleTargetOffsets(
  const std::vector<LateralInterval> & intervals,
  const double preferred_offset, const int sample_count,
  const double boundary_inset_m)
{
  if (sample_count <= 0 || intervals.empty()) {
    return {};
  }

  struct InteriorInterval
  {
    double minimum;
    double maximum;
  };
  std::vector<InteriorInterval> interiors;
  interiors.reserve(intervals.size());
  const double inset = std::max(0.0, boundary_inset_m);
  for (const auto & interval : intervals) {
    if (interval.max_d <= interval.min_d + kIntervalEpsilon) {
      continue;
    }
    double minimum = interval.min_d + inset;
    double maximum = interval.max_d - inset;
    if (maximum <= minimum + kIntervalEpsilon) {
      minimum = 0.5 * (interval.min_d + interval.max_d);
      maximum = minimum;
    }
    interiors.push_back({minimum, maximum});
  }
  if (interiors.empty()) {
    return {};
  }

  std::vector<double> targets;
  targets.reserve(static_cast<std::size_t>(sample_count));
  const auto add_unique = [&targets, sample_count](const double value) {
      if (static_cast<int>(targets.size()) >= sample_count) {
        return;
      }
      const bool duplicate = std::any_of(
        targets.begin(), targets.end(),
        [value](const double existing) {return std::abs(existing - value) <= 1e-4;});
      if (!duplicate) {
        targets.push_back(value);
      }
    };

  // Give every disconnected passage a preferred and a centered target before
  // using remaining samples at its boundaries.
  for (const auto & interval : interiors) {
    add_unique(std::clamp(preferred_offset, interval.minimum, interval.maximum));
  }
  for (const auto & interval : interiors) {
    add_unique(0.5 * (interval.minimum + interval.maximum));
  }
  for (const auto & interval : interiors) {
    add_unique(interval.minimum);
    add_unique(interval.maximum);
  }

  // If more density was requested, distribute it uniformly through each
  // passage. This never creates a target outside the common safe interval.
  for (int division = 2;
    static_cast<int>(targets.size()) < sample_count && division <= sample_count + 1;
    ++division)
  {
    for (const auto & interval : interiors) {
      for (int index = 1;
        index < division && static_cast<int>(targets.size()) < sample_count; ++index)
      {
        const double ratio = static_cast<double>(index) / static_cast<double>(division);
        add_unique(interval.minimum + ratio * (interval.maximum - interval.minimum));
      }
    }
  }
  return targets;
}

std::vector<std::vector<LateralTargetKnot>>
SafeCorridorBuilder::buildCorridorGuidedProfiles(
  const std::vector<SafeCorridorSample> & samples,
  const int begin_index, const int end_index, const bool left_side,
  const double preferred_offset, const int lateral_sample_count,
  const double boundary_inset_m, const int knot_stride, const int beam_width)
{
  if (samples.empty() || begin_index < 0 || end_index < begin_index ||
    end_index >= static_cast<int>(samples.size()) || lateral_sample_count <= 0 ||
    beam_width <= 0)
  {
    return {};
  }

  struct PartialProfile
  {
    std::vector<LateralTargetKnot> knots;
    double score{0.0};
  };

  // A coarse knot spacing is cheaper on ordinary corridors. If no quintic
  // connection can remain inside every intermediate slice, progressively add
  // knots down to every waypoint. This changes candidate geometry, not the
  // hard corridor or footprint limits.
  for (int active_stride = std::max(1, knot_stride); active_stride >= 1;
    active_stride = active_stride == 1 ? 0 : std::max(1, active_stride / 2))
  {
    std::vector<int> knot_indices;
    for (int index = begin_index; index <= end_index; index += active_stride) {
      knot_indices.push_back(index);
    }
    if (knot_indices.empty() || knot_indices.back() != end_index) {
      knot_indices.push_back(end_index);
    }

    std::vector<PartialProfile> beam;
    bool failed = false;
    for (const int knot_index : knot_indices) {
      const auto intervals = sideIntervals(
        samples[static_cast<std::size_t>(knot_index)], left_side);
      const auto targets = sampleTargetOffsets(
        intervals, preferred_offset, lateral_sample_count, boundary_inset_m);
      if (targets.empty()) {
        failed = true;
        break;
      }

      std::vector<PartialProfile> next_beam;
      if (beam.empty()) {
        next_beam.reserve(targets.size());
        for (const double target : targets) {
          PartialProfile profile;
          profile.knots.push_back({knot_index, target});
          profile.score = 0.10 * std::pow(target - preferred_offset, 2.0);
          next_beam.push_back(std::move(profile));
        }
      } else {
        next_beam.reserve(beam.size() * targets.size());
        for (const auto & partial : beam) {
          const auto & previous = partial.knots.back();
          for (const double target : targets) {
            if (!quinticSegmentInsideCorridor(
                samples, previous.waypoint_offset, previous.d,
                knot_index, target, left_side))
            {
              continue;
            }
            PartialProfile extended = partial;
            extended.knots.push_back({knot_index, target});
            const double span = std::max(1, knot_index - previous.waypoint_offset);
            extended.score +=
              std::pow(target - previous.d, 2.0) / static_cast<double>(span) +
              0.10 * std::pow(target - preferred_offset, 2.0);
            next_beam.push_back(std::move(extended));
          }
        }
      }
      if (next_beam.empty()) {
        failed = true;
        break;
      }
      std::sort(
        next_beam.begin(), next_beam.end(),
        [](const PartialProfile & left, const PartialProfile & right) {
          return left.score < right.score;
        });
      if (static_cast<int>(next_beam.size()) > beam_width) {
        next_beam.resize(static_cast<std::size_t>(beam_width));
      }
      beam = std::move(next_beam);
    }

    if (!failed && !beam.empty()) {
      std::vector<std::vector<LateralTargetKnot>> profiles;
      profiles.reserve(beam.size());
      for (auto & partial : beam) {
        profiles.push_back(std::move(partial.knots));
      }
      return profiles;
    }
  }
  return {};
}

std::vector<std::vector<LateralTargetKnot>>
SafeCorridorBuilder::buildFullHorizonGuidedProfiles(
  const std::vector<SafeCorridorSample> & samples,
  const int window_begin, const int collision_begin, const int collision_end,
  const int window_end, const bool left_side, const double start_d,
  const double end_d, const double preferred_offset,
  const int lateral_sample_count, const double boundary_inset_m,
  const int knot_stride, const int beam_width)
{
  if (samples.empty() || window_begin < 0 || collision_begin <= window_begin ||
    collision_end < collision_begin || window_end <= collision_end ||
    window_end >= static_cast<int>(samples.size()) || lateral_sample_count <= 0 ||
    beam_width <= 0 ||
    !corridorContains(samples[static_cast<std::size_t>(window_begin)], start_d) ||
    !corridorContains(samples[static_cast<std::size_t>(window_end)], end_d))
  {
    return {};
  }

  struct PartialProfile
  {
    std::vector<LateralTargetKnot> knots;
    double score{0.0};
    double previous_slope{0.0};
  };

  const auto segment_inside = [&](const LateralTargetKnot & from,
      const LateralTargetKnot & to) {
      for (int index = from.waypoint_offset; index <= to.waypoint_offset; ++index) {
        const double ratio = corridorInterpolationRatio(
          samples, from.waypoint_offset, to.waypoint_offset, index);
        const double blend = ratio * ratio * ratio *
          (10.0 + ratio * (-15.0 + 6.0 * ratio));
        const double d = from.d + (to.d - from.d) * blend;
        const bool inside_collision = index >= collision_begin && index <= collision_end;
        if ((inside_collision && left_side && d < -kIntervalEpsilon) ||
          (inside_collision && !left_side && d > kIntervalEpsilon) ||
          !corridorContains(samples[static_cast<std::size_t>(index)], d))
        {
          return false;
        }
      }
      return true;
    };

  for (int active_stride = std::max(1, knot_stride); active_stride >= 1;
    active_stride = active_stride == 1 ? 0 : std::max(1, active_stride / 2))
  {
    std::vector<int> knot_indices{window_begin};
    for (int index = window_begin + active_stride; index < window_end;
      index += active_stride)
    {
      knot_indices.push_back(index);
    }
    // Do not force knots exactly at the first and last occupied slices.  Doing
    // so pins the lateral profile to a nearly constant offset through a short
    // obstacle cluster.  On a curved reference this can drive 1-kappa*d close
    // to zero and create a cusp even though the interpolated profile remains
    // inside the safe corridor.  The regularly spaced knots (and the finer
    // stride retries below) still enforce every collision slice through
    // segment_inside(), while allowing the vehicle to pass the obstacle with
    // a continuous diagonal motion.
    knot_indices.push_back(window_end);
    std::sort(knot_indices.begin(), knot_indices.end());
    knot_indices.erase(std::unique(knot_indices.begin(), knot_indices.end()), knot_indices.end());

    std::vector<PartialProfile> beam(1);
    beam.front().knots.push_back({window_begin, start_d});
    bool failed = false;
    for (std::size_t knot_position = 1U; knot_position < knot_indices.size(); ++knot_position) {
      const int knot_index = knot_indices[knot_position];
      std::vector<double> targets;
      if (knot_index == window_end) {
        targets.push_back(end_d);
      } else {
        const bool inside_collision =
          knot_index >= collision_begin && knot_index <= collision_end;
        const auto intervals = inside_collision ?
          sideIntervals(samples[static_cast<std::size_t>(knot_index)], left_side) :
          samples[static_cast<std::size_t>(knot_index)].feasible_intervals;
        const double target_preference = inside_collision ? preferred_offset :
          (knot_index < collision_begin ?
          start_d + (preferred_offset - start_d) * corridorInterpolationRatio(
            samples, window_begin, collision_begin, knot_index) :
          preferred_offset + (end_d - preferred_offset) * corridorInterpolationRatio(
            samples, collision_end, window_end, knot_index));
        targets = sampleTargetOffsets(
          intervals, target_preference, lateral_sample_count, boundary_inset_m);
      }
      if (targets.empty()) {
        failed = true;
        break;
      }

      std::vector<PartialProfile> next_beam;
      next_beam.reserve(beam.size() * targets.size());
      for (const auto & partial : beam) {
        const auto & previous = partial.knots.back();
        const double previous_s = samples[static_cast<std::size_t>(
              previous.waypoint_offset)].reference.unwrapped_s;
        const double current_s =
          samples[static_cast<std::size_t>(knot_index)].reference.unwrapped_s;
        const double span = std::max(1e-3, current_s - previous_s);
        for (const double target : targets) {
          const LateralTargetKnot current{knot_index, target};
          if (!segment_inside(previous, current)) {
            continue;
          }
          PartialProfile extended = partial;
          extended.knots.push_back(current);
          const double slope = (target - previous.d) / span;
          const double preference = knot_index >= collision_begin &&
            knot_index <= collision_end ? preferred_offset : target;
          extended.score +=
            0.35 * slope * slope +
            0.80 * std::pow(slope - partial.previous_slope, 2.0) +
            0.10 * std::pow(target - preference, 2.0);
          extended.previous_slope = slope;
          next_beam.push_back(std::move(extended));
        }
      }
      if (next_beam.empty()) {
        failed = true;
        break;
      }
      std::sort(
        next_beam.begin(), next_beam.end(),
        [](const PartialProfile & a, const PartialProfile & b) {
          return a.score < b.score;
        });
      if (static_cast<int>(next_beam.size()) > beam_width) {
        next_beam.resize(static_cast<std::size_t>(beam_width));
      }
      beam = std::move(next_beam);
    }

    if (!failed && !beam.empty()) {
      std::vector<std::vector<LateralTargetKnot>> profiles;
      profiles.reserve(beam.size());
      for (auto & partial : beam) {
        // buildQuinticLatticePath adds the fixed window endpoints itself.
        partial.knots.erase(partial.knots.begin());
        partial.knots.pop_back();
        profiles.push_back(std::move(partial.knots));
      }
      return profiles;
    }
  }
  return {};
}

bool SafeCorridorBuilder::worldToMap(
  const nav_msgs::msg::OccupancyGrid & grid,
  const double origin_cos, const double origin_sin,
  const double x, const double y, int & column, int & row) const
{
  if (grid.info.resolution <= 0.0) {
    return false;
  }
  const double dx = x - grid.info.origin.position.x;
  const double dy = y - grid.info.origin.position.y;
  const double local_x = origin_cos * dx + origin_sin * dy;
  const double local_y = -origin_sin * dx + origin_cos * dy;
  column = static_cast<int>(std::floor(local_x / grid.info.resolution));
  row = static_cast<int>(std::floor(local_y / grid.info.resolution));
  return column >= 0 && column < static_cast<int>(grid.info.width) &&
         row >= 0 && row < static_cast<int>(grid.info.height);
}

void SafeCorridorBuilder::mapCellCenter(
  const nav_msgs::msg::OccupancyGrid & grid,
  const double origin_cos, const double origin_sin,
  const int column, const int row, double & x, double & y) const
{
  const double local_x = (static_cast<double>(column) + 0.5) * grid.info.resolution;
  const double local_y = (static_cast<double>(row) + 0.5) * grid.info.resolution;
  x = grid.info.origin.position.x + origin_cos * local_x - origin_sin * local_y;
  y = grid.info.origin.position.y + origin_sin * local_x + origin_cos * local_y;
}

bool SafeCorridorBuilder::isBlockingCell(
  const int8_t occupancy, bool & is_unknown) const
{
  is_unknown = occupancy < 0;
  if (is_unknown) {
    return config_.unknown_policy != UnknownCellPolicy::kTreatAsFree;
  }
  return occupancy > config_.occupied_threshold;
}

SafeCorridorResult SafeCorridorBuilder::build(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<CorridorReferenceSample> & references) const
{
  SafeCorridorResult result;
  result.samples.reserve(references.size());
  if (grid.info.resolution <= 0.0 || grid.info.width == 0U || grid.info.height == 0U ||
    grid.data.size() != static_cast<std::size_t>(grid.info.width * grid.info.height))
  {
    return result;
  }

  const auto & q = grid.info.origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double origin_cos = std::cos(origin_yaw);
  const double origin_sin = std::sin(origin_yaw);
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const double cell_radius = grid.info.resolution * std::sqrt(0.5);
  const double circular_support = config_.preserve_circular_collision_check ?
    config_.vehicle_radius_m + config_.path_clearance_margin_m : 0.0;
  const double rectangular_margin = config_.localization_margin_m +
    config_.safety_margin_m + config_.path_clearance_margin_m;
  const double lateral_support = std::max(
    circular_support,
    0.5 * config_.vehicle_width_m + rectangular_margin);
  const double longitudinal_support = std::max(
    circular_support,
    std::max(config_.vehicle_front_extent_m, config_.vehicle_rear_extent_m) +
    rectangular_margin);
  std::unordered_set<int> inflated_cells;

  for (const auto & reference : references) {
    SafeCorridorSample sample;
    sample.reference = reference;
    sample.track_min_d = -reference.d_right + lateral_support;
    sample.track_max_d = reference.d_left - lateral_support;
    sample.blocking_raw_min_d = std::numeric_limits<double>::infinity();
    sample.blocking_raw_max_d = -std::numeric_limits<double>::infinity();

    if (sample.track_max_d <= sample.track_min_d) {
      sample.blocked_intervals.push_back(
        {sample.track_min_d, sample.track_max_d, false});
      sample.center_blocked = true;
      result.samples.push_back(std::move(sample));
      continue;
    }

    const double longitudinal_half_width = std::max(
      config_.minimum_longitudinal_half_width_m,
      reference.longitudinal_half_span + longitudinal_support + cell_radius);
    const double lateral_search = std::max(reference.d_left, reference.d_right) +
      lateral_support + cell_radius;
    const double search_radius = std::hypot(longitudinal_half_width, lateral_search);
    const int radius_cells = static_cast<int>(std::ceil(search_radius / grid.info.resolution));
    int center_column = 0;
    int center_row = 0;
    if (!worldToMap(
        grid, origin_cos, origin_sin, reference.x, reference.y,
        center_column, center_row))
    {
      sample.center_blocked = true;
      result.samples.push_back(std::move(sample));
      continue;
    }

    for (int row = std::max(0, center_row - radius_cells);
      row <= std::min(height - 1, center_row + radius_cells); ++row)
    {
      for (int column = std::max(0, center_column - radius_cells);
        column <= std::min(width - 1, center_column + radius_cells); ++column)
      {
        const int map_index = row * width + column;
        bool is_unknown = false;
        if (!isBlockingCell(grid.data[map_index], is_unknown)) {
          continue;
        }
        double cell_x = 0.0;
        double cell_y = 0.0;
        mapCellCenter(grid, origin_cos, origin_sin, column, row, cell_x, cell_y);
        const double dx = cell_x - reference.x;
        const double dy = cell_y - reference.y;
        const double longitudinal = dx * std::cos(reference.yaw) +
          dy * std::sin(reference.yaw);
        if (std::abs(longitudinal) > longitudinal_half_width) {
          continue;
        }
        const double lateral = -dx * std::sin(reference.yaw) +
          dy * std::cos(reference.yaw);
        const double raw_min = lateral - cell_radius;
        const double raw_max = lateral + cell_radius;
        const LateralInterval inflated{
          std::max(sample.track_min_d, raw_min - lateral_support),
          std::min(sample.track_max_d, raw_max + lateral_support),
          is_unknown};
        if (inflated.max_d <= sample.track_min_d ||
          inflated.min_d >= sample.track_max_d)
        {
          continue;
        }
        sample.blocked_intervals.push_back(inflated);
        inflated_cells.insert(map_index);
        sample.inflated_cell_indices.push_back(map_index);
        if (intervalContains(inflated, 0.0)) {
          sample.center_blocked = true;
          sample.center_blocked_by_unknown =
            sample.center_blocked_by_unknown || is_unknown;
          sample.blocking_raw_min_d = std::min(sample.blocking_raw_min_d, raw_min);
          sample.blocking_raw_max_d = std::max(sample.blocking_raw_max_d, raw_max);
        }
      }
    }

    sample.blocked_intervals = mergeIntervals(std::move(sample.blocked_intervals));
    std::sort(sample.inflated_cell_indices.begin(), sample.inflated_cell_indices.end());
    sample.inflated_cell_indices.erase(
      std::unique(
        sample.inflated_cell_indices.begin(), sample.inflated_cell_indices.end()),
      sample.inflated_cell_indices.end());
    sample.feasible_intervals = subtractIntervals(
      sample.track_min_d, sample.track_max_d, sample.blocked_intervals);
    for (const auto & interval : sample.feasible_intervals) {
      sample.left_feasible = sample.left_feasible || interval.max_d > 0.0;
      sample.right_feasible = sample.right_feasible || interval.min_d < 0.0;
    }
    if (!std::isfinite(sample.blocking_raw_min_d)) {
      sample.blocking_raw_min_d = 0.0;
      sample.blocking_raw_max_d = 0.0;
    }
    result.samples.push_back(std::move(sample));
  }

  result.inflated_cell_indices.assign(inflated_cells.begin(), inflated_cells.end());
  std::sort(result.inflated_cell_indices.begin(), result.inflated_cell_indices.end());
  return result;
}

}  // namespace local_planning
