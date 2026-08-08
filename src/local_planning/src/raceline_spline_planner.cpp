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

#include "local_planning/raceline_spline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool finiteWaypoint(const f110_msgs::msg::Wpnt & waypoint)
{
  return std::isfinite(waypoint.s_m) && std::isfinite(waypoint.d_m) &&
         std::isfinite(waypoint.x_m) && std::isfinite(waypoint.y_m) &&
         std::isfinite(waypoint.psi_rad) && std::isfinite(waypoint.kappa_radpm) &&
         std::isfinite(waypoint.vx_mps) && std::isfinite(waypoint.ax_mps2) &&
         std::isfinite(waypoint.d_left) && std::isfinite(waypoint.d_right);
}

double pointDistance(
  const f110_msgs::msg::Wpnt & first,
  const f110_msgs::msg::Wpnt & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

// Monotone fifth-order smoothstep. Position, first derivative, and second derivative are all
// continuous with a constant-d segment at both ends, so entry/exit steering does not jump.
double quinticSmoothStep(double progress)
{
  const double u = clamp(progress, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double quinticBlend(double start, double finish, double progress)
{
  return start + (finish - start) * quinticSmoothStep(progress);
}

}  // namespace

struct RacelineSplinePlanner::ExpandedObstacle
{
  int id{-1};
  double start{0.0};
  double end{0.0};
  double center{0.0};
  double raw_d_right{0.0};
  double raw_d_left{0.0};
  double d_right{0.0};
  double d_left{0.0};
  double clearance{0.0};
};

struct RacelineSplinePlanner::Candidate
{
  bool valid{false};
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  double score{std::numeric_limits<double>::infinity()};
  double headroom{-std::numeric_limits<double>::infinity()};
  f110_msgs::msg::WpntArray path;
  std::vector<SplineControlPoint> control_points;
  std::string reason;
};

RacelineSplinePlanner::RacelineSplinePlanner(RacelineSplineParameters parameters)
: parameters_(std::move(parameters))
{
}

void RacelineSplinePlanner::setParameters(const RacelineSplineParameters & parameters)
{
  parameters_ = parameters;
}

bool RacelineSplinePlanner::setReference(
  const f110_msgs::msg::WpntArray & reference,
  std::string * error)
{
  auto reject = [&](const std::string & why) {
      reference_.wpnts.clear();
      track_length_ = 0.0;
      if (error != nullptr) {
        *error = why;
      }
      return false;
    };

  if (reference.wpnts.size() < 4U) {
    return reject("global reference needs at least four waypoints");
  }
  std::vector<double> spacing;
  spacing.reserve(reference.wpnts.size() - 1U);
  for (std::size_t i = 0; i < reference.wpnts.size(); ++i) {
    if (!finiteWaypoint(reference.wpnts[i])) {
      return reject("global reference contains a non-finite waypoint");
    }
    if (i > 0U) {
      const double ds = reference.wpnts[i].s_m - reference.wpnts[i - 1U].s_m;
      if (!(ds > kEpsilon)) {
        return reject("global reference s_m must be strictly increasing");
      }
      spacing.push_back(ds);
    }
  }
  std::sort(spacing.begin(), spacing.end());
  const double median_spacing = spacing[spacing.size() / 2U];
  const double inferred_length = reference.wpnts.back().s_m + median_spacing;
  if (!(inferred_length > reference.wpnts.back().s_m) || !std::isfinite(inferred_length)) {
    return reject("failed to infer a positive closed-track length");
  }

  reference_ = reference;
  track_length_ = inferred_length;
  return true;
}

bool RacelineSplinePlanner::ready() const
{
  return reference_.wpnts.size() >= 4U && track_length_ > 0.0;
}

double RacelineSplinePlanner::trackLength() const
{
  return track_length_;
}

double RacelineSplinePlanner::wrapS(double s) const
{
  if (!(track_length_ > 0.0)) {
    return s;
  }
  double wrapped = std::fmod(s, track_length_);
  if (wrapped < 0.0) {
    wrapped += track_length_;
  }
  return wrapped;
}

double RacelineSplinePlanner::forwardDistance(double from_s, double to_s) const
{
  return wrapS(to_s - from_s);
}

std::vector<int> RacelineSplinePlanner::blockingClusterIds(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed))
  {
    return {};
  }
  const auto cluster = nearestCluster(expandVisibleObstacles(ego, obstacles));
  std::vector<int> ids;
  ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    ids.push_back(obstacle.id);
  }
  return ids;
}

std::size_t RacelineSplinePlanner::nextReferenceIndex(double s) const
{
  const double wrapped_s = wrapS(s);
  const auto iterator = std::lower_bound(
    reference_.wpnts.begin(), reference_.wpnts.end(), wrapped_s,
    [](const f110_msgs::msg::Wpnt & waypoint, double value) {
      return waypoint.s_m < value;
    });
  return iterator == reference_.wpnts.end() ?
         0U : static_cast<std::size_t>(std::distance(reference_.wpnts.begin(), iterator));
}

std::size_t RacelineSplinePlanner::nearestReferenceIndex(double s) const
{
  const std::size_t next = nextReferenceIndex(s);
  const std::size_t previous =
    next == 0U ? reference_.wpnts.size() - 1U : next - 1U;
  const auto circular_distance = [this, s](std::size_t index) {
      const double forward = forwardDistance(s, reference_.wpnts[index].s_m);
      return std::min(forward, track_length_ - forward);
    };
  return circular_distance(next) < circular_distance(previous) ? next : previous;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::expandVisibleObstacles(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  // Obstacle input may already be an uncertainty Guard, but vehicle size, physical margin, and
  // closed-loop tracking reserve are applied exactly once here.
  const double clearance = parameters_.obstacleSafetyClearance();
  std::vector<ExpandedObstacle> visible;
  visible.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    if (!std::isfinite(obstacle.s_center) || !std::isfinite(obstacle.s_start) ||
      !std::isfinite(obstacle.s_end) || !std::isfinite(obstacle.d_left) ||
      !std::isfinite(obstacle.d_right))
    {
      continue;
    }
    const double center = forwardDistance(ego.s, obstacle.s_center);
    const double span_forward = forwardDistance(obstacle.s_start, obstacle.s_end);
    const double span_reverse = forwardDistance(obstacle.s_end, obstacle.s_start);
    double span = std::min(span_forward, span_reverse);
    if (!(span > kEpsilon)) {
      span = std::max(0.05, std::abs(obstacle.size));
    }
    const double half_span = 0.5 * span + parameters_.obstacle_longitudinal_padding_m;
    ExpandedObstacle expanded;
    expanded.id = obstacle.id;
    expanded.center = center;
    expanded.start = center - half_span;
    expanded.end = center + half_span;
    expanded.raw_d_right = std::min(obstacle.d_right, obstacle.d_left);
    expanded.raw_d_left = std::max(obstacle.d_right, obstacle.d_left);
    expanded.d_right = expanded.raw_d_right - clearance;
    expanded.d_left = expanded.raw_d_left + clearance;
    expanded.clearance = clearance;
    if (expanded.end >= 0.0 &&
      expanded.start <= parameters_.detection_lookahead_m)
    {
      visible.push_back(expanded);
    }
  }
  std::sort(
    visible.begin(), visible.end(),
    [](const ExpandedObstacle & first, const ExpandedObstacle & second) {
      return first.start < second.start;
    });
  return visible;
}

bool RacelineSplinePlanner::isBlockingRaceline(const ExpandedObstacle & obstacle) const
{
  const double envelope = parameters_.obstacleSafetyClearance();
  return obstacle.raw_d_right <= envelope && obstacle.raw_d_left >= -envelope;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::nearestCluster(
  const std::vector<ExpandedObstacle> & obstacles) const
{
  auto first = std::find_if(
    obstacles.begin(), obstacles.end(),
    [this](const ExpandedObstacle & obstacle) {return isBlockingRaceline(obstacle);});
  if (first == obstacles.end()) {
    return {};
  }

  std::vector<ExpandedObstacle> cluster;
  cluster.push_back(*first);
  double cluster_end = first->end;
  for (auto current = std::next(first); current != obstacles.end(); ++current) {
    if (current->start > cluster_end + parameters_.obstacle_cluster_gap_m) {
      break;
    }
    cluster.push_back(*current);
    cluster_end = std::max(cluster_end, current->end);
  }
  return cluster;
}

bool RacelineSplinePlanner::outsideIsLeft(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster) const
{
  if (cluster.empty()) {
    return false;
  }
  double cluster_start = cluster.front().start;
  double cluster_end = cluster.front().end;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
  }
  double curvature_sum = 0.0;
  const std::size_t obstacle_index = nearestReferenceIndex(
    wrapS(ego.s + 0.5 * (cluster_start + cluster_end)));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (obstacle_index + k) % reference_.wpnts.size();
    const double forward = forwardDistance(
      reference_.wpnts[obstacle_index].s_m, reference_.wpnts[index].s_m);
    if (forward > 2.0) {
      break;
    }
    curvature_sum += reference_.wpnts[index].kappa_radpm;
  }
  return curvature_sum < 0.0;
}

bool RacelineSplinePlanner::computeSideTarget(
  const std::vector<ExpandedObstacle> & cluster,
  bool go_left,
  double & cluster_start,
  double & cluster_end,
  double & target_d,
  std::string & reason) const
{
  if (cluster.empty()) {
    reason = "empty obstacle cluster";
    return false;
  }

  cluster_start = cluster.front().start;
  cluster_end = cluster.front().end;
  target_d = go_left ? cluster.front().d_left : cluster.front().d_right;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
    target_d = go_left ? std::max(target_d, obstacle.d_left) :
      std::min(target_d, obstacle.d_right);
  }
  if (go_left) {
    target_d = std::max(
      target_d,
      parameters_.minimum_target_offset_m);
  } else {
    target_d = std::min(
      target_d,
      -parameters_.minimum_target_offset_m);
  }
  if (std::abs(target_d) > parameters_.maximum_target_offset_m) {
    reason = "required d-offset exceeds maximum_target_offset_m";
    return false;
  }
  return true;
}

bool RacelineSplinePlanner::targetFitsTrackBounds(
  const EgoFrenetState & ego,
  double cluster_start,
  double cluster_end,
  bool go_left,
  double target_d,
  std::string & reason,
  double * min_headroom) const
{
  // Global waypoint d_left/d_right are centre-of-vehicle limits that already include vehicle
  // half-width and the generator's wall safety margin. Apply no additional local wall reserve.
  const double center_boundary_clearance = parameters_.trackBoundaryReserve();
  if (min_headroom != nullptr) {
    *min_headroom = std::numeric_limits<double>::infinity();
  }
  const auto fits_at = [&](const f110_msgs::msg::Wpnt & reference) {
      const double left_width = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right_width = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      const double headroom = go_left ?
        left_width - center_boundary_clearance - target_d :
        target_d + right_width - center_boundary_clearance;
      if (min_headroom != nullptr) {
        *min_headroom = std::min(*min_headroom, headroom);
      }
      return headroom >= -kEpsilon;
    };

  // The target offset is held across the expanded obstacle-cluster span. Reject an obviously
  // impossible side before fitting/sampling up to three splines, but retain the full candidate
  // validation because the transition can still meet a narrower wall before or after this span.
  const double check_start = std::max(0.0, cluster_start);
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference =
      reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > cluster_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    if (!fits_at(reference)) {
      reason = go_left ?
        "left target d exceeds track bound in obstacle span before spline construction" :
        "right target d exceeds track bound in obstacle span before spline construction";
      return false;
    }
  }

  // A very short obstacle span can fall between two global samples. Check its midpoint against the
  // nearest reference width so the fast gate remains useful without inventing a Cartesian wall.
  if (!checked_reference) {
    const double midpoint = 0.5 * (check_start + std::max(check_start, cluster_end));
    const auto & reference = reference_.wpnts[
      nearestReferenceIndex(wrapS(ego.s + midpoint))];
    if (!fits_at(reference)) {
      reason = go_left ?
        "left target d exceeds track bound in obstacle span before spline construction" :
        "right target d exceeds track bound in obstacle span before spline construction";
      return false;
    }
  }
  return true;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildGlobalHandoffPath(
  double ego_s, double state_tail_ratio, double speed_cap_mps) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego_s) || !(state_tail_ratio > 0.0) ||
    state_tail_ratio > 1.0 || !(speed_cap_mps > 0.0))
  {
    return path;
  }

  const std::size_t total = reference_.wpnts.size();
  const std::size_t tail_count = std::max<std::size_t>(
    1U,
    static_cast<std::size_t>(
      std::ceil(state_tail_ratio * static_cast<double>(total))));
  const std::size_t tail_begin = total - std::min(tail_count, total);
  const std::size_t ego_index = nearestReferenceIndex(ego_s);

  // controller에 충분한 전방 경로를 주기 위해 전체 global loop를 회전시켜 현재 ego를
  // 마지막 tail_ratio 구간의 첫 점에 놓는다. state_machine은 명시적인 handoff 표식을
  // 우선 사용하며, tail 배치는 기존 합류 판정과의 호환성을 유지한다.
  const std::size_t first_index = (ego_index + total - tail_begin) % total;
  path.wpnts.reserve(total);
  for (std::size_t k = 0; k < total; ++k) {
    auto waypoint = reference_.wpnts[(first_index + k) % total];
    waypoint.id = static_cast<int32_t>(k);
    waypoint.d_m = 0.0;
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), speed_cap_mps);
    path.wpnts.push_back(waypoint);
  }
  return path;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildEmergencyStopPath(
  const EgoFrenetState & ego) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d)) {
    return path;
  }
  const std::size_t count = std::min(
    reference_.wpnts.size(),
    std::max<std::size_t>(
      2U, static_cast<std::size_t>(parameters_.minimum_path_points)));
  const std::size_t first_index = nextReferenceIndex(ego.s);
  path.wpnts.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    const auto & global = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(k);
    waypoint.d_m = ego.d;
    waypoint.x_m = global.x_m - ego.d * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + ego.d * std::cos(global.psi_rad);
    waypoint.vx_mps = 0.0;
    waypoint.ax_mps2 = 0.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
}

RacelineSplineResult RacelineSplinePlanner::buildCommittedPathStop(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & committed_path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed) || committed_path.wpnts.empty())
  {
    result.reason = "cannot build a committed-path stop from invalid inputs";
    return result;
  }

  std::size_t start_index = 0U;
  double nearest_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < committed_path.wpnts.size(); ++i) {
    const double forward = forwardDistance(ego.s, committed_path.wpnts[i].s_m);
    if (forward < nearest_forward) {
      nearest_forward = forward;
      start_index = i;
    }
  }
  if (nearest_forward > 0.5 * track_length_) {
    result.reason = "committed path has no waypoint ahead for braking";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  double first_collision_forward = std::numeric_limits<double>::infinity();
  int first_collision_id = -1;
  for (std::size_t i = start_index; i < committed_path.wpnts.size(); ++i) {
    const auto & waypoint = committed_path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    for (const auto & obstacle : visible) {
      if (forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.d_right + kEpsilon &&
        waypoint.d_m < obstacle.d_left - kEpsilon)
      {
        first_collision_forward = forward_s;
        first_collision_id = obstacle.id;
        break;
      }
    }
    if (std::isfinite(first_collision_forward)) {
      break;
    }
  }
  if (!std::isfinite(first_collision_forward)) {
    result.reason = "committed path has no obstacle collision before which to stop";
    return result;
  }

  const double stop_at = std::max(
    0.0, first_collision_forward - parameters_.safe_stop_buffer_m);
  result.path.header = committed_path.header;
  for (std::size_t i = start_index; i < committed_path.wpnts.size(); ++i) {
    const double forward_s = forwardDistance(ego.s, committed_path.wpnts[i].s_m);
    if (forward_s > stop_at + kEpsilon) {
      break;
    }
    auto waypoint = committed_path.wpnts[i];
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.path.wpnts.clear();
    result.reason = "committed path has no collision-free braking prefix";
    return result;
  }

  const double first_forward = forwardDistance(ego.s, result.path.wpnts.front().s_m);
  const double first_lateral_delta =
    std::abs(result.path.wpnts.front().d_m - ego.d);
  const bool same_s_lateral_jump =
    first_forward <= kEpsilon && first_lateral_delta > kEpsilon;
  const bool excessive_entry_slope =
    first_forward > kEpsilon &&
    first_lateral_delta / first_forward > parameters_.maximum_lateral_slope;
  if (same_s_lateral_jump || excessive_entry_slope) {
    result.path.wpnts.clear();
    result.reason = "committed braking prefix is discontinuous from the current ego d";
    return result;
  }

  result.path.wpnts.back().vx_mps = 0.0;
  for (std::size_t reverse = result.path.wpnts.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t previous = reverse - 1U;
    const double distance = pointDistance(
      result.path.wpnts[previous], result.path.wpnts[reverse]);
    const double next_speed = std::max(0.0, result.path.wpnts[reverse].vx_mps);
    const double braking_speed = std::sqrt(
      next_speed * next_speed +
      2.0 * parameters_.safe_stop_deceleration_mps2 * std::max(0.0, distance));
    result.path.wpnts[previous].vx_mps = std::min(
      std::max(0.0, result.path.wpnts[previous].vx_mps), braking_speed);
  }
  updateGeometryAndAcceleration(result.path);

  std::string validation_reason;
  if (!validateCandidate(ego, result.path, visible, validation_reason, 0U, 2U)) {
    result.path.wpnts.clear();
    result.reason = "committed braking prefix rejected: " + validation_reason;
    return result;
  }
  result.kind = SplinePlanKind::kSafeStop;
  result.obstacle_id = first_collision_id;
  result.merge_s = result.path.wpnts.back().s_m;
  result.reason = "braking on the remaining committed geometry before a collision";
  return result;
}

RacelineSplinePlanner::Candidate RacelineSplinePlanner::buildCandidate(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  bool go_left,
  double entry_transition_scale,
  double exit_transition_scale,
  bool outside_is_left,
  double cluster_start,
  double cluster_end,
  double target_d) const
{
  Candidate candidate;
  candidate.go_left = go_left;

  if (go_left == outside_is_left) {
    entry_transition_scale *= parameters_.outside_line_transition_scale;
    exit_transition_scale *= parameters_.outside_line_transition_scale;
  }

  std::vector<double> knot_s;
  std::vector<double> knot_d;
  auto append_knot = [&](double forward_s, double d) {
      if (!knot_s.empty() && forward_s <= knot_s.back() + 1.0e-3) {
        if (forward_s > knot_s.back() - 1.0e-3) {
          knot_d.back() = d;
        }
        return;
      }
      knot_s.push_back(forward_s);
      knot_d.push_back(d);
    };

  const double pre_far = parameters_.pre_apex_distances_m[0] * entry_transition_scale;
  const double pre_middle = parameters_.pre_apex_distances_m[1] * entry_transition_scale;
  const double pre_near = parameters_.pre_apex_distances_m[2] * entry_transition_scale;
  const double post_near = parameters_.post_apex_distances_m[0] * exit_transition_scale;
  const double post_middle = parameters_.post_apex_distances_m[1] * exit_transition_scale;
  const double post_far = parameters_.post_apex_distances_m[2] * exit_transition_scale;

  // A long candidate is allowed to consume all currently available distance, but never starts
  // behind ego. If perception arrives late, anchoring entry at forward_s=0 still guarantees
  // d(0)=ego.d and zero Frenet slope/curvature at the new commitment boundary.
  if (!(cluster_start > kEpsilon)) {
    candidate.reason = "blocking obstacle has no positive quintic entry distance";
    return candidate;
  }
  const double entry_start = std::max(0.0, cluster_start - pre_far);
  const double entry_length = cluster_start - entry_start;
  if (!(entry_length > kEpsilon) || !(post_far > kEpsilon)) {
    candidate.reason = "quintic transition length is not positive";
    return candidate;
  }
  const double exit_end = cluster_end + post_far;
  const auto entry_offset = [&](double forward_s) {
      return quinticBlend(
        ego.d, target_d, (forward_s - entry_start) / entry_length);
    };
  const auto exit_offset = [&](double forward_s) {
      return quinticBlend(
        target_d, 0.0, (forward_s - cluster_end) / post_far);
    };
  const auto profile_offset = [&](double forward_s) {
      if (forward_s <= entry_start) {
        return ego.d;
      }
      if (forward_s < cluster_start) {
        return entry_offset(forward_s);
      }
      if (forward_s <= cluster_end) {
        return target_d;
      }
      if (forward_s < exit_end) {
        return exit_offset(forward_s);
      }
      return 0.0;
    };

  // Keep the familiar pre/apex/post marker layout, but sample each marker from the actual
  // quintic profile instead of pinning all pre/post points to d=0. This makes the intended
  // progressive lateral motion directly visible in RViz.
  append_knot(entry_start, ego.d);
  for (const double distance : {pre_middle, pre_near}) {
    const double forward_s = cluster_start - distance;
    if (forward_s > entry_start + kEpsilon && forward_s < cluster_start - kEpsilon) {
      append_knot(forward_s, entry_offset(forward_s));
    }
  }
  append_knot(cluster_start, target_d);
  if (cluster_end > cluster_start + 0.05) {
    append_knot(cluster_end, target_d);
  }
  for (const double distance : {post_near, post_middle}) {
    const double forward_s = cluster_end + distance;
    if (forward_s > cluster_end + kEpsilon && forward_s < exit_end - kEpsilon) {
      append_knot(forward_s, exit_offset(forward_s));
    }
  }
  append_knot(exit_end, 0.0);
  for (std::size_t i = 0; i < knot_s.size(); ++i) {
    candidate.control_points.push_back({knot_s[i], knot_d[i]});
  }

  const double spline_end = exit_end;
  // 고속에서 고정 2m tail은 state-machine handoff가 끝나기 전에 소진된다. 회피를 계획한
  // 순간의 ego 속도를 기준으로 최소 시간만큼 global d=0 구간을 확보하되, 저속에서는 기존
  // 거리 하한을 유지한다. 이 tail은 회피 형상을 바꾸지 않고 동일 ordered race-line 표본을
  // 뒤에 더 붙이는 것뿐이다.
  const double post_merge_tail = std::max(
    parameters_.post_merge_lookahead_m,
    std::abs(ego.speed) * parameters_.post_merge_min_time_sec);
  const double path_end = spline_end + post_merge_tail;
  const double clip_min = std::min({0.0, ego.d, target_d});
  const double clip_max = std::max({0.0, ego.d, target_d});
  const std::size_t first_index = nextReferenceIndex(ego.s);
  candidate.path.header = reference_.header;
  candidate.path.wpnts.reserve(reference_.wpnts.size());
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const auto & global = reference_.wpnts[index];
    const double forward_s = forwardDistance(ego.s, global.s_m);
    if (forward_s > path_end + kEpsilon) {
      break;
    }
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(candidate.path.wpnts.size());
    waypoint.d_m = clamp(profile_offset(forward_s), clip_min, clip_max);
    waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
    candidate.path.wpnts.push_back(waypoint);
  }
  if (candidate.path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    candidate.reason = "spline segment has too few global race-line samples";
    return candidate;
  }

  updateGeometryAndAcceleration(candidate.path);
  if (!validateCandidate(ego, candidate.path, visible, candidate.reason)) {
    return candidate;
  }

  candidate.valid = true;
  candidate.target_d = target_d;
  // merge_s는 발행 세그먼트 끝이 아니라 d-offset spline이 실제로 d=0에 복귀하는 지점이다.
  // tail 길이를 늘려도 합류 완료 판정 시점이 뒤로 밀리지 않아야 한다.
  candidate.merge_s = wrapS(ego.s + spline_end);
  candidate.score =
    std::abs(target_d) + 0.02 * entry_transition_scale + 0.002 * exit_transition_scale;
  return candidate;
}

void RacelineSplinePlanner::updateGeometryAndAcceleration(
  f110_msgs::msg::WpntArray & path) const
{
  auto & waypoints = path.wpnts;
  if (waypoints.size() < 2U) {
    return;
  }

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const std::size_t previous = (i == 0U) ? 0U : i - 1U;
    const std::size_t next = std::min(i + 1U, waypoints.size() - 1U);
    const double dx = waypoints[next].x_m - waypoints[previous].x_m;
    const double dy = waypoints[next].y_m - waypoints[previous].y_m;
    if (std::hypot(dx, dy) > kEpsilon) {
      waypoints[i].psi_rad = std::atan2(dy, dx);
    }
  }

  for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
    const auto & first = waypoints[i - 1U];
    const auto & middle = waypoints[i];
    const auto & last = waypoints[i + 1U];
    const double a = pointDistance(first, middle);
    const double b = pointDistance(middle, last);
    const double c = pointDistance(first, last);
    const double cross =
      (middle.x_m - first.x_m) * (last.y_m - first.y_m) -
      (middle.y_m - first.y_m) * (last.x_m - first.x_m);
    const double denominator = a * b * c;
    waypoints[i].kappa_radpm = (denominator > kEpsilon) ? 2.0 * cross / denominator : 0.0;
  }
  waypoints.front().kappa_radpm = waypoints[1].kappa_radpm;
  waypoints.back().kappa_radpm = waypoints[waypoints.size() - 2U].kappa_radpm;

  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const double distance = pointDistance(waypoints[i - 1U], waypoints[i]);
    if (distance > kEpsilon) {
      waypoints[i - 1U].ax_mps2 =
        (waypoints[i].vx_mps * waypoints[i].vx_mps -
        waypoints[i - 1U].vx_mps * waypoints[i - 1U].vx_mps) / (2.0 * distance);
    }
  }
  waypoints.back().ax_mps2 = 0.0;
}

bool RacelineSplinePlanner::validateCandidate(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<ExpandedObstacle> & visible,
  std::string & reason,
  std::size_t start_index,
  std::size_t minimum_points,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m) const
{
  if (failure != nullptr) {
    *failure = PathValidationFailure();
  }
  const auto reject = [&reason, failure](
    PathValidationFailureKind kind,
    const std::string & message,
    std::size_t waypoint_index = std::numeric_limits<std::size_t>::max(),
    const f110_msgs::msg::Wpnt * waypoint = nullptr,
    const ExpandedObstacle * obstacle = nullptr,
    double obstacle_s_start = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_s_end = std::numeric_limits<double>::quiet_NaN())
    {
      reason = message;
      if (failure != nullptr) {
        failure->kind = kind;
        failure->reason = message;
        failure->waypoint_index = waypoint_index;
        if (waypoint != nullptr) {
          failure->waypoint_s = waypoint->s_m;
          failure->waypoint_d = waypoint->d_m;
        }
        if (obstacle != nullptr) {
          failure->obstacle_id = obstacle->id;
          failure->obstacle_s_start = obstacle_s_start;
          failure->obstacle_s_end = obstacle_s_end;
          failure->obstacle_source_d_right = obstacle->raw_d_right;
          failure->obstacle_source_d_left = obstacle->raw_d_left;
          failure->obstacle_test_d_right = obstacle->d_right;
          failure->obstacle_test_d_left = obstacle->d_left;
          failure->obstacle_clearance = obstacle->clearance;
        }
      }
      return false;
    };
  if (start_index >= path.wpnts.size()) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "path has no waypoint ahead of ego");
  }
  if (minimum_points == 0U) {
    minimum_points = static_cast<std::size_t>(parameters_.minimum_path_points);
  }
  if (path.wpnts.size() - start_index < minimum_points) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "path does not meet minimum_path_points");
  }
  const double center_boundary_clearance = parameters_.trackBoundaryReserve();
  double previous_d = path.wpnts[start_index].d_m;
  double previous_s = 0.0;
  double previous_curvature = path.wpnts[start_index].kappa_radpm;
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const auto & waypoint = path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const std::size_t reference_index = nearestReferenceIndex(waypoint.s_m);
    const auto & reference = reference_.wpnts[reference_index];
    const double left_width = reference.d_left > 0.05 ?
      reference.d_left : parameters_.fallback_track_half_width_m;
    const double right_width = reference.d_right > 0.05 ?
      reference.d_right : parameters_.fallback_track_half_width_m;
    if (waypoint.d_m > left_width - center_boundary_clearance + 1.0e-6 ||
      waypoint.d_m < -right_width + center_boundary_clearance - 1.0e-6)
    {
      return reject(
        PathValidationFailureKind::kTrackBoundary,
        "d-offset leaves the global waypoint track bounds", i, &waypoint);
    }
    for (const auto & obstacle : visible) {
      if ((!maximum_collision_forward_m.has_value() ||
        forward_s <= maximum_collision_forward_m.value() + kEpsilon) &&
        forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.d_right + 1.0e-6 &&
        waypoint.d_m < obstacle.d_left - 1.0e-6)
      {
        return reject(
          PathValidationFailureKind::kObstacleCollision,
          "d-offset intersects an inflated static-obstacle box", i, &waypoint, &obstacle,
          wrapS(ego.s + obstacle.start), wrapS(ego.s + obstacle.end));
      }
    }
    if (i > start_index) {
      const double ds = forward_s - previous_s;
      if (!(ds > kEpsilon)) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "candidate no longer follows increasing global race-line order", i, &waypoint);
      }
      const double slope = std::abs(waypoint.d_m - previous_d) / ds;
      if (slope > parameters_.maximum_lateral_slope) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "quintic d-offset exceeds maximum_lateral_slope", i, &waypoint);
      }
      const double curvature_rate =
        std::abs(waypoint.kappa_radpm - previous_curvature) / ds;
      if (curvature_rate > parameters_.maximum_curvature_rate_radpm2) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "shifted race line exceeds maximum_curvature_rate_radpm2", i, &waypoint);
      }
    }
    if (std::abs(waypoint.kappa_radpm) > parameters_.maximum_curvature_radpm) {
      return reject(
        PathValidationFailureKind::kGeometry,
        "shifted race line exceeds maximum_curvature_radpm", i, &waypoint);
    }
    previous_d = waypoint.d_m;
    previous_s = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }
  return true;
}

bool RacelineSplinePlanner::validatePath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::string * error,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m) const
{
  if (failure != nullptr) {
    *failure = PathValidationFailure();
  }
  auto reject = [error, failure](
    PathValidationFailureKind kind,
    const std::string & reason)
    {
      if (error != nullptr) {
        *error = reason;
      }
      if (failure != nullptr) {
        failure->kind = kind;
        failure->reason = reason;
      }
      return false;
    };
  if (!ready()) {
    return reject(
      PathValidationFailureKind::kInput,
      "global race-line reference is not ready");
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    return reject(PathValidationFailureKind::kInput, "ego Frenet state is non-finite");
  }
  if (path.wpnts.empty()) {
    return reject(PathValidationFailureKind::kInput, "path is empty");
  }
  if (maximum_collision_forward_m.has_value() &&
    (!std::isfinite(maximum_collision_forward_m.value()) ||
    maximum_collision_forward_m.value() < 0.0))
  {
    return reject(
      PathValidationFailureKind::kInput,
      "maximum collision-forward distance is invalid");
  }

  std::size_t start_index = 0U;
  double nearest_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double forward = forwardDistance(ego.s, path.wpnts[i].s_m);
    if (forward < nearest_forward) {
      nearest_forward = forward;
      start_index = i;
    }
  }
  if (nearest_forward > 0.5 * track_length_) {
    return reject(
      PathValidationFailureKind::kNoForwardPath,
      "committed path has no remaining waypoint ahead of ego");
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  std::string reason;
  if (!validateCandidate(
      ego, path, visible, reason, start_index, 1U, failure,
      maximum_collision_forward_m))
  {
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  }
  return true;
}

RacelineSplineResult RacelineSplinePlanner::buildSafeStop(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const ExpandedObstacle & blocking) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  result.obstacle_id = blocking.id;
  const double stop_at = std::max(0.0, blocking.start - parameters_.safe_stop_buffer_m);
  const std::size_t first_index = nextReferenceIndex(ego.s);
  result.path.header = reference_.header;
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const double forward_s = forwardDistance(ego.s, reference_.wpnts[index].s_m);
    if (forward_s > stop_at + kEpsilon) {
      break;
    }
    const auto & global = reference_.wpnts[index];
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    waypoint.d_m = ego.d;
    waypoint.x_m = global.x_m - ego.d * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + ego.d * std::cos(global.psi_rad);
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(
        2.0 * parameters_.safe_stop_deceleration_mps2 *
        std::max(0.0, stop_at - forward_s)));
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.reason = "blocking obstacle is inside the safe-stop buffer";
    result.path.wpnts.clear();
    return result;
  }
  result.path.wpnts.back().vx_mps = 0.0;
  updateGeometryAndAcceleration(result.path);

  std::string validation_reason;
  if (!validateCandidate(ego, result.path, visible, validation_reason, 0U, 2U)) {
    result.path.wpnts.clear();
    result.reason = "safe-stop path rejected: " + validation_reason;
    return result;
  }
  result.kind = SplinePlanKind::kSafeStop;
  result.merge_s = result.path.wpnts.back().s_m;
  result.reason = "both spline sides rejected; braking before the static obstacle";
  return result;
}

RacelineSplineResult RacelineSplinePlanner::buildPreparationStop(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  RacelineSplineResult result;
  if (!ready()) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "global race-line reference is not ready";
    return result;
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "ego Frenet state is non-finite";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    result.kind = SplinePlanKind::kNoObstacle;
    result.reason = "no static obstacle blocks the global race line";
    return result;
  }

  result = buildSafeStop(ego, visible, cluster.front());
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    result.kind = SplinePlanKind::kPreparation;
    result.reason =
      "waiting for initial obstacle-cluster stabilization; braking before commitment";
  }
  return result;
}

RacelineSplineResult RacelineSplinePlanner::plan(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::optional<bool> & preferred_left,
  bool allow_side_switch) const
{
  RacelineSplineResult result;
  if (!ready()) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "global race-line reference is not ready";
    return result;
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
    result.kind = SplinePlanKind::kNoSafePath;
    result.reason = "ego Frenet state is non-finite";
    return result;
  }

  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    result.kind = SplinePlanKind::kNoObstacle;
    result.reason = "no static obstacle blocks the global race line";
    return result;
  }

  const bool outside_is_left = outsideIsLeft(ego, cluster);
  Candidate left;
  Candidate right;
  bool left_evaluated = false;
  bool right_evaluated = false;
  Candidate selected;

  auto run_selection = [&](
    const std::vector<ExpandedObstacle> & pass_visible,
    const std::vector<ExpandedObstacle> & pass_cluster) {
      auto evaluate_side = [&](bool go_left) {
          Candidate last;
          last.go_left = go_left;
          double cluster_start = 0.0;
          double cluster_end = 0.0;
          double target_d = 0.0;
          if (!computeSideTarget(
              pass_cluster, go_left, cluster_start, cluster_end, target_d, last.reason))
          {
            return last;
          }
          double headroom = -std::numeric_limits<double>::infinity();
          if (!targetFitsTrackBounds(
              ego, cluster_start, cluster_end, go_left, target_d, last.reason, &headroom))
          {
            return last;
          }
          // Entry and exit have different priorities. Use the longest feasible entry so a distant
          // obstacle is avoided early, but pair it with the shortest feasible exit so the old
          // maneuver releases promptly. Lengthen the exit only when validation requires it.
          for (auto entry_scale = parameters_.transition_distance_scales.rbegin();
            entry_scale != parameters_.transition_distance_scales.rend(); ++entry_scale)
          {
            for (const double exit_scale : parameters_.transition_distance_scales) {
              last = buildCandidate(
                ego, pass_visible, go_left, *entry_scale, exit_scale, outside_is_left,
                cluster_start, cluster_end, target_d);
              if (last.valid) {
                last.headroom = headroom;
                return last;
              }
            }
          }
          return last;
        };

      left = Candidate();
      right = Candidate();
      left_evaluated = false;
      right_evaluated = false;
      selected = Candidate();
      if (preferred_left.has_value()) {
        if (preferred_left.value()) {
          left = evaluate_side(true);
          left_evaluated = true;
          if (left.valid) {
            selected = std::move(left);
          } else if (allow_side_switch) {
            right = evaluate_side(false);
            right_evaluated = true;
            if (right.valid) {
              selected = std::move(right);
            }
          }
        } else {
          right = evaluate_side(false);
          right_evaluated = true;
          if (right.valid) {
            selected = std::move(right);
          } else if (allow_side_switch) {
            left = evaluate_side(true);
            left_evaluated = true;
            if (left.valid) {
              selected = std::move(left);
            }
          }
        }
      } else {
        left = evaluate_side(true);
        right = evaluate_side(false);
        left_evaluated = true;
        right_evaluated = true;
        if (left.valid && right.valid) {
          // A raceline-centred obstacle ties the score within perception jitter. Break ties
          // with the reference-width headroom, which does not jitter frame to frame, so the
          // chosen side cannot flap between replans.
          if (std::abs(left.score - right.score) <= parameters_.side_tie_epsilon_m &&
            left.headroom != right.headroom)
          {
            selected = left.headroom > right.headroom ? std::move(left) : std::move(right);
          } else {
            selected = left.score <= right.score ? std::move(left) : std::move(right);
          }
        } else if (left.valid) {
          selected = std::move(left);
        } else if (right.valid) {
          selected = std::move(right);
        }
      }
    };

  run_selection(visible, cluster);

  if (!selected.valid) {
    auto safe_stop = buildSafeStop(ego, visible, cluster.front());
    safe_stop.obstacle_ids.reserve(cluster.size());
    for (const auto & obstacle : cluster) {
      safe_stop.obstacle_ids.push_back(obstacle.id);
    }
    const std::string left_reason = left_evaluated ?
      left.reason : "not evaluated: side locked by active commitment";
    const std::string right_reason = right_evaluated ?
      right.reason : "not evaluated: side locked by active commitment";
    safe_stop.reason =
      left_evaluated && right_evaluated ?
      "both spline sides rejected; braking before the static obstacle" :
      "committed side rejected; alternate side locked after lateral engagement; braking before "
      "the static obstacle";
    safe_stop.reason += "; left: " + left_reason + "; right: " + right_reason;
    return safe_stop;
  }
  result.kind = SplinePlanKind::kAvoidance;
  result.path = std::move(selected.path);
  result.go_left = selected.go_left;
  result.target_d = selected.target_d;
  result.merge_s = selected.merge_s;
  result.obstacle_id = cluster.front().id;
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  result.control_points = std::move(selected.control_points);
  result.reason = "global race-line waypoints shifted by a local quintic d-offset";
  return result;
}

void RacelineSplinePlanner::toCartesian(
  double s, double d, double & x, double & y, double & yaw) const
{
  if (!ready()) {
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    return;
  }
  const double wrapped = wrapS(s);
  std::size_t first = 0U;
  for (std::size_t i = 1; i < reference_.wpnts.size(); ++i) {
    if (reference_.wpnts[i].s_m > wrapped) {
      break;
    }
    first = i;
  }
  const std::size_t second = (first + 1U) % reference_.wpnts.size();
  const double s0 = reference_.wpnts[first].s_m;
  const double s1 = second == 0U ? track_length_ : reference_.wpnts[second].s_m;
  const double fraction = clamp((wrapped - s0) / std::max(kEpsilon, s1 - s0), 0.0, 1.0);
  const auto & a = reference_.wpnts[first];
  const auto & b = reference_.wpnts[second];
  const double base_x = a.x_m + fraction * (b.x_m - a.x_m);
  const double base_y = a.y_m + fraction * (b.y_m - a.y_m);
  yaw = a.psi_rad + fraction * normalizeAngle(b.psi_rad - a.psi_rad);
  x = base_x - d * std::sin(yaw);
  y = base_y + d * std::cos(yaw);
}

}  // namespace local_planning
