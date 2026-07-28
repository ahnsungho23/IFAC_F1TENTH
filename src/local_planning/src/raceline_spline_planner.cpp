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

// Natural cubic interpolation in the unwrapped Frenet-s domain. The evaluated value is clipped by
// the caller to the control-point extrema, matching the upstream spliner's no-opposite-overshoot
// behavior while retaining a continuous cubic fit.
class NaturalCubicSpline
{
public:
  bool build(const std::vector<double> & x, const std::vector<double> & y)
  {
    if (x.size() < 2U || x.size() != y.size()) {
      return false;
    }
    for (std::size_t i = 1; i < x.size(); ++i) {
      if (!std::isfinite(x[i]) || !std::isfinite(y[i]) || x[i] <= x[i - 1]) {
        return false;
      }
    }
    if (!std::isfinite(x.front()) || !std::isfinite(y.front())) {
      return false;
    }

    x_ = x;
    a_ = y;
    const std::size_t n = x.size();
    b_.assign(n - 1U, 0.0);
    c_.assign(n, 0.0);
    d_.assign(n - 1U, 0.0);

    std::vector<double> h(n - 1U, 0.0);
    for (std::size_t i = 0; i + 1U < n; ++i) {
      h[i] = x[i + 1U] - x[i];
    }

    if (n > 2U) {
      std::vector<double> alpha(n, 0.0);
      for (std::size_t i = 1; i + 1U < n; ++i) {
        alpha[i] = 3.0 * (a_[i + 1U] - a_[i]) / h[i] -
          3.0 * (a_[i] - a_[i - 1U]) / h[i - 1U];
      }

      std::vector<double> lower(n, 1.0);
      std::vector<double> mu(n, 0.0);
      std::vector<double> z(n, 0.0);
      for (std::size_t i = 1; i + 1U < n; ++i) {
        lower[i] = 2.0 * (x[i + 1U] - x[i - 1U]) - h[i - 1U] * mu[i - 1U];
        if (std::abs(lower[i]) < kEpsilon) {
          return false;
        }
        mu[i] = h[i] / lower[i];
        z[i] = (alpha[i] - h[i - 1U] * z[i - 1U]) / lower[i];
      }
      for (std::size_t reverse = n - 1U; reverse > 0U; --reverse) {
        const std::size_t j = reverse - 1U;
        c_[j] = z[j] - mu[j] * c_[j + 1U];
        b_[j] = (a_[j + 1U] - a_[j]) / h[j] -
          h[j] * (c_[j + 1U] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1U] - c_[j]) / (3.0 * h[j]);
      }
    } else {
      b_[0] = (a_[1] - a_[0]) / h[0];
    }
    return true;
  }

  double evaluate(double x) const
  {
    if (x_.empty()) {
      return 0.0;
    }
    if (x <= x_.front()) {
      return a_.front();
    }
    if (x >= x_.back()) {
      return a_.back();
    }
    const auto upper = std::upper_bound(x_.begin(), x_.end(), x);
    const std::size_t i = static_cast<std::size_t>(upper - x_.begin() - 1);
    const double dx = x - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }

private:
  std::vector<double> x_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

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
};

struct RacelineSplinePlanner::Candidate
{
  bool valid{false};
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  double score{std::numeric_limits<double>::infinity()};
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

std::size_t RacelineSplinePlanner::nextReferenceIndex(double s) const
{
  std::size_t best = 0U;
  double best_forward = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference_.wpnts.size(); ++i) {
    const double distance = forwardDistance(s, reference_.wpnts[i].s_m);
    if (distance < best_forward) {
      best_forward = distance;
      best = i;
    }
  }
  return best;
}

std::size_t RacelineSplinePlanner::nearestReferenceIndex(double s) const
{
  std::size_t best = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference_.wpnts.size(); ++i) {
    const double forward = forwardDistance(s, reference_.wpnts[i].s_m);
    const double distance = std::min(forward, track_length_ - forward);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::expandVisibleObstacles(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
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
    expanded.d_right = expanded.raw_d_right -
      parameters_.obstacle_clearance_m;
    expanded.d_left = expanded.raw_d_left +
      parameters_.obstacle_clearance_m;
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
  const double envelope = parameters_.vehicle_half_width_m + parameters_.blocking_margin_m;
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

RacelineSplinePlanner::Candidate RacelineSplinePlanner::buildCandidate(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const std::vector<ExpandedObstacle> & cluster,
  bool go_left,
  double transition_scale) const
{
  Candidate candidate;
  candidate.go_left = go_left;
  if (cluster.empty()) {
    candidate.reason = "empty obstacle cluster";
    return candidate;
  }

  double cluster_start = cluster.front().start;
  double cluster_end = cluster.front().end;
  double target_d = go_left ? cluster.front().d_left : cluster.front().d_right;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
    target_d = go_left ? std::max(target_d, obstacle.d_left) :
      std::min(target_d, obstacle.d_right);
  }
  if (go_left) {
    target_d = std::max(target_d, parameters_.minimum_target_offset_m);
  } else {
    target_d = std::min(target_d, -parameters_.minimum_target_offset_m);
  }
  if (std::abs(target_d) > parameters_.maximum_target_offset_m) {
    candidate.reason = "required d-offset exceeds maximum_target_offset_m";
    return candidate;
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
  const bool outside_is_left = curvature_sum < 0.0;
  if (go_left == outside_is_left) {
    transition_scale *= parameters_.outside_line_transition_scale;
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

  const double pre_far = parameters_.pre_apex_distances_m[0] * transition_scale;
  const double pre_middle = parameters_.pre_apex_distances_m[1] * transition_scale;
  const double pre_near = parameters_.pre_apex_distances_m[2] * transition_scale;
  const double post_near = parameters_.post_apex_distances_m[0] * transition_scale;
  const double post_middle = parameters_.post_apex_distances_m[1] * transition_scale;
  const double post_far = parameters_.post_apex_distances_m[2] * transition_scale;

  append_knot(cluster_start - pre_far, 0.0);
  append_knot(cluster_start - pre_middle, 0.0);
  append_knot(cluster_start - pre_near, 0.0);
  append_knot(cluster_start, target_d);
  if (cluster_end > cluster_start + 0.05) {
    append_knot(cluster_end, target_d);
  }
  append_knot(cluster_end + post_near, 0.0);
  append_knot(cluster_end + post_middle, 0.0);
  append_knot(cluster_end + post_far, 0.0);

  if (knot_s.front() > 0.05) {
    knot_s.insert(knot_s.begin(), 0.0);
    knot_d.insert(knot_d.begin(), ego.d);
  } else if (knot_s.front() <= 0.05) {
    knot_s.front() = std::min(knot_s.front(), 0.0);
  }

  NaturalCubicSpline spline;
  if (!spline.build(knot_s, knot_d)) {
    candidate.reason = "cubic spline control points are not strictly ordered";
    return candidate;
  }
  for (std::size_t i = 0; i < knot_s.size(); ++i) {
    candidate.control_points.push_back({knot_s[i], knot_d[i]});
  }

  const double spline_end = cluster_end + post_far;
  const double path_end = spline_end + parameters_.post_merge_lookahead_m;
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
    waypoint.d_m = clamp(spline.evaluate(forward_s), clip_min, clip_max);
    waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
    candidate.path.wpnts.push_back(waypoint);
  }
  if (candidate.path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    candidate.reason = "spline segment has too few global race-line samples";
    return candidate;
  }

  updateGeometryAndSpeed(candidate.path, wrapS(ego.s + spline_end));
  if (!validateCandidate(ego, candidate.path, visible, candidate.reason)) {
    return candidate;
  }

  candidate.valid = true;
  candidate.target_d = target_d;
  candidate.merge_s = candidate.path.wpnts.back().s_m;
  candidate.score = std::abs(target_d) + 0.02 * transition_scale;
  return candidate;
}

void RacelineSplinePlanner::updateGeometryAndSpeed(
  f110_msgs::msg::WpntArray & path,
  double active_until_s) const
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

  const double start_s = waypoints.front().s_m;
  const double active_distance = forwardDistance(start_s, active_until_s);
  for (auto & waypoint : waypoints) {
    const double forward_s = forwardDistance(start_s, waypoint.s_m);
    double speed = std::max(0.0, waypoint.vx_mps);
    if (forward_s <= active_distance || std::abs(waypoint.d_m) > 0.01) {
      speed *= parameters_.avoidance_speed_scale;
    }
    const double curvature = std::abs(waypoint.kappa_radpm);
    if (curvature > kEpsilon) {
      speed = std::min(
        speed, std::sqrt(parameters_.maximum_lateral_accel_mps2 / curvature));
    }
    waypoint.vx_mps = speed;
  }

  for (std::size_t reverse = waypoints.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const double distance = pointDistance(waypoints[i], waypoints[i + 1U]);
    const double allowed = std::sqrt(
      waypoints[i + 1U].vx_mps * waypoints[i + 1U].vx_mps +
      2.0 * parameters_.maximum_longitudinal_decel_mps2 * distance);
    waypoints[i].vx_mps = std::min(waypoints[i].vx_mps, allowed);
  }
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const double distance = pointDistance(waypoints[i - 1U], waypoints[i]);
    const double allowed = std::sqrt(
      waypoints[i - 1U].vx_mps * waypoints[i - 1U].vx_mps +
      2.0 * parameters_.maximum_longitudinal_accel_mps2 * distance);
    waypoints[i].vx_mps = std::min(waypoints[i].vx_mps, allowed);
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
  std::string & reason) const
{
  if (path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    reason = "path does not meet minimum_path_points";
    return false;
  }
  const double center_boundary_clearance =
    parameters_.vehicle_half_width_m + parameters_.boundary_margin_m;
  double previous_d = path.wpnts.front().d_m;
  double previous_s = 0.0;
  double previous_curvature = path.wpnts.front().kappa_radpm;
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
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
      reason = "d-offset leaves the global waypoint track bounds";
      return false;
    }
    for (const auto & obstacle : visible) {
      if (forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.d_right + 1.0e-6 &&
        waypoint.d_m < obstacle.d_left - 1.0e-6)
      {
        reason = "d-offset intersects an inflated static-obstacle box";
        return false;
      }
    }
    if (i > 0U) {
      const double ds = forward_s - previous_s;
      if (!(ds > kEpsilon)) {
        reason = "candidate no longer follows increasing global race-line order";
        return false;
      }
      const double slope = std::abs(waypoint.d_m - previous_d) / ds;
      if (slope > parameters_.maximum_lateral_slope) {
        reason = "cubic d-offset exceeds maximum_lateral_slope";
        return false;
      }
      const double curvature_rate =
        std::abs(waypoint.kappa_radpm - previous_curvature) / ds;
      if (curvature_rate > parameters_.maximum_curvature_rate_radpm2) {
        reason = "shifted race line exceeds maximum_curvature_rate_radpm2";
        return false;
      }
    }
    if (std::abs(waypoint.kappa_radpm) > parameters_.maximum_curvature_radpm) {
      reason = "shifted race line exceeds maximum_curvature_radpm";
      return false;
    }
    previous_d = waypoint.d_m;
    previous_s = forward_s;
    previous_curvature = waypoint.kappa_radpm;
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
    auto waypoint = reference_.wpnts[index];
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    waypoint.d_m = 0.0;
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
  result.path.wpnts.back().ax_mps2 = 0.0;
  for (std::size_t i = 0; i + 1U < result.path.wpnts.size(); ++i) {
    const double distance = pointDistance(result.path.wpnts[i], result.path.wpnts[i + 1U]);
    if (distance > kEpsilon) {
      const double speed = result.path.wpnts[i].vx_mps;
      const double next_speed = result.path.wpnts[i + 1U].vx_mps;
      result.path.wpnts[i].ax_mps2 =
        (next_speed * next_speed - speed * speed) / (2.0 * distance);
    }
  }

  std::string validation_reason;
  if (!validateCandidate(ego, result.path, visible, validation_reason)) {
    result.path.wpnts.clear();
    result.reason = "safe-stop path rejected: " + validation_reason;
    return result;
  }
  result.kind = SplinePlanKind::kSafeStop;
  result.merge_s = result.path.wpnts.back().s_m;
  result.reason = "both spline sides rejected; braking before the static obstacle";
  return result;
}

RacelineSplineResult RacelineSplinePlanner::plan(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::optional<bool> & preferred_left) const
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

  std::vector<Candidate> left_candidates;
  std::vector<Candidate> right_candidates;
  for (const double scale : parameters_.transition_distance_scales) {
    left_candidates.push_back(buildCandidate(ego, visible, cluster, true, scale));
    right_candidates.push_back(buildCandidate(ego, visible, cluster, false, scale));
  }
  auto best_valid = [](const std::vector<Candidate> & candidates) -> const Candidate * {
      const Candidate * best = nullptr;
      for (const auto & candidate : candidates) {
        if (candidate.valid && (best == nullptr || candidate.score < best->score)) {
          best = &candidate;
        }
      }
      return best;
    };
  const Candidate * left = best_valid(left_candidates);
  const Candidate * right = best_valid(right_candidates);
  const Candidate * selected = nullptr;
  if (preferred_left.has_value()) {
    selected = preferred_left.value() ? left : right;
    if (selected == nullptr) {
      selected = preferred_left.value() ? right : left;
    }
  } else if (left != nullptr && right != nullptr) {
    selected = (left->score <= right->score) ? left : right;
  } else {
    selected = left != nullptr ? left : right;
  }

  if (selected == nullptr) {
    auto safe_stop = buildSafeStop(ego, visible, cluster.front());
    const std::string left_reason = left_candidates.empty() ?
      "not evaluated" : left_candidates.back().reason;
    const std::string right_reason = right_candidates.empty() ?
      "not evaluated" : right_candidates.back().reason;
    safe_stop.reason += "; left: " + left_reason + "; right: " + right_reason;
    return safe_stop;
  }
  result.kind = SplinePlanKind::kAvoidance;
  result.path = selected->path;
  result.go_left = selected->go_left;
  result.target_d = selected->target_d;
  result.merge_s = selected->merge_s;
  result.obstacle_id = cluster.front().id;
  result.control_points = selected->control_points;
  result.reason = "global race-line waypoints shifted by a local cubic d-offset";
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


