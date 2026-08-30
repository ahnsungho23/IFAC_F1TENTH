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
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>

#include "local_planning/candidate_rank.hpp"
#include "local_planning/research_instrumentation.hpp"

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

using ResearchClock = std::chrono::steady_clock;

double elapsedResearchUs(const ResearchClock::time_point & start)
{
  return std::chrono::duration<double, std::micro>(ResearchClock::now() - start).count();
}

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

struct AxisInterpolation
{
  std::size_t lower{0U};
  std::size_t upper{0U};
  double ratio{0.0};
};

struct CurveDerivatives
{
  double first{0.0};
  double second{0.0};
  bool valid{false};
};

CurveDerivatives fitLocalCubicDerivatives(
  const std::vector<std::pair<double, double>> & samples)
{
  if (samples.size() < 4U) {
    return {};
  }
  double scale = 0.0;
  for (const auto & sample : samples) {
    scale = std::max(scale, std::abs(sample.first));
  }
  if (!(scale > kEpsilon)) {
    return {};
  }

  // [1,t,t^2,t^3] least-squares normal equation. t를 [-1,1] 근방으로 스케일해
  // waypoint s가 큰 트랙에서도 endpoint one-sided fit의 조건수를 제한한다.
  std::array<std::array<double, 5>, 4> augmented{};
  for (const auto & [station, value] : samples) {
    const double t = station / scale;
    const std::array<double, 4> basis{1.0, t, t * t, t * t * t};
    for (std::size_t row = 0U; row < 4U; ++row) {
      for (std::size_t column = 0U; column < 4U; ++column) {
        augmented[row][column] += basis[row] * basis[column];
      }
      augmented[row][4U] += basis[row] * value;
    }
  }
  for (std::size_t pivot = 0U; pivot < 4U; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < 4U; ++row) {
      if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) {
        best = row;
      }
    }
    if (std::abs(augmented[best][pivot]) <= 1.0e-12) {
      return {};
    }
    if (best != pivot) {
      std::swap(augmented[best], augmented[pivot]);
    }
    const double divisor = augmented[pivot][pivot];
    for (std::size_t column = pivot; column < 5U; ++column) {
      augmented[pivot][column] /= divisor;
    }
    for (std::size_t row = 0U; row < 4U; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = augmented[row][pivot];
      for (std::size_t column = pivot; column < 5U; ++column) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }
  const double first = augmented[1U][4U] / scale;
  const double second = 2.0 * augmented[2U][4U] / (scale * scale);
  return {first, second, std::isfinite(first) && std::isfinite(second)};
}

AxisInterpolation interpolationFor(const std::vector<double> & bins, double value)
{
  if (bins.size() <= 1U || value <= bins.front()) {
    return {0U, 0U, 0.0};
  }
  if (value >= bins.back()) {
    const std::size_t last = bins.size() - 1U;
    return {last, last, 0.0};
  }
  const auto upper = std::upper_bound(bins.begin(), bins.end(), value);
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(bins.begin(), upper));
  const std::size_t lower_index = upper_index - 1U;
  const double span = bins[upper_index] - bins[lower_index];
  return {lower_index, upper_index, (value - bins[lower_index]) / span};
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
  double target_clearance{0.0};
  // Same gate evaluated at avoidance_minimum_speed_mps. The preferred gate above reserves the
  // race-line speed's tracking error, which keeps a wide gap fast; a gap that only opens once the
  // pass slows down would otherwise be discarded here, before the speed cap ever runs.
  double relaxed_d_right{0.0};
  double relaxed_d_left{0.0};
  double relaxed_target_clearance{0.0};
};

struct RacelineSplinePlanner::FootprintTrackBoundSample
{
  double centerline_clearance_m{std::numeric_limits<double>::infinity()};
  double footprint_clearance_m{std::numeric_limits<double>::infinity()};
  bool invalid{false};
  std::string minimum_side;
  std::size_t waypoint_index{std::numeric_limits<std::size_t>::max()};
  double waypoint_s_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_x_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_y_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_relative_to_reference_rad{std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
};

struct RacelineSplinePlanner::Candidate
{
  bool valid{false};
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  double entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  bool footprint_invalid{false};
  std::string footprint_violation_side;
  std::size_t footprint_violation_waypoint_index{
    std::numeric_limits<std::size_t>::max()};
  double footprint_violation_s_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_x_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_y_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double footprint_heading_relative_to_reference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
  double wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double minimum_curvature_margin_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double ego_braking_distance_deficit_m{0.0};
  // 클러스터를 지난 뒤에도 오프셋이 남아 다음(비클러스터) 장애물의 물리 엔벨로프에 닿는
  // 후보. 유효성은 그대로 두고 순위에서만 뒤로 민다 — P3ShadowCandidateTrace의 같은 이름
  // 필드에서 그대로 옮겨온다.
  bool exit_reaches_next_obstacle{false};
  std::size_t audit_index{std::numeric_limits<std::size_t>::max()};
  f110_msgs::msg::WpntArray path;
  std::vector<SplineControlPoint> control_points;
  std::string reason;
};

RacelineSplinePlanner::RacelineSplinePlanner(RacelineSplineParameters parameters)
: parameters_(std::move(parameters))
{
}

bool RacelineSplineParameters::hasTrackingErrorLut() const
{
  return !tracking_error_lut_speed_bins_mps.empty() &&
         !tracking_error_lut_curvature_bins_radpm.empty() &&
         !tracking_error_lut_values_m.empty();
}

bool RacelineSplineParameters::trackingErrorLutValid() const
{
  const bool all_empty = tracking_error_lut_speed_bins_mps.empty() &&
    tracking_error_lut_curvature_bins_radpm.empty() &&
    tracking_error_lut_values_m.empty();
  if (all_empty) {
    return true;
  }
  if (!hasTrackingErrorLut() ||
    tracking_error_lut_values_m.size() !=
    tracking_error_lut_speed_bins_mps.size() *
    tracking_error_lut_curvature_bins_radpm.size())
  {
    return false;
  }
  const auto valid_axis = [](const std::vector<double> & bins) {
      return std::all_of(
        bins.begin(), bins.end(), [](double value) {
          return std::isfinite(value) && value >= 0.0;
        }) &&
             std::adjacent_find(
        bins.begin(), bins.end(), std::greater_equal<double>()) == bins.end();
    };
  return valid_axis(tracking_error_lut_speed_bins_mps) &&
         valid_axis(tracking_error_lut_curvature_bins_radpm) &&
         std::all_of(
    tracking_error_lut_values_m.begin(), tracking_error_lut_values_m.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;});
}

bool RacelineSplineParameters::avoidanceVelocityLimitValid() const
{
  const bool all_empty = avoidance_velocity_limit_speed_bins_mps.empty() &&
    avoidance_velocity_limit_lateral_accel_mps2.empty();
  if (all_empty) {
    return true;
  }
  if (avoidance_velocity_limit_speed_bins_mps.size() < 2U ||
    avoidance_velocity_limit_speed_bins_mps.size() !=
    avoidance_velocity_limit_lateral_accel_mps2.size() ||
    std::abs(avoidance_velocity_limit_speed_bins_mps.front()) > kEpsilon)
  {
    return false;
  }
  const bool valid_speed_axis = std::all_of(
    avoidance_velocity_limit_speed_bins_mps.begin(),
    avoidance_velocity_limit_speed_bins_mps.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;}) &&
    std::adjacent_find(
    avoidance_velocity_limit_speed_bins_mps.begin(),
    avoidance_velocity_limit_speed_bins_mps.end(),
    std::greater_equal<double>()) == avoidance_velocity_limit_speed_bins_mps.end();
  const bool valid_lateral_limits = std::all_of(
    avoidance_velocity_limit_lateral_accel_mps2.begin(),
    avoidance_velocity_limit_lateral_accel_mps2.end(),
    [](double value) {return std::isfinite(value) && value > 0.0;}) &&
    std::adjacent_find(
    avoidance_velocity_limit_lateral_accel_mps2.begin(),
    avoidance_velocity_limit_lateral_accel_mps2.end(),
    std::less<double>()) == avoidance_velocity_limit_lateral_accel_mps2.end();
  return valid_speed_axis && valid_lateral_limits;
}

bool RacelineSplineParameters::longitudinalVelocityLimitValid() const
{
  if (avoidance_velocity_limit_accel_mps2.empty() &&
    avoidance_velocity_limit_decel_mps2.empty())
  {
    return true;   // 표 없음 = 종전 스칼라 거동. 옛 파라미터 파일도 그대로 뜬다.
  }
  if (avoidance_velocity_limit_speed_bins_mps.size() < 2U) {
    return false;
  }
  const auto column_valid = [this](const std::vector<double> & column) {
      return column.size() == avoidance_velocity_limit_speed_bins_mps.size() &&
             std::all_of(
        column.begin(), column.end(),
        [](double value) {return std::isfinite(value) && value > 0.0;});
    };
  // 두 열은 함께 있어야 한다 — 한쪽만 있으면 어느 패스는 표를, 어느 패스는 스칼라를 쓰게 되어
  // 플래너가 다시 차량 모델 두 개를 갖는다(2026-08-17 컨트롤러가 겪은 그 실패 형태다).
  return column_valid(avoidance_velocity_limit_accel_mps2) &&
         column_valid(avoidance_velocity_limit_decel_mps2);
}

double RacelineSplineParameters::accelLimitAt(double speed_mps) const
{
  if (avoidance_velocity_limit_accel_mps2.empty() || !longitudinalVelocityLimitValid()) {
    return 0.0;
  }
  const auto interpolation =
    interpolationFor(avoidance_velocity_limit_speed_bins_mps, std::max(0.0, speed_mps));
  const double lower = avoidance_velocity_limit_accel_mps2[interpolation.lower];
  const double upper = avoidance_velocity_limit_accel_mps2[interpolation.upper];
  return lower + interpolation.ratio * (upper - lower);
}

double RacelineSplineParameters::decelLimitAt(double speed_mps) const
{
  if (avoidance_velocity_limit_decel_mps2.empty() || !longitudinalVelocityLimitValid()) {
    return 0.0;
  }
  const auto interpolation =
    interpolationFor(avoidance_velocity_limit_speed_bins_mps, std::max(0.0, speed_mps));
  const double lower = avoidance_velocity_limit_decel_mps2[interpolation.lower];
  const double upper = avoidance_velocity_limit_decel_mps2[interpolation.upper];
  return lower + interpolation.ratio * (upper - lower);
}

bool RacelineSplineParameters::controlSteeringGeometryValid() const
{
  return std::isfinite(control_wheelbase_m) && control_wheelbase_m > kEpsilon &&
         std::isfinite(control_max_steering_left_rad) &&
         control_max_steering_left_rad > kEpsilon &&
         std::isfinite(control_max_steering_right_rad) &&
         control_max_steering_right_rad > kEpsilon;
}

double RacelineSplineParameters::maximumCurvatureFor(double signed_curvature_radpm) const
{
  const double legacy_limit = std::max(0.0, maximum_curvature_radpm);
  if (!controlSteeringGeometryValid()) {
    return legacy_limit;
  }
  // Frenet 규약: kappa>0 = 좌회전, kappa<0 = 우회전. 실차 우조향 도달각이
  // 더 작으므로 abs(kappa) 단일 상한으로 합치면 우코너의 물리 한계를 과대평가한다.
  const double steering_limit = signed_curvature_radpm >= 0.0 ?
    control_max_steering_left_rad : control_max_steering_right_rad;
  const double directional_limit = std::tan(steering_limit) / control_wheelbase_m;
  return std::min(legacy_limit, directional_limit);
}

double RacelineSplineParameters::modeledControlSteeringRad(
  double signed_curvature_radpm, double speed_mps) const
{
  if (!controlSteeringGeometryValid()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // control_map_node의 현재 bicycle 계약과 같다:
  // delta = L*kappa + K_us*a_lat = kappa*(L + K_us*v^2).
  const double gradient = signed_curvature_radpm >= 0.0 ?
    std::max(0.0, control_understeer_gradient_left_rad_per_mps2) :
    std::max(0.0, control_understeer_gradient_right_rad_per_mps2);
  const double speed = std::max(0.0, speed_mps);
  const double requested = signed_curvature_radpm *
    (control_wheelbase_m + gradient * speed * speed);
  // 제어기의 rate limiter 뒤에도 최종 좌우 clamp가 있다. 진단은 액추에이터가
  // 실제로 받을 목표각 사이의 변화율을 묻는다.
  return std::clamp(
    requested, -control_max_steering_right_rad, control_max_steering_left_rad);
}

double RacelineSplineParameters::limitedAvoidanceSpeed(
  double requested_speed_mps, double curvature_radpm) const
{
  const double requested_speed = std::abs(requested_speed_mps);
  const double curvature = std::abs(curvature_radpm);
  if (!(requested_speed > 0.0) || curvature <= kEpsilon ||
    avoidance_velocity_limit_speed_bins_mps.empty() || !avoidanceVelocityLimitValid())
  {
    return requested_speed;
  }

  const auto lateral_limit_at = [&](double speed_mps) {
      const auto interpolation = interpolationFor(
        avoidance_velocity_limit_speed_bins_mps, speed_mps);
      const double lower =
        avoidance_velocity_limit_lateral_accel_mps2[interpolation.lower];
      const double upper =
        avoidance_velocity_limit_lateral_accel_mps2[interpolation.upper];
      return lower + interpolation.ratio * (upper - lower);
    };
  const auto feasible = [&](double speed_mps) {
      return speed_mps * speed_mps * curvature <= lateral_limit_at(speed_mps);
    };
  if (feasible(requested_speed)) {
    return requested_speed;
  }

  double lower = 0.0;
  double upper = requested_speed;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (feasible(middle)) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return lower;
}

double RacelineSplineParameters::gapLimitedAvoidanceSpeed(
  double requested_speed_mps, double curvature_radpm, double admissible_reserve_m) const
{
  const double requested_speed = std::max(0.0, requested_speed_mps);
  const double floor_speed = std::max(0.0, avoidance_minimum_speed_mps);
  if (!std::isfinite(admissible_reserve_m) || requested_speed <= floor_speed) {
    return requested_speed;
  }
  const auto fits = [&](double speed_mps) {
      return trackingErrorReserve(speed_mps, curvature_radpm) <= admissible_reserve_m;
    };
  // Deciding whether to slow down at all carries the same tolerance the hard validator applies to
  // this boundary, so a candidate already sitting exactly on its clearance limit is not slowed by
  // rounding alone. The search below stays strict: a reduced speed whose reserve overshot by that
  // tolerance would spend room the path does not have, and the validator -- comparing with the
  // very same tolerance -- would reject the candidate the cap was meant to enable.
  if (trackingErrorReserve(requested_speed, curvature_radpm) <=
    admissible_reserve_m + kEpsilon)
  {
    return requested_speed;
  }
  // Even crawling does not fit this gap. Hold the floor and let the hard validator reject the
  // candidate: silently creeping through a gap the reserve says we cannot hold is not a decision
  // the speed policy is allowed to make.
  if (!fits(floor_speed)) {
    return floor_speed;
  }
  double lower = floor_speed;
  double upper = requested_speed;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (fits(middle)) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return lower;
}

double RacelineSplineParameters::trackingErrorReserve(
  double speed_mps, double curvature_radpm) const
{
  // localization_reserve_m is a constant floor added here (the single choke point) so the
  // gap-limited speed inversion, envelope expansion, and hard validation all account for the
  // same localization uncertainty. Constant offset keeps the speed-monotonicity the inversion
  // in gapLimitedAvoidanceSpeed relies on.
  const double localization = std::max(0.0, localization_reserve_m);
  // 게이트가 꺼져 있으면 표도 폴백 상수도 읽지 않는다 (선언부 주석 참고). 여기가 예약의
  // 유일한 관문이므로, 이 한 줄이 후보 검증·갭 역산·봉투 확장·entry 예산 네 곳을 동시에
  // 끈다 — 표를 0 으로 채우는 것과 값은 같지만, 표가 되살아나도 되살아나지 않는다.
  if (!obstacle_reserve_from_lut) {
    return localization;
  }
  if (!hasTrackingErrorLut() || !trackingErrorLutValid()) {
    return tracking_error_reserve_m + localization;
  }
  const auto speed = interpolationFor(
    tracking_error_lut_speed_bins_mps, std::abs(speed_mps));
  const auto curvature = interpolationFor(
    tracking_error_lut_curvature_bins_radpm, std::abs(curvature_radpm));
  const std::size_t curvature_count = tracking_error_lut_curvature_bins_radpm.size();
  const auto value_at = [&](std::size_t speed_index, std::size_t curvature_index) {
      return tracking_error_lut_values_m[speed_index * curvature_count + curvature_index];
    };
  const double lower =
    value_at(speed.lower, curvature.lower) + curvature.ratio *
    (value_at(speed.lower, curvature.upper) - value_at(speed.lower, curvature.lower));
  const double upper =
    value_at(speed.upper, curvature.lower) + curvature.ratio *
    (value_at(speed.upper, curvature.upper) - value_at(speed.upper, curvature.lower));
  return localization + lower + speed.ratio * (upper - lower);
}

double RacelineSplineParameters::avoidanceTrackingErrorReserve(
  double speed_mps, double curvature_radpm) const
{
  return trackingErrorReserve(
    limitedAvoidanceSpeed(speed_mps, curvature_radpm), curvature_radpm);
}

double RacelineSplineParameters::cappedCombinedExitScale(double combined_exit_scale) const
{
  if (!(maximum_exit_length_m > 0.0) || post_apex_distances_m.empty()) {
    return combined_exit_scale;
  }
  const double post_far = post_apex_distances_m.back();
  if (!(post_far > kEpsilon)) {
    return combined_exit_scale;
  }
  return std::min(combined_exit_scale, maximum_exit_length_m / post_far);
}

double RacelineSplineParameters::confirmedSpeedHoldEndForwardM(
  double padded_cluster_end_forward_m) const
{
  // padded_cluster_end에는 obstacle_longitudinal_padding_m이 이미 한 번 들어 있다.
  // 부족분만 더하면 detector 뒤 총 hold가 max(padding, post_hold)가 된다.
  const double extra = std::max(
    0.0, confirmed_speed_post_hold_distance_m -
    std::max(0.0, obstacle_longitudinal_padding_m));
  return padded_cluster_end_forward_m + extra;
}

double RacelineSplineParameters::obstacleBaseClearance() const
{
  return vehicle_half_width_m + safety_margin_m;
}

double RacelineSplineParameters::obstacleSafetyClearance(
  double speed_mps, double curvature_radpm, double reserve_scale) const
{
  const double scale = std::clamp(reserve_scale, 0.0, 1.0);
  return obstacleBaseClearance() +
         scale * avoidanceTrackingErrorReserve(speed_mps, curvature_radpm);
}

double RacelineSplineParameters::trackBoundaryReserve(
  double speed_mps, double curvature_radpm) const
{
  (void)speed_mps;
  (void)curvature_radpm;
  return wall_safety_margin_m;
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

void RacelineSplinePlanner::setActiveResearchCycle(PlanningResearchCycle * cycle) const
{
  active_research_cycle_ = cycle;
}

PlanningResearchCycle * RacelineSplinePlanner::activeResearchCycle() const
{
  return active_research_cycle_;
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

double RacelineSplinePlanner::maximumReferenceTrackingErrorReserve(
  const EgoFrenetState & ego, double start, double end, double speed_cap_mps) const
{
  const double check_start = std::max(0.0, start);
  const double check_end = std::max(check_start, end);
  double maximum = 0.0;
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > check_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    maximum = std::max(
      maximum,
      parameters_.avoidanceTrackingErrorReserve(
        std::min(reference.vx_mps, speed_cap_mps), reference.kappa_radpm));
  }
  if (!checked_reference) {
    const auto & reference = reference_.wpnts[
      nearestReferenceIndex(wrapS(ego.s + 0.5 * (check_start + check_end)))];
    maximum = parameters_.avoidanceTrackingErrorReserve(
      std::min(reference.vx_mps, speed_cap_mps), reference.kappa_radpm);
  }
  return maximum;
}

std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::expandVisibleObstacles(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  // Obstacle input may already be an uncertainty Guard, but vehicle size, physical margin, and
  // the reference-span maximum tracking reserve are applied exactly once to the target gate.
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
    expanded.target_clearance = parameters_.obstacleBaseClearance() +
      maximumReferenceTrackingErrorReserve(ego, expanded.start, expanded.end);
    expanded.d_right = expanded.raw_d_right - expanded.target_clearance;
    expanded.d_left = expanded.raw_d_left + expanded.target_clearance;
    expanded.relaxed_target_clearance = parameters_.obstacleBaseClearance() +
      maximumReferenceTrackingErrorReserve(
      ego, expanded.start, expanded.end, parameters_.avoidance_minimum_speed_mps);
    expanded.relaxed_d_right = expanded.raw_d_right - expanded.relaxed_target_clearance;
    expanded.relaxed_d_left = expanded.raw_d_left + expanded.relaxed_target_clearance;
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
  return obstacle.d_right <= 0.0 && obstacle.d_left >= 0.0;
}

bool RacelineSplinePlanner::physicallyBlocksRaceline(const ExpandedObstacle & obstacle) const
{
  // Physical predicate only: raw measured envelope plus the vehicle's physical footprint
  // clearance (half width + safety margin). The tracking-error reserve deliberately does not
  // participate — it decides whether avoidance/slow-down is wanted, not whether the line is
  // physically drivable.
  const double physical_clearance = parameters_.obstacleBaseClearance();
  return obstacle.raw_d_right - physical_clearance <= 0.0 &&
         obstacle.raw_d_left + physical_clearance >= 0.0;
}

bool RacelineSplinePlanner::clusterPhysicallyBlocksRaceline(
  const std::vector<ExpandedObstacle> & cluster) const
{
  return std::any_of(
    cluster.begin(), cluster.end(),
    [this](const ExpandedObstacle & obstacle) {return physicallyBlocksRaceline(obstacle);});
}

bool RacelineSplinePlanner::obstaclesPhysicallyBlockRaceline(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d) ||
    !std::isfinite(ego.speed))
  {
    return false;
  }
  return clusterPhysicallyBlocksRaceline(expandVisibleObstacles(ego, obstacles));
}

RacelineSplineResult RacelineSplinePlanner::buildMarginSlowPass(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  if (cluster.empty()) {
    result.reason = "margin slow pass requested without a blocking cluster";
    return result;
  }
  double cluster_end = cluster.front().end;
  double raw_d_sum = 0.0;
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    cluster_end = std::max(cluster_end, obstacle.end);
    raw_d_sum += 0.5 * (obstacle.raw_d_left + obstacle.raw_d_right);
    result.obstacle_ids.push_back(obstacle.id);
  }
  const double cap = parameters_.margin_pass_speed_cap_mps;
  if (!(cap > 0.0)) {
    result.reason = "margin slow pass disabled (margin_pass_speed_cap_mps <= 0)";
    return result;
  }
  // 접근 실현성 램프(2026-08-14 실차): 자차가 cap보다 빠를 때 flat cap을 자차 위치부터
  // 그대로 명령하면 계단 감속이 된다 — 서비스 브레이크가 포화하고 마찰 한계를 넘겨
  // 슬립(조향 상실)으로 이어졌다(4.4 m/s 접근에 flat 2.0 → 벽 충돌). 군집 시작 전
  // 구간은 실측 자차 속도에서 approach_feasibility_decel_mps2로 내려가는 프로파일까지
  // 허용하되, 그 감속으로 군집 시작까지 cap에 못 닿으면 닿는 만큼만 더 가파르게 잡는다
  // (여유가 전혀 없으면 기존 flat cap으로 수렴). 군집 스팬부터는 항상 cap 그대로다.
  double cluster_start = cluster.front().start;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
  }
  cluster_start = std::max(0.0, cluster_start);
  const double approach_decel = parameters_.approach_feasibility_decel_mps2;
  const double ego_speed =
    std::isfinite(ego.speed) ? std::max(0.0, std::abs(ego.speed)) : 0.0;
  const bool ramp_active =
    approach_decel > 0.0 && ego_speed > cap && cluster_start > kEpsilon;
  double ramp_decel = approach_decel;
  if (ramp_active) {
    ramp_decel = std::max(
      approach_decel,
      (ego_speed * ego_speed - cap * cap) / (2.0 * cluster_start));
  }
  // End far enough past the cluster that the merge-completion check (tail reach) fires with the
  // full vehicle clear of the obstacle span.
  const double end_at = std::min(
    cluster_end + parameters_.safe_stop_buffer_m + parameters_.vehicle_length_m,
    track_length_ - kEpsilon);
  result.path.header = reference_.header;
  const std::size_t first_index = nextReferenceIndex(ego.s);
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const double forward_s = forwardDistance(ego.s, reference_.wpnts[index].s_m);
    if (forward_s > end_at + kEpsilon) {
      break;
    }
    auto waypoint = reference_.wpnts[index];
    waypoint.id = static_cast<int32_t>(result.path.wpnts.size());
    waypoint.d_m = 0.0;
    double allowed = cap;
    if (ramp_active && forward_s < cluster_start) {
      allowed = std::sqrt(
        std::max(cap * cap, ego_speed * ego_speed - 2.0 * ramp_decel * forward_s));
    }
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), allowed);
    result.path.wpnts.push_back(waypoint);
  }
  if (result.path.wpnts.size() < 2U) {
    result.path.wpnts.clear();
    result.reason = "margin slow pass span is already behind ego";
    return result;
  }
  updateGeometryAndAcceleration(result.path);
  result.kind = SplinePlanKind::kAvoidance;
  result.margin_pass = true;
  // The pass stays on the line; the side flag only records which side of the obstacle that is.
  result.go_left = raw_d_sum < 0.0;
  result.target_d = 0.0;
  result.merge_s = result.path.wpnts.back().s_m;
  result.obstacle_id = result.obstacle_ids.front();
  result.reason =
    "margin-only blocking cluster (raw envelope + physical clearance stays clear of the race "
    "line); passing on the line at the margin_pass speed cap";
  return result;
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

bool RacelineSplinePlanner::computeSideTargetRange(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & cluster,
  bool go_left,
  double & cluster_start,
  double & cluster_end,
  double & minimum_clearance_target_d,
  double & maximum_track_target_d,
  std::string & reason,
  bool relaxed_clearance_gate) const
{
  if (cluster.empty()) {
    reason = "empty obstacle cluster";
    return false;
  }

  cluster_start = cluster.front().start;
  cluster_end = cluster.front().end;
  minimum_clearance_target_d = go_left ? cluster.front().d_left : cluster.front().d_right;
  double relaxed_clearance_target_d =
    go_left ? cluster.front().relaxed_d_left : cluster.front().relaxed_d_right;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
    minimum_clearance_target_d = go_left ?
      std::max(minimum_clearance_target_d, obstacle.d_left) :
      std::min(minimum_clearance_target_d, obstacle.d_right);
    relaxed_clearance_target_d = go_left ?
      std::max(relaxed_clearance_target_d, obstacle.relaxed_d_left) :
      std::min(relaxed_clearance_target_d, obstacle.relaxed_d_right);
  }
  if (relaxed_clearance_gate) {
    // 2차 시도 전용(evaluateP3Shadow 참고): strict 게이트의 전 후보가 exact validator에서
    // 기각된 뒤에만 들어온다. 감속 통과가 필요로 하는 게이트로 최소 target을 낮춰 "느리지만
    // 가능한" 통로를 고려 대상에 넣는다. 수용 기준(정확 검증·속도 상한 역산)은 그대로이므로
    // 고려 범위만 넓어질 뿐 수용이 넓어지지는 않는다.
    minimum_clearance_target_d = relaxed_clearance_target_d;
  }
  if (go_left) {
    minimum_clearance_target_d = std::max(
      minimum_clearance_target_d,
      parameters_.minimum_target_offset_m);
  } else {
    minimum_clearance_target_d = std::min(
      minimum_clearance_target_d,
      -parameters_.minimum_target_offset_m);
  }
  if (std::abs(minimum_clearance_target_d) > parameters_.maximum_target_offset_m) {
    reason = "required d-offset exceeds maximum_target_offset_m";
    return false;
  }

  // Find the farthest target that remains inside the centre-of-vehicle track boundary for the
  // complete obstacle span. The closest endpoint is determined only by obstacle clearance; the
  // other endpoint is determined only by track geometry and maximum_target_offset_m.
  maximum_track_target_d = go_left ? parameters_.maximum_target_offset_m :
    -parameters_.maximum_target_offset_m;
  const auto update_track_limit = [&](const f110_msgs::msg::Wpnt & reference) {
      // The hard validator checks the rotated vehicle rectangle, not the path centre, so the
      // centre may only approach the wall to within its own half width. Leaving that out here
      // offers target offsets the footprint check can never accept, and a gap that is only
      // reachable closer to the obstacle looks unreachable instead of merely slower.
      const double reserve = parameters_.trackBoundaryReserve(
        reference.vx_mps, reference.kappa_radpm) + parameters_.vehicle_half_width_m;
      const double left_width = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right_width = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      if (go_left) {
        maximum_track_target_d = std::min(maximum_track_target_d, left_width - reserve);
      } else {
        maximum_track_target_d = std::max(maximum_track_target_d, -right_width + reserve);
      }
    };

  const double check_start = std::max(0.0, cluster_start);
  bool checked_reference = false;
  const std::size_t first_index = nextReferenceIndex(wrapS(ego.s + check_start));
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const auto & reference = reference_.wpnts[(first_index + k) % reference_.wpnts.size()];
    const double forward_s = forwardDistance(ego.s, reference.s_m);
    if (forward_s + kEpsilon < check_start) {
      continue;
    }
    if (forward_s > cluster_end + kEpsilon) {
      break;
    }
    checked_reference = true;
    update_track_limit(reference);
  }
  if (!checked_reference) {
    const double midpoint = 0.5 * (check_start + std::max(check_start, cluster_end));
    update_track_limit(
      reference_.wpnts[nearestReferenceIndex(wrapS(ego.s + midpoint))]);
  }

  const auto fits_track = [&](double target_d) {
      return go_left ?
             target_d <= maximum_track_target_d + kEpsilon :
             target_d >= maximum_track_target_d - kEpsilon;
    };
  if (!fits_track(minimum_clearance_target_d)) {
    // The race-line-speed gate does not fit between the obstacle and the wall. Retry against the
    // gate the pass would need at avoidance_minimum_speed_mps: the speed cap lowers this waypoint
    // to match, and the hard validator still checks the candidate at whatever speed it ends up
    // with, so this widens what is considered without widening what is accepted.
    const double relaxed_target_d = go_left ?
      std::max(relaxed_clearance_target_d, parameters_.minimum_target_offset_m) :
      std::min(relaxed_clearance_target_d, -parameters_.minimum_target_offset_m);
    if (std::abs(relaxed_target_d) <= parameters_.maximum_target_offset_m &&
      fits_track(relaxed_target_d))
    {
      minimum_clearance_target_d = relaxed_target_d;
      return true;
    }
    reason = go_left ?
      "left target d exceeds track bound in obstacle span before spline construction" :
      "right target d exceeds track bound in obstacle span before spline construction";
    return false;
  }
  return true;
}

double RacelineSplinePlanner::maneuverScopeEnd(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<int> & cluster_ids,
  double cluster_end_forward_m) const
{
  const double nominal = cluster_end_forward_m + parameters_.post_merge_lookahead_m;
  if (!ready() || !std::isfinite(cluster_end_forward_m)) {
    return nominal;
  }
  // 이 기동의 클러스터보다 뒤에 있는 가장 가까운 장애물의 확장 앞면.
  double next_start = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : expandVisibleObstacles(ego, obstacles)) {
    const bool in_cluster = std::find(
      cluster_ids.begin(), cluster_ids.end(), obstacle.id) != cluster_ids.end();
    if (in_cluster || obstacle.start <= cluster_end_forward_m + kEpsilon) {
      continue;
    }
    next_start = std::min(next_start, obstacle.start);
  }
  if (!std::isfinite(next_start)) {
    return nominal;
  }
  // 클러스터 끝보다 앞으로 자르지는 않는다 — 자기 장애물은 반드시 검사해야 한다.
  return std::max(cluster_end_forward_m, std::min(nominal, next_start));
}

P3ShadowPlanningContext RacelineSplinePlanner::buildP3ShadowPlanningContext(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  bool relaxed_clearance_gate) const
{
  P3ShadowPlanningContext context;
  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto cluster = nearestCluster(visible);
  if (cluster.empty()) {
    context.reason = "no static obstacle blocks the global race line";
    return context;
  }
  context.valid = true;
  context.outside_is_left = outsideIsLeft(ego, cluster);
  context.cluster_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    context.cluster_ids.push_back(obstacle.id);
  }
  context.visible.reserve(visible.size());
  for (const auto & obstacle : visible) {
    // relaxed 모드에서는 코리도의 스테이션별 통과 가능 구간도 감속 게이트 외피로 계산해야
    // 한다 — 도메인 끝점만 낮추면 코리도 교집합이 레이스 속도 외피로 다시 닫아 버린다.
    context.visible.push_back({
        obstacle.id, obstacle.start, obstacle.end, obstacle.center,
        relaxed_clearance_gate ? obstacle.relaxed_d_right : obstacle.d_right,
        relaxed_clearance_gate ? obstacle.relaxed_d_left : obstacle.d_left});
  }
  const auto fill_side = [&](bool go_left, P3ShadowSideDomain & side) {
      side.go_left = go_left;
      side.valid = computeSideTargetRange(
        ego, cluster, go_left, side.cluster_start, side.cluster_end,
        side.minimum_target, side.maximum_target, side.reason, relaxed_clearance_gate);
    };
  fill_side(false, context.right);
  fill_side(true, context.left);
  return context;
}

double RacelineSplinePlanner::finalizeP3ShadowPath(
  f110_msgs::msg::WpntArray & path,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::array<double, 5> & maneuver_stations) const
{
  const auto visible = expandVisibleObstacles(ego, obstacles);
  // 속도 상한은 최종 x/y에서 계산한 geometry를 입력으로 사용한다. 속도 성형 뒤에는 ax만
  // 갱신해 해석 geometry를 legacy 3점 원으로 다시 덮어쓰지 않는다.
  const auto geometry_start = active_research_cycle_ == nullptr ?
    ResearchClock::time_point() : ResearchClock::now();
  updateGeometry(path);
  if (active_research_cycle_ != nullptr) {
    active_research_cycle_->runtime_geometry_recompute_us += elapsedResearchUs(geometry_start);
  }
  const auto velocity_start = active_research_cycle_ == nullptr ?
    ResearchClock::time_point() : ResearchClock::now();
  const double confirmed_critical_speed_mps = applyAvoidanceVelocityLimit(
    path, ego, visible, maneuver_stations);
  if (active_research_cycle_ != nullptr) {
    active_research_cycle_->runtime_velocity_shaping_us += elapsedResearchUs(velocity_start);
  }
  const auto acceleration_start = active_research_cycle_ == nullptr ?
    ResearchClock::time_point() : ResearchClock::now();
  updateAccelerationOnly(path);
  if (active_research_cycle_ != nullptr) {
    active_research_cycle_->runtime_geometry_recompute_us +=
      elapsedResearchUs(acceleration_start);
  }
  return confirmed_critical_speed_mps;
}

P3ShadowPathEvaluation RacelineSplinePlanner::validateP3ShadowPath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  double obstacle_reserve_scale,
  const std::optional<double> & collision_horizon) const
{
  P3ShadowPathEvaluation result;
  Candidate candidate;
  candidate.path = path;
  const auto visible = expandVisibleObstacles(ego, obstacles);
  const auto measurement_start = active_research_cycle_ == nullptr ?
    ResearchClock::time_point() : ResearchClock::now();
  measureCandidate(ego, visible, candidate);
  if (active_research_cycle_ != nullptr) {
    result.runtime_candidate_measurement_us = elapsedResearchUs(measurement_start);
    active_research_cycle_->runtime_candidate_measurement_us +=
      result.runtime_candidate_measurement_us;
  }
  PathValidationFailure failure;
  const auto validation_start = active_research_cycle_ == nullptr ?
    ResearchClock::time_point() : ResearchClock::now();
  result.hard_valid = validateCandidate(
    ego, candidate.path, visible, candidate.reason, 0U, 0U, &failure, collision_horizon,
    obstacle_reserve_scale);
  if (active_research_cycle_ != nullptr) {
    result.runtime_hard_validation_us = elapsedResearchUs(validation_start);
    active_research_cycle_->runtime_hard_validation_us += result.runtime_hard_validation_us;
  }
  result.minimum_normalized_safety_slack = candidate.minimum_normalized_safety_slack;
  result.minimum_center_track_margin_m = candidate.centerline_wall_clearance_m;
  result.minimum_track_margin_m = candidate.rectangular_footprint_wall_clearance_m;
  result.minimum_obstacle_margin_m = candidate.obstacle_clearance_m;
  result.peak_curvature_radpm = candidate.peak_curvature_radpm;
  result.minimum_curvature_margin_radpm = candidate.minimum_curvature_margin_radpm;
  result.peak_curvature_rate_radpm2 = candidate.peak_curvature_rate_radpm2;
  result.velocity_loss = candidate.velocity_loss;
  result.global_path_deviation_m = candidate.global_path_deviation_m;
  result.ego_braking_distance_deficit_m = candidate.ego_braking_distance_deficit_m;
  if (active_research_cycle_ != nullptr) {
    double peak_positive = 0.0;
    double peak_negative = 0.0;
    for (const auto & waypoint : candidate.path.wpnts) {
      peak_positive = std::max(peak_positive, waypoint.kappa_radpm);
      peak_negative = std::min(peak_negative, waypoint.kappa_radpm);
    }
    result.peak_positive_curvature_radpm = peak_positive;
    result.peak_negative_curvature_radpm = peak_negative;
  }
  result.rejection_reason = result.hard_valid ? std::string() : candidate.reason;
  if (active_research_cycle_ != nullptr &&
    active_research_cycle_->all_violation_audit_enabled)
  {
    result.all_observed_violation_flags = auditCandidateViolations(
      ego, candidate.path, visible, 0U, 0U, collision_horizon, obstacle_reserve_scale);
  }
  result.first_failure_kind = static_cast<int>(failure.kind);
  result.failure_waypoint_index =
    failure.waypoint_index == std::numeric_limits<std::size_t>::max() ?
    -1 : static_cast<std::int64_t>(failure.waypoint_index);
  if (!result.hard_valid) {
    result.failure_obstacle_id = failure.obstacle_id;
    result.failure_waypoint_s = failure.waypoint_s;
    result.failure_waypoint_d = failure.waypoint_d;
    result.failure_footprint_side = failure.footprint_violation_side;
    result.failure_heading_relative_rad = failure.heading_relative_to_reference;
    result.failure_corner_protrusion_m = failure.wallward_corner_protrusion;
  }
  return result;
}

P3ShadowPathEvaluation RacelineSplinePlanner::evaluateP3PathCurrent(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  double obstacle_reserve_scale,
  const std::optional<double> & collision_horizon) const
{
  if (path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    P3ShadowPathEvaluation result;
    result.rejection_reason = "spline segment has too few global race-line samples";
    return result;
  }
  return validateP3ShadowPath(
    ego, path, obstacles, obstacle_reserve_scale, collision_horizon);
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
  // Global waypoint d_left/d_right are centre-of-vehicle limits. Vehicle width, obstacle
  // clearance, and tracking-error reserve do not belong here; subtract only the wall reserve.
  if (min_headroom != nullptr) {
    *min_headroom = std::numeric_limits<double>::infinity();
  }
  const auto fits_at = [&](const f110_msgs::msg::Wpnt & reference) {
      const double center_boundary_clearance = parameters_.trackBoundaryReserve(
        reference.vx_mps, reference.kappa_radpm);
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
  const EgoFrenetState & ego, double state_tail_distance_m, double speed_cap_mps) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(state_tail_distance_m) ||
    !(state_tail_distance_m > 0.0) || !(speed_cap_mps > 0.0))
  {
    return path;
  }

  const std::size_t total = reference_.wpnts.size();
  // tail 창은 경로 끝에서 거꾸로 잰 호 길이[m]다. 비율이던 시절에는 창이 경로 길이에
  // 비례해 요동했다(전체 루프 43 m의 10% = 4.3 m). state_machine의
  // enter_global_tail_distance_m와 같은 정의·같은 값이어야 한다.
  std::size_t tail_count = 1U;
  double walked_m = 0.0;
  while (tail_count < total) {
    const std::size_t index = total - tail_count;
    const double segment = forwardDistance(
      reference_.wpnts[(index - 1U) % total].s_m, reference_.wpnts[index % total].s_m);
    if (walked_m + segment > state_tail_distance_m) {
      break;
    }
    walked_m += segment;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;
  const std::size_t ego_index = nearestReferenceIndex(ego.s);

  // 계획된 복귀 램프: ego의 현재 d에서 0까지 smoothstep으로 내려간다. 램프 없이 d=0
  // 라인만 주면 복귀가 컨트롤러 자연 수렴(실측 0.055 m/m)에 맡겨져, 연속 장애물에서
  // 다음 기동이 남은 오프셋 위에서 시작되고 FSM은 |d| 게이트에 오래 붙잡힌다.
  const double ramp_d0 = (std::isfinite(ego.d) ? ego.d : 0.0);
  const double ramp_length = std::max(
    std::max(0.0, parameters_.merge_ramp_min_length_m),
    std::abs(ego.speed) * std::max(0.0, parameters_.merge_ramp_time_sec));
  const bool apply_ramp = std::abs(ramp_d0) > 0.03 && ramp_length > 1.0e-6;

  // 전체 global loop를 회전시켜 현재 ego를 마지막 tail 구간의 **첫 점**에 놓는다.
  // state_machine은 이 경로의 ot_line=raceline_global_handoff 표식을 GLOBAL 복귀의
  // 전제로 요구하고(2026-08-16 계약), 그 위에서 tail 도달·횡오차·지속시간 검사를
  // 평가한다. ego를 tail 첫 점에 놓으므로 tail 도달 게이트는 즉시 성립하고, 실질
  // 결정은 물리적 |ego_d| 게이트와 지속시간이 한다.
  const std::size_t first_index = (ego_index + total - tail_begin) % total;
  path.wpnts.reserve(total);
  for (std::size_t k = 0; k < total; ++k) {
    auto waypoint = reference_.wpnts[(first_index + k) % total];
    waypoint.id = static_cast<int32_t>(k);
    waypoint.d_m = 0.0;
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), speed_cap_mps);
    path.wpnts.push_back(waypoint);
  }

  // 회전 배열에서 ego는 위치 tail_begin에 놓인다(전방 순서: tail_begin → total-1).
  // 그 구간에 전방 호 길이 기준으로 램프를 적용한다. 기본 tail 길이(≈수 m)가
  // ramp_length보다 짧으면 램프가 잘리지만, 잘린 끝에서도 d는 단조 감소라 안전 방향이다.
  if (apply_ramp) {
    // 벽 협착부 클램프: 고정 길이 램프가 좁아지는 구간에서 오프셋을 유지한 채 지나가면
    // 벽 여유가 깎인다(.regression_check10: FINALS 최소 벽 여유 0.117→0.082 m — s≈23
    // 왼쪽 협착부). 각 지점의 트랙 여유로 |d|를 제한하고, 클램프로 내려간 뒤 다시
    // 벌어지는 구간에서도 단조 감소를 유지해 차가 도로 바깥쪽으로 되돌지 않게 한다.
    const double wall_keepout =
      std::max(0.0, parameters_.vehicle_half_width_m) +
      std::max(0.0, parameters_.wall_safety_margin_m);
    double forward_m = 0.0;
    double previous_magnitude = std::abs(ramp_d0);
    const double sign = (ramp_d0 >= 0.0 ? 1.0 : -1.0);
    for (std::size_t k = tail_begin; k < total; ++k) {
      const auto & global = reference_.wpnts[(first_index + k) % total];
      if (k > tail_begin) {
        const auto & previous = reference_.wpnts[(first_index + k - 1U) % total];
        forward_m += std::hypot(
          global.x_m - previous.x_m, global.y_m - previous.y_m);
      }
      if (forward_m >= ramp_length) {
        break;
      }
      const double t = std::clamp(forward_m / ramp_length, 0.0, 1.0);
      // smoothstep의 여집합 (1-t)^2(1+2t): d(0)=d0, d(L)=0, 양 끝 기울기 0.
      const double profile = std::abs(ramp_d0) * (1.0 - t) * (1.0 - t) * (1.0 + 2.0 * t);
      const double side_room = (sign > 0.0 ? global.d_left : global.d_right);
      const double allowed =
        std::isfinite(side_room) ? std::max(0.0, side_room - wall_keepout) :
        previous_magnitude;
      const double magnitude = std::min({profile, allowed, previous_magnitude});
      previous_magnitude = magnitude;
      const double d = sign * magnitude;
      auto & waypoint = path.wpnts[k];
      waypoint.d_m = d;
      waypoint.x_m = global.x_m - d * std::sin(global.psi_rad);
      waypoint.y_m = global.y_m + d * std::cos(global.psi_rad);
    }
  }

  if (parameters_.handoff_speed_shaping_enable && total >= 3U) {
    shapeGlobalHandoffSpeed(path, ego);
  }
  return path;
}

// R1 핸드오프 속도 성형 (2026-08-21, run_220742 충돌 A·B / 검토 반영 재작업).
// 종전에는 flat 캡뿐이라 회피 종료 직후 자차 실측 속도와 무관하게 캡 6.0 이 계단으로
// 실렸고, 컨트롤러는 3.7 m/s² 로 재가속하다 코너 탈출에서 그립 권한(6.9)을 넘겼다
// ("그립 권한 포화" 경고, 두 충돌 모두 이 재가속 중).
//
// applyLongitudinalFeasibility 를 그대로 못 쓰는 이유: 그 함수는 배열 순서로 걷는데
// 이 루프는 자차가 배열 중간(tail_begin)에 있다. 더구나 tail_begin은 nearest reference라
// 자차 뒤일 수도 있다. 여기서는 ego 기준 실제 전방거리로 정렬해 자차 시드가 첫 전방 점에
// 정확한 남은 거리로 걸리게 한다.
void RacelineSplinePlanner::shapeGlobalHandoffSpeed(
  f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego) const
{
  const std::size_t total = path.wpnts.size();
  if (total < 3U) {
    return;
  }
  // buildGlobalHandoffPath의 배열은 state-machine tail 계약 때문에 회전돼 있고, 배열의
  // tail_begin 점은 ego에 가장 가까운 기준점이다. 그 점이 ego 뒤쪽이면 기존 구현은 다음
  // 점까지의 가속거리로 기준점 한 칸 전체를 써서 실제 남은 거리보다 크게 예산했다. 배열은
  // 바꾸지 않고, 속도 패스만 forwardDistance(ego.s, s)로 정렬해 실제 자차 위치에서 시드한다.
  // 2026-08-24 장애물 snapshot 10개/1,786 frame에 합성 handoff를 건 open-loop A/B에서
  // shaping ON의 ego>=1 m/s 가속 위반이 667→0, 경로 내부 감속 위반이 0으로 내려갔다.
  // 남은 위반은 launch floor(ego<1)와 이미 첫 cap보다 빠른 ego seam뿐이며 별도 진단 대상이다.
  std::vector<std::size_t> ego_forward_order;
  ego_forward_order.reserve(total);
  for (std::size_t index = 0U; index < total; ++index) {
    ego_forward_order.push_back(index);
  }
  std::stable_sort(
    ego_forward_order.begin(), ego_forward_order.end(), [&](std::size_t first, std::size_t second) {
      return forwardDistance(ego.s, path.wpnts[first].s_m) <
             forwardDistance(ego.s, path.wpnts[second].s_m);
    });
  const auto at = [&](std::size_t j) -> f110_msgs::msg::Wpnt & {
      return path.wpnts[ego_forward_order[j]];
    };
  // (1) 실제 기하 곡률(복귀 램프 포함)로 횡가속 캡. 라인 자체는 v²κ 가 표 안이라
  //     이 캡은 주로 램프가 더한 곡률에만 문다. Menger 곡률은 크기만 쓴다(캡 용도).
  for (std::size_t j = 0; j < total; ++j) {
    const auto & a = at((j + total - 1U) % total);
    auto & b = at(j);
    const auto & c3 = at((j + 1U) % total);
    const double la = pointDistance(a, b);
    const double lb = pointDistance(b, c3);
    const double lc = pointDistance(a, c3);
    double menger = 0.0;
    if (la > kEpsilon && lb > kEpsilon && lc > kEpsilon) {
      const double cross = std::abs(
        (b.x_m - a.x_m) * (c3.y_m - a.y_m) - (b.y_m - a.y_m) * (c3.x_m - a.x_m));
      menger = 2.0 * cross / (la * lb * lc);
    }
    const double kappa = std::max(std::abs(b.kappa_radpm), menger);
    b.vx_mps = parameters_.limitedAvoidanceSpeed(std::max(0.0, b.vx_mps), kappa);
  }
  // (2) 전진 가속 램프 — 시드는 자차 실측 속도 (velocity_limits.csv 가속 열 재사용).
  if (parameters_.longitudinalVelocityLimitValid() &&
    !parameters_.avoidance_velocity_limit_accel_mps2.empty())
  {
    double speed = std::max(
      parameters_.longitudinal_launch_speed_floor_mps, std::max(0.0, ego.speed));
    double previous_forward = 0.0;
    for (std::size_t j = 0; j < total; ++j) {
      auto & waypoint = at(j);
      const double forward = forwardDistance(ego.s, waypoint.s_m);
      const double ds = forward - previous_forward;
      if (ds > kEpsilon) {
        const double accel = parameters_.accelLimitAt(speed);
        if (accel > 0.0) {
          speed = std::sqrt(speed * speed + 2.0 * accel * ds);
        }
      }
      waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), speed);
      speed = std::max(0.0, waypoint.vx_mps);
      previous_forward = forward;
    }
  }
  // (3) 후방 감속 패스 — 자차 위치(seam) 앞에서 멈춘다. 캡이 만든 하강 계단을
  //     제동 가능 프로파일로 편다.
  for (std::size_t j = total - 1U; j > 0U; --j) {
    auto & earlier = at(j - 1U);
    const auto & later = at(j);
    const double ds =
      forwardDistance(ego.s, later.s_m) - forwardDistance(ego.s, earlier.s_m);
    if (!(ds > kEpsilon)) {
      continue;
    }
    const double later_speed = std::max(0.0, later.vx_mps);
    const double earlier_speed = std::max(0.0, earlier.vx_mps);
    double decel = parameters_.profileFeasibilityDecel();
    const double table_decel =
      parameters_.decelLimitAt(std::max(earlier_speed, later_speed));
    if (table_decel > 0.0) {
      decel = table_decel;
    }
    if (decel > 0.0) {
      earlier.vx_mps = std::min(
        earlier_speed, std::sqrt(later_speed * later_speed + 2.0 * decel * ds));
    }
  }
  // (4) 성형이 끝난 기하·속도로 ψ·부호 있는 κ·ax 를 재계산한다 — 복귀 램프가 기하를
  //     바꿨는데 필드가 기준선 값 그대로면 하류(컨트롤러 곡률 FF = 부호 κ 소비)가
  //     틀린 값을 먹는다. 배열이 전방 순서로 연속인 닫힌 루프라 open-경로용 스텐실의
  //     오차는 배열 양끝 2점(자차 전방 ~tail 거리)에 국한되고 이웃 복사로 흡수된다.
  updateGeometryAndAcceleration(path);
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildRawSlowdownPath(
  const EgoFrenetState & ego, double state_tail_distance_m,
  double obstacle_front_m, double obstacle_span_m,
  double cap_mps, double decel_mps2) const
{
  // 기하는 검증된 핸드오프 루프(복귀 램프·벽 협착 클램프 포함)를 그대로 쓰고 속도만
  // 다시 쓴다. 속도 상한을 무한대로 넘겨 루프의 flat 캡을 무효화한다.
  f110_msgs::msg::WpntArray path = buildGlobalHandoffPath(
    ego, state_tail_distance_m, std::numeric_limits<double>::infinity());
  applyRawSlowdownProfile(path, ego, obstacle_front_m, obstacle_span_m, cap_mps, decel_mps2);
  return path;
}

// B1 속도 오버레이 (2026-08-21 분리, 2026-08-23 release 램프 추가). 어떤 경로에든
// "전방 front 지점에서 cap 에 닿는 감속 프로파일 + 스팬 통과 cap 유지 + 차량 가속표로
// 원속도에 복귀하는 release 프로파일"을 min 으로만 씌운다. 속도만 낮추므로 기하·정지
// 프로파일(0)은 절대 되살리지 않는다.
//
// release 램프가 없으면 hold_until 바로 다음 점이 원속도다. 0.1 m 간격에서 2.8 -> 6.0은
// 140.8 m/s^2를 요구하므로, raw가 모든 feasibility 패스 뒤(publishResult)에 적용된다는
// 사실과 합쳐져 실차가 따라갈 수 없는 가속 계단이 된다. 여기서 오버레이 자체를 완결된
// envelope로 만들면 후보 생성·handoff seam 코드를 건드리지 않고 전 발행 분기를 닫을 수 있다.
//
// 경로 배열은 handoff 닫힌 루프에서 ego가 중간에 있을 수 있으므로 배열 순서를 신뢰하지 않고
// forwardDistance(ego.s, waypoint.s_m)로 ego-forward 순서를 만든다. release 가속도는 다른
// feasibility 패스와 같은 velocity_limits 표(accelLimitAt)를 사용하고, 옛 파라미터 파일처럼
// 표가 없을 때만 전달받은 decel을 보수적인 대칭 fallback으로 쓴다.
void RacelineSplinePlanner::applyRawSlowdownProfile(
  f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego,
  double obstacle_front_m, double obstacle_span_m,
  double cap_mps, double decel_mps2) const
{
  if (path.wpnts.empty() || !(cap_mps > 0.0) || !(decel_mps2 > 0.0) ||
    !std::isfinite(obstacle_front_m) || !std::isfinite(obstacle_span_m))
  {
    return;
  }
  const double front = std::max(0.0, obstacle_front_m);
  // 스팬 뒤 1 m 까지 cap 을 유지한다: s_end 은 라이다가 앞면만 봐서 과소평가되는 값이라
  // (run_192006 접촉 #3), 뒤끝 직후 재가속을 한 박자 늦춘다.
  const double hold_until = front + std::max(0.0, obstacle_span_m) +
    std::max(0.0, parameters_.raw_slowdown_post_hold_distance_m);
  struct AheadWaypoint
  {
    double forward{0.0};
    std::size_t index{0U};
  };
  std::vector<AheadWaypoint> ahead;
  ahead.reserve(path.wpnts.size());
  for (std::size_t index = 0U; index < path.wpnts.size(); ++index) {
    const double forward = forwardDistance(ego.s, path.wpnts[index].s_m);
    if (forward >= 0.5 * trackLength()) {
      continue;  // 자차 뒤쪽 절반은 건드리지 않는다
    }
    ahead.push_back({forward, index});
  }
  std::stable_sort(
    ahead.begin(), ahead.end(),
    [](const AheadWaypoint & first, const AheadWaypoint & second) {
      return first.forward < second.forward;
    });

  double release_speed = cap_mps;
  double previous_release_forward = hold_until;
  for (const auto & sample : ahead) {
    auto & waypoint = path.wpnts[sample.index];
    const double forward = sample.forward;
    double limit = std::numeric_limits<double>::infinity();
    if (forward <= front) {
      // 전방 front 지점에서 정확히 cap 에 닿는 감속 실현 가능 프로파일.
      limit = std::sqrt(cap_mps * cap_mps + 2.0 * decel_mps2 * (front - forward));
    } else if (forward <= hold_until) {
      limit = cap_mps;
    } else {
      const double ds = std::max(0.0, forward - previous_release_forward);
      double accel = parameters_.accelLimitAt(release_speed);
      if (!(accel > 0.0) || !std::isfinite(accel)) {
        accel = decel_mps2;
      }
      release_speed = std::sqrt(
        std::max(0.0, release_speed * release_speed + 2.0 * accel * ds));
      limit = release_speed;
      previous_release_forward = forward;
    }
    waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), limit);
    if (forward <= hold_until) {
      release_speed = std::min(cap_mps, waypoint.vx_mps);
    } else {
      release_speed = waypoint.vx_mps;
    }
  }
}

VelocityFeasibilityReport RacelineSplinePlanner::inspectVelocityFeasibility(
  const f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego) const
{
  VelocityFeasibilityReport report;
  if (path.wpnts.empty() || !ready()) {
    return report;
  }
  struct AheadWaypoint
  {
    double forward{0.0};
    const f110_msgs::msg::Wpnt * waypoint{nullptr};
  };
  std::vector<AheadWaypoint> ahead;
  ahead.reserve(path.wpnts.size());
  for (const auto & waypoint : path.wpnts) {
    const double forward = forwardDistance(ego.s, waypoint.s_m);
    if (forward < 0.5 * trackLength()) {
      ahead.push_back({forward, &waypoint});
    }
  }
  std::stable_sort(
    ahead.begin(), ahead.end(),
    [](const AheadWaypoint & first, const AheadWaypoint & second) {
      return first.forward < second.forward;
    });

  const bool has_longitudinal_table = parameters_.longitudinalVelocityLimitValid() &&
    !parameters_.avoidance_velocity_limit_accel_mps2.empty();
  const bool inspect_steering_rate = parameters_.controlSteeringGeometryValid() &&
    std::isfinite(parameters_.control_max_steering_rate_radps) &&
    parameters_.control_max_steering_rate_radps > 0.0;
  double previous_forward = 0.0;
  double previous_speed = std::max(0.0, ego.speed);
  double previous_steering = std::numeric_limits<double>::quiet_NaN();
  for (const auto & sample : ahead) {
    const auto & waypoint = *sample.waypoint;
    const double speed = std::max(0.0, waypoint.vx_mps);
    const double lateral_cap = parameters_.limitedAvoidanceSpeed(
      speed, waypoint.kappa_radpm);
    if (speed > lateral_cap + 1.0e-6) {
      ++report.lateral_violations;
      report.maximum_lateral_ratio = std::max(
        report.maximum_lateral_ratio,
        speed / std::max(lateral_cap, kEpsilon));
    }

    if (has_longitudinal_table) {
      const double ds = sample.forward - previous_forward;
      if (ds > kEpsilon) {
        const double required =
          (speed * speed - previous_speed * previous_speed) / (2.0 * ds);
        if (required > 0.0) {
          report.maximum_acceleration_mps2 = std::max(
            report.maximum_acceleration_mps2, required);
          if (required > parameters_.accelLimitAt(previous_speed) + 1.0e-6) {
            ++report.acceleration_violations;
          }
        } else {
          const double deceleration = -required;
          report.maximum_deceleration_mps2 = std::max(
            report.maximum_deceleration_mps2, deceleration);
          if (deceleration >
            parameters_.decelLimitAt(std::max(previous_speed, speed)) + 1.0e-6)
          {
            ++report.deceleration_violations;
          }
        }
      } else if (std::abs(speed - previous_speed) > 1.0e-3) {
        // 같은 station에서 다른 속도는 유한한 가감속도로 실현할 수 없는 진짜 step이다.
        if (speed > previous_speed) {
          ++report.acceleration_violations;
          report.maximum_acceleration_mps2 = std::numeric_limits<double>::infinity();
        } else {
          ++report.deceleration_violations;
          report.maximum_deceleration_mps2 = std::numeric_limits<double>::infinity();
        }
      }
    }
    const double steering = parameters_.modeledControlSteeringRad(
      waypoint.kappa_radpm, speed);
    if (inspect_steering_rate && std::isfinite(previous_steering)) {
      const double ds = sample.forward - previous_forward;
      const double average_speed = 0.5 * (previous_speed + speed);
      if (ds > kEpsilon && average_speed > kEpsilon) {
        const double required_rate =
          std::abs(steering - previous_steering) * average_speed / ds;
        report.maximum_steering_rate_radps = std::max(
          report.maximum_steering_rate_radps, required_rate);
        if (required_rate > parameters_.control_max_steering_rate_radps + 1.0e-6) {
          ++report.steering_rate_violations;
        }
      } else if (ds <= kEpsilon && std::abs(steering - previous_steering) > 1.0e-6) {
        ++report.steering_rate_violations;
        report.maximum_steering_rate_radps = std::numeric_limits<double>::infinity();
      }
    }
    previous_forward = sample.forward;
    previous_speed = speed;
    previous_steering = steering;
  }
  return report;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildEmergencyStopPath(
  const EgoFrenetState & ego, double minimum_forward_m) const
{
  f110_msgs::msg::WpntArray path;
  path.header = reference_.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.d)) {
    return path;
  }
  // 점 수 하한(minimum_path_points)과 **거리 하한** 중 큰 쪽을 쓴다 (F3, 2026-08-22).
  // 이 경로는 원본이 없어 extendStopGeometry 로 늘릴 수 없으므로 여기서 직접 채운다.
  // 레퍼런스 간격이 0.1 m 면 8 점이 0.7 m 라, 점 수만으로는 컨트롤러의 L1(0.6+0.32·v)
  // 을 못 덮고 목표가 경로 끝점에 물린다. 전부 vx=0 이므로 길어도 비용이 없다.
  std::size_t distance_count = 0U;
  if (minimum_forward_m > 0.0) {
    const std::size_t first = nextReferenceIndex(ego.s);
    for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
      ++distance_count;
      const auto & global = reference_.wpnts[(first + k) % reference_.wpnts.size()];
      if (forwardDistance(ego.s, global.s_m) >= minimum_forward_m) {
        break;
      }
    }
  }
  const std::size_t count = std::min(
    reference_.wpnts.size(),
    std::max<std::size_t>(
      std::max<std::size_t>(2U, static_cast<std::size_t>(parameters_.minimum_path_points)),
      distance_count));
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

double RacelineSplinePlanner::firstCollisionForward(
  const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path,
  const std::vector<ExpandedObstacle> & visible, int * obstacle_id) const
{
  if (obstacle_id != nullptr) {
    *obstacle_id = -1;
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
  if (path.wpnts.empty() || nearest_forward > 0.5 * track_length_) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const auto & waypoint = path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const double clearance = parameters_.obstacleSafetyClearance(
      waypoint.vx_mps, waypoint.kappa_radpm);
    for (const auto & obstacle : visible) {
      if (forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.raw_d_right - clearance + kEpsilon &&
        waypoint.d_m < obstacle.raw_d_left + clearance - kEpsilon)
      {
        if (obstacle_id != nullptr) {
          *obstacle_id = obstacle.id;
        }
        return forward_s;
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

double RacelineSplinePlanner::forwardSpanAheadOfEgo(
  const f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego) const
{
  if (!ready() || !std::isfinite(ego.s)) {
    return 0.0;
  }
  double span = 0.0;
  for (const auto & waypoint : path.wpnts) {
    const double forward = forwardDistance(ego.s, waypoint.s_m);
    if (forward > 0.0 && forward <= 0.5 * track_length_) {
      span = std::max(span, forward);
    }
  }
  return span;
}

// F3 (2026-08-22). 근거는 헤더 선언부 주석 참고.
std::size_t RacelineSplinePlanner::extendStopGeometry(
  f110_msgs::msg::WpntArray & stop_path,
  const f110_msgs::msg::WpntArray & source_path,
  const EgoFrenetState & ego,
  double minimum_forward_m) const
{
  if (!(minimum_forward_m > 0.0) || stop_path.wpnts.empty() || source_path.wpnts.empty() ||
    !ready() || !std::isfinite(ego.s))
  {
    return 0U;
  }
  if (forwardSpanAheadOfEgo(stop_path, ego) >= minimum_forward_m) {
    return 0U;
  }
  // 정지 접두부의 끝점이 원본의 어디인지 찾는다. 같은 waypoint 를 복사해 만든 것이므로
  // s 로 맞춘다 — id 는 접두부를 만들 때 0 부터 다시 매겨졌으므로 쓸 수 없다.
  const double tail_s = stop_path.wpnts.back().s_m;
  std::size_t resume_index = source_path.wpnts.size();
  double best_gap = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < source_path.wpnts.size(); ++i) {
    const double gap = std::abs(source_path.wpnts[i].s_m - tail_s);
    if (gap < best_gap) {
      best_gap = gap;
      resume_index = i;
    }
  }
  if (resume_index >= source_path.wpnts.size() || best_gap > kEpsilon) {
    return 0U;   // 접두부가 이 원본에서 나오지 않았다. 이어 붙이면 라인이 갈린다.
  }

  std::size_t appended = 0U;
  for (std::size_t i = resume_index + 1U; i < source_path.wpnts.size(); ++i) {
    if (forwardSpanAheadOfEgo(stop_path, ego) >= minimum_forward_m) {
      break;
    }
    const double forward = forwardDistance(ego.s, source_path.wpnts[i].s_m);
    if (!(forward > 0.0) || forward > 0.5 * track_length_) {
      break;   // 랩을 감았다. 뒤로 도는 기하를 붙이지 않는다.
    }
    auto waypoint = source_path.wpnts[i];
    waypoint.id = static_cast<int32_t>(stop_path.wpnts.size());
    // 🔴 제동 프로파일 불변: 정지 목표 뒤는 전부 0 이다. 이 점들은 조향 기하로만 쓰인다.
    waypoint.vx_mps = 0.0;
    waypoint.ax_mps2 = 0.0;
    stop_path.wpnts.push_back(waypoint);
    ++appended;
  }
  if (appended > 0U) {
    updateGeometryAndAcceleration(stop_path);
    // updateGeometryAndAcceleration 이 ax 를 다시 채우므로 꼬리의 0 속도를 다시 못 박는다.
    for (std::size_t i = stop_path.wpnts.size() - appended; i < stop_path.wpnts.size(); ++i) {
      stop_path.wpnts[i].vx_mps = 0.0;
    }
  }
  return appended;
}

f110_msgs::msg::WpntArray RacelineSplinePlanner::buildLastPathBrake(
  const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  f110_msgs::msg::WpntArray braked;
  braked.header = path.header;
  if (!ready() || !std::isfinite(ego.s) || !std::isfinite(ego.speed) ||
    path.wpnts.empty())
  {
    return braked;
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
    return braked;
  }
  const double deceleration = std::max(parameters_.safe_stop_deceleration_mps2, 0.1);
  const double speed = std::max(0.0, ego.speed);
  // 설정 감속으로 서려면 필요한 거리. 이것만으로 정지 목표를 잡으면 장애물이 그보다 가까울 때
  // 목표가 장애물 뒤에 놓인다 — 15:37 백의 충돌 4회가 정확히 그 형태였다.
  const double nominal_stop = nearest_forward + speed * speed / (2.0 * deceleration);
  const auto visible = expandVisibleObstacles(ego, obstacles);
  const double contact_forward = firstCollisionForward(ego, path, visible, nullptr);

  // 포함 한계는 **접촉 지점 직전**이다. firstCollisionForward는 처음으로 충돌하는
  // waypoint의 거리를 돌려주므로 그 점 자체를 포함하면 안 된다 — 경계에서 한 점만
  // 넘어가도 그 자리가 곧 접촉이다.
  std::vector<std::size_t> included;
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const double forward_s = forwardDistance(ego.s, path.wpnts[i].s_m);
    if (forward_s > nominal_stop + kEpsilon) {
      break;
    }
    if (std::isfinite(contact_forward) && forward_s >= contact_forward - kEpsilon) {
      break;
    }
    included.push_back(i);
  }
  if (included.size() < 2U) {
    return braked;   // 이미 접촉 지점 — 호출자가 zero-speed hold로 처리한다.
  }
  // 정지 목표는 실제로 포함된 마지막 점이다. 그래야 프로파일이 그 점에서 정확히 0에
  // 닿고, 접촉점 앞에서 멈추기 위해 필요한 감속이 설정값을 넘더라도 명령이 0에 도달한다.
  const double stop_at = forwardDistance(ego.s, path.wpnts[included.back()].s_m);
  for (const std::size_t i : included) {
    const double forward_s = forwardDistance(ego.s, path.wpnts[i].s_m);
    auto waypoint = path.wpnts[i];
    waypoint.id = static_cast<int32_t>(braked.wpnts.size());
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(2.0 * deceleration * std::max(0.0, stop_at - forward_s)));
    braked.wpnts.push_back(waypoint);
  }
  braked.wpnts.back().vx_mps = 0.0;
  updateGeometryAndAcceleration(braked);
  return braked;
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
  int first_collision_id = -1;
  const double first_collision_forward =
    firstCollisionForward(ego, committed_path, visible, &first_collision_id);
  if (!std::isfinite(first_collision_forward)) {
    result.reason = "committed path has no obstacle collision before which to stop";
    return result;
  }

  // safe_stop_buffer_m는 "여유가 있을 때 남겨두는 것"이지 "여유가 없으면 포기하는 조건"이
  // 아니다 (2026-08-16). 종전에는 충돌이 버퍼(2.60 m)보다 가까우면 접두부가 비고, 호출자가
  // 장애물을 전혀 보지 않는 buildLastPathBrake로 떨어져 장애물 뒤에 정지 목표를 세웠다.
  // 그것이 15:37 백의 충돌 4회다. 이제 버퍼를 넣을 수 없으면 충돌 직전까지로 좁혀서라도
  // 반드시 접촉점 이전에 세운다.
  auto build_prefix = [&](double stop_at) {
      f110_msgs::msg::WpntArray prefix;
      prefix.header = committed_path.header;
      for (std::size_t i = start_index; i < committed_path.wpnts.size(); ++i) {
        const double forward_s = forwardDistance(ego.s, committed_path.wpnts[i].s_m);
        if (forward_s > stop_at + kEpsilon) {
          break;
        }
        auto waypoint = committed_path.wpnts[i];
        waypoint.id = static_cast<int32_t>(prefix.wpnts.size());
        prefix.wpnts.push_back(waypoint);
      }
      return prefix;
    };
  bool buffer_sacrificed = false;
  result.path = build_prefix(
    std::max(0.0, first_collision_forward - parameters_.safe_stop_buffer_m));
  if (result.path.wpnts.size() < 2U) {
    // 접촉점 바로 앞까지. 여기서 요구 감속이 safe_stop_deceleration_mps2를 넘을 수 있고,
    // 그것이 의도다 — 명령이 0에 닿지 않는 것보다 급제동이 낫다.
    buffer_sacrificed = true;
    result.path = build_prefix(std::max(0.0, first_collision_forward - kEpsilon));
  }
  if (result.path.wpnts.size() < 2U) {
    result.path.wpnts.clear();
    result.reason =
      "SAFE_STOP_NO_COLLISION_FREE_PREFIX: committed path has no collision-free braking prefix";
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
  result.reason = buffer_sacrificed ?
    "SAFE_STOP_BUFFER_SACRIFICED: braking on the remaining committed geometry to the last "
    "collision-free point (safe_stop_buffer_m did not fit; deceleration may exceed the "
    "configured rate)" :
    "braking on the remaining committed geometry before a collision";
  return result;
}

void RacelineSplinePlanner::measureCandidate(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  Candidate & candidate) const
{
  if (candidate.path.wpnts.empty()) {
    return;
  }

  double wall_clearance = std::numeric_limits<double>::infinity();
  // Ranking-side wall room. Unlike the legacy centerline headroom above, this is measured from the
  // same reference the obstacle term uses: the vehicle body plus the tracking-error tube. Without
  // it the two ranking terms are not commensurate -- the obstacle side already spends half width +
  // safety margin + tube while the wall side spent only wall_safety_margin_m -- and maximizing
  // their minimum biases every selection toward the wall by (obstacle clearance - wall margin)/2.
  double body_wall_clearance = std::numeric_limits<double>::infinity();
  double obstacle_clearance = std::numeric_limits<double>::infinity();
  double peak_curvature = 0.0;
  double minimum_curvature_margin = std::numeric_limits<double>::infinity();
  double minimum_curvature_slack = std::numeric_limits<double>::infinity();
  double peak_curvature_rate = 0.0;
  double ego_braking_distance_deficit = 0.0;
  double velocity_loss_sum = 0.0;
  double deviation_sum = 0.0;
  bool measured_obstacle = false;
  double previous_forward = 0.0;
  double previous_curvature = candidate.path.wpnts.front().kappa_radpm;
  FootprintTrackBoundSample minimum_footprint;

  for (std::size_t i = 0; i < candidate.path.wpnts.size(); ++i) {
    const auto & waypoint = candidate.path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const auto & reference = reference_.wpnts[nearestReferenceIndex(waypoint.s_m)];
    const double reserve = parameters_.trackBoundaryReserve(
      waypoint.vx_mps, waypoint.kappa_radpm);
    const double left_width = reference.d_left > 0.05 ?
      reference.d_left : parameters_.fallback_track_half_width_m;
    const double right_width = reference.d_right > 0.05 ?
      reference.d_right : parameters_.fallback_track_half_width_m;
    wall_clearance = std::min(
      wall_clearance,
      std::min(left_width - reserve - waypoint.d_m,
      waypoint.d_m + right_width - reserve));
    // Track-bound VALIDATION still spends wall_safety_margin_m exactly once; this extra body and
    // tube allowance is a ranking quantity only and never rejects a candidate.
    const double body_reserve = reserve + parameters_.vehicle_half_width_m +
      parameters_.avoidanceTrackingErrorReserve(waypoint.vx_mps, waypoint.kappa_radpm);
    body_wall_clearance = std::min(
      body_wall_clearance,
      std::min(left_width - body_reserve - waypoint.d_m,
      waypoint.d_m + right_width - body_reserve));
    const auto footprint = measureFootprintTrackBound(waypoint, i);
    if (footprint.footprint_clearance_m < minimum_footprint.footprint_clearance_m) {
      minimum_footprint = footprint;
    }

    for (const auto & obstacle : visible) {
      if (forward_s + kEpsilon < obstacle.start || forward_s > obstacle.end + kEpsilon) {
        continue;
      }
      measured_obstacle = true;
      const double clearance = parameters_.obstacleSafetyClearance(
        waypoint.vx_mps, waypoint.kappa_radpm);
      const double test_right = obstacle.raw_d_right - clearance;
      const double test_left = obstacle.raw_d_left + clearance;
      double signed_clearance = 0.0;
      if (waypoint.d_m >= test_left) {
        signed_clearance = waypoint.d_m - test_left;
      } else if (waypoint.d_m <= test_right) {
        signed_clearance = test_right - waypoint.d_m;
      } else {
        signed_clearance = -std::min(
          waypoint.d_m - test_right, test_left - waypoint.d_m);
      }
      obstacle_clearance = std::min(obstacle_clearance, signed_clearance);
    }

    const double curvature_abs = std::abs(waypoint.kappa_radpm);
    const double directional_curvature_limit =
      parameters_.maximumCurvatureFor(waypoint.kappa_radpm);
    peak_curvature = std::max(peak_curvature, curvature_abs);
    minimum_curvature_margin = std::min(
      minimum_curvature_margin, directional_curvature_limit - curvature_abs);
    minimum_curvature_slack = std::min(
      minimum_curvature_slack,
      (directional_curvature_limit - curvature_abs) /
      std::max(kEpsilon, directional_curvature_limit));
    const double waypoint_speed = std::max(0.0, waypoint.vx_mps);
    const double ego_speed = std::max(0.0, ego.speed);
    if (ego_speed > waypoint_speed + kEpsilon) {
      double decel = parameters_.decelLimitAt(std::max(ego_speed, waypoint_speed));
      if (!(decel > 0.0) || !std::isfinite(decel)) {
        decel = parameters_.profileFeasibilityDecel();
      }
      if (decel > 0.0 && std::isfinite(decel)) {
        const double required_distance =
          ego_speed * std::max(0.0, parameters_.confirmed_speed_response_delay_sec) +
          (ego_speed * ego_speed - waypoint_speed * waypoint_speed) / (2.0 * decel);
        ego_braking_distance_deficit = std::max(
          ego_braking_distance_deficit, required_distance - forward_s);
      }
    }
    if (i > 0U) {
      const double ds = forward_s - previous_forward;
      if (ds > kEpsilon) {
        peak_curvature_rate = std::max(
          peak_curvature_rate,
          std::abs(waypoint.kappa_radpm - previous_curvature) / ds);
      }
    }
    const double reference_speed = std::max(0.0, reference.vx_mps);
    if (reference_speed > kEpsilon) {
      velocity_loss_sum += std::max(
        0.0, reference_speed - std::max(0.0, waypoint.vx_mps)) / reference_speed;
    }
    deviation_sum += std::abs(waypoint.d_m);
    previous_forward = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }

  const double normalization_distance = std::max(
    kEpsilon, parameters_.maximum_target_offset_m);
  if (!measured_obstacle) {
    obstacle_clearance = normalization_distance;
  }
  const double wall_slack = body_wall_clearance / normalization_distance;
  const double obstacle_slack = obstacle_clearance / normalization_distance;
  const double curvature_rate_slack =
    (parameters_.maximum_curvature_rate_radpm2 - peak_curvature_rate) /
    std::max(kEpsilon, parameters_.maximum_curvature_rate_radpm2);

  candidate.centerline_wall_clearance_m = wall_clearance;
  candidate.rectangular_footprint_wall_clearance_m =
    minimum_footprint.footprint_clearance_m;
  candidate.footprint_invalid = minimum_footprint.invalid;
  candidate.footprint_violation_side =
    minimum_footprint.invalid ? minimum_footprint.minimum_side : std::string();
  candidate.footprint_violation_waypoint_index = minimum_footprint.waypoint_index;
  candidate.footprint_violation_s_m = minimum_footprint.waypoint_s_m;
  candidate.footprint_violation_x_m = minimum_footprint.waypoint_x_m;
  candidate.footprint_violation_y_m = minimum_footprint.waypoint_y_m;
  candidate.footprint_violation_yaw_rad = minimum_footprint.waypoint_yaw_rad;
  candidate.footprint_heading_relative_to_reference_rad =
    minimum_footprint.heading_relative_to_reference_rad;
  candidate.wallward_corner_protrusion_m =
    minimum_footprint.wallward_corner_protrusion_m;
  // Ranking compares body-referenced room on both sides (see body_wall_clearance above). The
  // rectangular footprint stays an additional hard gate, not a ranking weight or objective, and
  // centerline_wall_clearance_m above keeps the legacy headroom for audit continuity.
  candidate.wall_clearance_m = body_wall_clearance;
  candidate.obstacle_clearance_m = obstacle_clearance;
  candidate.peak_curvature_radpm = peak_curvature;
  candidate.minimum_curvature_margin_radpm = minimum_curvature_margin;
  candidate.peak_curvature_rate_radpm2 = peak_curvature_rate;
  candidate.velocity_loss = velocity_loss_sum /
    static_cast<double>(candidate.path.wpnts.size());
  candidate.global_path_deviation_m = deviation_sum /
    static_cast<double>(candidate.path.wpnts.size());
  candidate.ego_braking_distance_deficit_m = std::max(0.0, ego_braking_distance_deficit);
  candidate.minimum_normalized_safety_slack = std::min(
    {wall_slack, obstacle_slack, minimum_curvature_slack, curvature_rate_slack});
}

RacelineSplinePlanner::FootprintTrackBoundSample
RacelineSplinePlanner::measureFootprintTrackBound(
  const f110_msgs::msg::Wpnt & waypoint,
  std::size_t waypoint_index) const
{
  FootprintTrackBoundSample result;
  result.waypoint_index = waypoint_index;
  result.waypoint_s_m = waypoint.s_m;
  result.waypoint_x_m = waypoint.x_m;
  result.waypoint_y_m = waypoint.y_m;
  result.waypoint_yaw_rad = waypoint.psi_rad;

  const auto & reference = reference_.wpnts[nearestReferenceIndex(waypoint.s_m)];
  const double reserve = parameters_.trackBoundaryReserve(
    waypoint.vx_mps, waypoint.kappa_radpm);
  const double left_track_width = reference.d_left > 0.05 ?
    reference.d_left : parameters_.fallback_track_half_width_m;
  const double right_track_width = reference.d_right > 0.05 ?
    reference.d_right : parameters_.fallback_track_half_width_m;
  result.centerline_clearance_m = std::min(
    left_track_width - reserve - waypoint.d_m,
    waypoint.d_m + right_track_width - reserve);

  // Global d_left/d_right are distances from the reference to the physical track boundaries.
  // Project every rotated rectangle corner onto the matching local reference segment instead of
  // subtracting a heading-independent half-width from the candidate centre. The s-window keeps a
  // nearby parallel branch of a snake-shaped track from becoming the corner's reference. The
  // existing wall reserve is applied once; simulator TTC/noise guards intentionally do not enter.
  const double half_length = 0.5 * parameters_.vehicle_length_m;
  const double half_width = parameters_.vehicle_half_width_m;
  const double footprint_radius = std::hypot(half_length, half_width);
  const double candidate_cos = std::cos(waypoint.psi_rad);
  const double candidate_sin = std::sin(waypoint.psi_rad);
  const std::size_t anchor_index = nearestReferenceIndex(waypoint.s_m);
  std::vector<std::size_t> local_reference_segments;
  local_reference_segments.reserve(10U);
  const auto append_segment = [&](std::size_t index) {
      if (std::find(
          local_reference_segments.begin(), local_reference_segments.end(), index) ==
        local_reference_segments.end())
      {
        local_reference_segments.push_back(index);
      }
    };
  double covered_forward_s = 0.0;
  std::size_t forward_index = anchor_index;
  for (std::size_t count = 0; count < reference_.wpnts.size(); ++count) {
    if (covered_forward_s > footprint_radius + kEpsilon) {
      break;
    }
    append_segment(forward_index);
    const std::size_t next = (forward_index + 1U) % reference_.wpnts.size();
    covered_forward_s += forwardDistance(
      reference_.wpnts[forward_index].s_m, reference_.wpnts[next].s_m);
    forward_index = next;
  }
  double covered_backward_s = 0.0;
  std::size_t backward_end_index = anchor_index;
  for (std::size_t count = 0; count < reference_.wpnts.size(); ++count) {
    if (covered_backward_s > footprint_radius + kEpsilon) {
      break;
    }
    const std::size_t previous = backward_end_index == 0U ?
      reference_.wpnts.size() - 1U : backward_end_index - 1U;
    append_segment(previous);
    covered_backward_s += forwardDistance(
      reference_.wpnts[previous].s_m, reference_.wpnts[backward_end_index].s_m);
    backward_end_index = previous;
  }
  struct CornerResult
  {
    double clearance_m{std::numeric_limits<double>::infinity()};
    double heading_relative_rad{0.0};
    double wallward_protrusion_m{0.0};
    bool left_is_minimum{true};
  };
  const auto project_corner = [&](double longitudinal, double lateral) {
      const double corner_x = waypoint.x_m + longitudinal * candidate_cos -
        lateral * candidate_sin;
      const double corner_y = waypoint.y_m + longitudinal * candidate_sin +
        lateral * candidate_cos;
      double nearest_distance_squared = std::numeric_limits<double>::infinity();
      CornerResult corner;
      for (const std::size_t index : local_reference_segments) {
        const std::size_t next = (index + 1U) % reference_.wpnts.size();
        const auto & first = reference_.wpnts[index];
        const auto & second = reference_.wpnts[next];
        const double segment_x = second.x_m - first.x_m;
        const double segment_y = second.y_m - first.y_m;
        const double segment_length_squared =
          segment_x * segment_x + segment_y * segment_y;
        if (!(segment_length_squared > kEpsilon)) {
          continue;
        }
        const double ratio = std::clamp(
          ((corner_x - first.x_m) * segment_x +
          (corner_y - first.y_m) * segment_y) / segment_length_squared,
          0.0, 1.0);
        const double projected_x = first.x_m + ratio * segment_x;
        const double projected_y = first.y_m + ratio * segment_y;
        const double delta_x = corner_x - projected_x;
        const double delta_y = corner_y - projected_y;
        const double distance_squared = delta_x * delta_x + delta_y * delta_y;
        if (!(distance_squared < nearest_distance_squared)) {
          continue;
        }
        nearest_distance_squared = distance_squared;
        const double segment_yaw = std::atan2(segment_y, segment_x);
        const double segment_sin = std::sin(segment_yaw);
        const double segment_cos = std::cos(segment_yaw);
        const double corner_d = -delta_x * segment_sin + delta_y * segment_cos;
        const double center_d =
          -(waypoint.x_m - projected_x) * segment_sin +
          (waypoint.y_m - projected_y) * segment_cos;
        const double interpolated_left =
          first.d_left + ratio * (second.d_left - first.d_left);
        const double interpolated_right =
          first.d_right + ratio * (second.d_right - first.d_right);
        const double left_width = interpolated_left > 0.05 ?
          interpolated_left : parameters_.fallback_track_half_width_m;
        const double right_width = interpolated_right > 0.05 ?
          interpolated_right : parameters_.fallback_track_half_width_m;
        const double left_clearance = left_width - reserve - corner_d;
        const double right_clearance = corner_d + right_width - reserve;
        corner.left_is_minimum = left_clearance <= right_clearance;
        corner.clearance_m = std::min(left_clearance, right_clearance);
        corner.heading_relative_rad = normalizeAngle(waypoint.psi_rad - segment_yaw);
        const double wallward_extent = corner.left_is_minimum ?
          corner_d - center_d : center_d - corner_d;
        corner.wallward_protrusion_m = std::max(
          0.0, wallward_extent - half_width);
      }
      return corner;
    };
  const std::array<CornerResult, 4> corners{
    project_corner(half_length, half_width),
    project_corner(half_length, -half_width),
    project_corner(-half_length, half_width),
    project_corner(-half_length, -half_width)};
  const auto minimum = std::min_element(
    corners.begin(), corners.end(),
    [](const CornerResult & first, const CornerResult & second) {
      return first.clearance_m < second.clearance_m;
    });
  result.minimum_side = minimum->left_is_minimum ? "left" : "right";
  result.footprint_clearance_m = minimum->clearance_m;
  result.invalid = result.footprint_clearance_m < -kEpsilon;
  result.heading_relative_to_reference_rad = minimum->heading_relative_rad;
  result.wallward_corner_protrusion_m = minimum->wallward_protrusion_m;
  return result;
}

double RacelineSplinePlanner::applyAvoidanceVelocityLimit(
  f110_msgs::msg::WpntArray & path,
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const std::array<double, 5> & maneuver_stations) const
{
  // 클러스터 hull (2026-08-16): 한 물리 상자가 스캔에 두 조각으로 갈라져 들어오면, 조각
  // 사이 s-틈의 waypoint는 어떤 장애물 스팬에도 덮이지 않아 캡 없이 라인 속도로 남는다.
  // 결과는 스팬 안 1.1 ↔ 5.8 m/s 빗살 프로파일 — 컨트롤러가 옆 통과 내내 급가감속 펄스를
  // 받는다 (2026-08-16 13:52 백 실측: s=15.3~16.6에서 1.07→5.70→1.37→6.29). 그래서
  // nearestCluster와 같은 간격 규칙(obstacle_cluster_gap_m)으로 hull을 만들고, 틈에 놓인
  // waypoint는 그 클러스터 모든 멤버에 대한 side room의 최솟값으로 캡한다. side room
  // 계산은 멤버별 면 기준 그대로라, 반대편에 떨어진 멤버(유령 벽 등)는 room이 커서
  // 제한하지 않는다 — hull이 d까지 합집합하는 것이 아니다.
  struct CapCluster
  {
    double start{0.0};
    double end{0.0};
    std::size_t first{0U};
    std::size_t last{0U};   // inclusive
  };
  std::vector<CapCluster> cap_clusters;
  for (std::size_t index = 0U; index < visible.size(); ++index) {
    const auto & obstacle = visible[index];
    if (!cap_clusters.empty() &&
      obstacle.start <= cap_clusters.back().end + parameters_.obstacle_cluster_gap_m)
    {
      cap_clusters.back().end = std::max(cap_clusters.back().end, obstacle.end);
      cap_clusters.back().last = index;
    } else {
      cap_clusters.push_back({obstacle.start, obstacle.end, index, index});
    }
  }
  const auto reserve_against = [this](const f110_msgs::msg::Wpnt & waypoint,
    const ExpandedObstacle & obstacle) {
      const double side_room = (waypoint.d_m <= obstacle.raw_d_right) ?
        obstacle.raw_d_right - waypoint.d_m :
        (waypoint.d_m >= obstacle.raw_d_left ? waypoint.d_m - obstacle.raw_d_left : 0.0);
      return side_room - parameters_.obstacleBaseClearance();
    };
  double critical_speed_mps = std::numeric_limits<double>::infinity();
  bool critical_speed_sampled = false;
  const bool stations_valid = std::all_of(
    maneuver_stations.begin(), maneuver_stations.end(),
    [](double station) {return std::isfinite(station);}) &&
    std::adjacent_find(
    maneuver_stations.begin(), maneuver_stations.end(), std::greater<double>()) ==
    maneuver_stations.end();
  for (auto & waypoint : path.wpnts) {
    const double curvature_limited_speed = parameters_.limitedAvoidanceSpeed(
      std::max(0.0, waypoint.vx_mps), waypoint.kappa_radpm);
    double speed = curvature_limited_speed;
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    if (parameters_.confirmed_obstacle_speed_envelope_enable && stations_valid &&
      forward_s + kEpsilon >= maneuver_stations.front() &&
      forward_s <= maneuver_stations[3] + kEpsilon)
    {
      // 선택 3B (2026-08-24): 고정 2.8 m/s가 아니라 최종 기하가 요구하는 가장 낮은
      // 곡률 제한 속도를 통과속도로 쓴다. global 원속도도 curvature_limited_speed에 이미
      // min으로 포함되므로 회피경로가 기준선보다 빨라지는 일은 없다.
      critical_speed_mps = std::min(critical_speed_mps, curvature_limited_speed);
      critical_speed_sampled = true;
    }

    // Lateral room left over between this waypoint and the face of every obstacle it is passing.
    // The hard validator spends exactly obstacleBaseClearance() + reserve here, so the reserve
    // this waypoint may afford is whatever remains once the base clearance is paid.
    double admissible_reserve = std::numeric_limits<double>::infinity();
    bool covered = false;
    for (const auto & obstacle : visible) {
      if (forward_s < obstacle.start || forward_s > obstacle.end) {
        continue;
      }
      covered = true;
      admissible_reserve = std::min(admissible_reserve, reserve_against(waypoint, obstacle));
    }
    if (!covered) {
      for (const auto & cluster : cap_clusters) {
        if (forward_s < cluster.start || forward_s > cluster.end) {
          continue;
        }
        for (std::size_t index = cluster.first; index <= cluster.last; ++index) {
          admissible_reserve = std::min(
            admissible_reserve, reserve_against(waypoint, visible[index]));
        }
      }
    }
    speed = parameters_.gapLimitedAvoidanceSpeed(
      speed, waypoint.kappa_radpm, admissible_reserve);
    // gapLimitedAvoidanceSpeed는 입력을 올리지 않고, 횡가속 표는 속도에 대해 비증가로
    // 검증된다. 따라서 1차 곡률 cap보다 낮아진 speed는 이미 곡률 제약을 만족한다. 종전의
    // 두 번째 limitedAvoidanceSpeed 호출은 모든 유효 입력에서 no-op이어서 제거한다.
    waypoint.vx_mps = speed;
  }

  if (parameters_.confirmed_obstacle_speed_envelope_enable && stations_valid &&
    critical_speed_sampled && std::isfinite(critical_speed_mps))
  {
    // 선택 1B/2B (2026-08-24): entry~padded cluster end에서 얻은 critical speed를 실제
    // 장애물 통과구간에 유지한다. 먼 exit의 코너/낮은 글로벌 속도는 critical 표본에서 빼고,
    // 그 구간 자체의 점별 곡률 cap과 아래 후방 감속 패스가 처리한다.
    //
    // station[3]에는 obstacle_longitudinal_padding_m이 이미 들어 있다. 부족한 속도 전용
    // post-hold만 더해 detector 뒤 총 보정이 max(padding, post_hold)가 되게 한다. 따라서
    // 현재 운영 padding=0의 뒤끝 구멍을 막으면서, 나중에 padding을 올려도 이중 계상하지 않는다.
    const double hold_end_forward_m =
      parameters_.confirmedSpeedHoldEndForwardM(maneuver_stations[3]);
    for (auto & waypoint : path.wpnts) {
      const double forward_s = forwardDistance(ego.s, waypoint.s_m);
      if (forward_s + kEpsilon >= maneuver_stations[1] &&
        forward_s <= hold_end_forward_m + kEpsilon)
      {
        waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), critical_speed_mps);
      }
    }
  }

  applyApproachFeasibilityRamp(path, ego, visible);
  applyLongitudinalFeasibility(path, ego);
  return critical_speed_sampled ? critical_speed_mps :
         std::numeric_limits<double>::quiet_NaN();
}

// 종방향 실현가능 후방 패스 (2026-08-16).
//
// 위의 캡들은 waypoint마다 **독립적으로** 곡률·간격 상한을 매길 뿐, 인접 waypoint 사이에
// 종방향 동역학을 전혀 걸지 않는다. 그래서 S자 전이의 변곡점처럼 곡률이 순간 0에 가까워지는
// 지점에서는 캡이 통째로 풀려 프로파일이 튄다 — 14:30 백 실측(s=32.9~36.2):
//
//   s=32.89 v=6.63 → s=33.14 v=4.58   0.25 m 만에 46 m/s² 제동 요구
//   s=34.39 v=7.00 → s=34.90 v=5.62   곡률 0 구간에서 라인 속도로 복귀했다가 다시 하락
//
// 접근 램프는 "장애물 스팬 앞"에만 걸리므로 스팬이 아닌 전이 구간의 이 계단들은 잡지
// 못한다. 여기서 경로 전체에 v[i] = min(v[i], sqrt(v[i+1]² + 2·a·ds))를 뒤에서 앞으로
// 적용해 모든 감속이 실제로 제동 가능한 기울기가 되게 한다. 낮추기만 하므로 위의 어떤
// 안전 캡도 무효화하지 않는다.
//
// 가속 방향(올라가는 계단)은 일부러 제한하지 않는다: 명령이 실측보다 높은 것은 차가 낼 수
// 있는 만큼 내는 것이라 안전 문제가 아니고, 여기서 조이면 근거 없이 속도만 잃는다.
//
// ⚠️ 이 패스는 계단을 없앨 뿐 속도를 되찾아주지 않는다. s=33~37의 달성률 57~63%는 전이
// 구간의 곡률 자체가 원인이며 그것은 별개 문제다.
void RacelineSplinePlanner::applyApproachFeasibilityRamp(
  f110_msgs::msg::WpntArray & path,
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible) const
{
  // 접근 실현성 후방 제동 램프(2026-08-14 실차): 위 캡들은 장애물 스팬 안에서만 속도를
  // 낮추므로, 접근 구간은 프로파일 속도 그대로다가 스팬 경계에서 속도가 계단으로
  // 떨어진다. 실차에서는 그 계단이 서비스 브레이크 포화 → 마찰 한계 초과 슬립 →
  // 조향 상실로 이어졌다. 스팬 시작 시점의 계획 속도에서 approach_feasibility_decel_mps2로
  // 거꾸로 올라가는 제동 프로파일을 접근 구간에 씌워(낮추기만 한다) 제동이 스팬 훨씬
  // 전에 완만하게 시작되게 한다. 스팬 내부 속도는 건드리지 않는다.
  const double approach_decel = parameters_.approach_feasibility_decel_mps2;
  if (!(approach_decel > 0.0) || path.wpnts.empty()) {
    return;
  }
  // 🔴 램프는 **스팬마다** 건다 (2026-08-16). 예전에는 가장 가까운 스팬 하나만
  // (min(obstacle.start)) 대상으로 삼아서, 경로가 장애물 스팬을 두 개 이상 지나면 두 번째
  // 스팬 앞에는 램프가 전혀 없었다. 캡은 스팬 안에서만 속도를 낮추므로 그 경계에 계단이
  // 그대로 남는다 — 2026-08-16 백에서 s=39.67 vx=4.62 → s=39.92 vx=1.00, 즉 0.25 m 만에
  // 3.6 m/s를 요구했다(decel 2.0으로는 5.0 m가 필요). 이 계단이 실차에서 브레이크 포화 →
  // 마찰 한계 초과 → 조향 상실로 이어진 형태이고, 이 램프는 애초에 그것 때문에 들어갔다.
  //
  // 램프는 낮추기만 하므로 여러 스팬의 프로파일을 waypoint별 min으로 합성해도 정의가
  // 깨지지 않는다. 목표 속도는 캡이 모두 반영된 프로파일에서 먼저 모아두고(램프끼리 서로의
  // 목표를 갉아먹어 순서 의존이 생기지 않도록) 그 다음에 일괄 적용한다.
  struct ApproachTarget
  {
    double span_start{0.0};
    double command_hold_start{0.0};
    double target_speed{0.0};
    double decel{0.0};
  };
  std::vector<ApproachTarget> targets;
  targets.reserve(visible.size());
  const double ego_speed = std::max(0.0, ego.speed);
  const double response_delay = std::max(0.0, parameters_.confirmed_speed_response_delay_sec);
  const double response_distance = ego_speed * response_delay;
  for (const auto & obstacle : visible) {
    const double span_start = obstacle.start;
    if (!(span_start > kEpsilon)) {
      continue;   // 스팬이 자차에 붙어 있음(정지 탈출 등) — 접근 구간이 없다.
    }
    for (const auto & waypoint : path.wpnts) {
      if (forwardDistance(ego.s, waypoint.s_m) >= span_start) {
        // 명령 프로파일은 cap에 v*delay만큼 먼저 도달해 유지한다. 대상
        // 속도는 여전히 **실제 장애물 span 시작점**에서 읽어야 지연 예약이
        // 앞쪽 라인 속도로 바뀌지 않는다.
        targets.push_back({
            span_start, std::max(0.0, span_start - response_distance),
            std::max(0.0, waypoint.vx_mps), approach_decel});
        break;   // 경로가 스팬까지 안 이어지면 대상에서 빠진다.
      }
    }
  }
  if (targets.empty()) {
    return;
  }
  // 적응 기울기 (2026-08-16): 필요 감속의 기준은 **자차의 실측 속도와 스팬까지의 거리**
  // 하나다 — required = (v_ego² − v_스팬²)/(2·스팬까지 거리). 이것이 "지금 이 속도로
  // 여기서 출발해 스팬 시작까지 목표 속도에 닿는 시컨트"이며, 이보다 완만한 램프는
  // 자차 위치의 명령 속도가 실측 속도보다 낮아져 그 자리에서 계단이 된다(체인/재계획
  // 순간 — 13:20 백에서 5~6.4 m/s로 다음 장애물 5~8 m 앞에서 재계획하던 그 상황).
  // 경로 waypoint의 원시 속도로 시컨트를 재면 안 된다: 스팬 직전 점은 램프가 아직 안
  // 내린 라인 속도(=계단 그 자체)라 필요치가 항상 무한대로 발산한다(회귀 테스트로 고정).
  // 여유 있는 접근은 base(2.0)가 그대로 남고, 자차가 이미 빠르고 가까운 경우만 필요한
  // 만큼 [base, max]로 가팔라진다. max로도 모자라면 max 램프를 깔고(잔여 계단은 종전보다
  // 작다) 종전대로 안전정지 사다리가 받친다.
  // 적응 램프의 상한도 차량 표에 종속시킨다 (2026-08-19). 3.5 는 velocity_limits.csv 가
  // 어느 속도에서도 허용하지 않는 값이다(저속 3.0, v>=4 는 2.0). 표가 없으면 종전대로
  // 파라미터 값을 그대로 쓴다. base(approach_decel) 아래로는 내리지 않는다 — 그러면
  // std::clamp 의 lo > hi 가 되어 미정의 동작이고, base 는 comfort 목표라 별개다.
  const double table_decel = parameters_.decelLimitAt(ego_speed);
  const double approach_decel_max = std::max(
    approach_decel,
    table_decel > 0.0 ?
    std::min(parameters_.approach_feasibility_decel_max_mps2, table_decel) :
    parameters_.approach_feasibility_decel_max_mps2);
  for (auto & target : targets) {
    const double excess =
      ego_speed * ego_speed - target.target_speed * target.target_speed;
    const double required = excess > 0.0 && target.command_hold_start > kEpsilon ?
      excess / (2.0 * target.command_hold_start) :
      (excess > 0.0 ? std::numeric_limits<double>::infinity() : 0.0);
    target.decel = std::clamp(required, approach_decel, approach_decel_max);
  }
  for (auto & waypoint : path.wpnts) {
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    for (const auto & target : targets) {
      if (forward_s >= target.span_start) {
        continue;
      }
      if (forward_s >= target.command_hold_start) {
        waypoint.vx_mps = std::min(
          std::max(0.0, waypoint.vx_mps), target.target_speed);
        continue;
      }
      const double braking_speed = std::sqrt(
        target.target_speed * target.target_speed +
        2.0 * target.decel * (target.command_hold_start - forward_s));
      waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), braking_speed);
    }
  }
}

void RacelineSplinePlanner::applyLongitudinalFeasibility(
  f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego) const
{
  if (path.wpnts.size() < 2U) {
    return;
  }
  const bool has_table = parameters_.longitudinalVelocityLimitValid() &&
    !parameters_.avoidance_velocity_limit_accel_mps2.empty();

  // ── 1) 전진(가속) 패스 — 2026-08-19 신설 ────────────────────────────────────
  // 이 패스가 없던 동안 프로파일의 **가속에는 제약이 하나도 없었다**. 아래 후방 패스는
  // "이 속도까지 제동으로 내려갈 수 있나"만 보므로, 캡이 한 점을 눌렀다가 다음 점이 라인
  // 속도로 되튀는 계단은 통과시킨다. 실차 백에서 발행 경로의 가속 요구가 velocity_limits.csv
  // 한계를 넘은 비율이 v 5~9 m/s 에서 93.5% 였다.
  //
  // 🔑 시드는 경로 첫 점의 계획 속도가 아니라 **자차의 실측 속도**다. 정지 후 출발 구간에서
  // 요구 가속 p90 이 30 m/s² 로 튀던 원인이 바로 첫 점에 라인 속도가 그대로 실려 있었던
  // 것이고, 접근 램프(applyApproachFeasibilityRamp)가 이미 같은 이유로 실측 속도를 쓴다.
  // 전진 패스는 min 으로만 쓰므로 속도를 올리는 일이 없다 — 안전정지 0 프로파일은 안전하다.
  if (has_table) {
    double speed = std::max(parameters_.longitudinal_launch_speed_floor_mps,
      std::max(0.0, ego.speed));
    double previous_s = ego.s;
    for (auto & waypoint : path.wpnts) {
      const double ds = forwardDistance(previous_s, waypoint.s_m);
      if (ds > kEpsilon) {
        const double accel = parameters_.accelLimitAt(speed);
        if (accel > 0.0) {
          speed = std::sqrt(speed * speed + 2.0 * accel * ds);
        }
      }
      waypoint.vx_mps = std::min(std::max(0.0, waypoint.vx_mps), speed);
      // 다음 구간은 실제로 채택된 속도에서 이어간다. 캡이 더 낮게 눌렀다면 그 자리에서
      // 다시 가속 한계를 밟아 올라가야 하기 때문이다.
      speed = std::max(0.0, waypoint.vx_mps);
      previous_s = waypoint.s_m;
    }
  }

  // ── 2) 후방(감속) 패스 ──────────────────────────────────────────────────────
  // 순서가 전진 → 후방이어야 한다. 후방 패스가 낮추는 점은 정의상 감속 구간이므로 가속
  // 제약이 그 자리에서 느슨해져 재위반이 생기지 않는다. 반대 순서면 전진 패스가 후방 패스의
  // 제동 프로파일을 다시 깎아 두 제약이 서로를 무효화한다.
  //
  // 감속 한계는 이제 속도 의존이다 (2026-08-19). 상수 3.5 는 csv 의 v>=4 값(2.0)을 1.75배
  // 넘어서, 실차 백에서 감속 요구가 한계를 넘은 비율이 v 5~9 에서 84.2% 였다. 두 점 중
  // **큰 속도**에서 조회한다 — csv 의 감속 한계는 속도가 오를수록 작아지므로 그쪽이 보수적이다.
  const double scalar_decel = parameters_.profileFeasibilityDecel();
  for (std::size_t index = path.wpnts.size() - 1U; index > 0U; --index) {
    auto & earlier = path.wpnts[index - 1U];
    const auto & later = path.wpnts[index];
    const double ds = forwardDistance(earlier.s_m, later.s_m);
    if (!(ds > kEpsilon)) {
      continue;
    }
    const double later_speed = std::max(0.0, later.vx_mps);
    const double earlier_speed = std::max(0.0, earlier.vx_mps);
    double decel = scalar_decel;
    if (has_table) {
      const double table_decel =
        parameters_.decelLimitAt(std::max(earlier_speed, later_speed));
      if (table_decel > 0.0) {
        decel = table_decel;
      }
    }
    if (!(decel > 0.0)) {
      continue;
    }
    const double reachable = std::sqrt(later_speed * later_speed + 2.0 * decel * ds);
    earlier.vx_mps = std::min(earlier_speed, reachable);
  }
}

void RacelineSplinePlanner::updateGeometry(f110_msgs::msg::WpntArray & path) const
{
  auto & waypoints = path.wpnts;
  if (waypoints.size() < 2U) {
    return;
  }

  if (parameters_.analytic_path_geometry_enable && waypoints.size() >= 4U) {
    const std::size_t count = waypoints.size();
    double open_length = 0.0;
    for (std::size_t index = 1U; index < count; ++index) {
      open_length += pointDistance(waypoints[index - 1U], waypoints[index]);
    }
    const double average_spacing = open_length / static_cast<double>(count - 1U);
    const bool closed = count >= 8U && average_spacing > kEpsilon &&
      pointDistance(waypoints.back(), waypoints.front()) <= 2.0 * average_spacing;

    std::vector<double> open_station(count, 0.0);
    for (std::size_t index = 1U; index < count; ++index) {
      open_station[index] = open_station[index - 1U] +
        pointDistance(waypoints[index - 1U], waypoints[index]);
    }
    std::vector<double> headings(count, 0.0);
    std::vector<double> curvatures(count, 0.0);
    bool all_valid = true;
    for (std::size_t index = 0U; index < count; ++index) {
      std::vector<std::pair<double, double>> x_samples;
      std::vector<std::pair<double, double>> y_samples;
      if (closed) {
        x_samples.reserve(5U);
        y_samples.reserve(5U);
        for (int offset = -2; offset <= 2; ++offset) {
          std::size_t sample_index = index;
          double station = 0.0;
          if (offset > 0) {
            for (int step = 0; step < offset; ++step) {
              const std::size_t next = (sample_index + 1U) % count;
              station += pointDistance(waypoints[sample_index], waypoints[next]);
              sample_index = next;
            }
          } else if (offset < 0) {
            for (int step = 0; step < -offset; ++step) {
              const std::size_t previous = (sample_index + count - 1U) % count;
              station -= pointDistance(waypoints[previous], waypoints[sample_index]);
              sample_index = previous;
            }
          }
          x_samples.emplace_back(station, waypoints[sample_index].x_m);
          y_samples.emplace_back(station, waypoints[sample_index].y_m);
        }
      } else {
        const std::size_t window = std::min<std::size_t>(5U, count);
        const std::size_t half = window / 2U;
        const std::size_t start = std::min(
          index > half ? index - half : 0U, count - window);
        x_samples.reserve(window);
        y_samples.reserve(window);
        for (std::size_t sample_index = start; sample_index < start + window; ++sample_index) {
          const double station = open_station[sample_index] - open_station[index];
          x_samples.emplace_back(station, waypoints[sample_index].x_m);
          y_samples.emplace_back(station, waypoints[sample_index].y_m);
        }
      }
      const CurveDerivatives x = fitLocalCubicDerivatives(x_samples);
      const CurveDerivatives y = fitLocalCubicDerivatives(y_samples);
      const double speed_squared = x.first * x.first + y.first * y.first;
      if (!x.valid || !y.valid || !(speed_squared > kEpsilon)) {
        all_valid = false;
        break;
      }
      const double denominator = std::pow(speed_squared, 1.5);
      const double curvature =
        (x.first * y.second - y.first * x.second) / denominator;
      if (!std::isfinite(curvature)) {
        all_valid = false;
        break;
      }
      headings[index] = std::atan2(y.first, x.first);
      curvatures[index] = curvature;
    }
    if (all_valid) {
      for (std::size_t index = 0U; index < count; ++index) {
        waypoints[index].psi_rad = headings[index];
        waypoints[index].kappa_radpm = curvatures[index];
      }
      return;
    }
    // 중복점/퇴화 창 하나라도 있으면 경로 일부만 새 방식으로 내보내지 않고 전체를 legacy로
    // 되돌린다. 혼합 geometry는 곡률 속도상한과 controller FF가 서로 다른 모델을 보게 한다.
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
}

void RacelineSplinePlanner::updateAccelerationOnly(f110_msgs::msg::WpntArray & path) const
{
  auto & waypoints = path.wpnts;
  if (waypoints.empty()) {
    return;
  }
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

void RacelineSplinePlanner::updateGeometryAndAcceleration(
  f110_msgs::msg::WpntArray & path) const
{
  updateGeometry(path);
  updateAccelerationOnly(path);
}

bool RacelineSplinePlanner::validateCandidate(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<ExpandedObstacle> & visible,
  std::string & reason,
  std::size_t start_index,
  std::size_t minimum_points,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m,
  double obstacle_reserve_scale,
  bool skip_entry_continuity) const
{
  if (active_research_cycle_ != nullptr) {
    ++active_research_cycle_->validate_candidate_executed_total_actual;
  }
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
    double obstacle_s_end = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_test_d_right = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_test_d_left = std::numeric_limits<double>::quiet_NaN(),
    double obstacle_clearance = std::numeric_limits<double>::quiet_NaN())
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
          failure->obstacle_test_d_right = obstacle_test_d_right;
          failure->obstacle_test_d_left = obstacle_test_d_left;
          failure->obstacle_clearance = obstacle_clearance;
        }
      }
      return false;
    };
  // 🔴 2026-08-17: 자차에서 경로 첫 점으로 **건너뛰는** 구간의 기울기 검사.
  //
  // 종전에는 경로 내부 waypoint 사이의 기울기만 봤다(아래 quintic d-offset 검사). 그래서
  // 경로 자체는 매끄러운데 자차와 첫 점 사이가 불연속인 후보가 하드 검증을 통과했다.
  //
  // 실해 — 2026-08-17 00:10 백, 랩마다 재현된 충돌 2건:
  //   자차        s=23.40  d=+0.043
  //   커밋 경로 첫 점 s=23.60  d=+0.4897    → 0.20 m 앞에서 0.45 m 옆 (기울기 2.3)
  // 안전정지가 8회 확인 후 해제되며 이 경로를 커밋했고, 차는 그 점을 향하다 오히려 반대로
  // 밀려(d: +0.04 → -0.14) s=23.85에서 장애물에 박았다.
  //
  // 같은 검사가 buildCommittedPathStop에는 이미 있었다(정지 접두부용). 회피 경로에만
  // 없었던 것이므로 여기로 옮겨 모든 후보가 통과하게 한다.
  //
  // 🔴 2026-08-17 수정 — 기울기 **단독** 판정은 회귀였다.
  // 기준점을 "자차보다 엄밀히 앞선 최근접 waypoint"로 잡으면 그 전방거리가 경로 샘플
  // 간격 아래로 얼마든지 작아진다(자차 s는 두 샘플 사이 아무 데나 있다). 그러면
  //   기울기 = |Δd| / entry_forward
  // 는 경로 기하가 아니라 **추종오차를 0에 가까운 수로 나눈 값**이 되어 발산한다.
  // 00:34 백에서 정상 추종 중 9회 무효화됐고, 그 중 3회가 곧바로(0.3~0.5 s 뒤) 안전정지
  // 영구 정지로 이어졌다(8.9 s 1건 포함 — 사람이 pose를 옮겨야 풀렸다). 정지 점유율이
  // 00:10의 11%에서 53%로 뛰었다.
  //
  // 경로 **내부**의 급한 횡변화는 아래 quintic d-offset 검사가 이미 담당한다. 이 검사가
  // 추가로 담당하는 것은 "경로가 자차가 있는 곳에서 시작하는가" 하나뿐이므로, 먼저
  // 간격 자체를 이 플래너가 이미 들고 있는 **추종 예산**과 비교한다:
  //   trackingErrorReserve(v, kappa)  — localization_reserve_m를 바닥으로 포함
  // 예산 안의 간격은 정상 추종오차다. 예산을 넘더라도 도달할 거리가 충분하면(기울기가
  // 한계 이하) 정상이다. **둘 다** 어긋날 때만 불연속이다. 새 상수는 없다.
  //
  // 실해 사례는 그대로 걸린다: 간격 0.447 m > 예산(약 0.14~0.20 m)이고 기울기 2.3 > 0.8.
  // 오탐은 사라진다: 추종오차 간격은 예산 안이므로 분모가 아무리 작아도 통과한다.
  // F5 (2026-08-22): 이 검사만 끌 수 있다. 근거·제약은 헤더 선언부 주석 참고.
  if (!skip_entry_continuity) {
    std::size_t entry_index = path.wpnts.size();
    double entry_forward = std::numeric_limits<double>::infinity();
    for (std::size_t index = start_index; index < path.wpnts.size(); ++index) {
      const double forward = forwardDistance(ego.s, path.wpnts[index].s_m);
      if (forward > kEpsilon && forward < entry_forward) {
        entry_forward = forward;
        entry_index = index;
      }
    }
    if (entry_index < path.wpnts.size() && entry_forward <= 0.5 * track_length_) {
      const auto & entry = path.wpnts[entry_index];
      const double entry_lateral = std::abs(entry.d_m - ego.d);
      // 예산에는 마진 정책과 무관한 하한을 건다 (2026-08-20). 예산이 0 이면 아래 AND 의
      // 첫 조건이 상시 참이 되어 오탐 방지 장치가 사라지고, 검사가 기울기 하나로 무너진다
      // — 자세한 근거는 entry_discontinuity_min_budget_m 선언부 주석 참고.
      const double tracking_budget_m = std::max(
        parameters_.trackingErrorReserve(ego.speed, entry.kappa_radpm),
        std::max(0.0, parameters_.entry_discontinuity_min_budget_m));
      const bool beyond_tracking_budget = entry_lateral > tracking_budget_m;
      // 분모에 바닥을 깐다 (2026-08-23). 최근접 점의 forward 는 경로 샘플 간격 안에서
      // 얼마든지 작아지므로, 그대로 나누면 기울기가 기하가 아니라 발산값이 된다 —
      // 근거·실측은 entry_continuity_baseline_m 선언부 주석 참고.
      const double entry_slope_baseline_m = std::max(
        entry_forward, std::max(0.0, parameters_.entry_continuity_baseline_m));
      const bool excessive_entry_slope =
        entry_lateral / entry_slope_baseline_m > parameters_.maximum_lateral_slope;
      if (beyond_tracking_budget && excessive_entry_slope) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "path entry is discontinuous from the current ego d", entry_index, &entry);
      }
    }
  }
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
  double previous_d = path.wpnts[start_index].d_m;
  double previous_s = 0.0;
  double previous_curvature = path.wpnts[start_index].kappa_radpm;
  for (std::size_t i = start_index; i < path.wpnts.size(); ++i) {
    const auto & waypoint = path.wpnts[i];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const std::size_t reference_index = nearestReferenceIndex(waypoint.s_m);
    const auto & reference = reference_.wpnts[reference_index];
    const double center_boundary_clearance = parameters_.trackBoundaryReserve(
      waypoint.vx_mps, waypoint.kappa_radpm);
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
    const auto footprint = measureFootprintTrackBound(waypoint, i);
    if (footprint.invalid) {
      const bool rejected = reject(
        PathValidationFailureKind::kTrackBoundary,
        "footprint_track_bound", i, &waypoint);
      if (failure != nullptr) {
        failure->centerline_wall_clearance = footprint.centerline_clearance_m;
        failure->rectangular_footprint_wall_clearance = footprint.footprint_clearance_m;
        failure->footprint_violation_side = footprint.minimum_side;
        failure->waypoint_x = footprint.waypoint_x_m;
        failure->waypoint_y = footprint.waypoint_y_m;
        failure->waypoint_yaw = footprint.waypoint_yaw_rad;
        failure->heading_relative_to_reference =
          footprint.heading_relative_to_reference_rad;
        failure->wallward_corner_protrusion =
          footprint.wallward_corner_protrusion_m;
      }
      return rejected;
    }
    for (const auto & obstacle : visible) {
      const double obstacle_clearance = parameters_.obstacleSafetyClearance(
        waypoint.vx_mps, waypoint.kappa_radpm, obstacle_reserve_scale);
      const double obstacle_test_d_right = obstacle.raw_d_right - obstacle_clearance;
      const double obstacle_test_d_left = obstacle.raw_d_left + obstacle_clearance;
      if ((!maximum_collision_forward_m.has_value() ||
        forward_s <= maximum_collision_forward_m.value() + kEpsilon) &&
        forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle_test_d_right + 1.0e-6 &&
        waypoint.d_m < obstacle_test_d_left - 1.0e-6)
      {
        return reject(
          PathValidationFailureKind::kObstacleCollision,
          "d-offset intersects an inflated static-obstacle box", i, &waypoint, &obstacle,
          wrapS(ego.s + obstacle.start), wrapS(ego.s + obstacle.end),
          obstacle_test_d_right, obstacle_test_d_left, obstacle_clearance);
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
    if (std::abs(waypoint.kappa_radpm) >
      parameters_.maximumCurvatureFor(waypoint.kappa_radpm))
    {
      return reject(
        PathValidationFailureKind::kGeometry,
        waypoint.kappa_radpm >= 0.0 ?
        "shifted race line exceeds left control steering curvature" :
        "shifted race line exceeds right control steering curvature", i, &waypoint);
    }
    previous_d = waypoint.d_m;
    previous_s = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }
  if (active_research_cycle_ != nullptr) {
    ++active_research_cycle_->hard_valid_total_actual;
  }
  return true;
}

std::vector<std::string> RacelineSplinePlanner::auditCandidateViolations(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<ExpandedObstacle> & visible,
  std::size_t start_index,
  std::size_t minimum_points,
  const std::optional<double> & maximum_collision_forward_m,
  double obstacle_reserve_scale) const
{
  std::vector<std::string> violations;
  const auto observe = [&violations](const std::string & flag) {
      if (std::find(violations.begin(), violations.end(), flag) == violations.end()) {
        violations.push_back(flag);
      }
    };

  std::size_t entry_index = path.wpnts.size();
  double entry_forward = std::numeric_limits<double>::infinity();
  for (std::size_t index = start_index; index < path.wpnts.size(); ++index) {
    const double forward = forwardDistance(ego.s, path.wpnts[index].s_m);
    if (forward > kEpsilon && forward < entry_forward) {
      entry_forward = forward;
      entry_index = index;
    }
  }
  if (entry_index < path.wpnts.size() && entry_forward <= 0.5 * track_length_) {
    const auto & entry = path.wpnts[entry_index];
    const double entry_lateral = std::abs(entry.d_m - ego.d);
    const double tracking_budget_m = std::max(
      parameters_.trackingErrorReserve(ego.speed, entry.kappa_radpm),
      std::max(0.0, parameters_.entry_discontinuity_min_budget_m));
    const double entry_slope_baseline_m = std::max(
      entry_forward, std::max(0.0, parameters_.entry_continuity_baseline_m));
    if (entry_lateral > tracking_budget_m &&
      entry_lateral / entry_slope_baseline_m > parameters_.maximum_lateral_slope)
    {
      observe("ENTRY_DISCONTINUITY");
    }
  }

  if (start_index >= path.wpnts.size()) {
    observe("NO_FORWARD_PATH");
    return violations;
  }
  if (minimum_points == 0U) {
    minimum_points = static_cast<std::size_t>(parameters_.minimum_path_points);
  }
  if (path.wpnts.size() - start_index < minimum_points) {
    observe("MINIMUM_PATH_POINTS");
  }

  double previous_d = path.wpnts[start_index].d_m;
  double previous_s = 0.0;
  double previous_curvature = path.wpnts[start_index].kappa_radpm;
  for (std::size_t index = start_index; index < path.wpnts.size(); ++index) {
    const auto & waypoint = path.wpnts[index];
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    const auto & reference = reference_.wpnts[nearestReferenceIndex(waypoint.s_m)];
    const double center_boundary_clearance = parameters_.trackBoundaryReserve(
      waypoint.vx_mps, waypoint.kappa_radpm);
    const double left_width = reference.d_left > 0.05 ?
      reference.d_left : parameters_.fallback_track_half_width_m;
    const double right_width = reference.d_right > 0.05 ?
      reference.d_right : parameters_.fallback_track_half_width_m;
    if (waypoint.d_m > left_width - center_boundary_clearance + 1.0e-6 ||
      waypoint.d_m < -right_width + center_boundary_clearance - 1.0e-6)
    {
      observe("CENTER_TRACK_BOUND");
    }
    if (measureFootprintTrackBound(waypoint, index).invalid) {
      observe("FOOTPRINT_TRACK_BOUND");
    }
    for (const auto & obstacle : visible) {
      const double clearance = parameters_.obstacleSafetyClearance(
        waypoint.vx_mps, waypoint.kappa_radpm, obstacle_reserve_scale);
      if ((!maximum_collision_forward_m.has_value() ||
        forward_s <= maximum_collision_forward_m.value() + kEpsilon) &&
        forward_s >= obstacle.start && forward_s <= obstacle.end &&
        waypoint.d_m > obstacle.raw_d_right - clearance + 1.0e-6 &&
        waypoint.d_m < obstacle.raw_d_left + clearance - 1.0e-6)
      {
        observe("OBSTACLE_COLLISION");
      }
    }
    if (index > start_index) {
      const double ds = forward_s - previous_s;
      if (!(ds > kEpsilon)) {
        observe("NON_INCREASING_RACELINE_ORDER");
      } else {
        if (std::abs(waypoint.d_m - previous_d) / ds >
          parameters_.maximum_lateral_slope)
        {
          observe("LATERAL_SLOPE");
        }
        if (std::abs(waypoint.kappa_radpm - previous_curvature) / ds >
          parameters_.maximum_curvature_rate_radpm2)
        {
          observe("CURVATURE_RATE");
        }
      }
    }
    if (std::abs(waypoint.kappa_radpm) >
      parameters_.maximumCurvatureFor(waypoint.kappa_radpm))
    {
      observe(waypoint.kappa_radpm >= 0.0 ? "LEFT_CURVATURE" : "RIGHT_CURVATURE");
    }
    previous_d = waypoint.d_m;
    previous_s = forward_s;
    previous_curvature = waypoint.kappa_radpm;
  }
  return violations;
}

bool RacelineSplinePlanner::validatePath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::string * error,
  PathValidationFailure * failure,
  const std::optional<double> & maximum_collision_forward_m,
  double obstacle_reserve_scale,
  bool skip_entry_continuity) const
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
      maximum_collision_forward_m, obstacle_reserve_scale, skip_entry_continuity))
  {
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  }
  return true;
}

std::size_t RacelineSplinePlanner::generateP3Candidates(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<ExpandedObstacle> & visible,
  const std::optional<bool> & preferred_left,
  bool allow_side_switch,
  bool stop_on_first_feasible,
  std::vector<Candidate> & candidates,
  std::string & reason) const
{
  // 이 패키지의 유일한 회피 후보 생성기. 후보 생성은 여기 한 곳에만 두어야 한다 —
  // 안전정지 탈출 검증(anyFeasibleCandidateFrom)이 같은 함수를 쓰므로 "정지점에서 회피
  // 가능"이라는 판정과 실제 재계획이 어긋날 수 없다. 두 번째 사본이 생기면 그 덫이 돌아온다.
  const P3ShadowResult p3 = evaluateP3Shadow(ego, obstacles, 0, 0U, 0U, "PLAN");
  if (!p3.invoked) {
    reason = p3.failure_classification.empty() ? "P3 not invoked" : p3.failure_classification;
    return 0U;
  }
  std::size_t feasible = 0U;
  std::size_t considered = 0U;
  for (const auto & trace : p3.candidates) {
    // 측 잠금: 커밋된 측이 있고 전환이 금지된 상태면 그 측 후보만 본다.
    if (preferred_left.has_value() && !allow_side_switch &&
      trace.go_left != preferred_left.value())
    {
      continue;
    }
    if (trace.path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
      continue;
    }
    ++considered;
    Candidate candidate;
    candidate.go_left = trace.go_left;
    candidate.target_d = trace.d_target;
    candidate.path = trace.path;
    candidate.audit_index = trace.generation_index;
    candidate.exit_reaches_next_obstacle = trace.exit_reaches_next_obstacle;
    candidate.entry_transition_scale = trace.entry_scale;
    candidate.exit_transition_scale = trace.exit_scale;
    candidate.effective_exit_transition_scale = trace.exit_scale;
    // merge_s는 발행 세그먼트 끝이 아니라 d-offset이 실제로 d=0으로 복귀하는 지점이다.
    // 뒤에서부터 |d|가 tolerance를 넘는 마지막 점을 찾고 그 다음 점을 합류점으로 쓴다.
    std::size_t last_offset = 0U;
    for (std::size_t i = 0U; i < candidate.path.wpnts.size(); ++i) {
      if (std::abs(candidate.path.wpnts[i].d_m) > 1.0e-3) {
        last_offset = i;
      }
    }
    const std::size_t merge_index =
      std::min(last_offset + 1U, candidate.path.wpnts.size() - 1U);
    candidate.merge_s = candidate.path.wpnts[merge_index].s_m;
    // P3 trace의 지표를 그대로 믿지 않고 P0와 동일한 안전 계층으로 재측정한다. 순위·감사·
    // 하류 필드가 전부 같은 출처에서 나오도록 하기 위함이다.
    //
    // 장애물 충돌 검사는 이번 기동이 책임지는 범위(클러스터 끝 + post_merge_lookahead)
    // 까지만 본다 (AGENTS.md: "A post-merge controller-tail obstacle must not make the
    // current maneuver fail" — 커밋 경로 재검증이 merge horizon으로 지키는 원칙과 동일).
    // 그 너머의 다음 장애물은 연쇄 기동과 안전정지 사다리의 몫이다: long exit은 설계상
    // 뒤따르는 장애물 위로 오프셋을 끌고 가고, 연쇄 재계획이 도착 전에 경로를 교체한다
    // (AGENTS의 maximum_exit_length 비활성 사유 참고). 이 horizon이 없으면 라인 위
    // 장애물(예: map s=40.6)이 lookahead 안에 있는 동안 앞선 장애물(s=31.6)의 모든 회피
    // 후보가 12 m 밖 꼬리 충돌로 전멸해 kNoSafePath→영구 크립이 된다 (2026-08-15 run18
    // 실측). 트랙 경계·기하 검사는 horizon과 무관하게 경로 전체에 적용된다.
    measureCandidate(ego, visible, candidate);
    std::string validation_reason;
    // 🔴 2026-08-16: 여기도 다음 클러스터 앞에서 자른다. 종전에는 이 세 번째 지점만
    // 고정 거리를 그대로 써서, P3 평가기와 커밋 재검증을 고친 뒤에도 plan()의 후보가
    // 여전히 다음 장애물로 심판받았다. 23:31 백 실측: obs9(s=32.3, 우측 통과 가능,
    // 우측 여유 1.14 m)를 피하는 후보가 4.4 m 뒤 obs10(s=37.2, 라인을 물고 좌측 통과
    // 필수)에서 전멸해 양측 실패 → margin pass로 떨어졌고, 라인 위를 2.0 m/s로 5 m
    // 넘게 기어갔다(라인 속도 7.0). 세 지점이 같은 정의를 쓰지 않으면 어느 하나를
    // 고쳐도 증상이 남는다.
    const std::optional<double> collision_horizon =
      std::isfinite(p3.cluster_end_forward_m) ?
      std::optional<double>(
      maneuverScopeEnd(ego, obstacles, p3.cluster_obstacle_ids, p3.cluster_end_forward_m)) :
      std::nullopt;
    candidate.valid = validateCandidate(
      ego, candidate.path, visible, validation_reason, 0U, 0U, nullptr, collision_horizon);
    candidate.reason = candidate.valid ? std::string() :
      (validation_reason.empty() ? trace.rejection_reason : validation_reason);
    if (candidate.valid) {
      ++feasible;
    }
    const bool candidate_valid = candidate.valid;
    candidates.push_back(std::move(candidate));
    if (candidate_valid && stop_on_first_feasible) {
      return feasible;
    }
  }
  if (feasible == 0U) {
    reason = considered == 0U ?
      (p3.failure_classification.empty() ? "no P3 candidate on the permitted side" :
      p3.failure_classification) :
      std::to_string(considered) + " P3 candidates rejected by the exact validator";
  } else {
    reason = std::to_string(feasible) + "/" + std::to_string(considered) +
      " P3 candidates feasible";
  }
  return feasible;
}

bool RacelineSplinePlanner::anyFeasibleCandidateFrom(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::vector<ExpandedObstacle> & visible,
  const std::vector<ExpandedObstacle> & cluster) const
{
  if (cluster.empty()) {
    return true;   // 막는 것이 없으면 굳이 정지할 이유도 없다
  }
  std::vector<Candidate> candidates;
  std::string reason;
  // plan()과 반드시 같은 생성기를 쓴다(위 generateP3Candidates 주석 참고).
  return generateP3Candidates(
    ego, obstacles, visible, std::nullopt, true, true, candidates, reason) > 0U;
}

void RacelineSplinePlanner::densifyPath(
  f110_msgs::msg::WpntArray & path, std::size_t minimum_points) const
{
  if (path.wpnts.size() < 2U || path.wpnts.size() >= minimum_points) {
    return;
  }
  // 가장 긴 구간을 반복해서 이등분한다. d가 일정한 정지 prefix라 선형 보간으로 충분하고,
  // 곡률·가속도는 뒤에서 updateGeometryAndAcceleration이 다시 계산한다.
  while (path.wpnts.size() < minimum_points) {
    std::size_t longest = 0U;
    double longest_length = -1.0;
    for (std::size_t i = 0U; i + 1U < path.wpnts.size(); ++i) {
      const double dx = path.wpnts[i + 1U].x_m - path.wpnts[i].x_m;
      const double dy = path.wpnts[i + 1U].y_m - path.wpnts[i].y_m;
      const double length = std::hypot(dx, dy);
      if (length > longest_length) {
        longest_length = length;
        longest = i;
      }
    }
    if (!(longest_length > kEpsilon)) {
      break;      // 모든 구간이 이미 퇴화 — 더 쪼개도 의미가 없다
    }
    const auto & a = path.wpnts[longest];
    const auto & b = path.wpnts[longest + 1U];
    f110_msgs::msg::Wpnt mid = a;
    mid.x_m = 0.5 * (a.x_m + b.x_m);
    mid.y_m = 0.5 * (a.y_m + b.y_m);
    mid.d_m = 0.5 * (a.d_m + b.d_m);
    mid.d_left = 0.5 * (a.d_left + b.d_left);
    mid.d_right = 0.5 * (a.d_right + b.d_right);
    mid.vx_mps = 0.5 * (a.vx_mps + b.vx_mps);
    mid.s_m = wrapS(a.s_m + 0.5 * forwardDistance(a.s_m, b.s_m));
    mid.psi_rad = a.psi_rad + 0.5 * normalizeAngle(b.psi_rad - a.psi_rad);
    mid.kappa_radpm = 0.5 * (a.kappa_radpm + b.kappa_radpm);
    path.wpnts.insert(path.wpnts.begin() + static_cast<std::ptrdiff_t>(longest) + 1, mid);
  }
  for (std::size_t i = 0U; i < path.wpnts.size(); ++i) {
    path.wpnts[i].id = static_cast<int32_t>(i);
  }
}

RacelineSplineResult RacelineSplinePlanner::buildSafeStop(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const std::vector<ExpandedObstacle> & cluster,
  const std::vector<f110_msgs::msg::Obstacle> & raw_obstacles,
  const ExpandedObstacle & blocking) const
{
  (void)cluster;   // 탈출 검증은 정지점 기준으로 다시 확장한 cluster를 쓴다(아래 참고)
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  result.obstacle_id = blocking.id;
  double stop_at = std::max(0.0, blocking.start - parameters_.safe_stop_buffer_m);

  // 🔴 정지점 탈출 검증 (2026-08-14). 정지 자체보다 **어디에 서느냐**가 교착을 만든다.
  // 실차에서 safe_stop_buffer_m 1.20 m는 탈출 임계(2.0~2.5 m)보다 작아서, 정지하는
  // 순간 이미 회피 후보가 0개 생성되는 구역이었다(run_0101_090639: 장애물 1.6 m 앞에
  // 28.8초 정지, 좌 1.17 m/우 1.36 m로 횡공간은 충분했음). 정지점에서 v=0으로
  // 재계획이 되는지 먼저 묻고, 안 되면 뒤로 물린다.
  bool escape_verified = !parameters_.safe_stop_escape_check_enable;
  if (parameters_.safe_stop_escape_check_enable) {
    const double requested_stop_at = stop_at;
    const auto escapable_at = [&](double forward) {
        if (active_research_cycle_ != nullptr) {
          ++active_research_cycle_->safe_stop_escape_evaluator_count;
        }
        EgoFrenetState at_stop;
        at_stop.s = wrapS(ego.s + forward);
        at_stop.d = ego.d;
        at_stop.speed = 0.0;
        // ⚠️ ExpandedObstacle의 center/start/end는 **자차 상대거리**다
        //    (expandVisibleObstacles: center = forwardDistance(ego.s, obstacle.s_center)).
        //    그래서 자차 s만 정지점으로 옮기고 기존 visible/cluster를 재사용하면 장애물이
        //    정지점에서도 여전히 같은 거리에 있는 것으로 보여, 검증이 "현재 위치에서
        //    v=0으로 회피 가능한가"를 물을 뿐 정지점과 무관해진다(이분탐색도 무의미해진다).
        //    반드시 절대 s를 담은 원본으로 정지점 기준 재확장해야 한다.
        const auto at_stop_visible = expandVisibleObstacles(at_stop, raw_obstacles);
        const auto at_stop_cluster = nearestCluster(at_stop_visible);
        return anyFeasibleCandidateFrom(
          at_stop, raw_obstacles, at_stop_visible, at_stop_cluster);
      };

    // 탈출 가능성은 정지점을 **뒤로 물릴수록**(=forward가 작을수록) 단조 증가한다:
    // 장애물까지 남는 진입 거리가 그만큼 길어지기 때문이다. 그래서 선형 후퇴 대신
    // 이분탐색으로 "탈출 가능한 가장 늦은 정지점"을 찾는다. 선형 후퇴는 탈출이 아예
    // 불가능한 경우에 매 사이클 (후퇴횟수 × 2면 × 36후보)를 다 태워 실측 23.8 ms가
    // 나왔다(40 Hz 예산 25 ms를 젯슨에서 확실히 초과). 이분탐색은 흔한 경우 1회,
    // 가망 없는 경우 2회 탐침으로 끝난다.
    // ⚠️ 트랙 폭이 구간마다 달라 단조성이 국소적으로 깨질 수 있다. 그때 이분탐색은
    // 중간의 통과 가능 지점을 놓칠 수 있는데, 결과는 "원래 정지점 유지"라 보수적이다.
    if (escapable_at(stop_at)) {
      escape_verified = true;                      // 탐침 1회 — 정상 경로
    } else if (stop_at > kEpsilon && escapable_at(0.0)) {
      // 자차 자리에서는 탈출 가능 → 그 사이 어딘가가 경계다. 가장 늦은 탈출 가능점 탐색.
      escape_verified = true;
      double feasible = 0.0;                       // 탈출 가능이 확인된 값
      double infeasible = stop_at;                 // 탈출 불가가 확인된 값
      const int probes = std::clamp(parameters_.safe_stop_escape_max_retreats, 0, 12);
      const double resolution = std::max(kEpsilon, parameters_.safe_stop_escape_retreat_step_m);
      for (int i = 0; i < probes && (infeasible - feasible) > resolution; ++i) {
        const double middle = 0.5 * (feasible + infeasible);
        if (escapable_at(middle)) {
          feasible = middle;
        } else {
          infeasible = middle;
        }
      }
      stop_at = feasible;
    }
    // 어느 지점에서도 탈출이 안 되면 후퇴는 아무것도 사지 못한다. 축소된 stop_at을 그대로
    // 쓰면 (a) 필요보다 훨씬 일찍 서고 (b) 정지 prefix가 짧아져 경로가 통째로 무효화된다
    // (이 복원이 없을 때 kSafeStop이 kNoSafePath로 퇴화하는 것을 확인했다).
    if (!escape_verified) {
      stop_at = requested_stop_at;
    }
  }

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
  // 🔴 minimum_path_points를 안전정지 경로에도 적용한다 (2026-08-14). 이전에는 가드가
  // size()<2뿐이라 2점 경로 [v, 0]이 그대로 나갔는데, 제어기는 룩어헤드 지점의 속도를
  // 읽으므로 2점에서는 룩어헤드가 곧바로 끝점 0에 걸려 감속 프로파일을 통째로 건너뛰고
  // 즉시 0을 명령한다(실차 관측: /local_waypoints [1.08, 0.00] → /drive_autonomous 0.00).
  densifyPath(result.path, static_cast<std::size_t>(std::max(2, parameters_.minimum_path_points)));
  // 보간으로 생긴 점에도 같은 제동 프로파일을 다시 씌운다 — 선형 보간된 속도는
  // sqrt(2·a·거리) 곡선보다 항상 크거나 같아 낙관적이다.
  for (auto & waypoint : result.path.wpnts) {
    const double forward_s = forwardDistance(ego.s, waypoint.s_m);
    waypoint.vx_mps = std::min(
      std::max(0.0, waypoint.vx_mps),
      std::sqrt(
        2.0 * parameters_.safe_stop_deceleration_mps2 *
        std::max(0.0, stop_at - forward_s)));
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
  result.safe_stop_escape_verified = escape_verified;
  result.safe_stop_forward_m = stop_at;
  result.reason = escape_verified ?
    "both spline sides rejected; braking before the static obstacle" :
    // 여기까지 왔다면 자차 위치까지 물러나도 회피 후보가 하나도 생성되지 않는다. 전진
    // 계획으로는 풀 수 없는 상태(후진이 필요)이므로, 조용히 매달려 있지 말고 알린다.
    "both spline sides rejected; braking before the static obstacle; "
    "WARNING no escapable stop point exists — the car will not be able to resume from this "
    "stop by forward planning";
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

  // A margin-only cluster never requires stopping: the line itself is physically drivable, so
  // stabilization is spent holding the line at the capped speed instead of braking to a halt
  // (and instead of a zero-speed hold when the cluster is first seen inside the stop buffer).
  if (!clusterPhysicallyBlocksRaceline(cluster)) {
    auto slow_pass = buildMarginSlowPass(ego, cluster);
    if (slow_pass.kind == SplinePlanKind::kAvoidance) {
      slow_pass.reason =
        "margin-only cluster during initial stabilization; holding the race line at the "
        "margin_pass speed cap";
      return slow_pass;
    }
  }

  result = buildSafeStop(ego, visible, cluster, obstacles, cluster.front());
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
  std::vector<Candidate> candidates;

  // 회피 후보 생성은 P3(analytic corridor) 한 곳뿐이다. P0 quintic 격자는 2026-08-15에
  // 제거됐다: 실차 시험에서 P0가 통과 가능한 모든 지점을 P3도 통과했고, 두 생성기를 함께
  // 두면 "어느 쪽이 답했는가"에 따라 연쇄 기동·안전정지 해제가 갈라진다.
  //
  // ⚠️ 사유 문자열은 P3가 돌려주는 이것 하나다. 예전에는 좌/우를 따로 평가하던 시절의
  // left_evaluated/right_evaluated 플래그로 "양측 기각" vs "반대측 측 잠금"을 갈라 적었는데,
  // P0 제거 후 그 두 변수는 false로 초기화된 뒤 갱신되는 곳이 없어져 **항상** "committed
  // side rejected; alternate side locked after lateral engagement"가 출력됐다. 실제로는
  // 양측을 모두 평가하고 양측 다 실패한 경우까지 그렇게 찍혀, 원인을 측 잠금 쪽으로
  // 오도했다 (2026-08-16 14:18 백: 실제 사유는 좌·우 모두 NO_VALID_SIDE_DOMAIN인 코너
  // 정점 기하 한계였는데 문구만 보고 측 잠금으로 반복 오진). 사유는 P3 것을 그대로 쓴다.
  std::string p3_reason;
  (void)generateP3Candidates(
    ego, obstacles, visible, preferred_left, allow_side_switch, false,
    candidates, p3_reason);

  std::vector<std::size_t> feasible_order;
  feasible_order.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].valid) {
      feasible_order.push_back(index);
    }
  }
  // 순위 정의는 candidate_rank.hpp 한 곳에만 둔다 (2026-08-21 통합). 항목·순서·근거는
  // 그 헤더의 주석에 있고, P3ShadowEvaluator::betterFeasible 도 같은 함수를 쓴다 —
  // 종전에는 같은 순위를 양쪽이 따로 구현하면서 동률 epsilon 이 1000 배 달랐다.
  const auto rank_key = [&](std::size_t index) {
      const auto & candidate = candidates[index];
      CandidateRankKey key;
      key.exit_reaches_next_obstacle = candidate.exit_reaches_next_obstacle;
      key.ego_braking_distance_deficit_m = candidate.ego_braking_distance_deficit_m;
      key.velocity_loss = candidate.velocity_loss;
      key.minimum_normalized_safety_slack = candidate.minimum_normalized_safety_slack;
      key.global_path_deviation_m = candidate.global_path_deviation_m;
      key.tiebreak_index = candidate.audit_index;
      return key;
    };
  const auto better_candidate = [&](std::size_t first_index, std::size_t second_index) {
      return betterCandidateRank(rank_key(first_index), rank_key(second_index));
    };
  std::stable_sort(feasible_order.begin(), feasible_order.end(), better_candidate);

  std::vector<int> ranks(candidates.size(), -1);
  for (std::size_t rank = 0; rank < feasible_order.size(); ++rank) {
    ranks[feasible_order[rank]] = static_cast<int>(rank + 1U);
  }
  // 강등 전 가상 순위(exit_reaches_next_obstacle 항을 뺀 순수 slack 순위). 강등이 실제로
  // 선택을 바꿨는지 감사 스트림에서 직접 확인하기 위한 것으로, 선택에는 쓰지 않는다:
  // rank_without_exit_demotion==1인 후보와 final_rank==1인 후보가 다르면 강등이 결정했다.
  const auto slack_only_candidate = [&](std::size_t first_index, std::size_t second_index) {
      const auto & first = candidates[first_index];
      const auto & second = candidates[second_index];
      const double slack_delta =
        first.minimum_normalized_safety_slack - second.minimum_normalized_safety_slack;
      if (std::abs(slack_delta) > kEpsilon) {
        return slack_delta > 0.0;
      }
      const double speed_loss_delta = first.velocity_loss - second.velocity_loss;
      if (std::abs(speed_loss_delta) > kEpsilon) {
        return speed_loss_delta < 0.0;
      }
      const double deviation_delta =
        first.global_path_deviation_m - second.global_path_deviation_m;
      if (std::abs(deviation_delta) > kEpsilon) {
        return deviation_delta < 0.0;
      }
      return first.audit_index < second.audit_index;
    };
  std::vector<std::size_t> slack_order = feasible_order;
  std::stable_sort(slack_order.begin(), slack_order.end(), slack_only_candidate);
  std::vector<int> slack_ranks(candidates.size(), -1);
  for (std::size_t rank = 0; rank < slack_order.size(); ++rank) {
    slack_ranks[slack_order[rank]] = static_cast<int>(rank + 1U);
  }
  const std::size_t selected_index = feasible_order.empty() ?
    std::numeric_limits<std::size_t>::max() : feasible_order.front();
  const auto build_audits = [&]() {
      std::vector<SplineCandidateAudit> audits;
      audits.reserve(candidates.size());
      for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto & candidate = candidates[index];
        SplineCandidateAudit audit;
        audit.generation_index = candidate.audit_index;
        audit.feasible = candidate.valid;
        audit.selected = index == selected_index;
        audit.go_left = candidate.go_left;
        audit.final_rank = ranks[index];
        audit.exit_reaches_next_obstacle = candidate.exit_reaches_next_obstacle;
        audit.rank_without_exit_demotion = slack_ranks[index];
        audit.target_d = candidate.target_d;
        audit.entry_fraction = candidate.entry_transition_scale;
        audit.exit_transition_scale = candidate.effective_exit_transition_scale;
        audit.requested_entry_length_m = candidate.requested_entry_length_m;
        audit.effective_entry_length_m = candidate.effective_entry_length_m;
        audit.exit_length_m = candidate.exit_length_m;
        audit.centerline_wall_clearance_m = candidate.centerline_wall_clearance_m;
        audit.rectangular_footprint_wall_clearance_m =
          candidate.rectangular_footprint_wall_clearance_m;
        audit.footprint_invalid = candidate.footprint_invalid;
        audit.footprint_violation_side = candidate.footprint_violation_side;
        audit.footprint_violation_waypoint_index =
          candidate.footprint_violation_waypoint_index ==
          std::numeric_limits<std::size_t>::max() ?
          -1 : static_cast<std::int64_t>(candidate.footprint_violation_waypoint_index);
        audit.footprint_violation_s_m = candidate.footprint_violation_s_m;
        audit.footprint_violation_x_m = candidate.footprint_violation_x_m;
        audit.footprint_violation_y_m = candidate.footprint_violation_y_m;
        audit.footprint_violation_yaw_rad = candidate.footprint_violation_yaw_rad;
        audit.footprint_heading_relative_to_reference_rad =
          candidate.footprint_heading_relative_to_reference_rad;
        audit.wallward_corner_protrusion_m = candidate.wallward_corner_protrusion_m;
        audit.wall_clearance_m = candidate.wall_clearance_m;
        audit.obstacle_clearance_m = candidate.obstacle_clearance_m;
        audit.peak_curvature_radpm = candidate.peak_curvature_radpm;
        audit.minimum_curvature_margin_radpm = candidate.minimum_curvature_margin_radpm;
        audit.peak_curvature_rate_radpm2 = candidate.peak_curvature_rate_radpm2;
        audit.velocity_loss = candidate.velocity_loss;
        audit.global_path_deviation_m = candidate.global_path_deviation_m;
        audit.minimum_normalized_safety_slack =
          candidate.minimum_normalized_safety_slack;
        audit.ego_braking_distance_deficit_m = candidate.ego_braking_distance_deficit_m;
        audit.rejection_reason = candidate.valid ? std::string() : candidate.reason;
        audits.push_back(std::move(audit));
      }
      return audits;
    };

  if (selected_index == std::numeric_limits<std::size_t>::max()) {
    // Both spline sides failed. If every cluster member blocks the line only through the
    // inflated tracking/uncertainty margin, the line itself is physically drivable: degrade to
    // a capped-speed lane hold instead of braking (2026-08-12 21:11 run: an off-line hairpin
    // obstacle escalated to a zero-speed hold on every lap through this branch).
    if (!clusterPhysicallyBlocksRaceline(cluster)) {
      auto slow_pass = buildMarginSlowPass(ego, cluster);
      if (slow_pass.kind == SplinePlanKind::kAvoidance) {
        slow_pass.reason += "; avoidance rejected: " + p3_reason;
        slow_pass.candidate_audits = build_audits();
        return slow_pass;
      }
    }
    auto safe_stop = buildSafeStop(ego, visible, cluster, obstacles, cluster.front());
    safe_stop.obstacle_ids.reserve(cluster.size());
    for (const auto & obstacle : cluster) {
      safe_stop.obstacle_ids.push_back(obstacle.id);
    }
    // 실제 사유를 먼저 적는다. 로그 라인이 길어 잘리는 환경에서도 원인이 남아야 한다
    // (14:18 백에서 NO_VALID_SIDE_DOMAIN이 잘려 나가 오진의 직접 원인이 됐다).
    // 측 제한이 실제로 걸린 경우에만 그렇게 표시한다 — plan()에 preferred_left가 있고
    // allow_side_switch가 false일 때가 유일한 그 경우다.
    const bool side_restricted = preferred_left.has_value() && !allow_side_switch;
    safe_stop.reason = "no avoidance candidate: " + p3_reason + "; braking before the static "
      "obstacle" + (side_restricted ?
      std::string(" (search restricted to the committed ") +
      (preferred_left.value() ? "left" : "right") + " side)" : std::string());
    // buildSafeStop이 붙인 탈출-불가 경고는 위 대입으로 지워지므로 여기서 다시 붙인다.
    if (!safe_stop.safe_stop_escape_verified) {
      safe_stop.reason +=
        "; WARNING no escapable stop point exists — the car will not be able to resume from "
        "this stop by forward planning";
    }
    safe_stop.candidate_audits = build_audits();
    return safe_stop;
  }
  Candidate selected = std::move(candidates[selected_index]);
  result.kind = SplinePlanKind::kAvoidance;
  result.path = std::move(selected.path);
  result.go_left = selected.go_left;
  result.target_d = selected.target_d;
  result.merge_s = selected.merge_s;
  result.entry_transition_scale = selected.entry_transition_scale;
  result.exit_transition_scale = selected.exit_transition_scale;
  result.effective_entry_transition_scale = selected.effective_entry_transition_scale;
  result.effective_exit_transition_scale = selected.effective_exit_transition_scale;
  result.requested_entry_length_m = selected.requested_entry_length_m;
  result.effective_entry_length_m = selected.effective_entry_length_m;
  result.exit_length_m = selected.exit_length_m;
  result.obstacle_id = cluster.front().id;
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  result.control_points = std::move(selected.control_points);
  result.candidate_audits = build_audits();
  result.reason =
    "selected maximum normalized safety-slack candidate from " +
    std::to_string(feasible_order.size()) + "/" + std::to_string(candidates.size()) +
    " feasible/generated local quintic paths";
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
