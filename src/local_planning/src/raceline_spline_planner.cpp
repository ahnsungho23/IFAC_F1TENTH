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

// ============================================================================
// 순수 기하 계산 보조 함수와 1차원 natural cubic spline
// ============================================================================

// 수치 비교, waypoint 간 최소 거리 및 spline 행렬 특이점 판정 허용 오차
constexpr double kEpsilon = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

// 값이 하한/상한을 넘지 않도록 제한한다.
double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

// heading 차이를 [-π, π] 범위로 정규화해 wrap 보간의 급격한 회전을 막는다.
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

// 경로 생성·검증에 쓰는 모든 waypoint 필드가 유한한지 확인한다.
bool finiteWaypoint(const f110_msgs::msg::Wpnt & waypoint)
{
  return std::isfinite(waypoint.s_m) && std::isfinite(waypoint.d_m) &&
         std::isfinite(waypoint.x_m) && std::isfinite(waypoint.y_m) &&
         std::isfinite(waypoint.psi_rad) && std::isfinite(waypoint.kappa_radpm) &&
         std::isfinite(waypoint.vx_mps) && std::isfinite(waypoint.ax_mps2) &&
         std::isfinite(waypoint.d_left) && std::isfinite(waypoint.d_right);
}

// 두 map-frame waypoint의 유클리드 거리
double pointDistance(
  const f110_msgs::msg::Wpnt & first,
  const f110_msgs::msg::Wpnt & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

// 펼친 Frenet s 영역에서 d(s)를 잇는 natural cubic spline이다. 호출부에서 평가값을
// 제어점 d의 최솟값/최댓값으로 제한해 연속 cubic 형상은 유지하면서 반대쪽 overshoot를 막는다.
class NaturalCubicSpline
{
public:
  // x는 엄격히 증가해야 한다. 각 구간의 a/b/c/d 계수를 삼대각 시스템으로 계산한다.
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
      // natural boundary(c[0]=c[n-1]=0)를 갖는 삼대각 시스템의 우변
      std::vector<double> alpha(n, 0.0);
      for (std::size_t i = 1; i + 1U < n; ++i) {
        alpha[i] = 3.0 * (a_[i + 1U] - a_[i]) / h[i] -
          3.0 * (a_[i] - a_[i - 1U]) / h[i - 1U];
      }

      std::vector<double> lower(n, 1.0);
      std::vector<double> mu(n, 0.0);
      std::vector<double> z(n, 0.0);

      // 삼대각 Thomas algorithm의 전진 소거
      for (std::size_t i = 1; i + 1U < n; ++i) {
        lower[i] = 2.0 * (x[i + 1U] - x[i - 1U]) - h[i - 1U] * mu[i - 1U];
        if (std::abs(lower[i]) < kEpsilon) {
          return false;
        }
        mu[i] = h[i] / lower[i];
        z[i] = (alpha[i] - h[i - 1U] * z[i - 1U]) / lower[i];
      }

      // 역대입으로 구간별 1·2·3차 계수를 완성한다.
      for (std::size_t reverse = n - 1U; reverse > 0U; --reverse) {
        const std::size_t j = reverse - 1U;
        c_[j] = z[j] - mu[j] * c_[j + 1U];
        b_[j] = (a_[j + 1U] - a_[j]) / h[j] -
          h[j] * (c_[j + 1U] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1U] - c_[j]) / (3.0 * h[j]);
      }
    } else {
      // 제어점 두 개뿐이면 유일한 직선 보간을 사용한다.
      b_[0] = (a_[1] - a_[0]) / h[0];
    }
    return true;
  }

  // 해당 x가 속한 구간을 이진 탐색하고 cubic 다항식을 평가한다.
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
  // x_[i]부터 x_[i+1] 사이에서 a + b·dx + c·dx² + d·dx³ 형태로 저장한다.
  std::vector<double> x_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

}  // namespace

// ego.s를 0으로 펼친 장애물 envelope. raw d는 진단용 원 경계이고 d_right/d_left에는
// 차량 중심 경로가 피해야 할 clearance가 반영되어 있다.
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

// 한 방향과 한 전환 길이에 대해 생성·검증한 spline 후보
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

// 파라미터 복사본을 소유하는 순수 계획 객체. ROS 인터페이스는 포함하지 않는다.
RacelineSplinePlanner::RacelineSplinePlanner(RacelineSplineParameters parameters)
: parameters_(std::move(parameters))
{
}

// 노드가 YAML 선언을 끝낸 후 검증된 설정으로 교체한다.
void RacelineSplinePlanner::setParameters(const RacelineSplineParameters & parameters)
{
  parameters_ = parameters;
}

// ============================================================================
// 글로벌 기준 경로 등록
// ============================================================================
// waypoint 순서를 계획의 기하 불변식으로 사용하므로 유한값과 엄격히 증가하는 s_m을 요구한다.
bool RacelineSplinePlanner::setReference(
  const f110_msgs::msg::WpntArray & reference,
  std::string * error)
{
  auto reject = [&](const std::string & why) {
      // 잘못된 새 reference가 들어오면 이전 경로를 계속 쓰지 않고 명시적으로 준비 해제한다.
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

  // 마지막 점 뒤의 폐곡선 간격은 인접 s 간격의 중앙값으로 추정해 outlier 영향을 줄인다.
  const double median_spacing = spacing[spacing.size() / 2U];
  const double inferred_length = reference.wpnts.back().s_m + median_spacing;
  if (!(inferred_length > reference.wpnts.back().s_m) || !std::isfinite(inferred_length)) {
    return reject("failed to infer a positive closed-track length");
  }

  reference_ = reference;
  track_length_ = inferred_length;
  return true;
}

// 최소 4개의 순서가 검증된 waypoint와 양의 트랙 길이가 있어야 계획 가능하다.
bool RacelineSplinePlanner::ready() const
{
  return reference_.wpnts.size() >= 4U && track_length_ > 0.0;
}

// 추정한 폐곡선 한 바퀴 길이
double RacelineSplinePlanner::trackLength() const
{
  return track_length_;
}

// 폐곡선 Frenet s를 [0, track_length_)로 감싼다.
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

// 트랙 wrap을 건너더라도 양수로 유지되는 전방 거리
double RacelineSplinePlanner::forwardDistance(double from_s, double to_s) const
{
  return wrapS(to_s - from_s);
}

// 현재 ego 앞에서 가장 먼저 레이스 라인을 막는 연결 장애물 군집의 ID만 반환한다.
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

// 주어진 s 이상인 첫 글로벌 waypoint index. 마지막을 넘으면 폐곡선 첫 점으로 돌아간다.
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

// 폐곡선 양방향 거리를 비교해 가장 가까운 글로벌 waypoint index를 선택한다.
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

// ============================================================================
// detector Frenet obstacle을 ego 기준의 충돌 검사용 envelope로 변환
// ============================================================================
std::vector<RacelineSplinePlanner::ExpandedObstacle>
RacelineSplinePlanner::expandVisibleObstacles(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const std::optional<double> & obstacle_clearance) const
{
  const double clearance = obstacle_clearance.value_or(parameters_.obstacle_clearance_m);
  std::vector<ExpandedObstacle> visible;
  visible.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    // 잘못된 한 물체 때문에 전체 계획을 중단하지 않고 해당 물체만 제외한다.
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

    // 경로는 차량 중심 궤적이므로 detector 경계 양쪽에 요구 clearance를 직접 더한다.
    expanded.d_right = expanded.raw_d_right - clearance;
    expanded.d_left = expanded.raw_d_left + clearance;
    expanded.clearance = clearance;
    if (expanded.end >= 0.0 &&
      expanded.start <= parameters_.detection_lookahead_m)
    {
      visible.push_back(expanded);
    }
  }

  // 가장 가까운 앞면부터 검사해야 첫 blocking cluster를 안정적으로 선택할 수 있다.
  std::sort(
    visible.begin(), visible.end(),
    [](const ExpandedObstacle & first, const ExpandedObstacle & second) {
      return first.start < second.start;
    });
  return visible;
}

// 원 장애물 경계가 race line 중심의 차량 폭 + blocking margin을 덮으면 blocking으로 본다.
bool RacelineSplinePlanner::isBlockingRaceline(const ExpandedObstacle & obstacle) const
{
  const double envelope = parameters_.vehicle_half_width_m + parameters_.blocking_margin_m;
  return obstacle.raw_d_right <= envelope && obstacle.raw_d_left >= -envelope;
}

// 첫 blocking obstacle부터 종방향 gap이 설정값 이하로 이어지는 물체들을 한 maneuver로 묶는다.
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

// 장애물 구간 주변 2 m의 signed curvature 합으로 코너 바깥쪽을 판단한다.
// 왼쪽 회전(양의 곡률)의 바깥은 오른쪽, 오른쪽 회전(음의 곡률)의 바깥은 왼쪽이다.
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

// 군집 전체의 좌/우 바깥 경계에 clearance reserve를 더해 목표 d를 계산한다.
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
    // 좌측은 양의 d, 우측은 음의 d이며 최소 회피 offset도 보장한다.
    target_d = std::max(
      target_d + parameters_.commitment_clearance_reserve_m,
      parameters_.minimum_target_offset_m);
  } else {
    target_d = std::min(
      target_d - parameters_.commitment_clearance_reserve_m,
      -parameters_.minimum_target_offset_m);
  }
  if (std::abs(target_d) > parameters_.maximum_target_offset_m) {
    reason = "required d-offset exceeds maximum_target_offset_m";
    return false;
  }
  return true;
}

// 장애물 span 동안 목표 d를 유지할 공간이 waypoint별 트랙 폭 안에 있는지 빠르게 검사한다.
bool RacelineSplinePlanner::targetFitsTrackBounds(
  const EgoFrenetState & ego,
  double cluster_start,
  double cluster_end,
  bool go_left,
  double target_d,
  std::string & reason) const
{
  const double center_boundary_clearance =
    parameters_.vehicle_half_width_m + parameters_.boundary_margin_m;
  const auto fits_at = [&](const f110_msgs::msg::Wpnt & reference) {
      const double left_width = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right_width = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      return go_left ?
             target_d <= left_width - center_boundary_clearance + kEpsilon :
             target_d >= -right_width + center_boundary_clearance - kEpsilon;
    };

  // 목표 offset은 팽창된 장애물 군집 span 전체에서 유지된다. 명백히 불가능한 방향은 여러
  // spline을 fitting/sampling하기 전에 제거한다. 단, 전환 구간의 더 좁은 벽은 이 검사로
  // 잡히지 않을 수 있으므로 살아남은 후보도 전체 경로 검증을 반드시 수행한다.
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

  // 매우 짧은 장애물 span이 두 글로벌 표본 사이에 있으면 midpoint에서 가장 가까운 기준 폭을
  // 검사한다. Cartesian 벽을 새로 추정하지 않으면서 빠른 gate의 누락을 막는다.
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
  // state_machine이 GLOBAL 전환을 확인할 때까지 비지 않은 d=0 한 바퀴를 제공한다.
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

// Frenet 위치는 알지만 정상 경로를 만들 수 없을 때 현재 d를 유지하는 zero-speed hold를 만든다.
// 최소 점 개수를 채워 downstream이 빈 경로로 해석하지 않게 한다.
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

// ============================================================================
// 활성 commitment 위에서 만드는 안전 정지 prefix
// ============================================================================
// 장애물 때문에 기존 경로가 무효화되어도 즉시 d=0으로 돌아가지 않는다. ego 앞의 committed
// waypoint부터 첫 충돌 전 safe_stop_buffer까지 같은 기하를 잘라 역방향 감속 프로파일을 만든다.
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

  // commitment 배열 중 현재 ego에서 가장 가까운 전방 waypoint를 시작점으로 찾는다.
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

  // 남은 commitment를 따라가며 가장 먼저 충돌하는 waypoint와 장애물 ID를 찾는다.
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

  // 충돌 지점보다 buffer만큼 앞까지만 기존 기하를 복사한다.
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

  // 첫 waypoint가 현재 ego.d와 불연속이면 차량이 순간 횡이동해야 하므로 이 prefix를 버린다.
  if (same_s_lateral_jump || excessive_entry_slope) {
    result.path.wpnts.clear();
    result.reason = "committed braking prefix is discontinuous from the current ego d";
    return result;
  }

  result.path.wpnts.back().vx_mps = 0.0;

  // v_prev² = v_next² + 2·a·Δx를 뒤에서 앞으로 적용해 기존 속도보다 높이지 않는 감속
  // 프로파일을 만든다.
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

  // 잘라낸 prefix도 트랙·장애물·기하 검사를 다시 통과해야 발행할 수 있다.
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

// ============================================================================
// 한 방향/전환 길이의 cubic d-offset 후보 생성
// ============================================================================
RacelineSplinePlanner::Candidate RacelineSplinePlanner::buildCandidate(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  bool go_left,
  double transition_scale,
  bool outside_is_left,
  double cluster_start,
  double cluster_end,
  double target_d) const
{
  Candidate candidate;
  candidate.go_left = go_left;

  // 코너 바깥쪽은 안쪽보다 전환 거리를 늘려 횡기울기와 곡률 변화를 완만하게 만든다.
  if (go_left == outside_is_left) {
    transition_scale *= parameters_.outside_line_transition_scale;
  }

  std::vector<double> knot_s;
  std::vector<double> knot_d;

  // 너무 가까운 s 제어점은 하나로 합쳐 spline 입력의 엄격한 증가 조건을 보장한다.
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

  // [현재 d 유지 3점] → [장애물 구간 target d] → [global d=0 복귀 3점]
  append_knot(cluster_start - pre_far, ego.d);
  append_knot(cluster_start - pre_middle, ego.d);
  append_knot(cluster_start - pre_near, ego.d);
  append_knot(cluster_start, target_d);
  if (cluster_end > cluster_start + 0.05) {
    append_knot(cluster_end, target_d);
  }
  append_knot(cluster_end + post_near, 0.0);
  append_knot(cluster_end + post_middle, 0.0);
  append_knot(cluster_end + post_far, 0.0);

  // 첫 maneuver는 가능한 기존 pre 점을 살리고, 연속 maneuver처럼 ego.d가 0이 아니면
  // 반드시 (forward_s=0, ego.d)에서 시작해 현재 차량 위치와 연속되게 한다.
  if (std::abs(ego.d) <= kEpsilon) {
    if (knot_s.front() > 0.05) {
      knot_s.insert(knot_s.begin(), 0.0);
      knot_d.insert(knot_d.begin(), ego.d);
    } else {
      knot_s.front() = std::min(knot_s.front(), 0.0);
    }
  } else {
    std::vector<double> forward_knot_s;
    std::vector<double> forward_knot_d;
    forward_knot_s.reserve(knot_s.size() + 1U);
    forward_knot_d.reserve(knot_d.size() + 1U);
    forward_knot_s.push_back(0.0);
    forward_knot_d.push_back(ego.d);
    for (std::size_t i = 0; i < knot_s.size(); ++i) {
      if (knot_s[i] > 1.0e-3) {
        forward_knot_s.push_back(knot_s[i]);
        forward_knot_d.push_back(knot_d[i]);
      }
    }
    knot_s = std::move(forward_knot_s);
    knot_d = std::move(forward_knot_d);
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

  // 글로벌 waypoint의 s/order/속도 프로파일은 유지하고 d와 map x/y만 바꾼다.
  for (std::size_t k = 0; k < reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % reference_.wpnts.size();
    const auto & global = reference_.wpnts[index];
    const double forward_s = forwardDistance(ego.s, global.s_m);
    if (forward_s > path_end + kEpsilon) {
      break;
    }
    auto waypoint = global;
    waypoint.id = static_cast<int32_t>(candidate.path.wpnts.size());

    // cubic overshoot가 반대쪽으로 튀지 않도록 [0, ego.d, target_d]의 min/max로 제한한다.
    waypoint.d_m = clamp(spline.evaluate(forward_s), clip_min, clip_max);
    waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
    candidate.path.wpnts.push_back(waypoint);
  }
  if (candidate.path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
    candidate.reason = "spline segment has too few global race-line samples";
    return candidate;
  }

  updateGeometryAndAcceleration(candidate.path);

  // 생성된 모든 표본이 트랙·장애물·기하 제약을 만족해야 후보를 유효 처리한다.
  if (!validateCandidate(ego, candidate.path, visible, candidate.reason)) {
    return candidate;
  }

  candidate.valid = true;
  candidate.target_d = target_d;
  // merge_s는 발행 세그먼트 끝이 아니라 d-offset spline이 실제로 d=0에 복귀하는 지점이다.
  // tail 길이를 늘려도 합류 완료 판정 시점이 뒤로 밀리지 않아야 한다.
  candidate.merge_s = wrapS(ego.s + spline_end);
  candidate.score = std::abs(target_d) + 0.02 * transition_scale;
  return candidate;
}

// d 이동으로 map 좌표가 바뀐 뒤 heading, signed curvature, longitudinal acceleration을
// 순서대로 다시 계산한다. 속도 자체는 글로벌 프로파일을 보존한다.
void RacelineSplinePlanner::updateGeometryAndAcceleration(
  f110_msgs::msg::WpntArray & path) const
{
  auto & waypoints = path.wpnts;
  if (waypoints.size() < 2U) {
    return;
  }

  // 양끝에서는 단방향 차분, 내부에서는 중앙 차분으로 heading을 계산한다.
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const std::size_t previous = (i == 0U) ? 0U : i - 1U;
    const std::size_t next = std::min(i + 1U, waypoints.size() - 1U);
    const double dx = waypoints[next].x_m - waypoints[previous].x_m;
    const double dy = waypoints[next].y_m - waypoints[previous].y_m;
    if (std::hypot(dx, dy) > kEpsilon) {
      waypoints[i].psi_rad = std::atan2(dy, dx);
    }
  }

  // 세 점의 외적과 변 길이 곱으로 signed curvature κ = 2·cross/(a·b·c)를 계산한다.
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

  // v_next² = v_prev² + 2·a·Δx 관계로 각 구간 종가속도를 재계산한다.
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

// ============================================================================
// 표본화된 경로의 통합 안전 검증
// ============================================================================
// 실패 시 PathValidationFailure에 정확한 원인과 waypoint/obstacle 경계를 채워 상위 노드가
// hard/soft collision을 구분하고 재현 가능한 로그를 남길 수 있게 한다.
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

  // ego 앞에 충분한 waypoint가 남았는지 먼저 확인한다.
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
  const double center_boundary_clearance =
    parameters_.vehicle_half_width_m + parameters_.boundary_margin_m;
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

    // waypoint별 좌/우 트랙 폭에서 차량 반폭과 boundary margin을 뺀 중심 허용 범위
    if (waypoint.d_m > left_width - center_boundary_clearance + 1.0e-6 ||
      waypoint.d_m < -right_width + center_boundary_clearance - 1.0e-6)
    {
      return reject(
        PathValidationFailureKind::kTrackBoundary,
        "d-offset leaves the global waypoint track bounds", i, &waypoint);
    }

    // 지정된 collision horizon 안에서 종/횡방향이 동시에 겹치면 장애물 충돌이다.
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

      // s 순서 보존은 이 planner의 핵심 불변식이며 branch jump를 원천 차단한다.
      if (!(ds > kEpsilon)) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "candidate no longer follows increasing global race-line order", i, &waypoint);
      }

      // |Δd/Δs|가 너무 크면 조향이 급격하므로 경로를 거부한다.
      const double slope = std::abs(waypoint.d_m - previous_d) / ds;
      if (slope > parameters_.maximum_lateral_slope) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "cubic d-offset exceeds maximum_lateral_slope", i, &waypoint);
      }

      // 곡률 변화율은 steering rate에 대응하는 기하 제한이다.
      const double curvature_rate =
        std::abs(waypoint.kappa_radpm - previous_curvature) / ds;
      if (curvature_rate > parameters_.maximum_curvature_rate_radpm2) {
        return reject(
          PathValidationFailureKind::kGeometry,
          "shifted race line exceeds maximum_curvature_rate_radpm2", i, &waypoint);
      }
    }

    // 점 자체의 절대 곡률도 차량의 최대 조향 가능 범위 안이어야 한다.
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

// 이미 발행한 경로를 현재 ego에서 남은 부분만 재검증하는 공개 함수다.
// obstacle_clearance로 hard/soft 검사용 팽창량을 바꾸고, maximum_collision_forward_m으로
// 현재 spline merge 뒤의 controller tail을 충돌 검사에서 제외할 수 있다.
bool RacelineSplinePlanner::validatePath(
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::string * error,
  PathValidationFailure * failure,
  const std::optional<double> & obstacle_clearance,
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
  if (obstacle_clearance.has_value() &&
    (!std::isfinite(obstacle_clearance.value()) || obstacle_clearance.value() < 0.0))
  {
    return reject(
      PathValidationFailureKind::kInput,
      "obstacle clearance override is invalid");
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

  // 폐곡선 wrap을 고려해 ego에서 가장 가까운 전방 waypoint를 찾는다.
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

  const auto visible = expandVisibleObstacles(ego, obstacles, obstacle_clearance);
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

// 좌우 회피 후보가 모두 실패했을 때 현재 ego.d를 유지하며 장애물 앞에 정지하는 경로를 만든다.
RacelineSplineResult RacelineSplinePlanner::buildSafeStop(
  const EgoFrenetState & ego,
  const std::vector<ExpandedObstacle> & visible,
  const ExpandedObstacle & blocking) const
{
  RacelineSplineResult result;
  result.kind = SplinePlanKind::kNoSafePath;
  result.obstacle_id = blocking.id;

  // 장애물 팽창 앞면보다 safe_stop_buffer_m만큼 앞을 목표 정지점으로 둔다.
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

    // v ≤ sqrt(2·a·남은거리)로 제한해 정지점에서 속도가 0이 되는 감속 상한을 만든다.
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

  // 정지 prefix 자체도 장애물/트랙/기하 검증을 통과해야 한다.
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

// 최초 blocking cluster를 여러 메시지 동안 관측하는 동안 사용할 준비 감속 경로를 만든다.
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

  // 준비 단계에서 군집의 모든 ID를 알려 상위 노드가 각 ID의 관측 횟수를 따로 셀 수 있게 한다.
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

// ============================================================================
// 좌/우 회피 계획의 최상위 함수
// ============================================================================
// 가장 가까운 blocking cluster에 대해 목표 d와 여러 전환 길이를 평가한다. preferred_left가
// 있으면 commitment 방향을 먼저 검사하고, allow_side_switch가 true인 진입 전 단계에서만
// 반대편 fallback을 허용한다.
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

  // 한 방향의 목표 d를 계산하고 짧은 전환부터 유효한 첫 spline을 찾는다.
  auto evaluate_side = [&](bool go_left) {
      Candidate last;
      last.go_left = go_left;
      double cluster_start = 0.0;
      double cluster_end = 0.0;
      double target_d = 0.0;
      if (!computeSideTarget(
          cluster, go_left, cluster_start, cluster_end, target_d, last.reason))
      {
        return last;
      }
      if (!targetFitsTrackBounds(
          ego, cluster_start, cluster_end, go_left, target_d, last.reason))
      {
        return last;
      }
      // transition_distance_scales는 엄격한 오름차순으로 검증됐다. 따라서 첫 유효 후보가
      // 최소 score이며 이후의 더 긴 spline은 평가할 필요가 없다.
      for (const double scale : parameters_.transition_distance_scales) {
        last = buildCandidate(
          ego, visible, go_left, scale, outside_is_left,
          cluster_start, cluster_end, target_d);
        if (last.valid) {
          break;
        }
      }
      return last;
    };

  Candidate left;
  Candidate right;
  bool left_evaluated = false;
  bool right_evaluated = false;
  Candidate selected;

  // 기존 commitment 방향이 있으면 그쪽을 우선하고, 방향 전환 허용 시에만 반대쪽을 검사한다.
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
    // 새 maneuver는 양쪽을 모두 평가해 더 작은 offset/짧은 전환 score를 선택한다.
    left = evaluate_side(true);
    right = evaluate_side(false);
    left_evaluated = true;
    right_evaluated = true;
    if (left.valid && right.valid) {
      selected = left.score <= right.score ? std::move(left) : std::move(right);
    } else if (left.valid) {
      selected = std::move(left);
    } else if (right.valid) {
      selected = std::move(right);
    }
  }

  if (!selected.valid) {
    // 어느 쪽도 안전하지 않으면 실패 이유를 보존한 검증된 감속 경로를 반환한다.
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

  // commitment/Guard 동결에 사용할 군집 전체 ID를 결과에 함께 싣는다.
  result.obstacle_ids.reserve(cluster.size());
  for (const auto & obstacle : cluster) {
    result.obstacle_ids.push_back(obstacle.id);
  }
  result.control_points = std::move(selected.control_points);
  result.reason = "global race-line waypoints shifted by a local cubic d-offset";
  return result;
}

// ============================================================================
// Frenet → map 좌표 보간
// ============================================================================
// s를 감싼 뒤 인접 글로벌 waypoint의 위치와 heading을 선형 보간하고, 기준 heading의 왼쪽
// 법선 방향으로 d만큼 이동한다. 이 변환도 글로벌 순서를 바꾸지 않는다.
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

  // wrapped s를 포함하는 기준 구간 [first, second]를 찾는다.
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

  // ±π 경계를 가로지르는 heading도 최단 각도 차로 보간한다.
  yaw = a.psi_rad + fraction * normalizeAngle(b.psi_rad - a.psi_rad);
  x = base_x - d * std::sin(yaw);
  y = base_y + d * std::cos(yaw);
}

}  // namespace local_planning
