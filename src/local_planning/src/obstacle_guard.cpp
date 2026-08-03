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

#include "local_planning/obstacle_guard.hpp"

#include <algorithm>
#include <cmath>

namespace local_planning
{
namespace
{

// 부동소수점 0 판정과 퇴화 구간 방지용 허용 오차
constexpr double kEpsilon = 1.0e-9;

// 폐곡선 Frenet s를 [0, track_length) 범위로 정규화한다.
double wrapS(double s, double track_length)
{
  if (!(track_length > kEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

// wrap을 건너더라도 항상 양수인 전방 진행 거리를 반환한다.
double forwardDistance(double from_s, double to_s, double track_length)
{
  return wrapS(to_s - from_s, track_length);
}

// s_start/s_end의 방향이 wrap 때문에 뒤집혀 보여도 더 짧은 쪽을 실제 장애물 길이로 본다.
double shortestSpan(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  const double forward = forwardDistance(obstacle.s_start, obstacle.s_end, track_length);
  const double reverse = forwardDistance(obstacle.s_end, obstacle.s_start, track_length);
  const double span = std::min(forward, reverse);
  if (std::isfinite(span) && span > kEpsilon) {
    return span;
  }
  return std::max(0.0, std::abs(obstacle.size));
}

// 잘못된 분산은 확장량을 키우지 않고, 유효한 분산만 표준편차로 변환한다.
double positionSigma(double variance)
{
  if (!std::isfinite(variance) || variance <= 0.0) {
    return 0.0;
  }
  return std::sqrt(variance);
}

}  // namespace

f110_msgs::msg::Obstacle buildUncertaintyGuard(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length,
  const ObstacleGuardParameters & parameters)
{
  auto guard = obstacle;

  // 종/횡방향 각각에 고정 센서 크기 오차와 Kalman 중심 위치의 kσ를 더한다.
  const double longitudinal_margin =
    parameters.minimum_longitudinal_margin_m +
    parameters.uncertainty_sigma_scale * positionSigma(obstacle.s_var);
  const double lateral_margin =
    parameters.minimum_lateral_margin_m +
    parameters.uncertainty_sigma_scale * positionSigma(obstacle.d_var);

  // 중심을 유지한 채 장애물의 전후 경계를 대칭으로 확장한다.
  const double half_span = 0.5 * shortestSpan(obstacle, track_length) + longitudinal_margin;
  guard.s_start = wrapS(obstacle.s_center - half_span, track_length);
  guard.s_end = wrapS(obstacle.s_center + half_span, track_length);

  // detector가 경계 순서를 바르게 제공하더라도 min/max로 방어적으로 정규화한다.
  const double raw_right = std::min(obstacle.d_right, obstacle.d_left);
  const double raw_left = std::max(obstacle.d_right, obstacle.d_left);
  guard.d_right = raw_right - lateral_margin;
  guard.d_left = raw_left + lateral_margin;
  guard.size = std::hypot(2.0 * half_span, guard.d_left - guard.d_right);
  return guard;
}

bool obstacleEnvelopeContained(
  const f110_msgs::msg::Obstacle & candidate,
  const f110_msgs::msg::Obstacle & guard,
  double track_length,
  double tolerance_m)
{
  // 유효하지 않은 track 길이 또는 비정상 경계로 포함 판정을 낙관하지 않는다.
  if (!(track_length > kEpsilon) ||
    !std::isfinite(candidate.s_center) || !std::isfinite(guard.s_center) ||
    !std::isfinite(candidate.d_right) || !std::isfinite(candidate.d_left) ||
    !std::isfinite(guard.d_right) || !std::isfinite(guard.d_left))
  {
    return false;
  }

  // 후보 중심을 Guard 중심 기준의 부호 있는 최단 거리로 바꿔 wrap 경계를 펼친다.
  double center_delta = forwardDistance(guard.s_center, candidate.s_center, track_length);
  if (center_delta > 0.5 * track_length) {
    center_delta -= track_length;
  }
  const double candidate_half_span = 0.5 * shortestSpan(candidate, track_length);
  const double guard_half_span = 0.5 * shortestSpan(guard, track_length);

  // 후보의 종방향 양 끝이 모두 동결 Guard의 양 끝 안에 있어야 한다.
  const bool longitudinally_contained =
    center_delta - candidate_half_span >= -guard_half_span - tolerance_m &&
    center_delta + candidate_half_span <= guard_half_span + tolerance_m;

  const double candidate_right = std::min(candidate.d_right, candidate.d_left);
  const double candidate_left = std::max(candidate.d_right, candidate.d_left);
  const double guard_right = std::min(guard.d_right, guard.d_left);
  const double guard_left = std::max(guard.d_right, guard.d_left);

  // 횡방향도 오른쪽/왼쪽 경계가 모두 Guard 내부인지 확인한다.
  const bool laterally_contained =
    candidate_right >= guard_right - tolerance_m &&
    candidate_left <= guard_left + tolerance_m;
  return longitudinally_contained && laterally_contained;
}

}  // namespace local_planning
