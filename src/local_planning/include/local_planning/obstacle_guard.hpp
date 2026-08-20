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

#ifndef LOCAL_PLANNING__OBSTACLE_GUARD_HPP_
#define LOCAL_PLANNING__OBSTACLE_GUARD_HPP_

#include <f110_msgs/msg/obstacle.hpp>

namespace local_planning
{

struct ObstacleGuardParameters
{
  double uncertainty_sigma_scale{3.0};
  double minimum_longitudinal_inflation_m{0.05};
  // Keep detector-owned lateral AABB bounds unchanged by default. Non-zero values are retained
  // only for explicit experiments; normal planning adds no covariance-based lateral extent.
  double minimum_lateral_inflation_m{0.0};
  double maximum_lateral_inflation_m{0.0};
  // Floor used once a face has enough of its own observations to speak for itself. It only has to
  // cover quantisation, not the worst face on the track, so it is far below
  // minimum_lateral_inflation_m. See ObstacleFaceUncertainty for why the two floors differ.
  double measured_lateral_inflation_floor_m{0.0};
};

// Observed standard deviation of each lateral face over a recent window of same-ID detections.
//
// 왜 면별인가 (2026-08-16 14:30 백 실측):
//   장애물 2 — 라인 쪽 면 σ=0.007 m, 반대쪽 면 σ=0.044 m (6배)
//   장애물 0 — 라인 쪽 면 σ=0.051 m, 반대쪽 면 σ=0.114 m
// 그런데 Obstacle::d_var는 박스 *중심*의 분산 하나뿐이고(위 두 장애물 모두 σ≈0.012),
// 어느 면도 대표하지 못한다. 그 하나를 양면에 대칭으로 물리면 정확히 측정된 면에서도
// 같은 마진을 걷어가고, 통과 가능한 간격이 계획 불가가 된다. 실제로 그날 여유 0.84 m
// 짜리 간격(필요 0.64 m)에서 차가 영구 정지해 사람이 꺼내야 했다.
//
// 음수는 "이 면에 대한 이력이 아직 부족하다"는 뜻이며, 그때는 종전의 중심 분산 경로를
// 그대로 쓴다 — 측정이 없을 때 상수 마진은 은폐가 아니라 정직한 사전분포다.
struct ObstacleFaceUncertainty
{
  double sigma_right_m{-1.0};
  double sigma_left_m{-1.0};
};

// Expand a projected Frenet AABB by configured fixed floors plus k standard deviations.
//
// Longitudinal extent always uses the Kalman centre variance. Each lateral face uses its own
// observed standard deviation when `faces` supplies one, and falls back to the centre variance
// with the larger fixed floor when it does not.
f110_msgs::msg::Obstacle buildUncertaintyGuard(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length,
  const ObstacleGuardParameters & parameters,
  const ObstacleFaceUncertainty & faces = ObstacleFaceUncertainty{});

// Return true only when the complete candidate Frenet envelope is contained in the frozen guard.
// Longitudinal containment is closed-track/wrap aware.
bool obstacleEnvelopeContained(
  const f110_msgs::msg::Obstacle & candidate,
  const f110_msgs::msg::Obstacle & guard,
  double track_length,
  double tolerance_m = 1.0e-6);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__OBSTACLE_GUARD_HPP_
