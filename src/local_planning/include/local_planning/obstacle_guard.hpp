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

// detector 분산과 AABB 크기 오차를 Frenet 안전 envelope에 반영하는 설정
struct ObstacleGuardParameters
{
  double uncertainty_sigma_scale{3.0};
  double minimum_longitudinal_margin_m{0.05};
  double minimum_lateral_margin_m{0.03};
};

// 투영된 Frenet AABB를 고정 크기 오차 하한 + Kalman 중심 위치의 kσ만큼 확장한다.
// Cartesian 필드는 RViz/진단에 쓸 원 측정 AABB이므로 변경하지 않는다.
f110_msgs::msg::Obstacle buildUncertaintyGuard(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length,
  const ObstacleGuardParameters & parameters);

// 후보 Frenet envelope 전체가 commitment 시점에 동결한 Guard 안에 있을 때만 true를 반환한다.
// 종방향 포함 검사는 폐곡선의 s=0 wrap을 고려한다.
bool obstacleEnvelopeContained(
  const f110_msgs::msg::Obstacle & candidate,
  const f110_msgs::msg::Obstacle & guard,
  double track_length,
  double tolerance_m = 1.0e-6);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__OBSTACLE_GUARD_HPP_
