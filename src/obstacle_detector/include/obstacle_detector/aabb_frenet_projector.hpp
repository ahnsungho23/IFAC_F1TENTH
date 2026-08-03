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

#ifndef OBSTACLE_DETECTOR__AABB_FRENET_PROJECTOR_HPP_
#define OBSTACLE_DETECTOR__AABB_FRENET_PROJECTOR_HPP_

#include <optional>

#include "global_planning/clcs_frenet_converter.hpp"

namespace obstacle_detector
{

// Cartesian AABB를 raceline 중심의 Frenet 경계로 투영한 결과이다.
struct FrenetAabbBounds
{
  double x_center{0.0};                 // map 좌표 AABB 중심 x
  double y_center{0.0};                 // map 좌표 AABB 중심 y
  double s_center{0.0};                 // AABB 중심의 폐루프 Frenet s
  double d_center{0.0};                 // AABB 중심의 부호 있는 Frenet d
  double s_start{0.0};                  // 종방향 경계 시작점
  double s_end{0.0};                    // 종방향 경계 끝점
  double d_right{0.0};                  // raceline 기준 오른쪽 경계
  double d_left{0.0};                   // raceline 기준 왼쪽 경계
  double closest_abs_d{0.0};            // 같은 경로 가지에서 raceline과 AABB 사이 최소 거리
  double diagonal{0.0};                 // Cartesian AABB 대각선 길이
  double longitudinal_half_extent{0.0}; // 중심 국소 접선에 투영한 종방향 반길이
};

// AABB 중심으로 CLCS 경로 가지를 고정한 뒤 네 모서리와 실제 곡선의 최근접 면을 함께
// 계산한다. 입력이 비정상적이거나 면적이 없는 상자이면 std::nullopt를 반환한다.
std::optional<FrenetAabbBounds> projectCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  double x_min,
  double x_max,
  double y_min,
  double y_max);

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__AABB_FRENET_PROJECTOR_HPP_
