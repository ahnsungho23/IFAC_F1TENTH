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

#include <gtest/gtest.h>

#include <limits>

#include "local_planning/obstacle_guard.hpp"

namespace local_planning
{
namespace
{

f110_msgs::msg::Obstacle makeObstacle(
  double s_center,
  double s_start,
  double s_end,
  double d_right = -0.2,
  double d_left = 0.2)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = 7;
  obstacle.s_center = s_center;
  obstacle.s_start = s_start;
  obstacle.s_end = s_end;
  obstacle.d_center = 0.5 * (d_right + d_left);
  obstacle.d_right = d_right;
  obstacle.d_left = d_left;
  obstacle.size = 0.4;
  return obstacle;
}

// 2026-08-16 14:30 백 회귀. 가드가 상수 대신 면별 실측 σ를 쓰는지 고정한다.
// 그날의 실해: 라인 쪽 면 σ=0.007, 반대쪽 면 σ=0.044인데 중심 분산 하나(σ≈0.012)를
// 양면에 대칭 적용해 조용한 면에서도 0.14를 걷어갔고, 여유 0.84 m 간격이 계획 불가가
// 되어 차가 영구 정지했다.
TEST(ObstacleGuard, QuietFaceKeepsItsCorridorWhileNoisyFaceStaysProtected)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2, -0.28, 0.33);
  obstacle.s_var = 0.0;
  obstacle.d_var = 0.000144;             // 중심 σ = 0.012 — 어느 면도 대표하지 못한다

  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 3.0;
  parameters.minimum_longitudinal_inflation_m = 0.0;
  parameters.minimum_lateral_inflation_m = 0.10;
  parameters.maximum_lateral_inflation_m = 0.15;
  parameters.measured_lateral_inflation_floor_m = 0.02;

  ObstacleFaceUncertainty faces;
  faces.sigma_right_m = 0.007;           // 라인 쪽: 정확한 curve-to-AABB 거리
  faces.sigma_left_m = 0.044;            // 반대쪽: 가려진 면
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters, faces);

  // 조용한 면은 자기 3σ(0.021)만큼만 부푼다 — 상수 0.10에 끌려가지 않는다.
  EXPECT_NEAR(guard.d_right, -0.28 - 0.021, 1.0e-9);
  // 잡음이 큰 면은 3σ=0.132로 종전과 같은 수준의 보호를 유지한다.
  EXPECT_NEAR(guard.d_left, 0.33 + 0.132, 1.0e-9);
  // 종전 상수 경로와 대조: 조용한 면이 되찾은 폭이 곧 회랑에 돌려준 폭이다.
  // (통과 가능 여부 자체는 곡률·기울기·이산 후보까지 보는 플래너가 정하므로 여기서
  //  산술로 단정하지 않는다 — 그 회귀는 test_raceline_spline.cpp에 있다.)
  ObstacleFaceUncertainty no_history;
  const auto constant_guard =
    buildUncertaintyGuard(obstacle, 100.0, parameters, no_history);
  EXPECT_GT(guard.d_right - constant_guard.d_right, 0.10)
    << "조용한 면이 상수 경로보다 회랑을 되찾지 못했다";
  // 잡음이 큰 면은 실측 3σ=0.132로, 종전 상수 0.136과 4 mm 차이다. 상수 쪽이 더 컸던 것은
  // 근거가 있어서가 아니라 바닥이 0.10이었기 때문이므로, 0.132가 그 면의 정직한 값이다.
  // 요구하는 성질은 "실질적으로 약해지지 않을 것"이며, 1 cm를 그 기준으로 둔다.
  EXPECT_GT(guard.d_left, constant_guard.d_left - 0.01)
    << "잡음이 큰 면의 보호가 실질적으로 약해졌다";
}

TEST(ObstacleGuard, VeryNoisyFaceIsStillCappedByTheMaximum)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2, -0.20, 0.20);
  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 3.0;
  parameters.minimum_lateral_inflation_m = 0.10;
  parameters.maximum_lateral_inflation_m = 0.15;
  parameters.measured_lateral_inflation_floor_m = 0.02;

  ObstacleFaceUncertainty faces;
  faces.sigma_right_m = 0.114;           // 3σ = 0.34 → 상한에서 멈춰야 한다
  faces.sigma_left_m = 0.114;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters, faces);
  EXPECT_NEAR(guard.d_right, -0.20 - 0.15, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.20 + 0.15, 1.0e-9);
}

TEST(ObstacleGuard, FaceWithoutHistoryFallsBackToTheConstantPrior)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2, -0.20, 0.20);
  obstacle.d_var = 0.0001;               // 중심 σ = 0.01 → 3σ = 0.03
  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 3.0;
  parameters.minimum_lateral_inflation_m = 0.10;
  parameters.maximum_lateral_inflation_m = 0.15;
  parameters.measured_lateral_inflation_floor_m = 0.02;

  ObstacleFaceUncertainty faces;                     // 양면 모두 이력 없음(-1)
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters, faces);
  EXPECT_NEAR(guard.d_right, -0.20 - 0.13, 1.0e-9);  // 0.10 + 3σ
  EXPECT_NEAR(guard.d_left, 0.20 + 0.13, 1.0e-9);

  // 한쪽만 이력이 있으면 그쪽만 실측으로 풀린다.
  faces.sigma_right_m = 0.005;
  const auto mixed = buildUncertaintyGuard(obstacle, 100.0, parameters, faces);
  EXPECT_NEAR(mixed.d_right, -0.20 - 0.02, 1.0e-9);  // floor
  EXPECT_NEAR(mixed.d_left, 0.20 + 0.13, 1.0e-9);    // 여전히 상수 경로
}

TEST(ObstacleGuard, AddsFixedInflationAndKalmanStandardDeviation)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = 0.01;
  obstacle.d_var = 0.0025;

  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 3.0;
  parameters.minimum_longitudinal_inflation_m = 0.05;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 1.0;  // test the uncapped inflation formula
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.s_start, 9.45, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.55, 1.0e-9);
  EXPECT_NEAR(guard.d_right, -0.38, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.38, 1.0e-9);
}

TEST(ObstacleGuard, KeepsSmallSameIdMotionInsideFrozenGuard)
{
  auto initial = makeObstacle(10.0, 9.8, 10.2);
  initial.s_var = 0.0004;
  initial.d_var = 0.0001;
  ObstacleGuardParameters parameters;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 0.15;
  const auto frozen_guard = buildUncertaintyGuard(initial, 100.0, parameters);

  auto shifted = makeObstacle(10.04, 9.84, 10.24, -0.18, 0.22);
  const auto shifted_envelope = buildUncertaintyGuard(shifted, 100.0, parameters);
  EXPECT_TRUE(obstacleEnvelopeContained(shifted_envelope, frozen_guard, 100.0));

  auto accumulated_shift = makeObstacle(10.10, 9.90, 10.30, -0.18, 0.22);
  const auto breached_envelope =
    buildUncertaintyGuard(accumulated_shift, 100.0, parameters);
  EXPECT_FALSE(obstacleEnvelopeContained(breached_envelope, frozen_guard, 100.0));
}

TEST(ObstacleGuard, HandlesClosedTrackWrap)
{
  ObstacleGuardParameters parameters;
  parameters.uncertainty_sigma_scale = 0.0;
  parameters.minimum_longitudinal_inflation_m = 0.10;
  parameters.minimum_lateral_inflation_m = 0.05;
  parameters.maximum_lateral_inflation_m = 0.15;

  const auto initial = makeObstacle(0.05, 99.85, 0.25);
  const auto frozen_guard = buildUncertaintyGuard(initial, 100.0, parameters);
  EXPECT_NEAR(frozen_guard.s_start, 99.75, 1.0e-9);
  EXPECT_NEAR(frozen_guard.s_end, 0.35, 1.0e-9);

  const auto shifted = makeObstacle(0.08, 99.90, 0.24);
  const auto shifted_envelope = buildUncertaintyGuard(shifted, 100.0, parameters);
  EXPECT_TRUE(obstacleEnvelopeContained(shifted_envelope, frozen_guard, 100.0));
}

TEST(ObstacleGuard, FallsBackToFixedInflationForInvalidVariance)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = std::numeric_limits<double>::quiet_NaN();
  obstacle.d_var = -1.0;
  ObstacleGuardParameters parameters;
  parameters.minimum_lateral_inflation_m = 0.03;
  parameters.maximum_lateral_inflation_m = 0.15;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.s_start, 9.75, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.25, 1.0e-9);
  EXPECT_NEAR(guard.d_right, -0.23, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.23, 1.0e-9);
}

TEST(ObstacleGuard, CapsLateralInflationAtConfiguredMaximum)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2);
  obstacle.s_var = 0.01;
  obstacle.d_var = 4.0;

  ObstacleGuardParameters parameters;
  parameters.maximum_lateral_inflation_m = 0.20;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  // 3 * sqrt(4.0) = 6 m of lateral inflation is capped at
  // maximum_lateral_inflation_m (0.20);
  // the longitudinal inflation stays uncapped.
  EXPECT_NEAR(guard.d_right, -0.40, 1.0e-9);
  EXPECT_NEAR(guard.d_left, 0.40, 1.0e-9);
  EXPECT_NEAR(guard.s_start, 9.45, 1.0e-9);
  EXPECT_NEAR(guard.s_end, 10.55, 1.0e-9);
}

TEST(ObstacleGuard, KeepsRawLateralBoundsWhenInflationIsDisabled)
{
  auto obstacle = makeObstacle(10.0, 9.8, 10.2, -0.17, 0.23);
  obstacle.s_var = 0.01;
  obstacle.d_var = 100.0;

  const ObstacleGuardParameters parameters;
  const auto guard = buildUncertaintyGuard(obstacle, 100.0, parameters);

  EXPECT_NEAR(guard.d_right, obstacle.d_right, 1.0e-9);
  EXPECT_NEAR(guard.d_left, obstacle.d_left, 1.0e-9);
  EXPECT_LT(guard.s_start, obstacle.s_start);
  EXPECT_GT(guard.s_end, obstacle.s_end);
}

}  // namespace
}  // namespace local_planning
