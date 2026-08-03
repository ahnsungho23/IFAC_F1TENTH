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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

// 직선 트랙에서 d-offset과 경계 판정을 쉽게 검증하기 위한 기준 경로 fixture
f110_msgs::msg::WpntArray makeStraightReference(
  int count = 300, double spacing = 0.1,
  double left_width = 1.5, double right_width = 1.5)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = 0.0;
    waypoint.psi_rad = 0.0;
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = left_width;
    waypoint.d_right = right_width;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

// 폐곡선 wrap과 글로벌 handoff를 검증하기 위한 원형 기준 경로 fixture
f110_msgs::msg::WpntArray makeCircularReference(
  int count = 200, double radius = 5.0)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  const double spacing = 2.0 * M_PI * radius / static_cast<double>(count);
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = radius * std::cos(angle);
    waypoint.y_m = radius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * M_PI;
    waypoint.kappa_radpm = 1.0 / radius;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = 1.5;
    waypoint.d_right = 1.5;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

// detector가 제공하는 Frenet AABB 계약을 간단히 구성하는 장애물 fixture
f110_msgs::msg::Obstacle makeObstacle(
  int id, double s, double d_right = -0.20, double d_left = 0.20)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = s;
  obstacle.s_start = s - 0.20;
  obstacle.s_end = s + 0.20;
  obstacle.d_center = 0.5 * (d_right + d_left);
  obstacle.d_right = d_right;
  obstacle.d_left = d_left;
  obstacle.size = 0.40;
  obstacle.is_static = true;
  return obstacle;
}

// 개별 테스트가 기하 조건에 집중하도록 다소 여유 있게 설정한 공통 planner 파라미터
RacelineSplineParameters testParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 12.0;
  parameters.obstacle_clearance_m = 0.25;
  parameters.vehicle_half_width_m = 0.12;
  parameters.boundary_margin_m = 0.08;
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 50.0;
  return parameters;
}

// 글로벌 waypoint의 s_m 순서가 중복되거나 역전되면 reference 자체를 거부해야 한다.
TEST(RacelineSplinePlanner, RejectsNonMonotonicGlobalReference)
{
  auto reference = makeStraightReference(20);
  reference.wpnts[10].s_m = reference.wpnts[9].s_m;
  RacelineSplinePlanner planner(testParameters());
  std::string error;
  EXPECT_FALSE(planner.setReference(reference, &error));
  EXPECT_FALSE(error.empty());
}

// 회피 경로가 새 Cartesian branch를 찾지 않고 원래 글로벌 표본의 s/order만 보존하는지 검사한다.
TEST(RacelineSplinePlanner, ShiftsOnlyOrderedGlobalRaceLineSamples)
{
  const auto reference = makeStraightReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_forward = -1.0;
  bool saw_offset = false;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    EXPECT_GT(forward, previous_forward);
    previous_forward = forward;
    const std::size_t reference_index = static_cast<std::size_t>(
      std::llround(waypoint.s_m / 0.1));
    ASSERT_LT(reference_index, reference.wpnts.size());
    const auto & global = reference.wpnts[reference_index];
    EXPECT_NEAR(waypoint.x_m, global.x_m, 1.0e-9);
    EXPECT_NEAR(waypoint.y_m, waypoint.d_m, 1.0e-9);
    EXPECT_EQ(waypoint.s_m, global.s_m);
    EXPECT_EQ(waypoint.vx_mps, global.vx_mps);
    saw_offset = saw_offset || std::abs(waypoint.d_m) > 0.20;
  }
  EXPECT_TRUE(saw_offset);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

// 실제 d=0 merge 뒤에 속도 기반 controller tail이 추가되는지 검사한다.
TEST(RacelineSplinePlanner, AppendsSpeedAwareGlobalTailAfterActualMerge)
{
  auto parameters = testParameters();
  parameters.post_merge_lookahead_m = 2.0;
  parameters.post_merge_min_time_sec = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const EgoFrenetState ego{0.0, 0.0, 6.0};
  const auto result = planner.plan(ego, {makeObstacle(12, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  const double tail_distance =
    planner.forwardDistance(result.merge_s, result.path.wpnts.back().s_m);
  EXPECT_GE(tail_distance, 5.8);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

// 글로벌 인계 경로가 폐곡선 한 바퀴이며 ego가 state 확인용 tail 시작에 놓이는지 검사한다.
TEST(RacelineSplinePlanner, BuildsClosedGlobalHandoffWithEgoInStateTail)
{
  const auto reference = makeCircularReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailRatio = 0.10;
  const double ego_s = reference.wpnts[37].s_m;
  const auto path = planner.buildGlobalHandoffPath(ego_s, kTailRatio, 2.5);
  ASSERT_EQ(path.wpnts.size(), reference.wpnts.size());

  std::size_t closest_index = 0U;
  double closest_gap = std::numeric_limits<double>::infinity();
  double path_length = 0.0;
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double gap = planner.forwardDistance(ego_s, path.wpnts[i].s_m);
    const double circular_gap = std::min(gap, planner.trackLength() - gap);
    if (circular_gap < closest_gap) {
      closest_gap = circular_gap;
      closest_index = i;
    }
    EXPECT_DOUBLE_EQ(path.wpnts[i].d_m, 0.0);
    EXPECT_LE(path.wpnts[i].vx_mps, 2.5);
    if (i > 0U) {
      path_length += std::hypot(
        path.wpnts[i].x_m - path.wpnts[i - 1U].x_m,
        path.wpnts[i].y_m - path.wpnts[i - 1U].y_m);
    }
  }
  const std::size_t tail_count = static_cast<std::size_t>(
    std::ceil(kTailRatio * static_cast<double>(path.wpnts.size())));
  EXPECT_GE(closest_index, path.wpnts.size() - tail_count);

  const double average_spacing =
    path_length / static_cast<double>(path.wpnts.size() - 1U);
  const double closing_gap = std::hypot(
    path.wpnts.front().x_m - path.wpnts.back().x_m,
    path.wpnts.front().y_m - path.wpnts.back().y_m);
  EXPECT_LE(closing_gap, 2.0 * average_spacing);
}

// 왼쪽 트랙 폭이 부족하면 가능한 오른쪽 회피를 선택하는지 검사한다.
TEST(RacelineSplinePlanner, UsesRightSideWhenLeftTrackSpaceIsInsufficient)
{
  auto reference = makeStraightReference(300, 0.1, 0.55, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, 0.0);
}

// 목표 d가 장애물 span의 벽을 넘으면 spline fitting 전에 해당 방향을 제거하는지 검사한다.
TEST(RacelineSplinePlanner, RejectsWallBlockedTargetsBeforeSplineConstruction)
{
  auto reference = makeStraightReference(300, 0.1, 0.55, 0.55);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});

  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_NE(
    result.reason.find("left target d exceeds track bound in obstacle span"),
    std::string::npos);
  EXPECT_NE(
    result.reason.find("right target d exceeds track bound in obstacle span"),
    std::string::npos);
}

// 기존 commitment 방향이 계속 안전하면 재계획에서도 같은 방향을 유지하는지 검사한다.
TEST(RacelineSplinePlanner, HonorsCommittedSideWhenItRemainsFeasible)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_TRUE(result.go_left);
}

// uncertainty Guard 바깥에 commitment_clearance_reserve가 별도로 더해지는지 검사한다.
TEST(RacelineSplinePlanner, AddsReserveOutsideValidatedObstacleClearance)
{
  auto parameters = testParameters();
  parameters.commitment_clearance_reserve_m = 0.05;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_NEAR(result.target_d, 0.50, 1.0e-9);
}

// 작은 AABB jitter가 기존 동결 경로를 불필요하게 무효화하지 않는지 검사한다.
TEST(RacelineSplinePlanner, KeepsCommittedPathValidAcrossSmallAabbJitter)
{
  auto parameters = testParameters();
  parameters.commitment_clearance_reserve_m = 0.05;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  std::string reason;
  EXPECT_TRUE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0, -0.22, 0.22)}, &reason)) << reason;
  EXPECT_FALSE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0, -0.35, 0.35)}, &reason));
}

// uncertainty envelope의 soft 충돌과 차량 폭 기준 hard 충돌을 구분할 수 있는지 검사한다.
TEST(RacelineSplinePlanner, DistinguishesSoftEnvelopeFromHardVehicleCollision)
{
  auto parameters = testParameters();
  parameters.commitment_clearance_reserve_m = 0.05;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(23, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const auto expanded_obstacle = makeObstacle(23, 7.0, -0.20, 0.30);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {expanded_obstacle}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_EQ(failure.obstacle_id, 23);
  EXPECT_TRUE(std::isfinite(failure.waypoint_s));
  EXPECT_TRUE(std::isfinite(failure.waypoint_d));
  EXPECT_NEAR(failure.obstacle_source_d_left, 0.30, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_test_d_left, 0.55, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_clearance, 0.25, 1.0e-9);

  constexpr double kHardVehicleClearance = 0.15;
  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {expanded_obstacle}, &reason, &failure,
      kHardVehicleClearance)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

// 현재 merge 뒤 controller tail의 장애물이 활성 commitment를 실패시키지 않는지 검사한다.
TEST(RacelineSplinePlanner, IgnoresPostMergeTailCollisionForCurrentCommitment)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(24, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const double next_obstacle_s = committed.merge_s + 0.5;
  const auto next_obstacle = makeObstacle(25, next_obstacle_s, -0.60, -0.05);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_GT(failure.waypoint_s, committed.merge_s);

  const double merge_horizon = planner.forwardDistance(ego.s, committed.merge_s);
  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure,
      std::nullopt, merge_horizon)) << reason;
}

// 연속 maneuver가 0이 아닌 현재 ego.d에서 불연속 없이 시작하는지 검사한다.
TEST(RacelineSplinePlanner, StartsNextManeuverContinuouslyFromNonzeroEgoD)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{9.05, 0.50, 2.0};
  const auto result = planner.plan(
    ego, {makeObstacle(26, 13.5)}, true, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_s = ego.s;
  double previous_d = ego.d;
  for (const auto & waypoint : result.path.wpnts) {
    const double ds = planner.forwardDistance(previous_s, waypoint.s_m);
    ASSERT_GT(ds, 0.0);
    EXPECT_LE(
      std::abs(waypoint.d_m - previous_d) / ds,
      testParameters().maximum_lateral_slope + 1.0e-6);
    previous_s = waypoint.s_m;
    previous_d = waypoint.d_m;
  }
  EXPECT_NEAR(result.path.wpnts.front().d_m, ego.d, 0.02);
}

// 방향 잠금 뒤 선택 방향이 막히면 반대쪽으로 뒤집지 않고 정지하는지 검사한다.
TEST(RacelineSplinePlanner, DoesNotReverseCommittedSideWhenItBecomesBlocked)
{
  auto reference = makeStraightReference(300, 0.1, 0.55, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto unlocked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, true);
  ASSERT_EQ(unlocked.kind, SplinePlanKind::kAvoidance) << unlocked.reason;
  EXPECT_FALSE(unlocked.go_left);

  const auto locked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, false);
  EXPECT_EQ(locked.kind, SplinePlanKind::kSafeStop) << locked.reason;
}

// 원 장애물 경계가 레이스 라인에서 충분히 떨어져 있으면 blocking으로 보지 않는지 검사한다.
TEST(RacelineSplinePlanner, IgnoresObstacleWithEnoughRawRacelineClearance)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(4, 7.0, 0.35, 0.55)});
  EXPECT_EQ(result.kind, SplinePlanKind::kNoObstacle) << result.reason;
  EXPECT_TRUE(result.path.wpnts.empty());
}

// 좌우가 모두 닫혔을 때 충돌하지 않는 감속 prefix를 만드는지 검사한다.
TEST(RacelineSplinePlanner, BuildsCollisionFreeStopWhenBothSidesAreClosed)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
}

// safe-stop이 글로벌 d=0으로 점프하지 않고 현재 횡방향 offset을 유지하는지 검사한다.
TEST(RacelineSplinePlanner, BuildsSafeStopAtCurrentLateralOffset)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.30, 2.0},
    {makeObstacle(27, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_NEAR(waypoint.d_m, 0.30, 1.0e-9);
  }
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
}

// 활성 commitment가 있으면 그 남은 기하 위에서 먼저 제동하는지 검사한다.
TEST(RacelineSplinePlanner, BrakesOnCommittedGeometryBeforeEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto committed = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(28, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const auto ego_waypoint = std::find_if(
    committed.path.wpnts.begin(), committed.path.wpnts.end(),
    [](const auto & waypoint) {return waypoint.s_m >= 3.0;});
  ASSERT_NE(ego_waypoint, committed.path.wpnts.end());
  const EgoFrenetState ego{ego_waypoint->s_m, ego_waypoint->d_m, 2.0};
  const auto stop = planner.buildCommittedPathStop(
    ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});

  ASSERT_EQ(stop.kind, SplinePlanKind::kSafeStop) << stop.reason;
  ASSERT_GE(stop.path.wpnts.size(), 2U);
  EXPECT_NEAR(stop.path.wpnts.front().d_m, ego.d, 0.05);
  EXPECT_NEAR(stop.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  EXPECT_TRUE(
    std::any_of(
      stop.path.wpnts.begin(), stop.path.wpnts.end(),
      [](const auto & waypoint) {return std::abs(waypoint.d_m) > 0.10;}));

  auto off_path_ego = ego;
  off_path_ego.d += 0.10;
  const auto discontinuous_stop = planner.buildCommittedPathStop(
    off_path_ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});
  EXPECT_EQ(discontinuous_stop.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(discontinuous_stop.path.wpnts.empty());
  EXPECT_NE(
    discontinuous_stop.reason.find("discontinuous from the current ego d"),
    std::string::npos);
}

// 최초 군집 안정화 중 장애물 앞 preparation 감속 경로를 만드는지 검사한다.
TEST(RacelineSplinePlanner, BuildsPreparationStopForInitialBlockingCluster)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kPreparation) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_EQ(result.obstacle_id, 5);
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
}

// 계획 결과에 가장 가까운 blocking 군집의 모든 ID가 포함되는지 검사한다.
TEST(RacelineSplinePlanner, ReportsWholeBlockingClusterInAvoidanceResult)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
}

// 다음 maneuver 연결에 사용할 가장 가까운 blocking cluster ID를 반환하는지 검사한다.
TEST(RacelineSplinePlanner, ReportsNearestBlockingClusterIdsForManeuverChaining)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto cluster_ids = planner.blockingClusterIds(
    EgoFrenetState{0.0, 0.0, 2.0},
      {
        makeObstacle(5, 7.0),
        makeObstacle(6, 7.6),
        makeObstacle(9, 10.0),
      });

  EXPECT_EQ(cluster_ids, (std::vector<int>{5, 6}));
}

// 장애물이 이미 safe-stop buffer 안이면 안정화 대기 없이 즉시 정지 실패로 처리하는지 검사한다.
TEST(RacelineSplinePlanner, RefusesPreparationDelayInsideSafeStopBuffer)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.0)});

  EXPECT_EQ(result.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(result.path.wpnts.empty());
  EXPECT_NE(result.reason.find("inside the safe-stop buffer"), std::string::npos);
}

// minimum_path_points보다 짧더라도 안전 정지에 충분한 2점 prefix를 허용하는지 검사한다.
TEST(RacelineSplinePlanner, AllowsShortValidatedSafeStopPrefix)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.65, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_GE(result.path.wpnts.size(), 2U);
  EXPECT_LT(result.path.wpnts.size(), 8U);
}

// 경로 prefix조차 만들 수 없을 때 현재 d의 zero-speed hold를 생성하는지 검사한다.
TEST(RacelineSplinePlanner, BuildsZeroSpeedEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto path = planner.buildEmergencyStopPath(EgoFrenetState{2.05, 0.18, 2.0});
  ASSERT_EQ(path.wpnts.size(), 8U);
  for (const auto & waypoint : path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.18);
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(waypoint.ax_mps2, 0.0);
  }
}

// s=0을 걸친 장애물의 span과 회피 경로가 정상 계산되는지 검사한다.
TEST(RacelineSplinePlanner, HandlesObstacleAcrossTrackWrap)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  auto obstacle = makeObstacle(9, 0.30);
  obstacle.s_start = 0.10;
  obstacle.s_end = 0.50;
  const auto result = planner.plan(EgoFrenetState{29.0, 0.0, 2.0}, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  bool wrapped = false;
  double previous = -1.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(29.0, waypoint.s_m);
    EXPECT_GT(forward, previous);
    previous = forward;
    wrapped = wrapped || waypoint.s_m < 1.0;
  }
  EXPECT_TRUE(wrapped);
}

// 가까이 붙은 뱀 모양 반대 branch로 점프하지 않고 ordered race-line만 이동하는 회귀 검사다.
TEST(RacelineSplinePlanner, NeverJumpsToNearbyWrongSnakeBranch)
{
  auto reference = makeStraightReference();
  for (std::size_t i = 150U; i < reference.wpnts.size(); ++i) {
    reference.wpnts[i].x_m = 30.0 - reference.wpnts[i].s_m;
    reference.wpnts[i].y_m = 0.55;
    reference.wpnts[i].psi_rad = 3.14159265358979323846;
  }
  auto parameters = testParameters();
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(11, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LT(waypoint.s_m, 15.0)
      << "planner selected a geometrically nearby but topologically wrong snake branch";
    EXPECT_LT(waypoint.y_m, 0.55)
      << "path must remain an offset of the ordered first race-line branch";
  }
}

}  // namespace
}  // namespace local_planning
