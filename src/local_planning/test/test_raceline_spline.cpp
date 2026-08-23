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
#include <optional>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

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

RacelineSplineParameters testParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 12.0;
  parameters.vehicle_half_width_m = 0.12;
  parameters.safety_margin_m = 0.03;
  // 이 파일의 시험들은 추종오차 예약 기구 자체를 검증하므로 게이트를 명시적으로 켠다
  // (2026-08-22). 운영 기본값은 off 이며, 그쪽 계약은 test_obstacle_reserve_gate 가 본다.
  parameters.obstacle_reserve_from_lut = true;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 50.0;
  // 기존 단위 테스트는 legacy zero-delay로 격리하고, response delay는
  // ConfirmedApproachReservesResponseDelayDistance에서만 명시적으로 켜 독립 검증한다.
  parameters.confirmed_speed_response_delay_sec = 0.0;
  // Legacy safety-constraint tests isolate one target. Multi-target behavior is exercised by
  // dedicated candidate-generation tests below.
  parameters.target_d_candidate_count = 1;
  return parameters;
}

f110_msgs::msg::WpntArray makeStraightCandidate(
  const f110_msgs::msg::WpntArray & reference,
  double d,
  double yaw,
  std::size_t count = 20U)
{
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  count = std::min(count, reference.wpnts.size());
  path.wpnts.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto waypoint = reference.wpnts[index];
    waypoint.id = static_cast<int32_t>(index);
    waypoint.d_m = d;
    waypoint.x_m = reference.wpnts[index].x_m;
    waypoint.y_m = reference.wpnts[index].y_m + d;
    waypoint.psi_rad = yaw;
    waypoint.kappa_radpm = 0.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
}

TEST(RacelineSplinePlanner, RejectsNonMonotonicGlobalReference)
{
  auto reference = makeStraightReference(20);
  reference.wpnts[10].s_m = reference.wpnts[9].s_m;
  RacelineSplinePlanner planner(testParameters());
  std::string error;
  EXPECT_FALSE(planner.setReference(reference, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RacelineSplinePlanner, P3NonpositiveEntryBoundaryFailsClosedWithoutThrowing)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  P3ShadowResult result;
  EXPECT_NO_THROW(
    result = planner.evaluateP3Shadow(
      EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(41, 0.2)},
      100, 1U, 1U, "NONPOSITIVE_BOUNDARY_TEST"));
  EXPECT_TRUE(result.invoked);
  EXPECT_FALSE(result.would_recover);
  EXPECT_TRUE(result.m1_invoked);
  EXPECT_EQ(result.m0_candidate_count, 0U);
  EXPECT_LE(result.candidate_count, 24U);
  ASSERT_EQ(result.cluster_obstacle_ids.size(), 1U);
  EXPECT_EQ(result.cluster_obstacle_ids.front(), 41);
  EXPECT_TRUE(std::isfinite(result.cluster_start_forward_m));
  EXPECT_TRUE(std::isfinite(result.cluster_end_forward_m));
  EXPECT_LE(result.cluster_start_forward_m, result.cluster_end_forward_m);
}

TEST(RacelineSplinePlanner, RejectsRotatedFootprintWhenCenterlineRemainsInsideLeftBound)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  // 자차 d를 경로 d에 맞춘다: 실제 경로는 언제나 자차의 현재 d에서 출발하므로,
  // d=0 자차에 옆으로 떨어진 합성 경로는 진입 불연속 검사(2026-08-17)에 먼저 걸려
  // 이 테스트가 원래 보려던 검사에 도달하지 못한다.
  const EgoFrenetState ego{0.0, 0.15, 2.0};
  const auto rotated = makeStraightCandidate(reference, 0.15, 0.40);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(ego, rotated, {}, &reason, &failure));
  EXPECT_EQ(reason, "footprint_track_bound");
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
  EXPECT_EQ(failure.footprint_violation_side, "left");
  EXPECT_GT(failure.centerline_wall_clearance, 0.0);
  EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  EXPECT_GT(failure.wallward_corner_protrusion, 0.0);
  EXPECT_NEAR(failure.heading_relative_to_reference, 0.40, 1.0e-12);
}

TEST(RacelineSplinePlanner, AcceptsHeadingAlignedFootprintAtSameCenterline)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const auto aligned = makeStraightCandidate(reference, 0.15, 0.0);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_TRUE(planner.validatePath(
      EgoFrenetState{0.0, 0.15, 2.0}, aligned, {}, &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

// F1 (2026-08-22) 의 안전 경계 — 왜 validatePath 하나로는 재발행을 못 막는가.
//
// 보류 게이트는 커밋 경로가 없을 때 **마지막으로 발행된 회피 기하**를 다시 검증해 계속
// 쓴다 (hold_recovery_gate.hpp). 처음 판은 "validatePath 를 매 콜백 통과해야 하니
// 소진되면 스스로 떨어진다" 로 충분하다고 봤는데, 이 시험이 그것이 **틀렸음**을 보였다:
//
//   validatePath 는 committed 경로 재검증용이라
//     validateCandidate(ego, path, visible, reason, start_index, **1U**, ...)
//   로 최소점수를 1 로 덮어쓴다. 그래서 앞에 1 점만 남아도 통과한다.
//
// 그 상태로 재발행하면 F1 이 고치려던 실패가 그대로 재현된다 — 컨트롤러의 walk_forward 가
// 열린 경로의 끝점에 멈추므로, 남은 점이 짧을수록 L1 목표가 코앞에 붙고 요구 횡가속이
// 튄다 (실차 최대 32.2 m/s², 그립 예산 6.80).
//
// 그래서 게이트는 minimum_path_points 를 **자기가 따로** 센다. 이 시험은 두 가지를 같이
// 고정한다: ① validatePath 단독은 끝점까지 유효하다(그러므로 하한이 필요하다),
// ② validatePath + 기하 하한은 끝점에 닿기 전에 종료하고 되돌아오지 않는다.
TEST(RacelineSplinePlanner, GuidanceRepublishNeedsItsOwnLengthFloorToTerminate)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  auto parameters = testParameters();
  parameters.minimum_path_points = 8;   // 운영 YAML 과 같은 값으로 고정한다.
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // 20 점 · s 0.0~1.9 m 짜리 회피 기하. 실차의 재발행 대상과 같은 성격이다.
  const auto guidance = makeStraightCandidate(reference, 0.15, 0.0, 20U);
  ASSERT_EQ(guidance.wpnts.size(), 20U);
  const double last_s = guidance.wpnts.back().s_m;

  ASSERT_TRUE(planner.validatePath(EgoFrenetState{0.0, 0.15, 2.0}, guidance, {}))
    << "출발점에서부터 무효면 이 시험은 아무것도 보지 못한다";

  // 노드가 재는 것과 같은 방식: ego 앞으로 경로가 뻗은 거리.
  const double floor_m = 1.0;   // 이 20 점(1.9 m) 경로에서 하한이 실제로 걸리는 값.

  std::optional<double> first_validate_invalid;
  std::optional<double> first_gate_stop;
  bool gate_recovered_after_stop = false;
  for (double s = 0.0; s <= last_s + 0.5; s += 0.05) {
    PathValidationFailure failure;
    const bool valid = planner.validatePath(
      EgoFrenetState{s, 0.15, 2.0}, guidance, {}, nullptr, &failure);
    if (!valid && !first_validate_invalid.has_value()) {
      first_validate_invalid = s;
      EXPECT_EQ(failure.kind, PathValidationFailureKind::kNoForwardPath)
        << "경로 소진이 아닌 이유로 떨어졌다: " << failure.reason;
    }
    const bool gate_publishes = valid &&
      planner.forwardSpanAheadOfEgo(guidance, EgoFrenetState{s, 0.15, 2.0}) >= floor_m;
    if (!gate_publishes && !first_gate_stop.has_value()) {
      first_gate_stop = s;
    } else if (gate_publishes && first_gate_stop.has_value()) {
      gate_recovered_after_stop = true;
    }
  }

  // ① validatePath 단독은 끝점까지 버틴다 — 이것이 하한이 필요한 이유다.
  ASSERT_TRUE(first_validate_invalid.has_value());
  EXPECT_GE(first_validate_invalid.value(), last_s)
    << "validatePath 가 minimum_path_points 를 보기 시작했다면 게이트의 자체 하한을 "
       "다시 검토하라 — 이 시험의 전제가 바뀐 것이다";

  // ② 하한을 얹으면 끝점에 닿기 한참 전에 종료한다.
  ASSERT_TRUE(first_gate_stop.has_value()) << "기하 하한이 끝내 안 걸린다 — 자기종료 실패";
  EXPECT_LT(first_gate_stop.value(), last_s)
    << "경로 끝에 닿은 뒤에야 멈춘다 — 그 사이 구간은 코앞의 끝점으로 조향한다";
  EXPECT_LT(first_gate_stop.value(), first_validate_invalid.value())
    << "하한이 validatePath 보다 먼저 걸려야 의미가 있다";
  EXPECT_FALSE(gate_recovered_after_stop)
    << "멈춘 뒤 다시 발행한다 — 재발행과 안전정지가 프레임 단위로 교대한다";
}

TEST(RacelineSplinePlanner, DetectsBothLeftAndRightFootprintViolations)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  for (const auto & test : std::vector<std::pair<double, std::string>>{
        {0.15, "left"}, {-0.15, "right"}})
  {
    PathValidationFailure failure;
    EXPECT_FALSE(planner.validatePath(
        // 자차 d를 경로 d에 맞춘다 — 실제 경로는 언제나 자차 d에서 출발한다.
        // 자차 d를 경로 d에 맞춘다 (진입 불연속 검사).
        EgoFrenetState{ego.s, test.first, ego.speed},
        makeStraightCandidate(reference, test.first, 0.40), {}, nullptr, &failure));
    EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
    EXPECT_EQ(failure.footprint_violation_side, test.second);
    EXPECT_GT(failure.centerline_wall_clearance, 0.0);
    EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  }
}

TEST(RacelineSplinePlanner, PreservesOtherHardConstraintsAfterFootprintValidation)
{
  const auto reference = makeStraightReference(100, 0.1, 2.0, 2.0);
  auto parameters = testParameters();
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  auto path = makeStraightCandidate(reference, 0.0, 0.0);
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(
      ego, path, {makeObstacle(501, 1.0)}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);

  path.wpnts.front().kappa_radpm = parameters.maximum_curvature_radpm + 0.1;
  EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);

  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1.0;
  RacelineSplinePlanner rate_planner(parameters);
  ASSERT_TRUE(rate_planner.setReference(reference));
  path = makeStraightCandidate(reference, 0.0, 0.0);
  path.wpnts[2].kappa_radpm = 1.0;
  EXPECT_FALSE(rate_planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);
}

TEST(RacelineSplinePlanner, UsesDirectionalControlSteeringCurvatureLimits)
{
  const auto reference = makeStraightReference(100, 0.1, 2.0, 2.0);
  auto parameters = testParameters();
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 100.0;
  parameters.control_wheelbase_m = 0.33;
  parameters.control_max_steering_left_rad = 0.410;
  parameters.control_max_steering_right_rad = 0.361;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  PathValidationFailure failure;

  auto left = makeStraightCandidate(reference, 0.0, 0.0);
  for (auto & waypoint : left.wpnts) {
    waypoint.kappa_radpm = 1.20;  // tan(0.410)/0.33 = 1.317: 좌조향 가능
  }
  EXPECT_TRUE(planner.validatePath(ego, left, {}, nullptr, &failure));

  auto right = left;
  for (auto & waypoint : right.wpnts) {
    waypoint.kappa_radpm = -1.20;  // tan(0.361)/0.33 = 1.144: 우조향 불가
  }
  EXPECT_FALSE(planner.validatePath(ego, right, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);
  EXPECT_NE(failure.reason.find("right control steering curvature"), std::string::npos);
}

TEST(RacelineSplinePlanner, FootprintValidationIsBitDeterministic)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  // 자차 d를 경로 d에 맞춘다: 실제 경로는 언제나 자차의 현재 d에서 출발하므로,
  // d=0 자차에 옆으로 떨어진 합성 경로는 진입 불연속 검사(2026-08-17)에 먼저 걸려
  // 이 테스트가 원래 보려던 검사에 도달하지 못한다.
  const EgoFrenetState ego{0.0, -0.15, 2.0};
  const auto path = makeStraightCandidate(reference, -0.15, -0.40);
  PathValidationFailure expected;
  ASSERT_FALSE(planner.validatePath(ego, path, {}, nullptr, &expected));

  for (int repetition = 0; repetition < 10; ++repetition) {
    PathValidationFailure actual;
    EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &actual));
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.waypoint_index, expected.waypoint_index);
    EXPECT_EQ(actual.footprint_violation_side, expected.footprint_violation_side);
    EXPECT_EQ(actual.centerline_wall_clearance, expected.centerline_wall_clearance);
    EXPECT_EQ(
      actual.rectangular_footprint_wall_clearance,
      expected.rectangular_footprint_wall_clearance);
    EXPECT_EQ(actual.waypoint_x, expected.waypoint_x);
    EXPECT_EQ(actual.waypoint_y, expected.waypoint_y);
    EXPECT_EQ(actual.waypoint_yaw, expected.waypoint_yaw);
    EXPECT_EQ(
      actual.heading_relative_to_reference,
      expected.heading_relative_to_reference);
    EXPECT_EQ(actual.wallward_corner_protrusion, expected.wallward_corner_protrusion);
  }
}

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

// 2026-08-16 시뮬 백(rosbag2_2026_08_16-08_50_21)의 실측 기하. 앞 장애물은 라인 왼쪽에
// 치우쳐 있어 우측으로 피하는데, 8 m 뒤 장애물은 라인 위에 걸쳐 있다. exit 스케일이 길면
// 복귀 램프가 오프셋을 유지한 채 뒤 장애물의 물리 엔벨로프를 지나가고, 그 경로는 커밋
// 재검증과 매 사이클 충돌해 25 ms마다 같은 후보를 다시 고르는 무한 재계획이 된다
// (실측: 랩당 hard collision 41회, s=28~30에서 완전 정지 랩당 2~4회).
// 뒤 장애물을 건드리지 않는 exit이 존재하면 그쪽이 선택되어야 한다.
TEST(RacelineSplinePlanner, PrefersExitThatClearsTheFollowingObstacle)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.entry_transition_fractions = {0.5, 0.75, 1.0};
  // 짧은/중간/아주 긴 exit. 마지막 값이 운영 YAML의 3.699 자리이며, 이것이 뒤 장애물을
  // 관통하는 후보를 만든다.
  parameters.transition_distance_scales = {0.5, 0.7, 3.7};
  RacelineSplinePlanner planner(parameters);
  // 백의 s=28~42 구간 회랑(d_left 1.18~1.28, d_right 0.73~0.90) 중 좁은 쪽으로 고정한다.
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.25, 1.20, 0.75)));

  auto blocking = makeObstacle(10, 3.57, 0.14, 0.47);   // 백 id10: s=31.44~31.77
  blocking.s_start = 3.40;
  blocking.s_end = 3.73;
  auto following = makeObstacle(0, 11.75, -0.18, 0.10);  // 백 id0: s=40.31~40.83, 라인 위
  following.s_start = 11.50;
  following.s_end = 12.00;

  const EgoFrenetState ego{0.0, -0.136, 2.17};
  const auto shadow = planner.evaluateP3Shadow(
    ego, {blocking, following}, 100, 1U, 1U, "FOLLOWING_OBSTACLE_EXIT_TEST");
  ASSERT_TRUE(shadow.invoked);
  ASSERT_FALSE(shadow.candidates.empty());
  ASSERT_NE(shadow.selected_path_digest, "NONE") << shadow.failure_classification;

  // 상황이 실제로 재현됐는지부터 확인한다: 뒤 장애물을 관통하는 exit 후보가 존재해야
  // 우선순위가 시험된다. 이게 0이면 테스트가 무의미하게 통과한다.
  std::size_t reaching = 0U;
  std::size_t clear_and_valid = 0U;
  for (const auto & candidate : shadow.candidates) {
    if (candidate.exit_reaches_next_obstacle) {
      ++reaching;
    } else if (candidate.hard_valid) {
      ++clear_and_valid;
    }
  }
  ASSERT_GT(reaching, 0U) << "no candidate carried its offset into the following obstacle; "
    "the ranking preference is not being exercised";
  ASSERT_GT(clear_and_valid, 0U) << "no clear alternative existed";

  // 선택된 후보는 그 관통 후보가 아니어야 한다.
  bool selected_found = false;
  for (const auto & candidate : shadow.candidates) {
    if (candidate.path_digest != shadow.selected_path_digest) {
      continue;
    }
    selected_found = true;
    EXPECT_FALSE(candidate.exit_reaches_next_obstacle)
      << "selected an exit ramp that carries offset into the following obstacle while a clear "
      "alternative existed";
  }
  EXPECT_TRUE(selected_found);

  // plan()의 순위도 같은 계약을 따라야 한다 (P3와 P0가 갈리면 서로 다른 경로를 커밋한다).
  const auto result = planner.plan(ego, {blocking, following});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  const double clearance = parameters.vehicle_half_width_m + parameters.safety_margin_m;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    if (forward + 1.0e-9 < following.s_start || forward > following.s_end + 1.0e-9) {
      continue;
    }
    if (std::abs(waypoint.d_m) <= 1.0e-3) {
      break;   // 합류 뒤 글로벌 꼬리 — 이 기동의 기하가 아니다.
    }
    EXPECT_FALSE(
      waypoint.d_m > following.d_right - clearance &&
      waypoint.d_m < following.d_left + clearance)
      << "plan() committed an exit ramp inside the following obstacle's envelope at forward="
      << forward << " d=" << waypoint.d_m;
  }
}

// 접근 램프의 적응 기울기 (2026-08-16): 여유 있는 접근은 base(2.0)를 그대로 쓰고, 가용
// 거리가 부족한 스팬만 max(3.5)까지 필요한 만큼 가팔라진다. 고정 상향은 여유 있는
// 장애물까지 늦고 세게 제동하게 만들므로 금지 — 이 테스트가 두 성질을 함께 고정한다.
// 한 물리 상자가 두 조각으로 갈라져 관측될 때, 조각 사이 s-틈의 waypoint도 클러스터
// hull 캡을 받아야 한다. 틈이 캡 없이 라인 속도로 남으면 스팬 안에서 1.1↔5.8 빗살
// 프로파일이 나와 옆 통과 내내 급가감속 펄스가 생긴다 (2026-08-16 13:52 백 실측).
// 안전정지 사유는 실제 P3 기각 사유를 담아야 하고, 측 제한이 걸리지 않았는데 "측 잠금"
// 이라고 적어서는 안 된다. 종전에는 left_evaluated/right_evaluated가 죽은 변수라 양측을
// 모두 평가하고 양측 다 실패한 경우에도 항상 "alternate side locked"가 찍혔고, 실제 사유
// (NO_VALID_SIDE_DOMAIN 등)는 뒤에 붙어 로그에서 잘려나갔다 (2026-08-16 14:18 백 오진).
// 2026-08-16 14:30 백의 s=12.28 영구 정지를 그 판정 지점에서 재현한다.
//
// 상황: 장애물 2가 s=15.6~16.1, d=[-0.28,+0.33]. 우측 회랑 -1.12 → 실여유 0.84 m.
// 시뮬 파라미터로 필요폭은 장애물면 0.393 + 벽 0.243 = 0.637 m이므로 물리적으로 통과
// 가능하다. 그런데 가드가 양면에 상수 0.14를 물리자 후보 14개가 전부 탈락하고 차가
// 멈춰 사람이 꺼내야 했다. 면별 실측(라인 쪽 σ=0.007 → 3σ=0.021)이면 통과한다.
//
// 이 테스트가 잡는 성질: 조용한 면의 팽창이 상수로 되돌아가면 즉시 깨진다.
RacelineSplineParameters fieldParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 15.0;
  parameters.obstacle_longitudinal_padding_m = 0.4149924657737441;
  parameters.vehicle_half_width_m = 0.15;
  parameters.vehicle_length_m = 0.56;
  parameters.safety_margin_m = 0.014789254299520768;
  // 실측표 재생용 파라미터이므로 예약 게이트를 켠다 (2026-08-22, 위 testParameters 주석 참고).
  parameters.obstacle_reserve_from_lut = true;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 1.5, 3.0, 4.5, 6.5};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 0.2, 0.5, 0.9, 1.316266519079011};
  parameters.tracking_error_lut_values_m = {                       // 시뮬 실측표
    0.115, 0.115, 0.115, 0.125, 0.125,
    0.175, 0.245, 0.280, 0.280, 0.280,
    0.175, 0.245, 0.280, 0.280, 0.280,
    0.185, 0.245, 0.280, 0.280, 0.280,
    0.185, 0.245, 0.280, 0.280, 0.280};
  parameters.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.6, 7.6, 7.6, 7.6, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5};
  // 종방향 한계표 — local_planning_velocity_limits.csv. 운영 YAML과 같은 값.
  parameters.avoidance_velocity_limit_accel_mps2 =
  {3.7, 3.7, 3.7, 3.7, 3.7, 3.47, 3.33, 3.0, 3.0, 3.0};
  parameters.avoidance_velocity_limit_decel_mps2 =
  {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  parameters.longitudinal_launch_speed_floor_mps = 1.0;
  parameters.avoidance_minimum_speed_mps = 1.0;
  parameters.localization_reserve_m = 0.06;
  parameters.wall_safety_margin_m = 0.10;
  parameters.maximum_target_offset_m = 1.50;
  parameters.target_d_candidate_count = 5;
  parameters.maximum_lateral_slope = 0.8;
  parameters.entry_discontinuity_min_budget_m = 0.20;
  parameters.maximum_curvature_radpm = 1.316266519079011;
  parameters.maximum_curvature_rate_radpm2 = 20.0;
  return parameters;
}

// 발행 프로파일의 모든 감속은 실제로 제동 가능해야 한다 (2026-08-16 14:30 백 회귀).
//
// 그날 실측: s=32.89 v=6.63 → s=33.14 v=4.58. 0.25 m 만에 (6.63²-4.58²)/(2·0.25) = 46 m/s²의
// 제동을 요구한다. 원인은 곡률·간격 캡이 waypoint마다 독립이라 S자 전이의 변곡점(κ≈0)에서
// 캡이 통째로 풀리는 것이었고, 접근 램프는 장애물 스팬 앞에만 걸려 이 구간을 못 잡았다.
// 🔴 2026-08-17 00:10 백 회귀. 랩마다 재현된 충돌 2건의 원인.
//
// 안전정지가 8회 확인 후 해제되며 커밋한 경로의 첫 점이 자차에서 불연속이었다:
//   자차          s=23.40  d=+0.043
//   경로 첫 점     s=23.60  d=+0.4897   → 0.20 m 앞에서 0.45 m 옆 (기울기 2.3)
// 경로 **내부**는 매끄러워 하드 검증을 통과했고, 차는 그 점을 향하다 오히려 반대로 밀려
// (d: +0.04 → -0.14) s=23.85에서 장애물에 박았다.
//
// 같은 검사가 buildCommittedPathStop에는 있었지만 회피 경로에는 없었다.
TEST(RacelineSplinePlanner, PathDiscontinuousFromEgoIsRejected)
{
  auto parameters = fieldParameters();
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference(300, 0.25, 1.20, 1.20);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{5.0, 0.043, 1.0};
  // 자차 0.20 m 앞에서 0.45 m 옆으로 시작하는 경로 — 기울기 2.25 (한계 0.8).
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  for (std::size_t index = 0; index < 20U; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = 5.20 + 0.25 * static_cast<double>(index);
    waypoint.d_m = 0.4897;                     // 경로 내부는 완전히 평탄하다
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = waypoint.d_m;
    waypoint.vx_mps = 1.5;
    path.wpnts.push_back(waypoint);
  }
  std::string reason;
  EXPECT_FALSE(planner.validatePath(ego, path, {}, &reason))
    << "자차에서 0.45 m 떨어진 곳에서 시작하는 경로가 통과했다";
  EXPECT_NE(reason.find("discontinuous"), std::string::npos) << reason;

  // 대조: 자차 d에서 시작하면 통과해야 한다 (검사가 과하게 걸리지 않는지).
  for (auto & waypoint : path.wpnts) {
    waypoint.d_m = ego.d;
    waypoint.y_m = waypoint.d_m;
  }
  std::string ok_reason;
  EXPECT_TRUE(planner.validatePath(ego, path, {}, &ok_reason)) << ok_reason;
}

// 🔴 2026-08-17 00:34 백 회귀 — 위 검사가 기울기 **단독**이었을 때의 오탐.
//
// 기준점은 자차보다 엄밀히 앞선 최근접 waypoint이고, 자차 s는 두 샘플 사이 아무 데나 있다.
// 그래서 그 전방거리는 샘플 간격 아래로 얼마든지 작아지고, 기울기 |Δd|/전방거리 는 경로
// 기하가 아니라 **추종오차를 0에 가까운 수로 나눈 값**이 되어 발산한다.
//
// 실해: 정상 추종 중 9회 무효화 → 그 중 3회가 0.3~0.5 s 뒤 안전정지 영구 정지(8.9 s 1건,
// 사람이 pose를 옮겨야 풀렸다). 정지 점유율 11%(00:10) → 53%(00:34).
//
// 판정은 간격을 **추종 예산**(trackingErrorReserve, localization_reserve_m 포함)과 먼저
// 비교하고, 예산을 넘으면서 기울기도 한계를 넘을 때만 불연속으로 본다.
// 🔴 0 마진 구성 회귀 (2026-08-20). 위 테스트는 fieldParameters() 가 localization_reserve_m
// 0.06 을 쓰기 때문에 예산이 항상 양수였고, 그래서 **마진을 전부 0 으로 둔 운영 구성을 한
// 번도 밟지 않았다**. 그 구성에서는 예산이 0 이 되어 검사의 AND 첫 조건이 상시 참이 되고,
// 판정이 기울기 하나로 무너진다 — 진입점이 자차 앞 몇 cm 라 분모가 작아 정상 추종오차도
// 불연속으로 기각된다.
//
// 실해(2026-08-19 run_001453, 자율 327.3 s): P3 기동 무효화 36 건 중 24 건(67%)이 이 검사였고
// 생성 후 p50 110 ms 만에 무효화됐다. 안전정지가 자율 시간의 24%(28 구간, 최장 11.6 s),
// 0.8 초에 커밋 5 회 교체, 경로 d 부호 전환 43 회(차가 1 초에 0.29 m 좌우로 끌림)로 이어졌다.
//
// entry_discontinuity_min_budget_m 하한이 그 붕괴를 막는다. 이 테스트는 마진을 전부 0 으로
// 둔 채 위 테스트와 같은 배치를 태운다.
TEST(RacelineSplinePlanner, ZeroMarginConfigStillHonoursTheTrackingBudgetFloor)
{
  auto parameters = fieldParameters();
  // 운영 프로토타입 구성: 장애물 마진을 전부 제거한다.
  parameters.localization_reserve_m = 0.0;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 7.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 0.46};
  parameters.tracking_error_lut_values_m = {0.0, 0.0, 0.0, 0.0};
  ASSERT_DOUBLE_EQ(parameters.trackingErrorReserve(3.0, 0.0), 0.0)
    << "이 테스트는 마진 예산이 0 일 때만 의미가 있다";
  ASSERT_GT(parameters.entry_discontinuity_min_budget_m, 0.0);

  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference(300, 0.25, 1.20, 1.20);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{5.0, 0.212, 2.96};
  // 하한 안쪽의 정상 추종오차. 진입점은 자차 0.01 m 앞이라 기울기는 발산한다.
  const double tracking_error = 0.5 * parameters.entry_discontinuity_min_budget_m;
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  for (std::size_t index = 0; index < 20U; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = 5.01 + 0.25 * static_cast<double>(index);
    waypoint.d_m = ego.d + tracking_error;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = waypoint.d_m;
    waypoint.vx_mps = 3.0;
    path.wpnts.push_back(waypoint);
  }
  ASSERT_GT(tracking_error / 0.01, parameters.maximum_lateral_slope)
    << "기울기가 한계를 넘지 않으면 회귀를 재현하지 못한다";
  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, path, {}, &reason))
    << "0 마진 구성에서 정상 추종오차가 불연속으로 기각됐다: " << reason;

  // 하한을 넘는 진짜 불연속은 그대로 걸려야 한다 (검사가 무력화되면 안 된다).
  for (auto & waypoint : path.wpnts) {
    waypoint.d_m = ego.d + 0.447;      // 코드 주석이 명시한 실해 사례 간격
    waypoint.y_m = waypoint.d_m;
  }
  reason.clear();
  EXPECT_FALSE(planner.validatePath(ego, path, {}, &reason))
    << "진짜 진입 불연속이 통과했다 — 하한이 검사를 무력화했다";
  EXPECT_NE(reason.find("discontinuous"), std::string::npos) << reason;
}

// 🔴 진입 불연속 검사의 AND 붕괴 (2026-08-23).
//
// 검사는 두 조건의 AND 로 설계됐다:
//     (간격 > 예산 하한 F)  AND  (간격 / 진입거리 > maximum_lateral_slope)
// 그런데 두 번째 조건은 진입거리 < 간격 / slope 일 때 참이고, 간격이 F 를 갓 넘은
// 경우 그 문턱은 F / slope = 0.20 / 0.8 = 0.25 m 다. 경로 샘플 간격이 0.251 m 이므로
// "자차보다 앞선 최근접 점"의 진입거리는 (0, 0.251] 이고 — 즉 **거의 항상** 문턱 아래다.
// 두 번째 조건이 자동으로 참이 되어 AND 가 무너지고, 검사는 "추종오차 > 0.20 m" 라는
// 단일 판정으로 퇴화한다.
//
// 실측 (롤보정 OFF 백 3개 072312/073013/204833 재생, 자율 구간만): 커밋 기동 무효화
// 16 건 중 12 건이 이 경로였고, 12 건 전부 간격 0.200~0.266(예산을 갓 넘김) / 진입거리
// p50 0.189 / 기울기 p50 1.10·최대 25.66 이었다. 무효화되면 후보를 새로 만드는데 그때
// 장애물이 0.32~1.04 m 앞이라 곡률·슬로프로 전멸한다.
//
// entry_continuity_baseline_m 가 분모에 바닥을 깔아 AND 를 되살린다.
TEST(RacelineSplinePlanner, EntrySlopeIsMeasuredOverABaselineTheCarCanActOn)
{
  auto parameters = fieldParameters();
  parameters.localization_reserve_m = 0.0;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.obstacle_reserve_from_lut = false;
  ASSERT_DOUBLE_EQ(parameters.trackingErrorReserve(3.0, 0.0), 0.0);
  ASSERT_GT(parameters.entry_continuity_baseline_m, 0.0);

  // 이 테스트가 재현하려는 붕괴가 파라미터상 실재하는지 먼저 못박는다. 하한이나 슬로프
  // 한계가 바뀌어 붕괴가 사라지면 이 단언이 먼저 깨져 테스트의 전제를 다시 보게 한다.
  // 실전 라인 간격: 39.20 m / 156 구간 = 0.2513 m.
  const double kSampleSpacingM = 0.2513;
  // 간격이 예산을 갓 넘으면 슬로프 조건은 진입거리 < 예산/슬로프 에서 참이 된다. 진입거리는
  // (0, 간격] 에 균등하므로, 그 문턱이 샘플 간격의 대부분을 덮으면 AND 는 사실상 무너진다.
  const double kAutoTrueThresholdM =
    parameters.entry_discontinuity_min_budget_m / parameters.maximum_lateral_slope;
  ASSERT_GT(kAutoTrueThresholdM, 0.9 * kSampleSpacingM)
    << "예산/슬로프 문턱이 샘플 간격을 거의 못 덮으면 AND 는 무너지지 않는다 — "
    << "문턱 " << kAutoTrueThresholdM << " m, 간격 " << kSampleSpacingM << " m";

  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference(300, kSampleSpacingM, 1.20, 1.20);
  ASSERT_TRUE(planner.setReference(reference));

  // 실측 중앙값 그대로: 간격 0.208 m, 진입거리 0.189 m.
  const double kMeasuredGapM = 0.208;
  const double kMeasuredEntryForwardM = 0.189;
  const EgoFrenetState ego{5.0, 0.0, 3.17};
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  for (std::size_t index = 0; index < 20U; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = ego.s + kMeasuredEntryForwardM +
      kSampleSpacingM * static_cast<double>(index);
    waypoint.d_m = ego.d + kMeasuredGapM;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = waypoint.d_m;
    waypoint.vx_mps = 3.17;
    path.wpnts.push_back(waypoint);
  }
  // 종전 판정을 재현: 예산을 넘었고, 최근접 점 기준 기울기도 한계를 넘는다.
  ASSERT_GT(kMeasuredGapM, parameters.entry_discontinuity_min_budget_m);
  ASSERT_GT(
    kMeasuredGapM / kMeasuredEntryForwardM, parameters.maximum_lateral_slope)
    << "종전 기준으로 기각되지 않으면 회귀를 재현하지 못한다";

  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, path, {}, &reason))
    << "정상 추종오차가 여전히 진입 불연속으로 기각된다: " << reason;

  // 진짜 불연속은 같은 진입거리에서도 그대로 걸려야 한다. 08-17 실해 사례 간격 0.447 m.
  for (auto & waypoint : path.wpnts) {
    waypoint.d_m = ego.d + 0.447;
    waypoint.y_m = waypoint.d_m;
  }
  ASSERT_GT(0.447 / parameters.entry_continuity_baseline_m, parameters.maximum_lateral_slope)
    << "바닥이 너무 크면 실해 사례를 놓친다 — 0.56 m 가 상한이다";
  reason.clear();
  EXPECT_FALSE(planner.validatePath(ego, path, {}, &reason))
    << "진짜 진입 불연속이 통과했다 — 바닥이 검사를 무력화했다";
  EXPECT_NE(reason.find("discontinuous"), std::string::npos) << reason;
}

// 바닥을 0 으로 두면 종전 거동으로 돌아간다 (되돌릴 수 있어야 한다).
TEST(RacelineSplinePlanner, ZeroBaselineRestoresThePreviousEntrySlopeBehaviour)
{
  auto parameters = fieldParameters();
  parameters.localization_reserve_m = 0.0;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.obstacle_reserve_from_lut = false;
  parameters.entry_continuity_baseline_m = 0.0;
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference(300, 0.25, 1.20, 1.20);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{5.0, 0.0, 3.17};
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  for (std::size_t index = 0; index < 20U; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = ego.s + 0.189 + 0.25 * static_cast<double>(index);
    waypoint.d_m = ego.d + 0.208;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = waypoint.d_m;
    waypoint.vx_mps = 3.17;
    path.wpnts.push_back(waypoint);
  }
  std::string reason;
  EXPECT_FALSE(planner.validatePath(ego, path, {}, &reason))
    << "바닥 0 인데 종전 기각이 재현되지 않았다";
  EXPECT_NE(reason.find("discontinuous"), std::string::npos) << reason;
}

TEST(RacelineSplinePlanner, NormalTrackingErrorOverATinyBaselineIsNotADiscontinuity)
{
  auto parameters = fieldParameters();
  const double budget = parameters.trackingErrorReserve(3.0, 0.0);
  ASSERT_GT(budget, 0.05) << "추종 예산이 0이면 이 검사는 의미가 없다";
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference(300, 0.25, 1.20, 1.20);
  ASSERT_TRUE(planner.setReference(reference));

  // 00:34 백 t=22.70 의 상태. 경로를 정상 추종 중이고 자차는 예산 안에서 옆으로 벗어나 있다.
  const EgoFrenetState ego{5.0, 0.212, 2.96};
  const double tracking_error = 0.5 * budget;
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  for (std::size_t index = 0; index < 20U; ++index) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    // 첫 점이 자차 **0.01 m** 앞에 온다 — 기울기는 발산하지만 간격은 추종오차 그대로다.
    waypoint.s_m = 5.01 + 0.25 * static_cast<double>(index);
    waypoint.d_m = ego.d + tracking_error;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = waypoint.d_m;
    waypoint.vx_mps = 3.0;
    path.wpnts.push_back(waypoint);
  }
  ASSERT_GT(tracking_error / 0.01, parameters.maximum_lateral_slope)
    << "이 배치에서 기울기가 한계를 넘지 않으면 회귀를 재현하지 못한다";
  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, path, {}, &reason))
    << "정상 추종오차가 불연속으로 기각됐다: " << reason;

  // 예산을 넘는 간격이라도 도달할 거리가 충분하면 정상이다 (판정의 나머지 절반).
  for (std::size_t index = 0; index < path.wpnts.size(); ++index) {
    path.wpnts[index].s_m = 7.00 + 0.25 * static_cast<double>(index);
    path.wpnts[index].d_m = ego.d + 0.45;
    path.wpnts[index].x_m = path.wpnts[index].s_m;
    path.wpnts[index].y_m = path.wpnts[index].d_m;
  }
  ASSERT_GT(0.45, budget);
  ASSERT_LT(0.45 / 2.0, parameters.maximum_lateral_slope);
  std::string reachable_reason;
  EXPECT_TRUE(planner.validatePath(ego, path, {}, &reachable_reason))
    << "2 m 앞의 0.45 m 이동은 기울기 0.225로 도달 가능한데 기각됐다: " << reachable_reason;
}

TEST(RacelineSplinePlanner, EveryDropInThePublishedProfileIsActuallyBrakeable)
{
  auto parameters = fieldParameters();
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.1, 1.20, 1.20);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 7.0;              // 라인 최고속 구간 — 여기서 캡이 풀리면 스파이크가 된다
  }
  ASSERT_TRUE(planner.setReference(reference));
  // 재현 조건: 라인 7.0 m/s에서 횡 ~1.0 m를 4.5 m 안에 옮긴다(백의 s=33~36과 같은 급도).
  // 전이가 완만하면 곡률 캡이 아예 안 걸려 결함이 나타나지 않는다 — 8 m 전이로는 κ_max가
  // 0.069뿐이라 프로파일이 7.00으로 평평하고, 이 테스트는 아무것도 잡지 못한다.
  const EgoFrenetState ego{0.0, -0.35, 7.0};
  const auto result = planner.plan(ego, {makeObstacle(2, 4.5, -0.60, 0.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 3U);

  const double limit = parameters.profileFeasibilityDecel();
  double worst = 0.0;
  double worst_s = 0.0;
  for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
    const auto & earlier = result.path.wpnts[i - 1];
    const auto & later = result.path.wpnts[i];
    const double ds = later.s_m - earlier.s_m;
    if (!(ds > 1.0e-9) || later.vx_mps >= earlier.vx_mps) {
      continue;                       // 가속 방향은 이 패스의 대상이 아니다.
    }
    const double required =
      (earlier.vx_mps * earlier.vx_mps - later.vx_mps * later.vx_mps) / (2.0 * ds);
    if (required > worst) {
      worst = required;
      worst_s = earlier.s_m;
    }
  }
  // 수치 오차만 허용한다. 이 값이 크게 튀면 캡 하나가 후방 패스 뒤에 적용되고 있다는 뜻이다.
  EXPECT_LT(worst, limit + 1.0e-6)
    << "s=" << worst_s << "에서 " << worst << " m/s² 제동을 요구한다 (한계 " << limit << ")";
}

// 전진(가속) 패스 회귀 — 2026-08-19. 이 패스가 생기기 전에는 발행 프로파일의 **가속에 제약이
// 하나도 없었다**. 후방 패스는 "제동으로 내려갈 수 있나"만 보므로, 캡이 한 점을 누른 뒤 다음
// 점이 라인 속도로 되튀는 계단을 그대로 통과시켰다(실차 백에서 v 5~9 m/s 구간 요구 가속의
// 93.5%가 velocity_limits.csv 한계 초과, 정지 출발 구간은 요구 p90 이 30 m/s²).
TEST(RacelineSplinePlanner, EveryRiseInThePublishedProfileIsActuallyReachable)
{
  auto parameters = fieldParameters();
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.1, 1.20, 1.20);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 7.0;
  }
  ASSERT_TRUE(planner.setReference(reference));
  // 🔑 자차는 느리고(1.2 m/s) 라인은 7.0 m/s다 — 전진 패스가 없으면 경로 첫 점이 곧바로
  // 7.0을 싣는다. 이것이 정지 후 출발에서 관측된 계단 그 자체다.
  const EgoFrenetState ego{0.0, -0.35, 1.2};
  const auto result = planner.plan(ego, {makeObstacle(2, 4.5, -0.60, 0.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 3U);

  // 첫 점부터 자차 실측 속도에서 도달 가능해야 한다.
  const double first_ds = result.path.wpnts.front().s_m - ego.s;
  if (first_ds > 1.0e-9) {
    const double reachable = std::sqrt(
      ego.speed * ego.speed + 2.0 * parameters.accelLimitAt(ego.speed) * first_ds);
    EXPECT_LE(result.path.wpnts.front().vx_mps, reachable + 1.0e-6)
      << "경로 첫 점이 자차 속도에서 도달 불가능하다";
  }

  double worst = 0.0;
  double worst_s = 0.0;
  for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
    const auto & earlier = result.path.wpnts[i - 1];
    const auto & later = result.path.wpnts[i];
    const double ds = later.s_m - earlier.s_m;
    if (!(ds > 1.0e-9) || later.vx_mps <= earlier.vx_mps) {
      continue;                       // 감속 방향은 후방 패스의 대상이다.
    }
    const double required =
      (later.vx_mps * later.vx_mps - earlier.vx_mps * earlier.vx_mps) / (2.0 * ds);
    const double allowed = parameters.accelLimitAt(earlier.vx_mps);
    if (required - allowed > worst) {
      worst = required - allowed;
      worst_s = earlier.s_m;
    }
  }
  EXPECT_LT(worst, 1.0e-6)
    << "s=" << worst_s << "에서 표 한계를 " << worst << " m/s² 초과하는 가속을 요구한다";
}

// 감속 한계가 스칼라가 아니라 속도의존 표에서 오는지 — 2026-08-19. 상수 3.5는 csv가 어느
// 속도에서도 허용하지 않는 값이라(저속 3.0, v>=4 는 2.0), 표를 붙이고도 스칼라를 쓰면
// 고속 구간에서 요구 제동이 실차 능력을 계속 넘는다.
TEST(RacelineSplinePlanner, ProfileDecelUsesTheSpeedDependentTableNotTheScalar)
{
  auto parameters = fieldParameters();
  EXPECT_GT(parameters.profileFeasibilityDecel(), parameters.decelLimitAt(7.0))
    << "이 테스트는 표가 스칼라보다 빡빡할 때만 의미가 있다";
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.1, 1.20, 1.20);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 7.0;
  }
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, -0.35, 7.0};
  const auto result = planner.plan(ego, {makeObstacle(2, 4.5, -0.60, 0.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  double worst = 0.0;
  for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
    const auto & earlier = result.path.wpnts[i - 1];
    const auto & later = result.path.wpnts[i];
    const double ds = later.s_m - earlier.s_m;
    if (!(ds > 1.0e-9) || later.vx_mps >= earlier.vx_mps) {
      continue;
    }
    const double required =
      (earlier.vx_mps * earlier.vx_mps - later.vx_mps * later.vx_mps) / (2.0 * ds);
    const double allowed =
      parameters.decelLimitAt(std::max(earlier.vx_mps, later.vx_mps));
    worst = std::max(worst, required - allowed);
  }
  EXPECT_LT(worst, 1.0e-6) << "표 한계를 " << worst << " m/s² 초과하는 제동을 요구한다";
}

TEST(RacelineSplinePlanner, MeasuredQuietFaceInflationKeepsTheEightyCentimetreGapPassable)
{
  RacelineSplinePlanner planner(fieldParameters());
  // 균일 회랑에서 우측 통과 밴드 = W - 0.9165 - 팽창 (0.9165 = 장애물면 0.393 + 벽 0.2435
  // + 박스 0.28). 실트랙(우측 1.15→1.075로 변동)의 임계는 하니스로 실측했고 팽창 0.07
  // 통과 / 0.10 실패였다. 여기서는 임계를 확실히 사이에 두도록 W=1.02를 쓴다:
  //   실측 팽창 0.021 → 밴드 +0.083 (통과)
  //   상수 팽창 0.14  → 밴드 -0.037 (불가)
  // 실트랙 증명은 stuck_case_harness가 담당한다 — 균일 회랑은 스팬 안 폭 변동을 못 담는다.
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.78, 1.02)));
  const EgoFrenetState ego{0.0, -0.019, 2.794};

  // 면별 실측 팽창: 라인 쪽 3σ=0.021, 반대쪽 3σ=0.132.
  const auto measured = planner.plan(
    ego, {makeObstacle(2, 6.25, -0.28 - 0.021, 0.33 + 0.132)});
  EXPECT_EQ(measured.kind, SplinePlanKind::kAvoidance)
    << "실여유 0.84 m 간격이 계획 불가가 됐다: " << measured.reason;

  // 종전 상수 경로(양면 0.14)로는 같은 간격에서 회피가 성립하지 않는다 — 그것이 정지였다.
  const auto constant = planner.plan(
    ego, {makeObstacle(2, 6.25, -0.28 - 0.14, 0.33 + 0.14)});
  EXPECT_NE(constant.kind, SplinePlanKind::kAvoidance)
    << "회귀 대조군이 성립하지 않는다 — 이 테스트는 상수 복귀를 잡지 못한다";
}

TEST(RacelineSplinePlanner, SafeStopReasonReportsTheActualRejectionNotAPhantomSideLock)
{
  auto parameters = testParameters();
  RacelineSplinePlanner planner(parameters);
  // 회랑을 양측 모두 통과 불가능하게 좁힌다 (코너 정점 배치의 축약판).
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.36, 0.36)));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(21, 8.0, -0.10, 0.10)});
  ASSERT_NE(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // 측 제한을 건 적이 없으므로 그 문구가 나오면 안 된다.
  EXPECT_EQ(result.reason.find("alternate side locked"), std::string::npos)
    << "phantom side-lock text in: " << result.reason;
  EXPECT_EQ(result.reason.find("side locked by active commitment"), std::string::npos)
    << "phantom side-lock text in: " << result.reason;
  // 실제 사유가 문자열 앞부분(로그 절단에도 살아남는 위치)에 있어야 한다.
  ASSERT_GE(result.reason.size(), 20U);
  EXPECT_NE(result.reason.substr(0, 120).find("no avoidance candidate"), std::string::npos)
    << "actual rejection not at the front of: " << result.reason;
}

TEST(RacelineSplinePlanner, GapCapBridgesFragmentGapsInsideOneCluster)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.tracking_error_lut_speed_bins_mps = {1.0, 5.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.50};
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.25, 1.20, 0.75);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 5.0;
  }
  ASSERT_TRUE(planner.setReference(reference));

  // 같은 면(d)을 가진 두 조각. 가장자리 간격 1.5 m — 확장 패딩(0.415×2)을 빼도
  // 0.67 m의 비커버 틈이 남고, 클러스터 규칙(gap 0.8)으로는 한 클러스터다.
  auto fragment_a = makeObstacle(10, 10.00, -0.10, 0.45);
  fragment_a.s_start = 9.90;
  fragment_a.s_end = 10.10;
  auto fragment_b = makeObstacle(11, 11.70, -0.10, 0.45);
  fragment_b.s_start = 11.60;
  fragment_b.s_end = 11.80;

  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto result = planner.plan(ego, {fragment_a, fragment_b});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // hull(첫 조각 시작 ~ 둘째 조각 끝) 안의 모든 waypoint는 스팬 내부와 같은 수준으로
  // 캡돼야 한다. 수리 전에는 틈 waypoint가 라인 속도(5.0)로 남았다.
  // 🔴 2026-08-16: 종전에는 span_cap을 max(span_cap, 0.0)으로 계산해 **항상 0**이었고,
  // 실제로는 임계 3.0만 보고 있었다. 그 상수는 당시 선택되던 후보에 맞춘 값이라, 후보
  // 순위가 바뀌면(A안: 속도 우선) 더 빠른 경로가 선택되면서 의도와 무관하게 깨진다.
  // 이 테스트가 지켜야 할 성질은 "틈 waypoint가 라인 속도로 남지 않고 스팬과 같은 수준으로
  // 캡되는가"이므로, 스팬 속도를 실제로 재서 그것과 비교한다.
  double span_max = 0.0;
  double gap_max = 0.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    if ((forward >= 9.49 && forward <= 10.51) || (forward >= 11.19 && forward <= 12.21)) {
      span_max = std::max(span_max, waypoint.vx_mps);   // 확장 스팬 내부
    }
    if (forward > 10.60 && forward < 11.15) {           // 확장 스팬 사이 비커버 틈
      gap_max = std::max(gap_max, waypoint.vx_mps);
    }
  }
  ASSERT_GT(gap_max, 0.0) << "no waypoint landed in the fragment gap; spacing broke";
  ASSERT_GT(span_max, 0.0) << "no waypoint landed inside the fragment spans";
  EXPECT_LT(gap_max, 5.0 - 1.0e-9)
    << "fragment-gap waypoint kept the raceline speed (5.0): " << gap_max;
  EXPECT_LE(gap_max, span_max + 1.0e-6)
    << "fragment-gap waypoint is faster than the span it bridges (sawtooth): "
    << gap_max << " vs span " << span_max;
}

TEST(RacelineSplinePlanner, ApproachRampSteepensOnlyWhenGeometryRequiresIt)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  parameters.approach_feasibility_decel_max_mps2 = 3.5;
  parameters.tracking_error_lut_speed_bins_mps = {1.0, 5.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.50};
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.25, 1.20, 0.75);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 5.0;
  }
  ASSERT_TRUE(planner.setReference(reference));

  // 케이스 1: 여유 있는 단일 장애물 (접근 10 m) — base 2.0을 넘는 감속이 있으면 안 된다.
  {
    auto lone = makeObstacle(10, 10.00, -0.10, 0.47);
    lone.s_start = 9.85;
    lone.s_end = 10.15;
    const EgoFrenetState ego{0.0, 0.0, 3.0};
    const auto result = planner.plan(ego, {lone});
    ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
    for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
      const auto & previous = result.path.wpnts[i - 1U];
      const auto & current = result.path.wpnts[i];
      const double ds = planner.forwardDistance(previous.s_m, current.s_m);
      if (!(ds > 1.0e-6) || current.vx_mps >= previous.vx_mps) {
        continue;
      }
      const double decel =
        (previous.vx_mps * previous.vx_mps - current.vx_mps * current.vx_mps) / (2.0 * ds);
      EXPECT_LE(decel, 2.0 * 1.10)
        << "generous approach was braked harder than the comfort rate at forward="
        << planner.forwardDistance(ego.s, current.s_m);
    }
  }

  // 케이스 2: 자차가 이미 빠르고(5.0) 스팬이 가까움(5 m) — 필요 기울기
  // (25-v_span²)/10 ≈ 2.3~2.5가 base를 넘으므로 램프가 그만큼만 가팔라져야 하고,
  // 그래도 max(3.5)를 넘는 감속을 명령하면 안 된다.
  {
    auto first = makeObstacle(10, 5.00, -0.10, 0.47);
    first.s_start = 4.85;
    first.s_end = 5.15;
    const EgoFrenetState ego{0.0, 0.0, 5.0};
    const auto result = planner.plan(ego, {first});
    ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
    double worst = 0.0;
    for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
      const auto & previous = result.path.wpnts[i - 1U];
      const auto & current = result.path.wpnts[i];
      const double ds = planner.forwardDistance(previous.s_m, current.s_m);
      if (!(ds > 1.0e-6) || current.vx_mps >= previous.vx_mps) {
        continue;
      }
      worst = std::max(
        worst,
        (previous.vx_mps * previous.vx_mps - current.vx_mps * current.vx_mps) / (2.0 * ds));
    }
    EXPECT_LE(worst, 3.5 * 1.10) << "tight gap demanded deceleration beyond the adaptive cap";
  }
}

TEST(RacelineSplinePlanner, ConfirmedApproachReservesResponseDelayDistance)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.obstacle_longitudinal_padding_m = 0.0;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  parameters.approach_feasibility_decel_max_mps2 = 3.5;
  parameters.tracking_error_lut_speed_bins_mps = {1.0, 5.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.50};
  auto reference = makeStraightReference(300, 0.25, 1.20, 0.75);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 5.0;
  }
  auto obstacle = makeObstacle(40, 8.20, -0.10, 0.47);
  obstacle.s_start = 8.00;
  obstacle.s_end = 8.40;
  const EgoFrenetState ego{0.0, 0.0, 5.0};

  const auto plan_with_delay = [&](double delay_sec) {
      auto copy = parameters;
      copy.confirmed_speed_response_delay_sec = delay_sec;
      RacelineSplinePlanner planner(copy);
      EXPECT_TRUE(planner.setReference(reference));
      return planner.plan(ego, {obstacle});
    };
  const auto legacy = plan_with_delay(0.0);
  const auto delayed = plan_with_delay(0.20);  // 5 m/s * 0.20 s = 1.0 m early hold
  ASSERT_EQ(legacy.kind, SplinePlanKind::kAvoidance) << legacy.reason;
  ASSERT_EQ(delayed.kind, SplinePlanKind::kAvoidance) << delayed.reason;

  const auto speed_near = [](const f110_msgs::msg::WpntArray & path, double station) {
      const auto closest = std::min_element(
        path.wpnts.begin(), path.wpnts.end(),
        [station](const auto & first, const auto & second) {
          return std::abs(first.s_m - station) < std::abs(second.s_m - station);
        });
      return closest == path.wpnts.end() ? std::numeric_limits<double>::quiet_NaN() :
             closest->vx_mps;
    };
  const double legacy_pre_span = speed_near(legacy.path, 7.50);
  const double delayed_pre_span = speed_near(delayed.path, 7.50);
  const double delayed_span = speed_near(delayed.path, obstacle.s_start);
  ASSERT_TRUE(std::isfinite(legacy_pre_span));
  ASSERT_TRUE(std::isfinite(delayed_pre_span));
  ASSERT_TRUE(std::isfinite(delayed_span));
  EXPECT_LT(delayed_pre_span, legacy_pre_span - 1.0e-6);
  EXPECT_LE(delayed_pre_span, delayed_span + 1.0e-6)
    << "response-reservation band must hold the span target before the obstacle";
}

// 접근 제동 램프는 스팬마다 걸려야 한다. 예전에는 가장 가까운 스팬 하나만 대상이라, 두 번째
// 장애물 앞에서 gap 캡이 그대로 계단으로 나타났다 (2026-08-16 백: 0.25 m 만에 4.62 → 1.00,
// decel 2.0으로는 5.0 m가 필요한 감속을 요구).
TEST(RacelineSplinePlanner, BrakingRampCoversEveryObstacleSpanNotOnlyTheNearest)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  // gap 기반 캡은 추종오차 tube가 속도에 따라 커질 때만 속도를 끌어내린다. 평평한
  // fallback reserve로는 감속해도 tube가 그대로라 캡 자체가 동작하지 않는다.
  parameters.tracking_error_lut_speed_bins_mps = {1.0, 5.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.50};
  RacelineSplinePlanner planner(parameters);
  auto reference = makeStraightReference(300, 0.25, 1.20, 0.75);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 5.0;   // 캡이 실제로 속도를 끌어내리도록 여유를 준다.
  }
  ASSERT_TRUE(planner.setReference(reference));

  // 두 장애물 모두 라인을 넘어 오른쪽까지 걸쳐 있어, 우측 통과 폭이 tube보다 좁다 —
  // 그래야 gap 캡이 실제로 속도를 끌어내린다.
  auto first = makeObstacle(10, 4.00, -0.10, 0.47);
  first.s_start = 3.85;
  first.s_end = 4.15;
  auto second = makeObstacle(0, 11.00, -0.14, 0.45);
  second.s_start = 10.85;
  second.s_end = 11.15;

  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto result = planner.plan(ego, {first, second});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // 두 스팬 모두에서 캡이 실제로 걸렸는지 먼저 확인한다. 안 걸렸으면 테스트가 무의미하다.
  const auto span_minimum = [&](double start, double end) {
      double minimum = std::numeric_limits<double>::infinity();
      for (const auto & waypoint : result.path.wpnts) {
        const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
        if (forward >= start && forward <= end) {
          minimum = std::min(minimum, waypoint.vx_mps);
        }
      }
      return minimum;
    };
  ASSERT_LT(span_minimum(first.s_start, first.s_end), 5.0);
  ASSERT_LT(span_minimum(second.s_start, second.s_end), 5.0);

  // 어떤 연속 구간도 approach_feasibility_decel_mps2로 실현 불가능한 감속을 요구하면 안 된다.
  for (std::size_t i = 1; i < result.path.wpnts.size(); ++i) {
    const auto & previous = result.path.wpnts[i - 1U];
    const auto & current = result.path.wpnts[i];
    const double ds = planner.forwardDistance(previous.s_m, current.s_m);
    if (!(ds > 1.0e-6) || current.vx_mps >= previous.vx_mps) {
      continue;   // 가속 구간은 이 램프의 대상이 아니다.
    }
    const double required_decel =
      (previous.vx_mps * previous.vx_mps - current.vx_mps * current.vx_mps) / (2.0 * ds);
    EXPECT_LE(required_decel, parameters.approach_feasibility_decel_mps2 * 1.10)
      << "unreachable deceleration step at forward="
      << planner.forwardDistance(ego.s, current.s_m)
      << " (" << previous.vx_mps << " -> " << current.vx_mps << " over " << ds << " m)";
  }
}

TEST(RacelineSplinePlanner, RankingCentresPassBetweenObstacleAndWall)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 9;
  parameters.entry_transition_fractions = {1.0};
  parameters.transition_distance_scales = {1.0};
  parameters.maximum_lateral_slope = 100.0;
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(311, 8.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  // Obstacle left face 0.20 against a 1.20 wall leaves a 1.00 m gap the car does not need all of.
  // Both ranking terms are measured from the vehicle body, so the selected offset must leave
  // comparable room on each side instead of hugging the wall. The pre-fix centerline metric
  // biased this by (obstacle clearance - wall margin) / 2 and left 0.21 m more room at the
  // obstacle than at the wall.
  const double body_to_wall = 1.20 - result.target_d - parameters.vehicle_half_width_m;
  const double body_to_obstacle = result.target_d - 0.20 - parameters.vehicle_half_width_m;
  EXPECT_GT(body_to_wall, 0.0);
  EXPECT_GT(body_to_obstacle, 0.0);
  EXPECT_LT(std::abs(body_to_wall - body_to_obstacle), 0.10);
}

TEST(RacelineSplinePlanner, RepeatedCandidateSelectionIsBitDeterministic)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(121, 8.0)};
  const auto reference = planner.plan(ego, obstacles);
  ASSERT_EQ(reference.kind, SplinePlanKind::kAvoidance) << reference.reason;

  for (int repetition = 0; repetition < 10; ++repetition) {
    const auto repeated = planner.plan(ego, obstacles);
    ASSERT_EQ(repeated.kind, reference.kind);
    ASSERT_EQ(repeated.target_d, reference.target_d);
    ASSERT_EQ(repeated.go_left, reference.go_left);
    ASSERT_EQ(repeated.candidate_audits.size(), reference.candidate_audits.size());
    ASSERT_EQ(repeated.path.wpnts.size(), reference.path.wpnts.size());
    for (std::size_t index = 0; index < reference.path.wpnts.size(); ++index) {
      EXPECT_EQ(repeated.path.wpnts[index].s_m, reference.path.wpnts[index].s_m);
      EXPECT_EQ(repeated.path.wpnts[index].d_m, reference.path.wpnts[index].d_m);
      EXPECT_EQ(repeated.path.wpnts[index].x_m, reference.path.wpnts[index].x_m);
      EXPECT_EQ(repeated.path.wpnts[index].y_m, reference.path.wpnts[index].y_m);
      EXPECT_EQ(
        repeated.path.wpnts[index].kappa_radpm,
        reference.path.wpnts[index].kappa_radpm);
    }
    for (std::size_t index = 0; index < reference.candidate_audits.size(); ++index) {
      EXPECT_EQ(
        repeated.candidate_audits[index].final_rank,
        reference.candidate_audits[index].final_rank);
      EXPECT_EQ(
        repeated.candidate_audits[index].selected,
        reference.candidate_audits[index].selected);
    }
  }
}

TEST(RacelineSplinePlanner, KeepsQuinticAvoidanceValidOnCurvedReference)
{
  auto parameters = testParameters();
  parameters.transition_distance_scales = {1.0};
  // 운영 기본값은 false지만, 최종 x/y 해석 미분을 켠 실제 P3 후보도 전체 검증을
  // 통과하는지 별도로 고정한다. 끝점 단측 창의 수치 정확도는 handoff 시험이 맡는다.
  parameters.analytic_path_geometry_enable = true;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeCircularReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto obstacle = makeObstacle(19, 8.0);

  const auto result = planner.plan(ego, {obstacle}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, result.path, {obstacle}, &reason)) << reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_TRUE(std::isfinite(waypoint.psi_rad));
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_LE(std::abs(waypoint.kappa_radpm), parameters.maximum_curvature_radpm);
  }
}

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

TEST(RacelineSplinePlanner, BuildsClosedGlobalHandoffWithEgoInStateTail)
{
  const auto reference = makeCircularReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailRatio = 0.10;
  const double ego_s = reference.wpnts[37].s_m;
  local_planning::EgoFrenetState handoff_ego;
  handoff_ego.s = ego_s;
  handoff_ego.d = 0.0;  // 라인 위에서의 핸드오프 — 램프 없이 순수 레이스라인이어야 한다
  handoff_ego.speed = 2.5;
  const auto path = planner.buildGlobalHandoffPath(handoff_ego, kTailRatio, 2.5);
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

TEST(RacelineSplinePlanner, RampedGlobalHandoffDecaysEgoOffsetToZero)
{
  const auto reference = makeCircularReference();
  auto ramp_params = testParameters();
  ramp_params.merge_ramp_min_length_m = 3.0;  // 기본 0 = 비활성이므로 명시 활성화
  ramp_params.merge_ramp_time_sec = 1.5;
  RacelineSplinePlanner planner(ramp_params);
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailDistanceM = 6.28;   // 구 0.20 비율과 같은 호 길이 (0.2 * 31.4 m)
  local_planning::EgoFrenetState ego;
  ego.s = reference.wpnts[37].s_m;
  ego.d = 0.40;   // 완료 시점에 남아 있는 회피 오프셋
  ego.speed = 2.0;  // ramp_length = max(3.0, 2.0*1.5) = 3.0 m
  const auto path = planner.buildGlobalHandoffPath(ego, kTailDistanceM, 2.5);
  ASSERT_EQ(path.wpnts.size(), reference.wpnts.size());

  const std::size_t total = path.wpnts.size();
  // 구현과 같은 정의: 경로 끝에서 거꾸로 호 길이를 걸어 tail 창을 정한다(균일 간격).
  const double spacing = 2.0 * M_PI * 5.0 / static_cast<double>(total);
  std::size_t tail_count = 1U;
  double walked = 0.0;
  while (tail_count < total && walked + spacing <= kTailDistanceM) {
    walked += spacing;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;

  // ego 위치(회전 배열의 tail 첫 점)에서 d는 ego.d로 시작한다.
  EXPECT_NEAR(path.wpnts[tail_begin].d_m, ego.d, 1.0e-9);

  // 램프는 단조 감소하고, 3 m 전방 이후에는 정확히 0(레이스라인)이다.
  double forward_m = 0.0;
  double previous_d = path.wpnts[tail_begin].d_m;
  bool reached_zero = false;
  for (std::size_t k = tail_begin + 1U; k < total; ++k) {
    forward_m += std::hypot(
      path.wpnts[k].x_m - path.wpnts[k - 1U].x_m,
      path.wpnts[k].y_m - path.wpnts[k - 1U].y_m);
    EXPECT_LE(path.wpnts[k].d_m, previous_d + 1.0e-9);
    EXPECT_GE(path.wpnts[k].d_m, -1.0e-9);
    previous_d = path.wpnts[k].d_m;
    if (forward_m >= 3.1) {
      EXPECT_NEAR(path.wpnts[k].d_m, 0.0, 1.0e-9);
      reached_zero = true;
    }
  }
  EXPECT_TRUE(reached_zero);

  // 램프 구간의 좌표는 레이스라인 법선으로 d만큼 밀려 있어야 한다(경로-참조점 거리 = d).
  const auto & ramp_start = path.wpnts[tail_begin];
  const auto & reference_at_ego = reference.wpnts[37];  // ego_s = wpnts[37].s_m
  const double offset_distance = std::hypot(
    ramp_start.x_m - reference_at_ego.x_m, ramp_start.y_m - reference_at_ego.y_m);
  EXPECT_NEAR(offset_distance, std::abs(ego.d), 1.0e-6);

  // 램프 앞(한 바퀴 돌아오는 원거리 구간)은 순수 레이스라인이다.
  for (std::size_t k = 0; k < tail_begin; ++k) {
    EXPECT_DOUBLE_EQ(path.wpnts[k].d_m, 0.0);
  }
}

TEST(RacelineSplinePlanner, RampedGlobalHandoffClampsInsideWallPinch)
{
  auto reference = makeCircularReference();
  // ego(인덱스 37) 전방 5~12점 구간을 왼쪽 벽 협착부로 만든다.
  for (std::size_t i = 42; i <= 49; ++i) {
    reference.wpnts[i].d_left = 0.20;
  }
  auto params = testParameters();
  params.merge_ramp_min_length_m = 3.0;  // 기본 0 = 비활성이므로 명시 활성화
  params.merge_ramp_time_sec = 1.5;
  RacelineSplinePlanner planner(params);
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailDistanceM = 6.28;   // 구 0.20 비율과 같은 호 길이
  local_planning::EgoFrenetState ego;
  ego.s = reference.wpnts[37].s_m;
  ego.d = 0.40;  // 왼쪽 오프셋 → 왼쪽 협착부가 클램프를 강제한다
  ego.speed = 2.0;
  const auto path = planner.buildGlobalHandoffPath(ego, kTailDistanceM, 2.5);
  ASSERT_FALSE(path.wpnts.empty());

  const std::size_t total = path.wpnts.size();
  const double spacing = 2.0 * M_PI * 5.0 / static_cast<double>(total);
  std::size_t tail_count = 1U;
  double walked = 0.0;
  while (tail_count < total && walked + spacing <= kTailDistanceM) {
    walked += spacing;
    ++tail_count;
  }
  const std::size_t tail_begin = total - tail_count;
  const double keepout = params.vehicle_half_width_m + params.wall_safety_margin_m;
  const double allowed_in_pinch = std::max(0.0, 0.20 - keepout);

  double previous_d = path.wpnts[tail_begin].d_m;
  for (std::size_t k = tail_begin; k < total; ++k) {
    const std::size_t reference_index =
      static_cast<std::size_t>((37 + (k - tail_begin)) % total);
    if (reference_index >= 42 && reference_index <= 49) {
      // 협착부에서는 프로파일이 아직 크더라도 벽 여유 한도 안으로 눌린다.
      EXPECT_LE(path.wpnts[k].d_m, allowed_in_pinch + 1.0e-9)
        << "k=" << k << " ref=" << reference_index;
    }
    // 클램프 후 다시 넓어져도 되돌아 나가지 않는다(단조 비증가).
    EXPECT_LE(path.wpnts[k].d_m, previous_d + 1.0e-9);
    previous_d = path.wpnts[k].d_m;
  }
}

TEST(RacelineSplinePlanner, UsesRightSideWhenLeftTrackSpaceIsInsufficient)
{
  auto reference = makeStraightReference(300, 0.1, 0.45, 1.5);
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, 0.0);
}

TEST(RacelineSplinePlanner, RejectsWallBlockedTargetsBeforeSplineConstruction)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});

  // 양쪽 모두 트랙 경계에 막히면 스플라인을 만들기 전에 안전정지로 간다. 사유 문자열은
  // 후보 생성기(P3)의 것이므로 문구가 아니라 판정만 계약으로 본다.
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_FALSE(result.reason.empty());
}

TEST(RacelineSplinePlanner, AppliesPhysicalVehicleClearanceOnceToTrackBounds)
{
  // d_left/d_right describe physical boundaries. At heading=0 with half-width 0.12, d=0.23 is
  // exactly tangent for a 0.35 m left boundary and remains valid at numerical tolerance.
  const auto feasible_reference = makeStraightReference(300, 0.1, 0.35, 0.35);
  RacelineSplinePlanner feasible_planner(testParameters());
  ASSERT_TRUE(feasible_planner.setReference(feasible_reference));
  std::string reason;
  EXPECT_TRUE(feasible_planner.validatePath(
      EgoFrenetState{0.0, 0.23, 2.0},
      makeStraightCandidate(feasible_reference, 0.23, 0.0), {}, &reason)) << reason;

  const auto blocked_reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner blocked_planner(testParameters());
  ASSERT_TRUE(blocked_planner.setReference(blocked_reference));
  EXPECT_FALSE(blocked_planner.validatePath(
      EgoFrenetState{0.0, 0.23, 2.0},
      makeStraightCandidate(blocked_reference, 0.23, 0.0), {}, &reason));
}

TEST(RacelineSplinePlanner, BreaksCentredObstacleScoreTieWithTrackHeadroom)
{
  auto reference = makeCircularReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.d_left = 0.9;
  }
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // The right side offers more balanced obstacle/wall safety slack than the narrow left side.
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 8.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, -0.35);
}

TEST(RacelineSplinePlanner, HonorsCommittedSideWhenItRemainsFeasible)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_TRUE(result.go_left);
}

TEST(RacelineSplinePlanner, AppliesSingleSafetyMarginToAvoidanceTarget)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + vehicle half-width 0.12 + the only safety margin 0.03 = 0.35 최소치.
  // P3는 최소 clearance 지점이 아니라 slack이 가장 큰 후보를 고르므로 그보다 멀 수 있다.
  // 여기서 지켜야 할 계약은 "안전마진이 정확히 한 번만 적용된다"이므로 하한으로 검사한다.
  EXPECT_GE(result.target_d, 0.35 - 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesTrackingErrorReserveAsSeparateClearanceTerm)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.14;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.29, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.0);

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + half-width 0.12 + safety 0.03 + tracking reserve 0.14 = 0.49 최소치.
  // 추종오차 예약이 별도 항으로 한 번 더 들어간다는 계약을 하한으로 검사한다(위 참고).
  EXPECT_GE(result.target_d, 0.49 - 1.0e-9);
}

TEST(RacelineSplinePlanner, BilinearlyInterpolatesTrackingErrorLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.99;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 0.5};
  parameters.tracking_error_lut_values_m = {0.02, 0.04, 0.06, 0.10};

  ASSERT_TRUE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(-1.0, -0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(10.0, 2.0), 0.10, 1.0e-9);

  parameters.tracking_error_lut_values_m.pop_back();
  EXPECT_FALSE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.99, 1.0e-9);
}

TEST(RacelineSplinePlanner, LimitsAvoidanceSpeedFromVelocityTable)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5};

  ASSERT_TRUE(parameters.avoidanceVelocityLimitValid());
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(6.5, 0.0), 6.5);
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(3.0, 0.5), 3.0);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.2), 5.754463, 1.0e-6);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.5), std::sqrt(14.0), 1.0e-9);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(-6.5, -0.9), std::sqrt(7.0 / 0.9), 1.0e-9);

  // 횡가속 표는 속도에 대해 비증가이고, 1차 cap 이하의 속도는 계속 feasible하다. 따라서
  // gap cap 뒤에 같은 함수를 다시 부르던 종전 호출은 멱등(no-op)임을 계약으로 고정한다.
  const double first_cap = parameters.limitedAvoidanceSpeed(6.5, 0.5);
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(first_cap, 0.5), first_cap);
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(0.8 * first_cap, 0.5), 0.8 * first_cap);

  parameters.avoidance_velocity_limit_lateral_accel_mps2[2] = 7.5;
  EXPECT_FALSE(parameters.avoidanceVelocityLimitValid());
}

TEST(RacelineSplinePlanner, UsesLimitedAvoidanceSpeedForObstacleTrackingLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 4.0, 6.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.04, 0.12};
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};

  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.0, 0.5), 2.0, 1.0e-9);
  EXPECT_NEAR(parameters.avoidanceTrackingErrorReserve(6.0, 0.5), 0.02, 1.0e-9);
  EXPECT_NEAR(parameters.obstacleSafetyClearance(6.0, 0.5), 0.17, 1.0e-9);
}

TEST(RacelineSplinePlanner, CapsPublishedAvoidanceWaypointSpeeds)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};
  auto reference = makeStraightReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 6.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(33, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LE(
      waypoint.vx_mps * waypoint.vx_mps * std::abs(waypoint.kappa_radpm),
      2.0 + 1.0e-9);
  }
}

TEST(RacelineSplinePlanner, HoldsCriticalCurvatureSpeedPastDetectorRearWithZeroPadding)
{
  auto parameters = testParameters();
  parameters.obstacle_reserve_from_lut = false;
  parameters.localization_reserve_m = 0.0;
  parameters.confirmed_obstacle_speed_envelope_enable = true;
  parameters.analytic_path_geometry_enable = true;
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 10.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};
  parameters.avoidance_velocity_limit_accel_mps2 = {3.0, 3.0};
  parameters.avoidance_velocity_limit_decel_mps2 = {2.0, 2.0};
  // 운영값을 그대로 재현한다. padding=0이어도 속도 hold는 detector 뒤 1.0 m를 보장해야
  // 한다. 기하 padding을 올려 이 결함을 가리면 후보 모양과 채택률까지 함께 바뀐다.
  parameters.obstacle_longitudinal_padding_m = 0.0;
  parameters.confirmed_speed_post_hold_distance_m = 1.0;
  auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 6.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 6.0};
  const auto result = planner.evaluateP3Shadow(
    ego, {makeObstacle(330, 7.0)}, 1, 1U, 1U, "CONFIRMED_SPEED_ENVELOPE_TEST");

  bool checked_candidate = false;
  bool checked_flat_plateau = false;
  bool checked_post_detector_hold = false;
  for (const auto & candidate : result.candidates) {
    if (!std::isfinite(candidate.confirmed_critical_speed_mps) || candidate.path.wpnts.empty()) {
      continue;
    }
    checked_candidate = true;
    EXPECT_LT(
      candidate.confirmed_speed_hold_start_forward_m,
      candidate.confirmed_speed_hold_end_forward_m);
    // detector s_end=7.2 m, padding=0, speed-only post hold=1.0 m이므로 8.2 m다.
    EXPECT_NEAR(candidate.confirmed_speed_hold_end_forward_m, 8.2, 1.0e-9);
    for (const auto & waypoint : candidate.path.wpnts) {
      const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
      if (forward + 1.0e-6 < candidate.confirmed_speed_hold_start_forward_m ||
        forward > candidate.confirmed_speed_hold_end_forward_m + 1.0e-6)
      {
        continue;
      }
      EXPECT_LE(waypoint.vx_mps, candidate.confirmed_critical_speed_mps + 1.0e-9);
      if (forward > 7.2 + 1.0e-6) {
        checked_post_detector_hold = true;
      }
      const double pointwise_cap = parameters.limitedAvoidanceSpeed(6.0, waypoint.kappa_radpm);
      if (pointwise_cap > candidate.confirmed_critical_speed_mps + 0.1) {
        checked_flat_plateau = true;
      }
    }
  }
  EXPECT_TRUE(checked_candidate);
  EXPECT_TRUE(checked_flat_plateau);
  EXPECT_TRUE(checked_post_detector_hold);
}

TEST(RacelineSplinePlanner, ConfirmedPostHoldDoesNotDoubleCountLongitudinalPadding)
{
  RacelineSplineParameters parameters;
  parameters.confirmed_speed_post_hold_distance_m = 1.0;

  // 인자는 detector s_end에 padding이 이미 더해진 station[3]이다.
  parameters.obstacle_longitudinal_padding_m = 0.0;
  EXPECT_NEAR(parameters.confirmedSpeedHoldEndForwardM(7.2), 8.2, 1.0e-12);
  parameters.obstacle_longitudinal_padding_m = 0.5;
  EXPECT_NEAR(parameters.confirmedSpeedHoldEndForwardM(7.7), 8.2, 1.0e-12);
  parameters.obstacle_longitudinal_padding_m = 1.2;
  EXPECT_NEAR(parameters.confirmedSpeedHoldEndForwardM(8.4), 8.4, 1.0e-12);
}

TEST(RacelineSplinePlanner, ConfirmedCriticalSpeedIgnoresRemoteExitLowSpeed)
{
  auto parameters = testParameters();
  parameters.obstacle_reserve_from_lut = false;
  parameters.localization_reserve_m = 0.0;
  parameters.confirmed_obstacle_speed_envelope_enable = true;
  parameters.analytic_path_geometry_enable = true;
  parameters.obstacle_longitudinal_padding_m = 0.0;
  parameters.confirmed_speed_post_hold_distance_m = 1.0;
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 10.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};
  parameters.avoidance_velocity_limit_accel_mps2 = {3.0, 3.0};
  parameters.avoidance_velocity_limit_decel_mps2 = {2.0, 2.0};
  auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  for (auto & waypoint : reference.wpnts) {
    // detector 뒤끝(7.2 m)보다 먼 exit 구간만 매우 낮춘다. 종전 station[4] 표본이면 이
    // 0.25 m/s가 장애물 통과 전체를 묶지만, station[3] 표본이면 exit 자신의 점별 cap으로만
    // 남아야 한다.
    waypoint.vx_mps = waypoint.s_m >= 7.4 && waypoint.s_m <= 11.0 ? 0.25 : 6.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.evaluateP3Shadow(
    EgoFrenetState{0.0, 0.0, 6.0}, {makeObstacle(331, 7.0)},
    1, 1U, 1U, "CONFIRMED_EXIT_WINDOW_TEST");

  bool checked_candidate = false;
  for (const auto & candidate : result.candidates) {
    if (!std::isfinite(candidate.confirmed_critical_speed_mps)) {
      continue;
    }
    checked_candidate = true;
    EXPECT_GT(candidate.confirmed_critical_speed_mps, 0.25 + 1.0e-6);
  }
  EXPECT_TRUE(checked_candidate);
}

TEST(RacelineSplinePlanner, UsesObstacleSpanMaximumTrackingLutReserveForTarget)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.10};
  // 트랙을 좁혀 유효 target 창을 [0.45, 0.46]으로 만든다: 벽 상한 = 0.58 − 0.12(반폭)
  // − 0.0(기본 벽마진) = 0.46. 스팬 최대 LUT 예약(0.10)을 빠뜨린 플래너라면 0.45 미만
  // (예: [0.35, 0.46]의 slack 최대 지점)을 골라 하한 검사에 걸린다.
  auto reference = makeStraightReference(300, 0.1, 0.58, 0.58);
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = waypoint.s_m >= 6.0 && waypoint.s_m <= 8.0 ? 3.0 : 0.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(31, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + base clearance 0.15 + maximum span LUT reserve 0.10 = 0.45 최소치.
  EXPECT_GE(result.target_d, 0.45 - 1.0e-9);
  EXPECT_LE(result.target_d, 0.46 + 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesOnlyWallMarginToTrackBounds)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.0, 0.10, 0.10};
  parameters.wall_safety_margin_m = 0.04;
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(3.0, 1.0), 0.04);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.62, 0.62)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  // 하한 = 장애물 clearance(0.45), 상한 = 벽 한계 0.62 − 0.12(반폭) − 0.04(벽마진) = 0.46.
  // P3는 이 창 안에서 slack 최대 지점을 고르므로 정확값이 아니라 창 준수를 계약으로 본다.
  EXPECT_GE(feasible.target_d, 0.45 - 1.0e-9);
  EXPECT_LE(feasible.target_d, 0.46 + 1.0e-9);

  // 0.60 m of room no longer safe-stops: the strict gate does not fit, but slowing the pass to
  // avoidance_minimum_speed_mps shrinks the reserve enough that a target does. Only a corridor too
  // narrow even at that floor is refused, and the target gate now says so before any spline is
  // built instead of generating candidates the footprint check must throw away.
  RacelineSplinePlanner slowed_planner(parameters);
  ASSERT_TRUE(slowed_planner.setReference(makeStraightReference(300, 0.1, 0.60, 0.60)));
  const auto slowed = slowed_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(slowed.kind, SplinePlanKind::kAvoidance) << slowed.reason;
  EXPECT_LT(slowed.target_d, 0.45);
  EXPECT_LE(slowed.target_d + parameters.vehicle_half_width_m, 0.60 - 0.04 + 1.0e-9);

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.55, 0.55)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
  // 사유 문자열은 후보 생성기(P3)의 것이므로 판정만 계약으로 본다.
  EXPECT_FALSE(blocked.reason.empty());
}

TEST(RacelineSplinePlanner, AppliesIndependentWallSafetyMarginOnlyToTrackBounds)
{
  auto parameters = testParameters();
  parameters.wall_safety_margin_m = 0.05;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.15, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.05);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.53, 0.53)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  // 하한 = clearance 0.35, 상한 = 0.53 − 0.12(반폭) − 0.05(벽마진) = 0.36 (위 테스트 참고).
  EXPECT_GE(std::abs(feasible.target_d), 0.35 - 1.0e-9);
  EXPECT_LE(std::abs(feasible.target_d), 0.36 + 1.0e-9);

  const auto tangent_reference = makeStraightReference(300, 0.1, 0.40, 0.40);
  RacelineSplinePlanner tangent_planner(parameters);
  ASSERT_TRUE(tangent_planner.setReference(tangent_reference));
  std::string reason;
  EXPECT_TRUE(tangent_planner.validatePath(
      EgoFrenetState{0.0, 0.23, 2.0},
      makeStraightCandidate(tangent_reference, 0.23, 0.0), {}, &reason)) << reason;

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.39, 0.39)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
}

TEST(RacelineSplinePlanner, RejectsCommittedPathWhenObstacleEnvelopeGrows)
{
  // 왼쪽 폭 0.475로 커밋 경로의 plateau를 [0.35, 0.355]로 강제한다(벽 상한 = 0.475 −
  // 0.12(반폭) − 0.0(기본 벽마진) = 0.355). 그래야 장애물 1 cm 성장(외피 0.35 → 0.36)이
  // 실제 위반이 된다. 넓은 트랙에서는 P3가 slack 최대 지점을 골라 위반이 안 난다.
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.475, 1.5)));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  std::string reason;
  EXPECT_TRUE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0)}, &reason)) << reason;
  EXPECT_FALSE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0, -0.21, 0.21)}, &reason));
}

TEST(RacelineSplinePlanner, UsesSameClearanceForGuardAndRawObstacleInputs)
{
  // 위 테스트와 같은 이유로 plateau를 [0.35, 0.355]로 강제한다.
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.475, 1.5)));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(23, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  // Soft/hard behavior differs only by the input envelope. Both calls add the same 0.15 m:
  // the uncertainty Guard reaches d=0.21 and collides, while raw geometry reaches d=0.19
  // and remains clear.
  const auto uncertainty_guard = makeObstacle(23, 7.0, -0.20, 0.21);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {uncertainty_guard}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_EQ(failure.obstacle_id, 23);
  EXPECT_TRUE(std::isfinite(failure.waypoint_s));
  EXPECT_TRUE(std::isfinite(failure.waypoint_d));
  EXPECT_NEAR(failure.obstacle_source_d_left, 0.21, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_test_d_left, 0.36, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_clearance, 0.15, 1.0e-9);

  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {makeObstacle(23, 7.0, -0.20, 0.19)},
      &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

TEST(RacelineSplinePlanner, IgnoresPostMergeTailCollisionForCurrentCommitment)
{
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.5, 3.0, 4.0};
  parameters.transition_distance_scales = {1.0};
  // P3 경로는 merge가 s≈15에 온다. merge 뒤에 놓는 다음 장애물이 기본 lookahead(12 m)
  // 밖으로 나가 검증이 공허하게 통과하지 않도록 늘린다.
  parameters.detection_lookahead_m = 20.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(24, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const double next_obstacle_s = committed.merge_s + 0.7;
  const auto next_obstacle = makeObstacle(25, next_obstacle_s, -0.20, 0.20);
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
      merge_horizon)) << reason;
}

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

TEST(RacelineSplinePlanner, DoesNotReverseCommittedSideWhenItBecomesBlocked)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto unlocked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, true);
  ASSERT_EQ(unlocked.kind, SplinePlanKind::kAvoidance) << unlocked.reason;
  EXPECT_FALSE(unlocked.go_left);

  const auto locked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, false);
  EXPECT_EQ(locked.kind, SplinePlanKind::kSafeStop) << locked.reason;
}

TEST(RacelineSplinePlanner, IgnoresObstacleWithEnoughRawRacelineClearance)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(4, 7.0, 0.35, 0.55)});
  EXPECT_EQ(result.kind, SplinePlanKind::kNoObstacle) << result.reason;
  EXPECT_TRUE(result.path.wpnts.empty());
}

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
  EXPECT_TRUE(std::any_of(
      result.path.wpnts.begin(), result.path.wpnts.end(),
      [&parameters](const auto & waypoint) {
        return waypoint.ax_mps2 <
               -0.99 * parameters.safe_stop_deceleration_mps2;
      }));

  const auto repeated = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(repeated.kind, SplinePlanKind::kSafeStop) << repeated.reason;
  ASSERT_EQ(repeated.path.wpnts.size(), result.path.wpnts.size());
  for (std::size_t index = 0; index < result.path.wpnts.size(); ++index) {
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].vx_mps, result.path.wpnts[index].vx_mps);
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].ax_mps2, result.path.wpnts[index].ax_mps2);
  }
}

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

TEST(RacelineSplinePlanner, TagsCommittedStopWithoutCollisionFreePrefix)
{
  RacelineSplinePlanner planner(testParameters());
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  auto committed_path = makeStraightCandidate(reference, 0.0, 0.0, 20U);
  for (auto & waypoint : committed_path.wpnts) {
    waypoint.vx_mps = 2.0;
  }

  // 첫 waypoint부터 장애물 종·횡 envelope 안이다. 충돌 전 점이 하나뿐이므로 제동 prefix를
  // 만들 수 없고, 일반 문자열이 아니라 운영 진단에서 검색 가능한 고정 reason을 내야 한다.
  const auto result = planner.buildCommittedPathStop(
    EgoFrenetState{0.0, 0.0, 2.0}, committed_path,
    {makeObstacle(290, 0.05, -1.0, 1.0)});

  EXPECT_EQ(result.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(result.path.wpnts.empty());
  EXPECT_EQ(result.reason.rfind("SAFE_STOP_NO_COLLISION_FREE_PREFIX:", 0U), 0U);
}

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

// 짧은 안전정지 prefix는 여전히 허용된다(minimum_path_points 미달을 이유로 거부하지
// 않는다). 다만 2026-08-14부터는 그 상태로 내보내지 않고 minimum_path_points까지
// 세분 보간한다. 제어기가 룩어헤드 지점의 속도를 읽기 때문에, 2~3점짜리 경로에서는
// 룩어헤드가 곧바로 끝점 0에 걸려 감속 프로파일을 통째로 건너뛰고 즉시 정지를 명령한다
// (실차 관측: /local_waypoints [1.08, 0.00] -> /drive_autonomous 0.00, 28.8초 교착).
// 보간은 점 수만 늘릴 뿐 정지 지점(기하 구간)을 늘려서는 안 된다.
TEST(RacelineSplinePlanner, DensifiesShortSafeStopPrefixToMinimumPoints)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.65, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 8U);

  // 기하 구간은 짧은 그대로여야 한다 — 보간이 정지점을 밀어내지 않았음을 확인한다.
  const double span = result.path.wpnts.back().s_m - result.path.wpnts.front().s_m;
  EXPECT_LT(span, 1.30);

  // 감속 프로파일이 살아 있어야 한다: 단조 비증가 + 종점 0.
  EXPECT_DOUBLE_EQ(result.path.wpnts.back().vx_mps, 0.0);
  EXPECT_GT(result.path.wpnts.front().vx_mps, 0.0);
  for (std::size_t i = 1U; i < result.path.wpnts.size(); ++i) {
    EXPECT_LE(result.path.wpnts[i].vx_mps, result.path.wpnts[i - 1U].vx_mps + 1e-9)
      << "index " << i;
  }
}

// 정지점 탈출 검증: 정지한 자리에서 회피 후보가 하나도 생성되지 않으면 그 사실이
// 결과에 남아야 한다. 이전에는 아무 표시 없이 정지해 현장에서 30초씩 매달렸다.
TEST(RacelineSplinePlanner, ReportsWhenSafeStopPointIsNotEscapable)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  parameters.safe_stop_escape_check_enable = true;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  // 트랙 폭을 가득 막는 장애물 — 어느 지점에서도 회피가 불가능하다.
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 3.0, -1.20, 1.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_FALSE(result.safe_stop_escape_verified);
  EXPECT_NE(result.reason.find("no escapable stop point"), std::string::npos);
  // 탈출이 어차피 불가능하면 후퇴는 아무것도 사지 못하므로 원래 정지점을 지켜야 한다.
  EXPECT_GT(result.safe_stop_forward_m, 1.0);
}

// 🔴 좌표계 회귀 가드 (2026-08-14 리뷰). ExpandedObstacle의 center/start/end는 자차
// 상대거리라, 가상의 정지점으로 ego.s만 옮기고 기존 visible/cluster를 재사용하면 장애물이
// 정지점에서도 같은 거리에 있는 것으로 보여 검증이 통째로 무의미해진다(이분탐색도 항상
// 같은 답을 낸다). buildSafeStop은 반드시 절대 s 원본으로 정지점 기준 재확장해야 한다.
//
// 검사 방법: 버퍼를 탈출 임계보다 크게 잡아 요청 정지점을 "장애물에서 너무 먼" 쪽이 아니라
// 자차에 가까운 쪽으로 두고, 요청 정지점과 실제 채택된 정지점이 다른지 본다. 좌표계가
// 틀렸다면 재확장이 없으므로 후퇴 탐색이 아무 효과를 못 내고 요청값 그대로 남는다.
TEST(RacelineSplinePlanner, EscapeCheckReexpandsObstaclesAtTheCandidateStopPoint)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  // 임계보다 작은 버퍼 → 요청 정지점은 탈출 불가 구역 안. 검증이 살아 있으면 뒤로 물린다.
  parameters.safe_stop_buffer_m = 0.30;
  parameters.safe_stop_escape_check_enable = true;
  parameters.safe_stop_escape_retreat_step_m = 0.10;
  parameters.safe_stop_escape_max_retreats = 10;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto obstacle = makeObstacle(5, 6.0, -0.40, 0.40);
  const auto result = planner.plan(ego, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;

  if (result.safe_stop_escape_verified) {
    // 후퇴가 성공했다면 채택된 정지점에서 실제로 회피가 나와야 한다 — 판정과 재계획이
    // 같은 후보 생성기를 쓰므로 이 두 값은 반드시 일치한다.
    const EgoFrenetState at_stop{
      ego.s + result.safe_stop_forward_m, ego.d, 0.0};
    const auto replan = planner.plan(at_stop, {obstacle});
    EXPECT_EQ(replan.kind, SplinePlanKind::kAvoidance)
      << "정지점에서 회피 가능하다고 판정했는데 실제 재계획은 실패했다: " << replan.reason;
  }
  // 좌표계가 틀렸을 때 나타나는 형태: 정지점이 자차 뒤로 가거나 장애물을 넘어선다.
  EXPECT_GE(result.safe_stop_forward_m, 0.0);
  EXPECT_LT(result.safe_stop_forward_m, 6.0);
}

// 탈출 검증을 끄면 이전 동작(정지점 무검증)으로 돌아간다 — 회귀 시 즉시 되돌릴 수 있어야 한다.
TEST(RacelineSplinePlanner, SafeStopEscapeCheckCanBeDisabled)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  parameters.safe_stop_escape_check_enable = false;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 3.0, -1.20, 1.20)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_TRUE(result.safe_stop_escape_verified);   // 검증 자체를 안 했으므로 참으로 둔다
  EXPECT_EQ(result.reason.find("no escapable stop point"), std::string::npos);
}

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

TEST(RacelineSplinePlanner, HandlesObstacleAcrossTrackWrap)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  auto obstacle = makeObstacle(9, 0.30);
  obstacle.s_start = 0.10;
  obstacle.s_end = 0.50;
  constexpr double kEgoS = 24.0;
  const auto result = planner.plan(EgoFrenetState{kEgoS, 0.0, 2.0}, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  bool wrapped = false;
  double previous = -1.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(kEgoS, waypoint.s_m);
    EXPECT_GT(forward, previous);
    previous = forward;
    wrapped = wrapped || waypoint.s_m < 1.0;
  }
  EXPECT_TRUE(wrapped);
}

TEST(RacelineSplinePlanner, NeverJumpsToNearbyWrongSnakeBranch)
{
  auto reference = makeStraightReference();
  for (std::size_t i = 220U; i < reference.wpnts.size(); ++i) {
    reference.wpnts[i].x_m = 30.0 - reference.wpnts[i].s_m;
    reference.wpnts[i].y_m = 0.55;
    reference.wpnts[i].psi_rad = 3.14159265358979323846;
  }
  // 복귀 가지가 y=0.55에 있으므로 자유폭도 그에 맞게 제한한다. 폭을 1.5로 두면 P3가
  // slack 최대 지점(0.86)까지 합법적으로 벌려 기하 모순(가지 관통)이 생긴다. 0.70이면
  // plateau ≤ 0.70 − 0.12 − 0.10 = 0.48 < 0.55로 어느 측을 골라도 가지 앞에서 멈춘다.
  for (auto & waypoint : reference.wpnts) {
    waypoint.d_left = 0.70;
    waypoint.d_right = 0.70;
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
    EXPECT_LT(waypoint.s_m, 22.0)
      << "planner selected a geometrically nearby but topologically wrong snake branch";
    EXPECT_LT(waypoint.y_m, 0.55)
      << "path must remain an offset of the ordered first race-line branch";
  }
}

// A gap wide enough for the full-speed reserve must not cost any speed: obstacle-free laps and
// roomy avoidances keep the race-line profile.
TEST(RacelineSplinePlanner, WideGapKeepsRaceLineSpeed)
{
  const auto reference = makeStraightReference();
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.10;
  parameters.avoidance_minimum_speed_mps = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 3.0);
  }
}

// The lateral-acceleration cap is blind to a straight, so before the gap-driven limit an obstacle
// on a straight was planned at full race-line speed and reserved the widest tracking error the LUT
// has. Squeeze the corridor until only the slow end of the LUT fits and the pass must slow down.
TEST(RacelineSplinePlanner, TightGapOnStraightSlowsDownInsteadOfReservingFullSpeedError)
{
  // 0.40 m of room each side: the race-line-speed reserve (0.30) cannot fit beside the obstacle,
  // the slow-end reserve (0.02) can.
  const auto reference = makeStraightReference(300, 0.1, 0.45, 0.45);
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 1.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 1.5, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.02, 0.02, 0.02, 0.02, 0.30, 0.30};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0, -0.05, 0.05)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  double slowest = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    slowest = std::min(slowest, waypoint.vx_mps);
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1.0e-9);
    EXPECT_GE(waypoint.vx_mps, parameters.avoidance_minimum_speed_mps - 1.0e-9);
  }
  EXPECT_LT(slowest, 3.0) << "the pass should have been slowed to afford its reserve";
}

// Speed may only be traded for reserve down to the configured floor. Below it the maneuver is
// reported infeasible rather than crawled through, so safe-stop stays the authority.
TEST(RacelineSplinePlanner, GapLimitedSpeedNeverFallsBelowTheFloor)
{
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 2.5;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 6.5};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.05, 0.30, 0.30};

  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 1.0), 6.0);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, -1.0), 2.5);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(2.0, 0.0, -1.0), 2.0);

  const double capped = parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 0.15);
  EXPECT_GE(capped, 2.5);
  EXPECT_LT(capped, 6.0);
  EXPECT_LE(parameters.trackingErrorReserve(capped, 0.0), 0.15 + 1.0e-6);
}

TEST(RacelineSplinePlanner, MaximumExitLengthCapsCombinedExitScale)
{
  // The slack ranking always prefers the gentlest (longest) exit, and without an absolute cap
  // post_apex_far x transition_long extends the published avoidance path up to ~18 m past the
  // obstacle, deferring the merge back to the global line by that whole tail.
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.0, 2.0, 5.0};
  parameters.maximum_exit_length_m = 6.0;
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(0.5), 0.5);
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 6.0 / 5.0);

  parameters.maximum_exit_length_m = 0.0;  // non-positive disables the cap
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 3.58);
}

// P0 격자를 제거하고 plan()의 후보 생성기를 P3로 옮긴 뒤에도, plan()은 여전히
// "회피가 가능하면 kAvoidance"라는 계약을 지켜야 한다. 이 계약이 깨지면 연쇄 기동
// (tryEarlyChainedManeuver)·안전정지 해제 조건 B·안정화 중 조기 회피가 전부 죽는다 —
// 2026-08-15 시뮬에서 실제로 그렇게 되어 차가 다음 장애물 앞에서 정지했다.
TEST(RacelineSplinePlanner, PlanReturnsAvoidanceForAPassableObstacle)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(401, 8.0)}, std::nullopt, true);
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.path.wpnts.empty());
  EXPECT_GT(std::abs(result.target_d), 0.0);
}

// 연쇄 기동이 쓰는 형태: 자차가 이미 라인에서 벗어나 있고(직전 회피의 여파) 다음 장애물이
// 앞에 있는 상태. 여기서 kAvoidance가 나오지 않으면 연쇄가 성립하지 않는다.
TEST(RacelineSplinePlanner, PlanChainsFromANonZeroEgoOffset)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 1.20, 1.20)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, -0.35, 2.0}, {makeObstacle(402, 9.0)}, std::nullopt, true);
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.path.wpnts.empty());
}

TEST(RacelineSplinePlanner, MarginOnlyClusterDegradesToSlowPassInsteadOfSafeStop)
{
  // Track so narrow that both spline sides fail. The obstacle sits entirely left of the line:
  // margin-blocking through the 0.3 m fallback reserve, but its raw envelope plus the physical
  // clearance (0.12 + 0.03) never reaches d = 0, so the line itself stays drivable.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto margin_only = makeObstacle(90, 5.0, 0.20, 0.60);

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 접근 실현성 램프: 자차(3.0 m/s)가 cap(2.0)보다 빠르므로 앞머리는 flat 2.0이 아니라
  // 자차 속도에서 내려오는 프로파일이다. 전 구간 단조 비증가 + 자차 속도 이하이고,
  // 장애물 스팬(및 그 이후)은 cap을 넘지 않아야 한다.
  double previous = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
    EXPECT_LE(waypoint.vx_mps, ego.speed + 1e-9);
    EXPECT_LE(waypoint.vx_mps, previous + 1e-9);
    previous = waypoint.vx_mps;
    if (waypoint.s_m >= margin_only.s_start) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  EXPECT_GT(result.path.wpnts.front().vx_mps, 2.5);   // 계단(즉시 2.0) 금지 = 램프 실존
  EXPECT_GT(result.merge_s, margin_only.s_end);

  // Contrast: the same track with a genuinely line-straddling obstacle must still stop.
  const auto physically_blocking = planner.plan(ego, {makeObstacle(91, 5.0)});
  EXPECT_EQ(physically_blocking.kind, SplinePlanKind::kSafeStop);
  EXPECT_FALSE(physically_blocking.margin_pass);
}

TEST(RacelineSplinePlanner, MarginPassApproachRampReachesCapBeforeClusterStart)
{
  // 0814 실차 회귀(run_0814_111210): 4.4 m/s 접근에 flat 2.0 margin pass가 발행돼 계단
  // 감속 → 서비스 브레이크 포화 → 마찰 한계 초과 슬립 → 벽. 램프는 실측 자차 속도에서
  // approach_feasibility_decel로 내려가되 군집 시작 전에 cap에 도달해야 한다.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto margin_only = makeObstacle(92, 9.0, 0.20, 0.60);
  const EgoFrenetState ego{0.0, 0.0, 4.4};

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 필요 감속 (4.4²-2.0²)/(2·~8.5) ≈ 0.9 < 2.0 → 완만한 파라미터 감속이 그대로 쓰이고,
  // cap 도달 지점은 (4.4²-2.0²)/(2·2.0) = 3.84 m — 군집 시작(≈8.5 m)보다 훨씬 앞이다.
  const double reach_cap_at = (ego.speed * ego.speed - 4.0) / (2.0 * 2.0);
  for (const auto & waypoint : result.path.wpnts) {
    // 프로파일 상한(3.0)은 절대 넘지 않는다: 램프값이 그보다 커도 참조 프로파일이 이긴다.
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1e-9);
    if (waypoint.s_m >= reach_cap_at + 0.2) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  // 앞머리는 램프를 따른다(참조 프로파일 3.0에 클램프): flat 2.0이 아니어야 한다.
  EXPECT_NEAR(result.path.wpnts.front().vx_mps, 3.0, 1e-9);

  // 비활성(<=0)이면 구 거동(flat cap) 그대로다.
  parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner flat_planner(parameters);
  ASSERT_TRUE(flat_planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto flat = flat_planner.plan(ego, {margin_only});
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  for (const auto & waypoint : flat.path.wpnts) {
    EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
  }
}

TEST(RacelineSplinePlanner, AvoidanceApproachBrakesBeforeSpanInsteadOfStepping)
{
  // 회피 스플라인의 접근 구간: 간극/곡률 캡은 스팬 안에서만 작동하므로 접근은 프로파일
  // 속도 그대로다가 스팬 경계에서 계단으로 떨어진다. 후방 제동 램프는 그 계단을 스팬
  // 시작 속도로 미리 내려가는 프로파일로 바꾼다(낮추기만 함). 스팬 내부는 비트 동일.
  auto ramp_parameters = testParameters();
  // 속도에 가파르게 커지는 추적오차 예약 LUT + 좁은 왼쪽 통로: 스팬 안에서 간극 캡이
  // 프로파일(3.0)보다 확실히 낮은 속도를 강제해 "접근 빠름 / 스팬 느림" 계단을 만든다.
  ramp_parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0, 4.0};
  ramp_parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  ramp_parameters.tracking_error_lut_values_m = {0.05, 0.08, 0.60};
  ramp_parameters.approach_feasibility_decel_mps2 = 2.0;
  auto flat_parameters = ramp_parameters;
  flat_parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner ramp_planner(ramp_parameters);
  RacelineSplinePlanner flat_planner(flat_parameters);
  const auto reference = makeStraightReference(300, 0.1, 0.6, 0.6);
  ASSERT_TRUE(ramp_planner.setReference(reference));
  ASSERT_TRUE(flat_planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  // 비대칭 장애물(오른쪽으로 치우침): 후보 랭킹 1순위(safety slack)에서 왼쪽이 명확히
  // 이기게 해, 램프로 달라지는 2순위(velocity_loss)가 후보 선택을 못 바꾸게 고정한다.
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(93, 8.0, -0.35, 0.05)};

  const auto ramped = ramp_planner.plan(ego, obstacles);
  const auto flat = flat_planner.plan(ego, obstacles);
  ASSERT_EQ(ramped.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(ramped.path.wpnts.size(), flat.path.wpnts.size());
  const double span_start = obstacles.front().s_start;
  bool lowered_somewhere = false;
  for (std::size_t index = 0; index < ramped.path.wpnts.size(); ++index) {
    const auto & with_ramp = ramped.path.wpnts[index];
    const auto & without = flat.path.wpnts[index];
    ASSERT_DOUBLE_EQ(with_ramp.s_m, without.s_m);
    // 램프는 어디서도 속도를 올리지 않는다.
    EXPECT_LE(with_ramp.vx_mps, without.vx_mps + 1e-9);
    if (with_ramp.s_m >= span_start) {
      // 스팬 및 그 이후는 손대지 않는다(간극/예약 캡 보존).
      EXPECT_DOUBLE_EQ(with_ramp.vx_mps, without.vx_mps);
    } else if (with_ramp.vx_mps < without.vx_mps - 1e-6) {
      lowered_somewhere = true;
    }
  }
  EXPECT_TRUE(lowered_somewhere);
}

TEST(RacelineSplinePlanner, RetentionReserveScaleHoldsCommittedPathThroughEnvelopeGrowth)
{
  // Full clearance = 0.15 (base) + 0.30 (reserve) = 0.45; retention 0.5 keeps 0.15 + 0.15.
  // The obstacle's grown left edge (0.42) violates the full margin against a path at d = 0.8
  // (0.42 + 0.45 > 0.8) but stays inside the retention band (0.42 + 0.30 < 0.8).
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.8, 2.0};
  const auto path = makeStraightCandidate(reference, 0.8, 0.0, 60U);

  const auto grown = makeObstacle(7, 2.0, -0.2, 0.42);
  PathValidationFailure failure;
  EXPECT_FALSE(planner.validatePath(ego, path, {grown}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_TRUE(planner.validatePath(ego, path, {grown}, nullptr, nullptr, std::nullopt, 0.5));

  // Growth past the retention band must still invalidate the committed path.
  const auto beyond_retention = makeObstacle(8, 2.0, -0.2, 0.55);
  EXPECT_FALSE(
    planner.validatePath(ego, path, {beyond_retention}, nullptr, nullptr, std::nullopt, 0.5));

  // The same contract through the P3 suffix validator.
  EXPECT_FALSE(planner.evaluateP3PathCurrent(ego, path, {grown}).hard_valid);
  EXPECT_TRUE(planner.evaluateP3PathCurrent(ego, path, {grown}, 0.5).hard_valid);
}

TEST(RacelineSplinePlanner, LocalizationReserveAddsConstantFloorAndScalesWithRetention)
{
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  const double base = parameters.obstacleBaseClearance();
  const double without = parameters.obstacleSafetyClearance(2.0, 0.0);
  EXPECT_NEAR(without, base + 0.30, 1e-9);

  parameters.localization_reserve_m = 0.06;
  // Full reserve: the localization floor adds verbatim.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), base + 0.36, 1e-9);
  // Retention scale halves the WHOLE reserve including the localization floor.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.5), base + 0.18, 1e-9);
  // The base clearance itself is never touched.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.0), base, 1e-9);
  // The gap-limited speed inversion sees the same floor: a gap that admits exactly the
  // tracking reserve no longer fits once the localization floor is added, so the requested
  // speed must drop to the avoidance floor (the constant reserve cannot be shed by slowing).
  const double at_floor = parameters.gapLimitedAvoidanceSpeed(4.0, 0.0, 0.30);
  EXPECT_NEAR(at_floor, parameters.avoidance_minimum_speed_mps, 1e-9);
}

TEST(RacelineSplinePlanner, BuildLastPathBrakeStopsAlongGivenGeometry)
{
  RacelineSplinePlanner planner(testParameters());
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const auto path = makeStraightCandidate(reference, 0.0, 0.0, 60U);
  const EgoFrenetState ego{1.0, 0.0, 2.0};

  const auto braked = planner.buildLastPathBrake(ego, path, {});
  ASSERT_GE(braked.wpnts.size(), 2U);
  // Stop distance from 2.0 m/s at the default 2.5 m/s^2 deceleration is 0.8 m.
  EXPECT_LE(planner.forwardDistance(ego.s, braked.wpnts.back().s_m), 0.8 + 0.2);
  EXPECT_DOUBLE_EQ(braked.wpnts.back().vx_mps, 0.0);
  for (std::size_t i = 1; i < braked.wpnts.size(); ++i) {
    EXPECT_LE(braked.wpnts[i].vx_mps, braked.wpnts[i - 1].vx_mps + 1e-9);
  }

  f110_msgs::msg::WpntArray empty_path;
  EXPECT_TRUE(planner.buildLastPathBrake(ego, empty_path, {}).wpnts.empty());

  // 🔴 2026-08-16 회귀: 장애물이 제동거리보다 가까우면 정지 목표가 장애물 뒤에 놓여
  // 차가 그 경로를 따라 들어갔다(15:37 백, 랩당 1회씩 4회 충돌). 정지 목표는 반드시
  // 접촉점 이전이어야 하고, 그 때문에 요구 감속이 설정값을 넘는 것은 의도된 동작이다.
  // 접촉점을 설정 제동거리(2.0 m/s, 2.5 m/s² → 0.8 m)보다 가깝게 둔다.
  // 확장 앞면 = (s_center − ego.s) − (0.5·span + obstacle_longitudinal_padding_m).
  const f110_msgs::msg::Obstacle blocker = makeObstacle(9, 2.1, -0.30, 0.30);
  const double contact_forward =
    (2.1 - ego.s) - (0.20 + testParameters().obstacle_longitudinal_padding_m);
  ASSERT_LT(contact_forward, 0.8) << "이 기하로는 클램프가 발동하지 않는다";
  const auto guarded = planner.buildLastPathBrake(ego, path, {blocker});
  ASSERT_GE(guarded.wpnts.size(), 2U);
  EXPECT_DOUBLE_EQ(guarded.wpnts.back().vx_mps, 0.0);
  for (const auto & waypoint : guarded.wpnts) {
    EXPECT_LT(planner.forwardDistance(ego.s, waypoint.s_m), contact_forward)
      << "경로가 접촉점(자차 +" << contact_forward << " m)을 넘어 연장됐다";
  }
  // 그리고 실제로 잘려야 한다 — 장애물이 없었다면 0.8 m까지 갔을 경로다.
  const double stop_forward = planner.forwardDistance(ego.s, guarded.wpnts.back().s_m);
  EXPECT_LT(stop_forward, 0.8) << "제동거리가 접촉점 기준으로 좁혀지지 않았다";
}

TEST(RacelineSplinePlanner, StandstillCloseBehindObstacleStillPlansEscape)
{
  // 2026-08-13 실차 재현 (run_0813_221339 s≈29.8): 코너 뒤 늦은 발견으로 장애물
  // ~1.5 m 앞에 정지. 갭은 기하학적으로 충분한데(비대칭 코리도 1.97 m, 반대쪽 여유
  // ~1.4 m) 정지 상태 재계획이 회피를 내지 못하면 safe-stop 홀드에서 영원히 못
  // 나온다. 진입 길이는 자차→클러스터 실거리에 비례하므로 짧은 거리에서도 후보가
  // 성립해야 한다.
  auto reference = makeStraightReference(300, 0.1, 1.40, 0.57);
  RacelineSplineParameters parameters;
  parameters.maximum_curvature_radpm = 3.2;
  parameters.maximum_curvature_rate_radpm2 = 60.0;
  // 실차 yaml과 같은 예약 구조: 기어가기 속도에서 LUT 바닥 0.20 + 위치추정 0.06.
  // 이 재현은 예약이 살아 있던 시절의 실차 사례이므로 게이트를 켠다 (2026-08-22).
  parameters.obstacle_reserve_from_lut = true;
  parameters.tracking_error_reserve_m = 0.20;
  parameters.localization_reserve_m = 0.06;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // 자차: 정지, 라인 살짝 오른쪽(-0.10). 장애물: 우벽 쪽 박스(폭 0.4), 전방
  // 클러스터 시작 ≈ 1.5 m (s_start 2.05 − 종방향 패딩 0.35 − 자차 s 0.2).
  const EgoFrenetState ego{0.2, -0.10, 0.0};
  const auto obstacle = makeObstacle(29, 2.45, -0.45, -0.05);

  const auto result = planner.plan(ego, {obstacle});
  EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());
  // 탈출은 여유가 있는 왼쪽으로 나가야 한다.
  double max_d = -10.0;
  for (const auto & waypoint : result.path.wpnts) {
    max_d = std::max(max_d, static_cast<double>(waypoint.d_m));
  }
  EXPECT_GT(max_d, 0.10);
}

// ── F3 (2026-08-22): 잘려 나간 정지 경로의 조향 기하를 되돌린다 ────────────────
//
// 안전정지 생성기는 정지 목표를 첫 접촉 지점 이전으로 자른다. 종방향으로는 옳지만 같은
// 절단이 횡방향 기하까지 잘라, 차가 장애물 바로 옆일 때(= 무효화가 나는 그 상황) 2~8 점
// 짜리 경로가 남는다. 컨트롤러의 walk_forward 는 열린 경로에서 끝점에 멈추므로 L1 목표가
// 전방 0.18 m 가 되고 요구 횡가속이 32 m/s² 로 튄다 (실차 073013 t=93.7, IMU 1.52 g).
//
// 이 시험들이 고정하는 계약: **기하는 늘고, 제동 프로파일은 안 바뀐다.**

TEST(RacelineSplinePlanner, ExtendStopGeometryLengthensTheSteeringPathToTheFloor)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.15, 4.6};

  // 원본 회피 기하 (40 점, 0.0~3.9 m) 와 그 접두부만 잘라 낸 정지 경로 (3 점, 0.5 m).
  const auto source = makeStraightCandidate(reference, 0.15, 0.0, 40U);
  f110_msgs::msg::WpntArray stop;
  stop.header = source.header;
  for (std::size_t i = 1; i <= 3U; ++i) {
    auto waypoint = source.wpnts[i];
    waypoint.id = static_cast<int32_t>(stop.wpnts.size());
    waypoint.vx_mps = static_cast<float>(1.2 - 0.4 * static_cast<double>(stop.wpnts.size()));
    stop.wpnts.push_back(waypoint);
  }
  stop.wpnts.back().vx_mps = 0.0f;
  const double span_before = planner.forwardSpanAheadOfEgo(stop, ego);
  ASSERT_NEAR(span_before, 0.3, 1.0e-9);
  const std::vector<float> profile_before{
    stop.wpnts[0].vx_mps, stop.wpnts[1].vx_mps, stop.wpnts[2].vx_mps};

  const std::size_t appended = planner.extendStopGeometry(stop, source, ego, 2.5);

  EXPECT_GT(appended, 0U);
  EXPECT_GE(planner.forwardSpanAheadOfEgo(stop, ego), 2.5)
    << "하한까지 안 늘었다 — L1 이 여전히 끝점에 물린다";

  // 🔴 제동 프로파일 불변: 원래 있던 점의 속도가 한 톨도 안 바뀌어야 한다.
  for (std::size_t i = 0; i < profile_before.size(); ++i) {
    EXPECT_FLOAT_EQ(stop.wpnts[i].vx_mps, profile_before[i])
      << "덧붙이기가 원래 제동 프로파일을 건드렸다 (index " << i << ")";
  }
  // 덧붙인 점은 전부 0 — 정지 지점이 뒤로 밀리면 장애물 안에서 서게 된다.
  for (std::size_t i = profile_before.size(); i < stop.wpnts.size(); ++i) {
    EXPECT_FLOAT_EQ(stop.wpnts[i].vx_mps, 0.0f)
      << "덧붙인 점에 속도가 실렸다 (index " << i << ") — 정지 목표가 밀린다";
  }
  // 기하는 원본을 그대로 이어야 한다 (라인이 갈리면 조향이 튄다).
  for (std::size_t i = 0; i < stop.wpnts.size(); ++i) {
    EXPECT_FLOAT_EQ(stop.wpnts[i].d_m, source.wpnts[i + 1].d_m);
    EXPECT_FLOAT_EQ(stop.wpnts[i].s_m, source.wpnts[i + 1].s_m);
  }
}

TEST(RacelineSplinePlanner, ExtendStopGeometryLeavesAnAlreadyLongPathAlone)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.15, 2.0};
  const auto source = makeStraightCandidate(reference, 0.15, 0.0, 40U);
  auto stop = source;

  EXPECT_EQ(planner.extendStopGeometry(stop, source, ego, 2.5), 0U);
  EXPECT_EQ(stop.wpnts.size(), source.wpnts.size());
}

TEST(RacelineSplinePlanner, ExtendStopGeometryRefusesASourceItDidNotComeFrom)
{
  // 🔴 안전 경계. 접두부가 이 원본에서 잘린 게 아니면 이어 붙이는 순간 라인이 갈린다 —
  //    d 가 다른 두 기하를 접합하면 그 이음매가 곧 조향 계단이다.
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.15, 2.0};
  const auto source = makeStraightCandidate(reference, 0.15, 0.0, 40U);

  // s 가 원본의 어느 점과도 안 맞는 정지 접두부.
  f110_msgs::msg::WpntArray alien;
  alien.header = source.header;
  for (std::size_t i = 0; i < 3U; ++i) {
    auto waypoint = source.wpnts[i + 1];
    waypoint.s_m += 0.037;
    waypoint.vx_mps = 0.0;
    alien.wpnts.push_back(waypoint);
  }
  const std::size_t before = alien.wpnts.size();

  EXPECT_EQ(planner.extendStopGeometry(alien, source, ego, 2.5), 0U);
  EXPECT_EQ(alien.wpnts.size(), before) << "다른 기하를 접합했다";
}

TEST(RacelineSplinePlanner, ExtendStopGeometryIsOffWhenTheFloorIsZero)
{
  // 되돌릴 길: controller_lookahead_floor_m 0 이면 종전 거동 그대로.
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.15, 4.6};
  const auto source = makeStraightCandidate(reference, 0.15, 0.0, 40U);
  f110_msgs::msg::WpntArray stop;
  stop.header = source.header;
  stop.wpnts.assign(source.wpnts.begin() + 1, source.wpnts.begin() + 4);

  EXPECT_EQ(planner.extendStopGeometry(stop, source, ego, 0.0), 0U);
  EXPECT_EQ(stop.wpnts.size(), 3U);
}

TEST(RacelineSplinePlanner, ExtendStopGeometryStopsAtTheSourceEndRatherThanWrapping)
{
  // 원본이 하한보다 짧으면 도달한 만큼만 늘리고 멈춘다 — 랩을 감아 뒤쪽 기하를 붙이면
  // 컨트롤러가 지나온 길을 향해 조향한다.
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.15, 4.6};
  const auto source = makeStraightCandidate(reference, 0.15, 0.0, 10U);   // 0.9 m 뿐
  f110_msgs::msg::WpntArray stop;
  stop.header = source.header;
  stop.wpnts.assign(source.wpnts.begin() + 1, source.wpnts.begin() + 4);

  const std::size_t appended = planner.extendStopGeometry(stop, source, ego, 2.5);

  EXPECT_EQ(appended, source.wpnts.size() - 4U) << "원본을 다 쓰지 않았거나 넘어섰다";
  EXPECT_LT(planner.forwardSpanAheadOfEgo(stop, ego), 2.5)
    << "원본보다 멀리 뻗었다 — 어디선가 기하를 지어냈다";
  EXPECT_EQ(stop.wpnts.back().s_m, source.wpnts.back().s_m);
}


}  // namespace
}  // namespace local_planning
