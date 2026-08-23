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

// 추종오차 예약 게이트 + raw 감속 힌트 커밋 skip (2026-08-22).
//
// 이 두 개가 지키는 계약:
//   ① 게이트가 꺼지면 표가 **살아 있어도** 예약이 0 이다. 표를 0 으로 채우는 것과 값은
//      같지만, 표가 병합으로 되살아나도 되살아나지 않는다는 점이 다르다.
//   ② 게이트가 꺼지면 갭 역산은 항등함수다 — 좁은 갭을 저속 통과하는 사다리가 사라진다.
//      이것은 부작용이 아니라 의도된 결과이므로 시험으로 못박는다.
//   ③ raw 캡은 "지금 confirmed" AND "지금 커밋된 기동이 책임짐" 일 때만 건너뛴다.
//      하나라도 빠지면 캡이 걸린다 — 특히 confirmed 에서 떨어져 나간 장애물.

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/raw_slowdown_gate.hpp"

namespace local_planning
{
namespace
{

// config/local_planning.yaml 의 08-19 실측표 (게이트 시험이므로 값 자체는 대표값만).
RacelineSplineParameters parametersWithLut()
{
  RacelineSplineParameters p;
  p.vehicle_half_width_m = 0.15;
  p.safety_margin_m = 0.05;
  p.localization_reserve_m = 0.0;
  p.tracking_error_reserve_m = 0.20;
  p.tracking_error_lut_speed_bins_mps = {0.0, 1.0, 1.6, 2.2, 2.9, 4.5, 7.0};
  p.tracking_error_lut_curvature_bins_radpm = {0.0, 0.1, 0.2, 0.35, 0.46};
  p.tracking_error_lut_values_m = {
    0.095, 0.095, 0.095, 0.130, 0.130,
    0.100, 0.165, 0.165, 0.165, 0.165,
    0.100, 0.165, 0.165, 0.165, 0.165,
    0.145, 0.165, 0.190, 0.230, 0.230,
    0.275, 0.275, 0.390, 0.390, 0.390,
    0.275, 0.275, 0.390, 0.390, 0.390,
    0.275, 0.275, 0.390, 0.390, 0.390};
  p.avoidance_minimum_speed_mps = 1.0;
  return p;
}

TEST(ObstacleReserveGate, DefaultIsOffSoARestoredTableStaysInert)
{
  // 기본값이 off 라는 것 자체가 계약이다 — 표가 병합으로 되돌아와도 예약은 0 이다.
  const auto p = parametersWithLut();
  EXPECT_FALSE(p.obstacle_reserve_from_lut);
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(4.5, 0.28), 0.0);
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(7.0, 0.46), 0.0);
}

TEST(ObstacleReserveGate, TurningItOnRestoresTheMeasuredTableExactly)
{
  // 되돌리기 경로가 종전과 비트 단위로 같아야 한다.
  auto p = parametersWithLut();
  p.obstacle_reserve_from_lut = true;
  EXPECT_NEAR(p.trackingErrorReserve(1.0, 0.0), 0.100, 1e-9);
  EXPECT_NEAR(p.trackingErrorReserve(2.9, 0.20), 0.390, 1e-9);
  EXPECT_NEAR(p.trackingErrorReserve(7.0, 0.46), 0.390, 1e-9);
}

TEST(ObstacleReserveGate, LocalizationReserveStillPassesThroughWhenGated)
{
  // 게이트는 **LUT** 를 끄는 것이지 위치추정 예비량을 끄는 것이 아니다. 그 둘을 한 번에
  // 끄면 나중에 위치추정 마진만 되살리고 싶을 때 방법이 없어진다.
  auto p = parametersWithLut();
  p.localization_reserve_m = 0.06;
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(4.5, 0.28), 0.06);
}

TEST(ObstacleReserveGate, GatedFallbackConstantIsAlsoIgnored)
{
  // 표가 깨져도(빈 배열) 폴백 상수로 예약이 되살아나면 안 된다.
  auto p = parametersWithLut();
  p.tracking_error_lut_speed_bins_mps.clear();
  p.tracking_error_lut_curvature_bins_radpm.clear();
  p.tracking_error_lut_values_m.clear();
  p.tracking_error_reserve_m = 0.20;
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(3.0, 0.3), 0.0)
    << "게이트가 꺼져 있으면 폴백 상수도 읽지 않아야 한다";
  p.obstacle_reserve_from_lut = true;
  EXPECT_DOUBLE_EQ(p.trackingErrorReserve(3.0, 0.3), 0.20)
    << "켜면 종전대로 폴백 상수를 쓴다";
}

TEST(ObstacleReserveGate, ObstacleClearanceCollapsesToHalfWidthPlusMargin)
{
  const auto p = parametersWithLut();
  EXPECT_DOUBLE_EQ(p.obstacleBaseClearance(), 0.20);
  EXPECT_DOUBLE_EQ(p.obstacleSafetyClearance(6.0, 0.30, 1.0), 0.20)
    << "예약이 0 이므로 속도·곡률과 무관하게 반폭+마진 뿐이다";
}

TEST(ObstacleReserveGate, GapInversionBecomesIdentity)
{
  // 의도된 부작용: 좁은 갭을 저속으로 통과하는 사다리가 사라진다. 갭은 통과 가부의
  // 이진 판정이 되고, 그 문턱은 obstacleBaseClearance() 다.
  const auto p = parametersWithLut();
  for (const double room : {0.001, 0.05, 0.20, 0.75, 3.0}) {
    EXPECT_DOUBLE_EQ(p.gapLimitedAvoidanceSpeed(6.5, 0.30, room), 6.5)
      << "room=" << room;
  }
}

TEST(ObstacleReserveGate, GapInversionStillLaddersWhenTheGateIsOn)
{
  // 되돌리기 경로에서는 종전 사다리가 그대로 살아 있어야 한다.
  auto p = parametersWithLut();
  p.obstacle_reserve_from_lut = true;
  const double capped = p.gapLimitedAvoidanceSpeed(6.5, 0.28, 0.34);
  EXPECT_GT(capped, 1.0);
  EXPECT_LT(capped, 2.9) << "0.34 m 여유는 v>=2.9 의 0.390 예약을 못 담는다";
}

// ── raw 감속 힌트 커밋 skip ────────────────────────────────────────────────

TEST(RawSlowdownSkip, SkipsOnlyWhenConfirmedAndCommitted)
{
  EXPECT_TRUE(rawSlowdownHandledByCommitment(true, 7, true, 7, {}, {7, 9}));
  EXPECT_TRUE(rawSlowdownHandledByCommitment(true, 9, true, -1, {3, 9}, {9}));
}

TEST(RawSlowdownSkip, DoesNotSkipWhenTheTrackFellOutOfConfirmed)
{
  // 가장 위험한 순간이다 — 커밋은 살아 있는데 검출이 끊겼다. 캡을 되돌려야 한다.
  EXPECT_FALSE(rawSlowdownHandledByCommitment(true, 7, true, 7, {7}, {}));
  EXPECT_FALSE(rawSlowdownHandledByCommitment(true, 7, true, 7, {7}, {3, 9}));
}

TEST(RawSlowdownSkip, DoesNotSkipObstaclesTheCommitmentDoesNotOwn)
{
  // 커밋 밖의 다음 장애물은 접근 보호가 그대로 필요하다.
  EXPECT_FALSE(rawSlowdownHandledByCommitment(true, 11, true, 7, {7, 9}, {7, 9, 11}));
}

TEST(RawSlowdownSkip, DoesNotSkipWithoutAnActiveAvoidanceCommitment)
{
  // 안전정지·준비 상태거나 커밋이 없으면 승격 대기 보호가 유일한 방어다.
  EXPECT_FALSE(rawSlowdownHandledByCommitment(true, 7, false, 7, {7}, {7}));
}

TEST(RawSlowdownSkip, DisabledFlagKeepsTheLegacyBehaviour)
{
  EXPECT_FALSE(rawSlowdownHandledByCommitment(false, 7, true, 7, {7}, {7}));
}

TEST(RawSlowdownSkip, NegativeIdNeverSkips)
{
  // 깜빡임 브리지가 id 를 기억하지 못한 경우(-1)는 보수적으로 캡을 유지한다.
  EXPECT_FALSE(rawSlowdownHandledByCommitment(true, -1, true, -1, {}, {}));
}

}  // namespace
}  // namespace local_planning

// 🔵 2026-08-23: raw 캡의 거리 스케일. 핵심은 **오직 완화만 한다**는 것이다 —
// 가까운 거리에서는 종전 상수와 비트 단위로 같아야 하고, 먼 거리에서만 풀려야 한다.
TEST(RawSlowdownSpeedCap, NearRangeIsBitIdenticalToTheLegacyConstant)
{
  const double kFloor = 2.8;
  const double kDecel = 2.0;
  // 교차점 d* = floor^2 / (2a).
  const double crossover = kFloor * kFloor / (2.0 * kDecel);
  EXPECT_NEAR(crossover, 1.96, 1e-9);
  for (const double front : {0.0, 0.1, 0.5, 1.0, 1.5, crossover - 1e-9}) {
    EXPECT_DOUBLE_EQ(
      local_planning::rawSlowdownSpeedCap(true, kFloor, kDecel, front), kFloor)
      << "전방 " << front << " m 에서 종전 거동이 바뀌었다";
  }
}

TEST(RawSlowdownSpeedCap, FarRangeRelaxesAlongTheBrakingCurve)
{
  const double kFloor = 2.8;
  const double kDecel = 2.0;
  // 실측 전방거리 분포의 p50 / p90 근방.
  EXPECT_NEAR(local_planning::rawSlowdownSpeedCap(true, kFloor, kDecel, 3.27), 3.617, 1e-3);
  EXPECT_NEAR(local_planning::rawSlowdownSpeedCap(true, kFloor, kDecel, 7.06), 5.315, 1e-3);
  EXPECT_NEAR(local_planning::rawSlowdownSpeedCap(true, kFloor, kDecel, 12.0), 6.928, 1e-3);
  // 단조 증가여야 한다.
  double previous = 0.0;
  for (double front = 0.0; front <= 12.0; front += 0.25) {
    const double cap = local_planning::rawSlowdownSpeedCap(true, kFloor, kDecel, front);
    EXPECT_GE(cap, previous - 1e-12) << "전방 " << front;
    EXPECT_GE(cap, kFloor) << "바닥 아래로 내려갔다 — min/max 방향이 뒤집혔다";
    previous = cap;
  }
}

TEST(RawSlowdownSpeedCap, DisabledOrDegenerateInputsFallBackToTheConstant)
{
  const double kFloor = 2.8;
  EXPECT_DOUBLE_EQ(local_planning::rawSlowdownSpeedCap(false, kFloor, 2.0, 12.0), kFloor);
  EXPECT_DOUBLE_EQ(local_planning::rawSlowdownSpeedCap(true, kFloor, 0.0, 12.0), kFloor);
  EXPECT_DOUBLE_EQ(local_planning::rawSlowdownSpeedCap(true, kFloor, -1.0, 12.0), kFloor);
  EXPECT_DOUBLE_EQ(local_planning::rawSlowdownSpeedCap(true, kFloor, 2.0, -3.0), kFloor);
  EXPECT_DOUBLE_EQ(
    local_planning::rawSlowdownSpeedCap(
      true, kFloor, 2.0, std::numeric_limits<double>::quiet_NaN()), kFloor);
}
