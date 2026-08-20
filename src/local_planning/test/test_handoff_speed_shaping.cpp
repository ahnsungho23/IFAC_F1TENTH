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

// R1 핸드오프 속도 성형(shapeGlobalHandoffSpeed)과 B1 오버레이(applyRawSlowdownProfile)의
// 계약을 고정한다 (2026-08-21):
//  - 성형은 플래그 뒤에서만 실행되고, 꺼져 있으면 구 flat-cap 거동과 비트 단위로 같다.
//  - 성형은 자차-전방 순서(tail_begin 회전)로 걷는다 — seam(랩 경계) 넘어서도 자차
//    실측 속도에서 가속 램프가 시작되어야 한다.
//  - 성형 후 κ 는 실제 기하의 부호 있는 값으로 재계산된다.
//  - 오버레이는 min 캡만 적용한다(속도를 절대 올리지 않는다).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 10.0;
constexpr std::size_t kCount = 120U;

f110_msgs::msg::WpntArray ringReference(double vx)
{
  f110_msgs::msg::WpntArray reference;
  const double step = 2.0 * kPi * kRadius / static_cast<double>(kCount);
  reference.wpnts.reserve(kCount);
  for (std::size_t index = 0U; index < kCount; ++index) {
    const double angle = 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(kCount);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<std::int32_t>(index);
    waypoint.s_m = step * static_cast<double>(index);
    waypoint.x_m = kRadius * std::cos(angle);
    waypoint.y_m = kRadius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * kPi;
    waypoint.kappa_radpm = 1.0 / kRadius;
    waypoint.vx_mps = vx;
    waypoint.d_left = 3.0;
    waypoint.d_right = 3.0;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

RacelineSplineParameters shapingParameters(bool enable)
{
  RacelineSplineParameters parameters;
  parameters.handoff_speed_shaping_enable = enable;
  // 속도 무관 평평한 표: lateral 4.0 → 반경 10 에서 캡 √(4/0.1) = 6.32 m/s.
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 10.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {4.0, 4.0};
  parameters.avoidance_velocity_limit_accel_mps2 = {3.0, 3.0};
  parameters.avoidance_velocity_limit_decel_mps2 = {3.0, 3.0};
  parameters.longitudinal_launch_speed_floor_mps = 1.0;
  return parameters;
}

RacelineSplinePlanner plannerWith(const RacelineSplineParameters & parameters, double vx)
{
  RacelineSplinePlanner planner;
  planner.setParameters(parameters);
  std::string error;
  EXPECT_TRUE(planner.setReference(ringReference(vx), &error)) << error;
  return planner;
}

// 자차-전방 순서로 (전방거리, 속도) 목록을 만든다.
std::vector<std::pair<double, double>> forwardProfile(
  const RacelineSplinePlanner & planner, const f110_msgs::msg::WpntArray & path,
  double ego_s)
{
  std::vector<std::pair<double, double>> profile;
  profile.reserve(path.wpnts.size());
  for (const auto & waypoint : path.wpnts) {
    profile.emplace_back(planner.forwardDistance(ego_s, waypoint.s_m), waypoint.vx_mps);
  }
  std::sort(profile.begin(), profile.end());
  return profile;
}

TEST(HandoffSpeedShaping, DisabledKeepsFlatCapAndReferenceKappa)
{
  auto planner = plannerWith(shapingParameters(false), 8.0);
  EgoFrenetState ego;
  ego.s = 30.0;
  ego.d = 0.0;
  ego.speed = 1.0;
  const auto path = planner.buildGlobalHandoffPath(ego, 6.0, 6.0);
  ASSERT_EQ(path.wpnts.size(), kCount);
  for (const auto & waypoint : path.wpnts) {
    // 구 거동: min(라인 속도, flat 캡) 그대로 — 자차 속도와 무관하게 즉시 6.0.
    EXPECT_NEAR(waypoint.vx_mps, 6.0, 1.0e-9);
    // κ 필드도 기준선 값 그대로 (재계산 없음).
    EXPECT_NEAR(waypoint.kappa_radpm, 1.0 / kRadius, 1.0e-9);
  }
}

TEST(HandoffSpeedShaping, EnabledSeedsAccelRampFromMeasuredEgoSpeed)
{
  auto planner = plannerWith(shapingParameters(true), 8.0);
  EgoFrenetState ego;
  ego.s = 30.0;
  ego.d = 0.0;
  ego.speed = 1.0;
  const auto path = planner.buildGlobalHandoffPath(ego, 6.0, 8.0);
  ASSERT_EQ(path.wpnts.size(), kCount);
  const auto profile = forwardProfile(planner, path, ego.s);
  // 자차 바로 앞 점은 실측 속도 시드의 가속 램프 안이어야 한다. 시드는 자차의 최근접
  // 웨이포인트(최대 한 칸 뒤)에서 시작하므로 spacing 한 칸만큼의 여유를 준다 —
  // 이 경계가 지키는 것은 "라인 속도(8.0)나 곡률 캡(6.32)이 아니라 자차 속도에서
  // 램프가 시작된다"는 계약이다.
  const double spacing = 2.0 * kPi * kRadius / static_cast<double>(kCount);
  bool checked_near = false;
  for (const auto & [forward, speed] : profile) {
    if (forward < 2.0) {
      EXPECT_LE(speed, std::sqrt(1.0 + 2.0 * 3.0 * (forward + spacing)) + 1.0e-6)
        << "forward=" << forward;
      checked_near = true;
    }
  }
  EXPECT_TRUE(checked_near);
  // 인접 점 사이 가속 요구가 표 한계(3.0)를 넘지 않는다 (자차-전방 순서).
  for (std::size_t i = 1; i < profile.size(); ++i) {
    const double ds = profile[i].first - profile[i - 1].first;
    if (!(ds > 1.0e-6)) {
      continue;
    }
    const double accel =
      (profile[i].second * profile[i].second -
      profile[i - 1].second * profile[i - 1].second) / (2.0 * ds);
    EXPECT_LE(accel, 3.0 + 1.0e-6) << "forward=" << profile[i].first;
  }
}

TEST(HandoffSpeedShaping, EnabledRampCrossesLapSeam)
{
  auto planner = plannerWith(shapingParameters(true), 8.0);
  const double track_length = planner.trackLength();
  EgoFrenetState ego;
  ego.s = track_length - 1.0;   // 랩 경계 직전 — 전방 점들이 s wrap 을 넘는다
  ego.d = 0.0;
  ego.speed = 1.0;
  const auto path = planner.buildGlobalHandoffPath(ego, 6.0, 8.0);
  const auto profile = forwardProfile(planner, path, ego.s);
  const double spacing = 2.0 * kPi * kRadius / static_cast<double>(kCount);
  bool checked = false;
  for (const auto & [forward, speed] : profile) {
    if (forward > 0.5 && forward < 2.5) {   // wrap 너머의 초반 구간
      EXPECT_LE(speed, std::sqrt(1.0 + 2.0 * 3.0 * (forward + spacing)) + 1.0e-6)
        << "forward=" << forward;
      checked = true;
    }
  }
  // seam 회전이 틀렸다면(배열 순서로 걸었다면) 이 구간은 라인 속도(8.0 캡)로 남는다.
  EXPECT_TRUE(checked);
}

TEST(HandoffSpeedShaping, EnabledCapsToActualCurvatureAndRecomputesSignedKappa)
{
  auto planner = plannerWith(shapingParameters(true), 8.0);
  EgoFrenetState ego;
  ego.s = 30.0;
  ego.d = 0.0;
  ego.speed = 6.0;
  const auto path = planner.buildGlobalHandoffPath(ego, 6.0, 8.0);
  const double curvature_cap = std::sqrt(4.0 * kRadius);   // √(a_lat/κ) = 6.32
  std::size_t recomputed = 0;
  for (const auto & waypoint : path.wpnts) {
    EXPECT_LE(waypoint.vx_mps, curvature_cap + 1.0e-6);
    // CCW 링 = 좌선회 = 양의 κ. 재계산된 부호·크기 확인 (배열 양끝 스텐실 오차 허용).
    if (std::abs(waypoint.kappa_radpm - 1.0 / kRadius) < 0.02) {
      EXPECT_GT(waypoint.kappa_radpm, 0.0);
      ++recomputed;
    }
  }
  EXPECT_GT(recomputed, kCount - 6U);
}

TEST(RawSlowdownProfile, OverlayOnlyLowersSpeedAheadOfObstacle)
{
  auto planner = plannerWith(shapingParameters(false), 8.0);
  EgoFrenetState ego;
  ego.s = 30.0;
  ego.d = 0.0;
  ego.speed = 5.0;
  auto path = planner.buildGlobalHandoffPath(ego, 6.0, 6.0);
  // 전방 5 m 에 장애물, 스팬 0.4 m: cap 2.8, decel 2.0.
  planner.applyRawSlowdownProfile(path, ego, 5.0, 0.4, 2.8, 2.0);
  for (const auto & waypoint : path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    if (forward >= 0.5 * planner.trackLength()) {
      EXPECT_NEAR(waypoint.vx_mps, 6.0, 1.0e-9);   // 자차 뒤쪽 절반은 불변
      continue;
    }
    if (forward <= 5.0) {
      const double limit = std::sqrt(2.8 * 2.8 + 2.0 * 2.0 * (5.0 - forward));
      EXPECT_LE(waypoint.vx_mps, std::min(6.0, limit) + 1.0e-6);
    } else if (forward <= 5.0 + 0.4 + 1.0) {
      EXPECT_LE(waypoint.vx_mps, 2.8 + 1.0e-6);   // 스팬+1 m 동안 cap 유지
    } else {
      EXPECT_NEAR(waypoint.vx_mps, 6.0, 1.0e-9);   // 그 뒤는 원속도
    }
  }
  // min 전용 계약: 이미 0 인 점(정지 프로파일)은 절대 올리지 않는다.
  auto stop_path = planner.buildGlobalHandoffPath(ego, 6.0, 6.0);
  for (auto & waypoint : stop_path.wpnts) {
    waypoint.vx_mps = 0.0;
  }
  planner.applyRawSlowdownProfile(stop_path, ego, 5.0, 0.4, 2.8, 2.0);
  for (const auto & waypoint : stop_path.wpnts) {
    EXPECT_NEAR(waypoint.vx_mps, 0.0, 1.0e-12);
  }
}

}  // namespace
}  // namespace local_planning
