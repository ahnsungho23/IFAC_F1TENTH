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
//
// P3 회피 후보 생성의 회귀 안전망.
//
// 왜 필요한가 (2026-08-16): P3의 후보 배치를 고치려 하는데, CMakeLists에 이 파일의 슬롯만
// 있고 파일 자체가 없어 검사가 **조용히 건너뛰어지고** 있었다. 즉 후보 생성을 어떻게 바꿔도
// 아무도 잡아주지 않는 상태였다. 후보 수를 늘리는 방향의 변경은 "없던 해를 억지로 만들어
// 내는" 쪽으로 틀어지기 쉬워서, 고치기 전에 안전망이 먼저 있어야 한다.
//
// 시나리오는 실제 주행 백에서 뽑는다(tools/extract_p3_scenarios.py). 기준선까지 파일에
// 넣는 이유는 offline_trajectory_generator/output/이 gitignore라 다른 환경에서 재현되지
// 않기 때문이다.
//
// 세 종류를 담는다:
//   kNoCandidate  지금 실패하는 장면 — 수리 후 성립해야 한다(현재는 실패가 정상)
//   kRecovers     지금 성공하는 장면 — 수리 후에도 유지돼야 한다(회귀 방지)
//   kImpossible   기하적으로 막힌 장면 — 수리 후에도 실패해야 한다(거짓 통과 방지)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/p3_shadow.hpp"
#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

struct Frame
{
  EgoFrenetState ego;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};

struct Stream
{
  std::string scenario;
  std::map<std::string, double> scalars;
  std::map<std::string, std::vector<double>> vectors;
  f110_msgs::msg::WpntArray reference;
  std::vector<Frame> frames;
};

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> tokens;
  std::size_t begin = 0U;
  while (true) {
    const std::size_t end = line.find('\t', begin);
    tokens.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      return tokens;
    }
    begin = end + 1U;
  }
}

double number(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite token: " + token);
  }
  return value;
}

Stream readStream(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "PATH_FAMILY_STREAM_V1") {
    throw std::runtime_error("unsupported scenario stream: " + path.string());
  }
  Stream stream;
  stream.reference.header.frame_id = "map";
  while (std::getline(input, line)) {
    const auto tokens = splitTabs(line);
    if (tokens[0] == "SCENARIO" && tokens.size() == 2U) {
      stream.scenario = tokens[1];
    } else if (tokens[0] == "SOURCE_BAG" || tokens[0] == "CONFIG_PATH") {
      continue;
    } else if (tokens[0] == "PARAM" && tokens.size() == 3U) {
      stream.scalars[tokens[1]] = number(tokens[2]);
    } else if (tokens[0] == "PARAMV" && tokens.size() >= 3U) {
      auto & output = stream.vectors[tokens[1]];
      for (std::size_t index = 3U; index < tokens.size(); ++index) {
        output.push_back(number(tokens[index]));
      }
    } else if (tokens[0] == "REFERENCE") {
      continue;
    } else if (tokens[0] == "W" && tokens.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(std::llround(number(tokens[1])));
      waypoint.s_m = number(tokens[2]);
      waypoint.d_m = number(tokens[3]);
      waypoint.x_m = number(tokens[4]);
      waypoint.y_m = number(tokens[5]);
      waypoint.d_right = number(tokens[6]);
      waypoint.d_left = number(tokens[7]);
      waypoint.psi_rad = number(tokens[8]);
      waypoint.kappa_radpm = number(tokens[9]);
      waypoint.vx_mps = number(tokens[10]);
      waypoint.ax_mps2 = number(tokens[11]);
      stream.reference.wpnts.push_back(waypoint);
    } else if (tokens[0] == "FRAME" && tokens.size() == 7U) {
      Frame frame;
      frame.ego.s = number(tokens[3]);
      frame.ego.d = number(tokens[4]);
      frame.ego.speed = number(tokens[5]);
      const auto obstacles = static_cast<std::size_t>(std::llround(number(tokens[6])));
      for (std::size_t index = 0U; index < obstacles; ++index) {
        if (!std::getline(input, line)) {
          throw std::runtime_error("truncated obstacle frame");
        }
        const auto fields = splitTabs(line);
        if (fields.size() != 10U || fields[0] != "O") {
          throw std::runtime_error("malformed obstacle record");
        }
        f110_msgs::msg::Obstacle obstacle;
        obstacle.id = static_cast<std::int32_t>(std::llround(number(fields[1])));
        obstacle.s_center = number(fields[2]);
        obstacle.s_start = number(fields[3]);
        obstacle.s_end = number(fields[4]);
        obstacle.d_right = number(fields[5]);
        obstacle.d_left = number(fields[6]);
        obstacle.size = number(fields[7]);
        obstacle.s_var = number(fields[8]);
        obstacle.d_var = number(fields[9]);
        obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
        obstacle.is_static = true;
        obstacle.is_visible = true;
        frame.obstacles.push_back(obstacle);
      }
      if (!std::getline(input, line) || line != "END_FRAME") {
        throw std::runtime_error("missing END_FRAME");
      }
      stream.frames.push_back(std::move(frame));
    } else if (tokens[0] == "END_STREAM") {
      break;
    } else if (!line.empty()) {
      throw std::runtime_error("unknown stream record: " + line);
    }
  }
  if (stream.scenario.empty() || stream.reference.wpnts.empty() || stream.frames.empty()) {
    throw std::runtime_error("incomplete scenario stream: " + path.string());
  }
  return stream;
}

// 스트림이 실은 값만 쓴다. at()이 없는 키에 대해 던지므로, 추출기와 이 목록이 어긋나면
// 조용히 구조체 기본값으로 통과하는 대신 테스트가 실패한다.
RacelineSplineParameters parametersOf(const Stream & stream)
{
  RacelineSplineParameters value;
  const auto scalar = [&](const char * name) {return stream.scalars.at(name);};
  const auto vector = [&](const char * name) {return stream.vectors.at(name);};
  value.detection_lookahead_m = scalar("detection_lookahead_m");
  value.obstacle_cluster_gap_m = scalar("obstacle_cluster_gap_m");
  value.obstacle_longitudinal_padding_m = scalar("obstacle_longitudinal_padding_m");
  value.vehicle_length_m = scalar("vehicle_length_m");
  value.vehicle_half_width_m = scalar("vehicle_half_width_m");
  value.safety_margin_m = scalar("safety_margin_m");
  value.tracking_error_reserve_m = scalar("tracking_error_reserve_m");
  value.wall_safety_margin_m = scalar("wall_safety_margin_m");
  value.fallback_track_half_width_m = scalar("fallback_track_half_width_m");
  value.outside_line_transition_scale = scalar("outside_line_transition_scale");
  value.post_merge_lookahead_m = scalar("post_merge_lookahead_m");
  value.post_merge_min_time_sec = scalar("post_merge_min_time_sec");
  value.minimum_target_offset_m = scalar("minimum_target_offset_m");
  value.maximum_target_offset_m = scalar("maximum_target_offset_m");
  value.target_d_candidate_count =
    static_cast<int>(std::llround(scalar("target_d_candidate_count")));
  value.maximum_lateral_slope = scalar("maximum_lateral_slope");
  value.maximum_curvature_radpm = scalar("maximum_curvature_radpm");
  value.maximum_curvature_rate_radpm2 = scalar("maximum_curvature_rate_radpm2");
  value.safe_stop_buffer_m = scalar("safe_stop_buffer_m");
  value.safe_stop_deceleration_mps2 = scalar("safe_stop_deceleration_mps2");
  value.minimum_path_points = static_cast<int>(std::llround(scalar("minimum_path_points")));
  value.localization_reserve_m = scalar("localization_reserve_m");
  value.avoidance_minimum_speed_mps = scalar("avoidance_minimum_speed_mps");
  value.margin_pass_speed_cap_mps = scalar("margin_pass_speed_cap_mps");
  value.commitment_retention_reserve_fraction =
    scalar("commitment_retention_reserve_fraction");
  value.tracking_error_lut_speed_bins_mps = vector("tracking_error_lut_speed_bins_mps");
  value.tracking_error_lut_curvature_bins_radpm =
    vector("tracking_error_lut_curvature_bins_radpm");
  value.tracking_error_lut_values_m = vector("tracking_error_lut_values_m");
  value.avoidance_velocity_limit_speed_bins_mps =
    vector("avoidance_velocity_limit_speed_bins_mps");
  value.avoidance_velocity_limit_lateral_accel_mps2 =
    vector("avoidance_velocity_limit_lateral_accel_mps2");
  value.pre_apex_distances_m = vector("pre_apex_distances_m");
  value.post_apex_distances_m = vector("post_apex_distances_m");
  value.entry_transition_fractions = vector("entry_transition_fractions");
  value.transition_distance_scales = vector("transition_distance_scales");
  return value;
}

std::filesystem::path scenarioPath(const std::string & name)
{
  return std::filesystem::path(P3_SCENARIO_DIR) / (name + ".stream");
}

// 각 프레임에서 P3가 신규 회피를 만들어내는지.
std::vector<bool> recoversPerFrame(const Stream & stream)
{
  RacelineSplinePlanner planner(parametersOf(stream));
  EXPECT_TRUE(planner.setReference(stream.reference)) << stream.scenario;
  std::vector<bool> recovers;
  for (const auto & frame : stream.frames) {
    const auto result = planner.evaluateP3Shadow(
      frame.ego, frame.obstacles, 0, 0U, 1U, "PARITY_ORACLE");
    recovers.push_back(result.would_recover);
  }
  return recovers;
}

}  // namespace

// 🔴 지금은 실패하는 것이 정상이다. 후보 배치를 고치면 이 단정이 뒤집혀야 한다.
//
// 실패 원인(2026-08-16 17:02 백 638건 분석): 후보를 도메인의 고정 비율(1/64, 1/4, 1/2, 1/1)
// 로 찍는데, 그 비율은 트랙 폭에서 온 값이라 장애물 위치와 무관하다. 그 결과 얕은 쪽 세 점이
// 전부 장애물 clearance에 걸리고, 유일하게 장애물을 벗어나는 가장 깊은 점은 곡률·기울기·
// 트랙 경계에 걸린다. 그 사이(도메인의 절반)에 후보가 하나도 없다.
TEST(P3ProductionParity, CurrentlyFailingScenariosAreStillReproduced)
{
  for (const char * name : {"failing_cluster11", "failing_cluster1"}) {
    const auto stream = readStream(scenarioPath(name));
    const auto recovers = recoversPerFrame(stream);
    ASSERT_FALSE(recovers.empty()) << name;
    for (std::size_t index = 0; index < recovers.size(); ++index) {
      EXPECT_FALSE(recovers[index])
        << name << " frame " << index
        << ": 이 장면은 아직 실패해야 한다. 성공한다면 후보 배치가 이미 바뀌었다는 뜻이고,"
        " 그때는 이 테스트를 CurrentlyFailing → Recovers 로 옮길 것";
    }
  }
}

// 지금 성공하는 장면. 후보 배치를 어떻게 바꾸든 여기가 깨지면 회귀다.
TEST(P3ProductionParity, PassingScenariosKeepRecovering)
{
  const auto stream = readStream(scenarioPath("passing_mixed"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 4U);
  for (std::size_t index = 0; index < recovers.size(); ++index) {
    EXPECT_TRUE(recovers[index])
      << "passing_mixed frame " << index << ": 되던 회피가 안 된다(회귀)";
  }
}

// 후보를 늘리는 방향의 변경은 "없던 해를 억지로 만들어내는" 쪽으로 틀어지기 쉽다.
// 물리적으로 통과 불가능한 배치에서는 계속 실패해야 한다.
TEST(P3ProductionParity, GeometricallyImpossibleGapStaysImpossible)
{
  const auto stream = readStream(scenarioPath("passing_mixed"));
  auto parameters = parametersOf(stream);
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(stream.reference));

  const auto & frame = stream.frames.front();
  // 같은 기준선 위에, 자차 바로 앞 회랑을 통째로 막는 장애물을 놓는다. 폭은 이 트랙의
  // 최대 회랑 폭보다 넓게 잡아 어떤 오프셋으로도 빠져나갈 수 없게 한다.
  double widest = 0.0;
  for (const auto & waypoint : stream.reference.wpnts) {
    widest = std::max(widest, waypoint.d_left + waypoint.d_right);
  }
  f110_msgs::msg::Obstacle blocker;
  blocker.id = 999;
  blocker.s_center = std::fmod(frame.ego.s + 6.0, stream.reference.wpnts.back().s_m);
  blocker.s_start = blocker.s_center - 0.25;
  blocker.s_end = blocker.s_center + 0.25;
  blocker.d_right = -widest;
  blocker.d_left = widest;
  blocker.d_center = 0.0;
  blocker.size = 2.0 * widest;
  blocker.is_static = true;
  blocker.is_visible = true;

  const auto result = planner.evaluateP3Shadow(
    frame.ego, {blocker}, 0, 0U, 1U, "PARITY_ORACLE");
  EXPECT_FALSE(result.would_recover)
    << "회랑을 통째로 막은 장애물에 회피가 성립했다 — 거짓 통과: "
    << result.failure_classification;
}

}  // namespace local_planning
