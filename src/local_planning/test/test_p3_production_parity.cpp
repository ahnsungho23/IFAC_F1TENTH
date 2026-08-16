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
    if (std::getenv("P3_PARITY_DUMP") != nullptr) {
      printf(
        "DUMP %s ego s=%.3f d=%+.4f v=%.2f | R[%+.4f,%+.4f] L[%+.4f,%+.4f] | %s\n",
        stream.scenario.c_str(), frame.ego.s, frame.ego.d, frame.ego.speed,
        result.right_domain.minimum_target, result.right_domain.maximum_target,
        result.left_domain.minimum_target, result.left_domain.maximum_target,
        result.failure_classification.c_str());
      for (const auto & candidate : result.candidates) {
        printf(
          "     %2zu %s d_target=%+.4f d_mid=%+.4f entry=%.3f exit=%.3f %s "
          "| obs=%d at s=%.3f d=%+.4f | %s\n",
          candidate.generation_index, candidate.go_left ? "L" : "R", candidate.d_target,
          candidate.d_mid, candidate.entry_scale, candidate.exit_scale,
          candidate.hard_valid ? "OK" : "XX", candidate.validation.failure_obstacle_id,
          candidate.validation.failure_waypoint_s, candidate.validation.failure_waypoint_d,
          candidate.rejection_reason.c_str());
      }
    }
  }
  return recovers;
}

}  // namespace

// 🔴 2026-08-16 수리 회귀. 이 장면들은 수리 전에는 전부 실패했다.
//
// 원인(안전망 덤프로 확정): obs9(s=32.7)는 우측 통과 필수, obs10(s=37.4)은 좌측 통과 필수
// 이고 간격이 4.2 m인데 검증 지평이 cluster_end + 5.0 m라 obs10을 삼켰다. 모든 후보가
// obs10에서 걸렸고, 특히 짧은 탈출 후보는 s=36.907에서 **d=+0.0000** — 라인으로 합류를
// 마친 상태에서 걸렸다. obs10의 박스가 d=0을 물고 있으니 라인 복귀 자체가 충돌이었다.
// 깊이나 후보 수로는 절대 풀리지 않는 형태다.
//
// 수리: 검증 지평을 다음 클러스터의 확장 앞면에서 자른다(maneuverScopeEnd).
TEST(P3ProductionParity, CoupledOppositeSideObstaclesRecover)
{
  const auto stream = readStream(scenarioPath("failing_cluster11"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 3U);
  for (std::size_t index = 0; index < recovers.size(); ++index) {
    EXPECT_TRUE(recovers[index])
      << "failing_cluster11 frame " << index
      << ": 다음 클러스터가 지평 안에 들어와 회피가 다시 막혔다";
  }
}

// 아직 실패하는 것이 정상인 장면 — 원인이 다르다.
//
// 장애물 1(s=8.27)을 우측으로 피하려면 d<=-0.68까지 나가야 하는데, 그 앞 s=6.0~6.8에서
// 회랑 우측이 -0.70까지 좁아진다. 전이 곡선은 양 끝점만 보고 그려지므로 그 잘록한 구간을
// 관통하고 footprint_track_bound로 걸린다. 속도를 3.41 -> 2.0으로 낮춰도 회복하지 않으므로
// run-up 부족이 아니다(실측). 회랑 중간 제약을 반영하는 전이 형상이 필요하며, 이는 검증
// 지평과 무관한 별개 사안이다.
TEST(P3ProductionParity, CorridorPinchDuringTransitionIsStillUnsolved)
{
  const auto stream = readStream(scenarioPath("failing_cluster1"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 1U);
  EXPECT_FALSE(recovers.front())
    << "회랑 잘록 구간 문제가 풀렸다면 이 테스트를 회복 케이스로 옮길 것";
}

// 지금 성공하는 장면. 후보 배치를 어떻게 바꾸든 여기가 깨지면 회귀다.
// 🔴 2026-08-16 23:31 백 회귀. 직선(라인 속도 7.0)에서 차가 라인 위를 2.0 m/s로 5 m 넘게
// 기어갔다. 원인은 margin pass — "라인이 물리적으로는 주행 가능하니 상한 속도로 그냥 통과"
// 인데, 그 전제인 "양측 회피 실패"가 잘못 성립한 것이다.
//
// obs9(s=32.3)는 우측 여유가 1.14 m나 되어 통과 가능한데, 4.4 m 뒤 obs10(s=37.2, 라인을
// 물고 있어 좌측 통과 필수)이 검증 지평 안에 들어와 모든 후보를 기각시켰다. plan()의
// 후보 검증(generateP3Candidates)이 세 지평 사용처 중 유일하게 고정 거리를 쓰고 있어서,
// P3 평가기와 커밋 재검증을 먼저 고친 뒤에도 증상이 남았다.
//
// 세 곳이 같은 정의(maneuverScopeEnd)를 쓰는지 이 테스트가 지킨다.
TEST(P3ProductionParity, StraightAvoidanceDoesNotDegradeToMarginCrawl)
{
  const auto stream = readStream(scenarioPath("straight_margin_crawl"));
  RacelineSplinePlanner planner(parametersOf(stream));
  ASSERT_TRUE(planner.setReference(stream.reference));
  ASSERT_GE(stream.frames.size(), 1U);
  for (std::size_t index = 0; index < stream.frames.size(); ++index) {
    const auto & frame = stream.frames[index];
    const auto result = planner.plan(frame.ego, frame.obstacles);
    EXPECT_EQ(result.kind, SplinePlanKind::kAvoidance)
      << "frame " << index << ": " << result.reason;
    EXPECT_FALSE(result.margin_pass)
      << "frame " << index
      << ": 우측이 열려 있는데 라인 위 저속 통과로 떨어졌다 (margin crawl). target_d="
      << result.target_d << ", " << result.reason;
  }
}

// A안 회귀 (2026-08-16): 실현 가능한 후보끼리는 속도가 slack보다 먼저다.
// 선택된 경로의 최저 속도가 다른 실현가능 후보들의 최저보다 낮으면 순위가 되돌아간 것이다.
TEST(P3ProductionParity, SelectedPathIsTheFastestFeasibleOne)
{
  const auto stream = readStream(scenarioPath("straight_margin_crawl"));
  RacelineSplinePlanner planner(parametersOf(stream));
  ASSERT_TRUE(planner.setReference(stream.reference));
  for (std::size_t index = 0; index < stream.frames.size(); ++index) {
    const auto & frame = stream.frames[index];
    const auto result = planner.plan(frame.ego, frame.obstacles);
    ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
    // exit_reaches_next_obstacle 강등은 속도보다 우선한다(다음 기동의 성립 여부를 좌우하는
    // 정합성 조건이다). 따라서 비교는 선택된 후보와 **같은 강등 등급** 안에서만 한다.
    bool selected_demoted = false;
    for (const auto & audit : result.candidate_audits) {
      if (audit.selected) {
        selected_demoted = audit.exit_reaches_next_obstacle;
      }
    }
    double selected_loss = std::numeric_limits<double>::quiet_NaN();
    double best_loss = std::numeric_limits<double>::infinity();
    for (const auto & audit : result.candidate_audits) {
      if (!audit.feasible || audit.exit_reaches_next_obstacle != selected_demoted) {
        continue;
      }
      best_loss = std::min(best_loss, audit.velocity_loss);
      if (audit.selected) {
        selected_loss = audit.velocity_loss;
      }
    }
    if (std::getenv("P3_PARITY_DUMP") != nullptr) {
      for (const auto & audit : result.candidate_audits) {
        if (!audit.feasible) {
          continue;
        }
        printf(
          "  RANK frame%zu #%d%s target_d=%+.3f entry=%.3f loss=%.4f slack=%.4f exit_next=%d\n",
          index, audit.final_rank, audit.selected ? "*" : " ", audit.target_d,
          audit.entry_fraction, audit.velocity_loss,
          audit.minimum_normalized_safety_slack,
          static_cast<int>(audit.exit_reaches_next_obstacle));
      }
    }
    if (!std::isfinite(selected_loss) || !std::isfinite(best_loss)) {
      continue;   // 감사 정보가 없는 경로(핸드오프 등)는 대상이 아니다.
    }
    EXPECT_LE(selected_loss, best_loss + 1.0e-9)
      << "frame " << index << ": 더 빠른 실현가능 후보가 있는데 느린 쪽을 골랐다 ("
      << selected_loss << " vs " << best_loss << ")";
  }
}

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
