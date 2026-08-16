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
        "DUMP %s ego s=%.3f d=%+.4f v=%.2f | clus[%.3f,%.3f] Rc[%.3f,%.3f] Lc[%.3f,%.3f] "
        "ctx=%zu cand=%zu(trace %zu) segrej=%zu | R[%+.4f,%+.4f] L[%+.4f,%+.4f] | %s\n",
        stream.scenario.c_str(), frame.ego.s, frame.ego.d, frame.ego.speed,
        result.cluster_start_forward_m, result.cluster_end_forward_m,
        result.right_domain.cluster_start, result.right_domain.cluster_end,
        result.left_domain.cluster_start, result.left_domain.cluster_end,
        result.m1_context_count, result.m1_candidate_count, result.candidates.size(),
        result.m1_positive_segment_rejection_count,
        result.right_domain.minimum_target, result.right_domain.maximum_target,
        result.left_domain.minimum_target, result.left_domain.maximum_target,
        result.failure_classification.c_str());
      for (const auto & candidate : result.candidates) {
        printf(
          "     %2zu %s d_target=%+.4f d_mid=%+.4f entry=%.3f exit=%.3f %s "
          "peakK=%.3f slope=%.3f | at s=%.3f d=%+.4f | %s\n",
          candidate.generation_index, candidate.go_left ? "L" : "R", candidate.d_target,
          candidate.d_mid, candidate.entry_scale, candidate.exit_scale,
          candidate.hard_valid ? "OK" : "XX", candidate.peak_curvature_radpm,
          candidate.peak_lateral_slope,
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

// 🔴 배치 과적합 방지 (2026-08-17). 시나리오가 한 배치에서만 나오면, 그 배치에만 맞춘
// 변경이 그대로 통과한다. 실제로 전이 형상 파라미터(pre_apex_distances_m 등)는 "FINALS
// 3장애물 맵"에서 CMA-ES로 뽑은 값이고, 배치를 바꾼 2026-08-17 00:10 백에서 충돌 3건과
// 정지 다수가 나왔다.
//
// 그래서 성공 케이스를 **서로 다른 두 배치**에서 가져온다:
//   passing_mixed    2026-08-16 17:02 백 (장애물 s=2.6/8.3/22.5/23.1/23.5/26.7/32.7/37.4)
//   layoutB_passing  2026-08-17 00:10 백 (장애물 s=9.9/13.1/23.5/24.3/30.9/34.7/36.4)
//   layoutC_passing  2026-08-17 00:34 백 (장애물 s=13.2/18.5/23.5/24.4/31.0/34.7/36.4)
//
// ⚠️ 랩 경계(s가 트랙 길이 근처, 43.44 m) 프레임은 백에서 성공(NONE)인데 오프라인 재현에서
// 실패한다 — 배치 B의 s=36.40, 배치 C의 s=38.66에서 두 번 확인했다. 재현되지 않는 프레임을
// 안전망에 두면 엉뚱한 이유로 실패하므로 제외했다. wrap 구간의 온·오프라인 차이는 별도
// 확인이 필요하며, 그 자체가 잠재적 결함일 수 있다.
// 앞으로의 수리는 **세 배치를 동시에** 만족해야 한다.
TEST(P3ProductionParity, PassingScenariosKeepRecoveringOnEveryLayout)
{
  for (const auto & entry : {std::make_pair("passing_mixed", 4U),
      std::make_pair("layoutB_passing", 5U),
      std::make_pair("layoutC_passing", 4U)})
  {
    const auto stream = readStream(scenarioPath(entry.first));
    const auto recovers = recoversPerFrame(stream);
    ASSERT_EQ(recovers.size(), entry.second) << entry.first;
    for (std::size_t index = 0; index < recovers.size(); ++index) {
      EXPECT_TRUE(recovers[index])
        << entry.first << " frame " << index << ": 되던 회피가 안 된다(회귀)";
    }
  }
}

// 배치 B에서 드러난 실패. 2026-08-17 00:10 백에서 랩마다 s≈21에서 재계획이 전멸해
// 안전정지로 떨어졌고, 그 뒤 해제되면서 실행 불가능한 회피를 커밋해 s≈23.85에서 obs4에
// 충돌했다(2회).
//
// 2026-08-17 수리 완료 — 진입 눈금 이분법으로 3프레임 전부 회복한다. 배치 B의 재계획
// 전멸도 pinch_failure와 **같은 원인**이었다(고정 눈금 0.5146은 곡률 초과, 1.311은 회랑
// 침범, 실현 구간은 그 사이). 이분법이 entry=0.913을 찾는다.
TEST(P3ProductionParity, LayoutBReplanRecovers)
{
  const auto stream = readStream(scenarioPath("layoutB_failing"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 3U);
  for (std::size_t index = 0; index < recovers.size(); ++index) {
    EXPECT_TRUE(recovers[index])
      << "layoutB_failing frame " << index << ": 되던 회피가 안 된다(회귀)";
  }
}

// 후보를 늘리는 방향의 변경은 "없던 해를 억지로 만들어내는" 쪽으로 틀어지기 쉽다.
// 물리적으로 통과 불가능한 배치에서는 계속 실패해야 한다.
// 배치 C에서 드러난 실패: BOUNDARY_HANDOFF_UNRESOLVED.
//
// 🔴 진단 정정 (2026-08-17). 처음에는 "자차가 옆으로 깊이 나가 있을 때"로 봤다. 그 상관은
// 있었지만(|d|>=0.45에서 3배) **혼동 변수**였다 — 회피 중이라 옆으로 나가 있었을 뿐이다.
// 실제 조건은 `cluster_start_forward_m < 0`, 즉 자차가 이미 클러스터 앞단을 지나 옆에
// 나란히 있는 상태다. 00:34 백 전수 대조:
//   cluster_start <  0   13건 → BOUNDARY_HANDOFF 13건 (100%)
//   cluster_start >= 0  129건 → BOUNDARY_HANDOFF  0건 (  0%)
//
// 메커니즘 (p3_shadow.cpp stationsFor):
//   entry_length = cluster_start * pre_apex_distances_m.front() * entry / detection_lookahead_m
// cluster_start가 음수면 entry_length도 음수가 되어 첫 구간 길이가 음수가 되고,
// strictPositiveSegments가 모든 M1 후보를 기각한다(segrej=4, cand=0).
//
// ⚠️ 이 13건은 **전부 v>=2.9 m/s의 주행 중 과도 상태**이고 정지와 무관하다(저속 0건).
// 그 백의 정지 7건은 전혀 다른 원인이다 — 안전정지 래치이며, 그 방아쇠 3건은 진입 불연속
// 검사의 오탐이었다(RacelineSplinePlanner.NormalTrackingErrorOverATinyBaselineIsNotADiscontinuity
// 참조). 즉 이 테스트가 고정하는 것은 랩타임 손실이 아니라 기하 구성의 결함이다.
//
// 아직 고치지 않았으므로 실패가 정상이다. 고쳐지면 회복 케이스로 옮길 것.
TEST(P3ProductionParity, ReplanBesideAClusterIsStillUnsolved)
{
  const auto stream = readStream(scenarioPath("layoutC_failing"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 3U);
  for (std::size_t index = 0; index < recovers.size(); ++index) {
    EXPECT_FALSE(recovers[index])
      << "layoutC_failing frame " << index << ": 이미 고쳐졌다면 회복 케이스로 옮길 것";
  }
}

// 좁은 길목 직후 장애물 — 같은 기하를 거리만 달리해 평가하면 결과가 갈린다.
//
// 2026-08-17 두 백 대조. 자차 s≈20.85에서 둘 다 똑같이 실패하고, 00:10은 검출이 끊겨
// 래치가 0.33 s 만에 풀린 덕에 1.8 m 더 굴러가 s=22.60에서 성공했다. 01:05은 검출이
// 안정화되어(끊김 50회 → 5회) 그 우연한 탈출구가 사라졌고 9.9 s 정지했다.
//
//   pinch_success  s=22.60  v=1.99  cluster_start=1.02  → 성공
//   pinch_failure  s=20.88  v=3.28  cluster_start=2.77  → 수리 전 실패 / 수리 후 성공
//
// 두 램프의 절대 위치·길이는 사실상 같다(22.60~23.62 L=1.02 / 22.56~23.65 L=1.09).
// 그런데도 갈리는 이유를 이 두 스트림으로 고정한다.
TEST(P3ProductionParity, SamePinchGeometryAtTwoDistances)
{
  const auto success = recoversPerFrame(readStream(scenarioPath("pinch_success")));
  ASSERT_EQ(success.size(), 1U);
  EXPECT_TRUE(success[0]) << "가까이서 되던 회피가 안 된다(회귀)";

  // 2026-08-17 수리 완료 — 진입 눈금 이분법(evaluateSide)으로 두 프레임 모두 회복한다.
  // 이분법이 2스텝 만에 entry=0.913을 찾는다(고정 눈금은 0.5146과 1.311뿐이었다).
  const auto failure = recoversPerFrame(readStream(scenarioPath("pinch_failure")));
  ASSERT_EQ(failure.size(), 2U);
  for (std::size_t index = 0; index < failure.size(); ++index) {
    EXPECT_TRUE(failure[index])
      << "pinch_failure frame " << index << ": 되던 회피가 안 된다(회귀)";
  }
}

// 🔴 평가기의 내부 불변식 위반이 노드를 죽이면 안 된다 (2026-08-17).
//
// p3_shadow.cpp에는 후보 예산·구간 포함관계를 지키는 throw std::runtime_error가 6개 있고,
// evaluateP3Shadow는 (a) 타이머 콜백에서, (b) plan() 안에서 불린다. ROS 2 콜백을 넘어간
// 예외는 executor를 타고 나가 local_planner_node를 통째로 종료시킨다 — 주행 중이면 회피도
// 안전정지도 남지 않는다. 노드 전체를 통틀어 catch는 진단용 std::stoll 하나뿐이었다.
//
// 여기서는 불변식을 직접 깨뜨릴 수 없으므로, 평가기가 어떤 입력에도 예외를 밖으로 흘리지
// 않는다는 계약만 고정한다. 극단 입력(NaN·역전 구간·거대 폭·트랙 길이 초과)을 넣는다.
TEST(P3ProductionParity, EvaluatorNeverThrowsOutOfTheCallback)
{
  const auto stream = readStream(scenarioPath("passing_mixed"));
  auto parameters = parametersOf(stream);
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(stream.reference));
  const double track_length = stream.reference.wpnts.back().s_m;
  const auto & frame = stream.frames.front();

  const auto make = [&](double s_start, double s_end, double d_right, double d_left) {
      f110_msgs::msg::Obstacle obstacle;
      obstacle.id = 991;
      obstacle.s_start = s_start;
      obstacle.s_end = s_end;
      obstacle.s_center = 0.5 * (s_start + s_end);
      obstacle.d_right = d_right;
      obstacle.d_left = d_left;
      obstacle.d_center = 0.5 * (d_right + d_left);
      obstacle.size = d_left - d_right;
      obstacle.is_static = true;
      obstacle.is_visible = true;
      return obstacle;
    };
  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::vector<f110_msgs::msg::Obstacle>> hostile{
    {},                                                            // 장애물 없음
    {make(frame.ego.s + 2.0, frame.ego.s + 1.0, -0.3, 0.3)},       // 구간 역전
    {make(frame.ego.s + 2.0, frame.ego.s + 2.0, -0.3, 0.3)},       // 길이 0
    {make(frame.ego.s + 2.0, frame.ego.s + 2.5, 0.3, -0.3)},       // 좌우 역전
    {make(frame.ego.s + 2.0, frame.ego.s + 2.5, -1.0e6, 1.0e6)},   // 거대 폭
    {make(frame.ego.s + 3.0 * track_length, frame.ego.s + 3.1 * track_length, -0.3, 0.3)},
    {make(quiet_nan, quiet_nan, quiet_nan, quiet_nan)},            // NaN
    {make(frame.ego.s + 2.0, frame.ego.s + 2.5, -0.3, 0.3),
      make(frame.ego.s + 2.1, frame.ego.s + 2.4, -0.2, 0.2)},      // 완전 겹침
  };
  for (std::size_t index = 0; index < hostile.size(); ++index) {
    EXPECT_NO_THROW({
        const auto result = planner.evaluateP3Shadow(
        frame.ego, hostile[index], 0, 0U, 1U, "PARITY_ORACLE");
        (void)result;
    }) << "입력 " << index << ": 평가기가 예외를 콜백 밖으로 흘렸다 — 노드가 죽는다";
    // plan()도 내부에서 같은 평가기를 부른다.
    EXPECT_NO_THROW({
        const auto plan = planner.plan(frame.ego, hostile[index]);
        (void)plan;
    }) << "입력 " << index << ": plan()이 예외를 콜백 밖으로 흘렸다 — 노드가 죽는다";
  }
}

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
