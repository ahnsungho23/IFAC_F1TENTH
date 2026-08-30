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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/p3_shadow.hpp"
#include "p3_scenario_stream.hpp"
#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/research_instrumentation.hpp"

namespace local_planning
{
namespace
{

using test_stream::Frame;
using test_stream::Stream;
using test_stream::readStream;
using test_stream::parametersOf;
using test_stream::scenarioPath;

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

TEST(P3ProductionParity, ResearchTracePreservesDiscardedSidesAndM1ProbeRoots)
{
  bool saw_discarded_side = false;
  bool saw_m1_analytic_probe = false;
  for (const char * scenario : {"failing_cluster11", "failing_cluster1", "layoutB_failing",
      "layoutB_passing", "layoutC_failing", "layoutC_passing", "passing_mixed",
      "pinch_failure", "pinch_success", "straight_margin_crawl"})
  {
    const auto stream = readStream(scenarioPath(scenario));
    RacelineSplinePlanner planner(parametersOf(stream));
    ASSERT_TRUE(planner.setReference(stream.reference));
    for (const auto & frame : stream.frames) {
      PlanningResearchCycle cycle;
      planner.setActiveResearchCycle(&cycle);
      const auto result = planner.evaluateP3Shadow(
        frame.ego, frame.obstacles, 0, 0U, 1U, "RESEARCH_PROVENANCE_TEST");
      planner.setActiveResearchCycle(nullptr);
      EXPECT_EQ(result.research_constructed_total_actual, result.research_all_candidates.size());
      EXPECT_GE(result.research_constructed_total_actual, result.candidate_count);
      EXPECT_FALSE(cycle.evaluations.empty());
      for (const auto & evaluation : cycle.evaluations) {
        EXPECT_EQ(evaluation.lineage.evaluation_sequence, 1U);
        EXPECT_EQ(evaluation.lineage.evaluation_role, "RESEARCH_PROVENANCE_TEST");
        EXPECT_FALSE(evaluation.lineage.input_snapshot_id.empty());
        EXPECT_FALSE(evaluation.lineage.ego_snapshot_id.empty());
        EXPECT_FALSE(evaluation.lineage.obstacle_snapshot_id.empty());
        EXPECT_FALSE(evaluation.lineage.reference_snapshot_id.empty());
        EXPECT_EQ(evaluation.lineage.reference_generation, 1U);
      }
      for (const auto & trace : result.research_all_candidates) {
        if (!trace.path.wpnts.empty()) {
          EXPECT_TRUE(std::isfinite(trace.waypoint0_s_m));
          EXPECT_TRUE(std::isfinite(trace.waypoint0_d_m));
          EXPECT_TRUE(std::isfinite(trace.waypoint0_yaw_rad));
          EXPECT_TRUE(std::isfinite(trace.waypoint0_footprint_track_margin_m));
        }
        saw_discarded_side = saw_discarded_side || trace.discarded_side;
        if (trace.generator_stage == "M1" && trace.root_type != "ZERO_INTERFACE") {
          EXPECT_TRUE(std::isfinite(trace.s_probe));
          EXPECT_TRUE(std::isfinite(trace.d_probe));
          EXPECT_GE(trace.root_index, 0);
          EXPECT_NE(trace.probe_location_rule, "NONE");
          EXPECT_NE(trace.probe_anchor_rule, "NONE");
          saw_m1_analytic_probe = true;
        }
      }
    }
  }
  {
    auto stream = readStream(scenarioPath("straight_margin_crawl"));
    for (auto & waypoint : stream.reference.wpnts) {
      waypoint.d_left = 2.0;
      waypoint.d_right = 2.0;
    }
    RacelineSplinePlanner planner(parametersOf(stream));
    ASSERT_TRUE(planner.setReference(stream.reference));
    const auto ego = stream.frames.front().ego;
    f110_msgs::msg::Obstacle obstacle;
    obstacle.id = 501;
    obstacle.s_center = ego.s + 5.0;
    obstacle.s_start = obstacle.s_center - 0.3;
    obstacle.s_end = obstacle.s_center + 0.3;
    obstacle.d_right = -0.1;
    obstacle.d_left = 0.1;
    obstacle.d_center = 0.0;
    obstacle.size = 0.2;
    obstacle.is_static = true;
    obstacle.is_visible = true;
    PlanningResearchCycle cycle;
    planner.setActiveResearchCycle(&cycle);
    const auto result = planner.evaluateP3Shadow(
      ego, {obstacle}, 0, 0U, 1U, "RESEARCH_DISCARDED_SIDE_TEST");
    planner.setActiveResearchCycle(nullptr);
    EXPECT_GT(result.research_m0_v1_constructed_right, 0U);
    EXPECT_GT(result.research_m0_v1_constructed_left, 0U);
    EXPECT_GT(result.research_discarded_side_candidate_count, 0U);
    saw_discarded_side = std::any_of(
      result.research_all_candidates.begin(), result.research_all_candidates.end(),
      [](const auto & trace) {return trace.discarded_side && !trace.returned_by_policy;});
  }
  EXPECT_TRUE(saw_discarded_side);
  EXPECT_TRUE(saw_m1_analytic_probe);
}

TEST(P3ProductionParity, ResearchLineageSeparatesEvaluatorInvocationsWithinOneCallback)
{
  const auto stream = readStream(scenarioPath("passing_mixed"));
  RacelineSplinePlanner planner(parametersOf(stream));
  ASSERT_TRUE(planner.setReference(stream.reference));
  ASSERT_FALSE(stream.frames.empty());
  PlanningResearchCycle cycle;
  cycle.obstacle_sequence = 17U;
  cycle.source_epoch = 4U;
  cycle.reference_generation = 8U;
  planner.setActiveResearchCycle(&cycle);
  const auto & frame = stream.frames.front();
  (void)planner.evaluateP3Shadow(
    frame.ego, frame.obstacles, 101, 4U, 8U, "PRIMARY");
  auto shifted_ego = frame.ego;
  shifted_ego.d += 0.01;
  (void)planner.evaluateP3Shadow(
    shifted_ego, frame.obstacles, 101, 4U, 8U, "SAFE_STOP_ESCAPE");
  planner.setActiveResearchCycle(nullptr);

  std::map<std::uint64_t, std::string> roles;
  std::map<std::uint64_t, std::string> inputs;
  for (const auto & evaluation : cycle.evaluations) {
    roles[evaluation.lineage.evaluation_sequence] = evaluation.lineage.evaluation_role;
    inputs[evaluation.lineage.evaluation_sequence] = evaluation.lineage.input_snapshot_id;
    EXPECT_EQ(evaluation.lineage.obstacle_sequence, 17U);
  }
  ASSERT_EQ(roles.size(), 2U);
  EXPECT_EQ(roles[1U], "PRIMARY");
  EXPECT_EQ(roles[2U], "SAFE_STOP_ESCAPE");
  EXPECT_NE(inputs[1U], inputs[2U]);
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
// 🔴 2026-08-17 수리 — stationsFor가 cluster_start <= 0 을 표현할 수 있게 됐다.
// 진입 램프를 클러스터 앞에 놓을 자리가 없으면 램프를 자차에서 시작시키고, 목표 도달 지점을
// 기울기 한계가 정하는 물리적 최소 이동거리(|target - ego_d| / maximum_lateral_slope)로 잡는다.
//
// 그 결과 두 경우가 비로소 갈린다:
//   frame 2 (s=30.91, d=+0.657): 자차 d가 이미 좌측 도메인 [+0.525,+0.932] 안 → 그 오프셋을
//           유지하고 빠져나가면 된다 → **성공**. 종전에는 표현 자체가 안 돼 정지했다.
//   frame 1 (s=13.03, d=-0.017): 좌측 도메인 [+0.150,+0.457]까지 옮겨야 하는데 이미 옆에
//           붙어 거리가 없다 → 후보는 생성되나(segrej 4->0, cand 0->3) 하드 검증에서 기각.
//           전진으로 해결 불가가 물리적 사실이므로 실패가 정상이다.
//   frame 0: 이 프레임에는 라인을 막는 장애물이 없다(회피 대상 아님).
TEST(P3ProductionParity, ReplanBesideAClusterUsesTheRemainingSpan)
{
  const auto stream = readStream(scenarioPath("layoutC_failing"));
  const auto recovers = recoversPerFrame(stream);
  ASSERT_EQ(recovers.size(), 3U);
  EXPECT_FALSE(recovers[0]) << "frame 0: 막는 장애물이 없는데 회피를 만들었다";
  EXPECT_FALSE(recovers[1]) << "frame 1: 전진으로 도달 불가인데 회피를 만들었다 — 거짓 통과";
  EXPECT_TRUE(recovers[2]) << "frame 2: 이미 비켜 있는 오프셋을 유지하면 되는데 실패했다";
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
