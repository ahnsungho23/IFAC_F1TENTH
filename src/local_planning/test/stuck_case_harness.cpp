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
// 실차 정지 교착 재현 하니스 (진단 전용, 런타임 코드 아님).
//
// 실제 글로벌 라인 CSV + bag에서 뽑은 자차/장애물 상태를 그대로 넣어 plan()을 호출하고,
// 각 후보의 거부 이유를 출력한다. 2026-08-14 실차에서 "옆 여유가 1.1~1.8 m나 되는데도
// 회피를 시도조차 않고 정지"한 사건의 원인을 찾기 위해 만들었다.
//
// 사용: stuck_case_harness <waypoints.csv> <ego_s> <ego_d> <ego_v>
//                          <obs_s_start> <obs_s_end> <obs_d_right> <obs_d_left>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

using local_planning::EgoFrenetState;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;
using local_planning::SplinePlanKind;

namespace
{

f110_msgs::msg::WpntArray loadReference(const std::string & path)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  std::ifstream file(path);
  if (!file) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(2);
  }
  std::string line;
  std::getline(file, line);                 // header
  std::vector<std::string> header;
  {
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      header.push_back(cell);
    }
  }
  auto column = [&](const std::string & name) {
      for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };
  const int c_s = column("s") >= 0 ? column("s") : column("s_m");
  const int c_x = column("x_m"), c_y = column("y_m"), c_psi = column("psi_rad");
  const int c_k = column("kappa_radpm"), c_v = column("vx_mps");
  const int c_dl = column("d_left"), c_dr = column("d_right");
  int id = 0;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::vector<double> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell.empty() ? 0.0 : std::stod(cell));
    }
    f110_msgs::msg::Wpnt w;
    w.id = id++;
    w.s_m = cells[c_s];
    w.x_m = cells[c_x];
    w.y_m = cells[c_y];
    w.psi_rad = cells[c_psi];
    w.kappa_radpm = cells[c_k];
    w.vx_mps = cells[c_v];
    w.d_left = cells[c_dl];
    w.d_right = cells[c_dr];
    reference.wpnts.push_back(w);
  }
  return reference;
}

// 운영 YAML(local_planning.yaml)과 같은 값. 하니스가 조용히 다른 마진으로 통과시키는 것을
// 막기 위해 여기 명시한다.
//
// ⚠️ 2026-08-14 리뷰에서 실제로 어긋나 있던 것이 발견됐다: vehicle_length 0.58 vs 0.56,
// localization_reserve 0.12 vs 0.06, maximum_target_offset 1.20 vs 1.50,
// target_d_candidate_count 3 vs 5. 뒤의 둘은 후보를 60개가 아니라 36개만, 그것도 좁은
// 오프셋 범위에서만 생성하게 만들어 하니스를 운영보다 **비관적**으로 만들었고, 그 상태로
// 잰 탈출 임계(2.0~2.5 m)로 safe_stop_buffer_m을 정했다. YAML을 고칠 때 이 표도 같이
// 고칠 것 — 아래 test/test_params_match_yaml.py가 이를 자동으로 검사한다.
RacelineSplineParameters operationalParameters()
{
  RacelineSplineParameters p;
  p.detection_lookahead_m = 15.0;
  // config/local_planning.yaml 과 일치해야 한다(params_match_yaml).
  p.obstacle_longitudinal_padding_m = 0.0;
  p.vehicle_half_width_m = 0.15;
  p.vehicle_length_m = 0.56;
  p.safety_margin_m = 0.05;   // 2026-08-23: 0.50 m 자유폭 예산 (YAML 과 동기)
  // 예약 게이트 (2026-08-22). 운영이 "none" 이므로 하니스도 꺼야 같은 질문에 답한다 —
  // 켜 두면 하니스만 0.39 짜리 예약을 쓰고 운영보다 훨씬 비관적인 진단을 낸다.
  p.obstacle_reserve_from_lut = false;
  p.tracking_error_reserve_m = 0.0;
  // 아래 표는 게이트가 꺼져 있어 읽히지 않지만, 되돌리기(mode: "lut") 때 운영과 같아야
  // 하므로 그대로 둔다 — 2026-08-20 복원된 08-19 실측표.
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
  p.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  p.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.6, 7.6, 7.6, 7.6, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5};
  // 종방향 한계표 — local_planning_velocity_limits.csv와 동기.
  // test/test_velocity_limits_match_csv.py 가 YAML ↔ csv 일치를 검사한다.
  p.avoidance_velocity_limit_accel_mps2 =
  {3.7, 3.7, 3.7, 3.7, 3.7, 3.47, 3.33, 3.0, 3.0, 3.0};
  p.avoidance_velocity_limit_decel_mps2 =
  {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  p.longitudinal_launch_speed_floor_mps = 1.0;
  p.handoff_speed_shaping_enable = true;
  p.confirmed_obstacle_speed_envelope_enable = true;
  p.confirmed_speed_post_hold_distance_m = 1.0;
  p.confirmed_speed_response_delay_sec = 0.15;
  p.avoidance_minimum_speed_mps = 1.0;
  p.margin_pass_speed_cap_mps = 2.0;
  p.approach_feasibility_decel_mps2 = 2.0;
  p.approach_feasibility_decel_max_mps2 = 3.5;
  p.commitment_retention_reserve_fraction = 0.5;
  p.localization_reserve_m = 0.0;
  p.wall_safety_margin_m = 0.04;   // 2026-08-23: 0.50 m 자유폭 예산 (YAML 과 동기)
  p.fallback_track_half_width_m = 1.50;
  p.pre_apex_distances_m = {11.442, 7.628, 3.814};
  p.post_apex_distances_m = {2.0595099500051189, 4.1190199000102378, 6.1785298500153566};
  p.entry_transition_fractions = {0.5145, 0.75, 1.00};
  p.transition_distance_scales = {0.497, 0.699, 3.698};
  p.outside_line_transition_scale = 0.4060036444074003;
  p.maximum_target_offset_m = 1.50;
  p.target_d_candidate_count = 5;
  p.maximum_lateral_slope = 0.8;
  p.entry_discontinuity_min_budget_m = 0.20;
  p.entry_continuity_baseline_m = 0.50;   // 2026-08-23: 진입 기울기 분모 바닥 (YAML 과 동기)
  p.maximum_curvature_radpm = 1.316266519079011;
  p.maximum_curvature_rate_radpm2 = 20.0;
  p.control_wheelbase_m = 0.33;
  p.control_max_steering_left_rad = 0.410;
  p.control_max_steering_right_rad = 0.361;
  p.control_understeer_gradient_left_rad_per_mps2 = 0.014;
  p.control_understeer_gradient_right_rad_per_mps2 = 0.019;
  p.control_max_steering_rate_radps = 20.0;
  // 🔴 2026-08-16: 아래 8개는 종전에 하니스가 설정하지 않아 **구조체 기본값**이 쓰였다.
  // 특히 post_merge_lookahead_m이 기본 2.0 / 운영 5.0으로 어긋나 하니스의 충돌 검사 범위가
  // 운영보다 3 m 짧았고, 그래서 "다음 장애물에 걸리는" 종류의 실패를 하니스가 통과시켰다.
  // 이 세션에서 결합 장애물 진단이 하니스와 실제 백에서 두 번 엇갈린 원인이다.
  // test/test_params_match_yaml.py가 이제 누락도 검사한다.
  p.obstacle_cluster_gap_m = 0.8;
  p.minimum_target_offset_m = 0.15;
  p.maximum_exit_length_m = 0.0;
  p.merge_ramp_min_length_m = 0.0;
  p.merge_ramp_time_sec = 0.0;
  p.post_merge_lookahead_m = 5.0;
  p.post_merge_min_time_sec = 1.0;
  p.profile_feasibility_decel_mps2 = 3.5;
  p.raw_slowdown_post_hold_distance_m = 1.0;
  p.safe_stop_buffer_m = 2.60;
  p.safe_stop_deceleration_mps2 = 1.8;
  p.minimum_path_points = 8;
  p.safe_stop_escape_check_enable = true;
  p.safe_stop_escape_retreat_step_m = 0.30;
  p.safe_stop_escape_max_retreats = 8;
  return p;
}

// 시뮬(gym+MCL) 오버레이 — config/local_planning_sim.yaml과 같은 두 값만 바꾼다.
// HARNESS_SIM=1일 때만 적용한다. 기본값이 항상 운영값인 이유: 시뮬 백을 진단하면서 하니스가
// 운영 마진으로 답하면 "실차에서는 못 지나간다"를 "시뮬에서 못 지나갔다"로 오독하게 되고,
// 반대로 기본을 시뮬로 두면 운영 진단이 조용히 통과해버린다. 어느 쪽인지 배너로 찍는다.
//
// ⚠️ 대입 대상 이름이 p가 아니라 sim인 것은 의도적이다 — test_params_match_yaml.py는
// `p.<name> = <number>;`만 운영 YAML과 대조하므로, 여기 값이 그 검사에 섞이면 안 된다.
void applySimulationOverlay(RacelineSplineParameters & sim)
{
  // 축이 7행(0,1,1.6,2.2,2.9,4.5,7)으로 바뀌어 시뮬 오버레이도 같은 모양으로 확장.
  sim.tracking_error_lut_values_m = {
    0.115, 0.115, 0.115, 0.125, 0.125,
    0.175, 0.245, 0.280, 0.280, 0.280,
    0.175, 0.245, 0.280, 0.280, 0.280,
    0.175, 0.245, 0.280, 0.280, 0.280,
    0.185, 0.245, 0.280, 0.280, 0.280,
    0.185, 0.245, 0.280, 0.280, 0.280,
    0.185, 0.245, 0.280, 0.280, 0.280};
  sim.localization_reserve_m = 0.06;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 9) {
    std::cerr << "usage: stuck_case_harness <csv> <ego_s> <ego_d> <ego_v> "
              << "<obs_s_start> <obs_s_end> <obs_d_right> <obs_d_left>\n";
    return 2;
  }
  const auto reference = loadReference(argv[1]);
  auto parameters = operationalParameters();
  const char * sim_env = std::getenv("HARNESS_SIM");
  const bool simulation = sim_env != nullptr && std::string(sim_env) == "1";
  if (simulation) {
    applySimulationOverlay(parameters);
  }
  std::printf(
    "파라미터: %s (LUT[1.5m/s,직선]=%.3f, localization_reserve=%.3f)\n",
    simulation ? "시뮬 오버레이 local_planning_sim.yaml" : "운영 local_planning.yaml",
    parameters.tracking_error_lut_values_m[5], parameters.localization_reserve_m);
  // 선택 인자 9: localization_reserve_m 오버라이드 (MCL 수리 전/후 비교용)
  if (argc >= 10) {
    parameters.localization_reserve_m = std::atof(argv[9]);
  }
  // 선택 인자 10: 안전정지 탈출 검증 on/off (비용 측정·회귀 비교용)
  if (argc >= 11) {
    parameters.safe_stop_escape_check_enable = std::atoi(argv[10]) != 0;
  }
  // 선택 인자 11: plan() 반복 횟수 (비용 측정용)
  const int repeat = (argc >= 12) ? std::max(1, std::atoi(argv[11])) : 1;
  RacelineSplinePlanner planner(parameters);
  if (!planner.setReference(reference)) {
    std::cerr << "reference rejected\n";
    return 2;
  }

  EgoFrenetState ego;
  ego.s = std::atof(argv[2]);
  ego.d = std::atof(argv[3]);
  ego.speed = std::atof(argv[4]);

  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = 1;
  obstacle.s_start = std::atof(argv[5]);
  obstacle.s_end = std::atof(argv[6]);
  obstacle.s_center = 0.5 * (obstacle.s_start + obstacle.s_end);
  obstacle.d_right = std::atof(argv[7]);
  obstacle.d_left = std::atof(argv[8]);
  obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
  obstacle.size = obstacle.d_left - obstacle.d_right;
  obstacle.is_static = true;

  std::vector<f110_msgs::msg::Obstacle> obstacles{obstacle};
  // 추가 장애물: HARNESS_OBS2="s0,s1,dr,dl" (노드는 시야 내 모든 장애물을 함께 계획한다 —
  // 단일 장애물 재현이 노드와 다르게 성공할 때 여기부터 의심할 것)
  if (const char * extra = std::getenv("HARNESS_OBS2")) {
    f110_msgs::msg::Obstacle second = obstacle;
    second.id = 2;
    if (std::sscanf(
        extra, "%lf,%lf,%lf,%lf",
        &second.s_start, &second.s_end, &second.d_right, &second.d_left) == 4)
    {
      second.s_center = 0.5 * (second.s_start + second.s_end);
      second.d_center = 0.5 * (second.d_right + second.d_left);
      second.size = second.d_left - second.d_right;
      obstacles.push_back(second);
      std::printf(
        "obs2 s=[%.2f,%.2f] d=[%+.2f,%+.2f]\n",
        second.s_start, second.s_end, second.d_right, second.d_left);
    }
  }
  for (int i = 0; i < repeat - 1; ++i) {
    (void)planner.plan(ego, obstacles);
  }
  const auto result = planner.plan(ego, obstacles);
  const char * kind = "?";
  switch (result.kind) {
    case SplinePlanKind::kAvoidance: kind = "kAvoidance"; break;
    case SplinePlanKind::kSafeStop: kind = "kSafeStop"; break;
    case SplinePlanKind::kNoSafePath: kind = "kNoSafePath"; break;
    default: break;
  }
  std::printf("ego s=%.2f d=%+.2f v=%.2f | obs s=[%.2f,%.2f] d=[%+.2f,%+.2f]\n",
    ego.s, ego.d, ego.speed, obstacle.s_start, obstacle.s_end,
    obstacle.d_right, obstacle.d_left);
  double sel_wall = std::nan("");
  for (const auto & a : result.candidate_audits) {
    if (a.selected) {
      sel_wall = a.centerline_wall_clearance_m;
    }
  }
  std::printf(
    "결과: %s (margin_pass=%d, 점 %zu개) target_d=%+.3f 선택후보 벽여유=%.3f\n"
    "  이유: %s\n",
    kind, static_cast<int>(result.margin_pass), result.path.wpnts.size(),
    result.target_d, sel_wall, result.reason.c_str());
  if (result.kind == SplinePlanKind::kSafeStop) {
    std::printf(
      "  안전정지: 정지점 자차+%.2f m, 탈출검증 %s\n",
      result.safe_stop_forward_m, result.safe_stop_escape_verified ? "통과" : "❌실패");
  }
  std::printf("후보 %zu개:\n", result.candidate_audits.size());
  for (const auto & a : result.candidate_audits) {
    std::printf(
      "  [%zu] %s target_d=%+.2f entry_frac=%.3f entry_len=%.2f exit_len=%.2f "
      "wall=%.3f footprint=%.3f%s\n      %s\n",
      a.generation_index, a.go_left ? "좌" : "우", a.target_d, a.entry_fraction,
      a.effective_entry_length_m, a.exit_length_m,
      a.centerline_wall_clearance_m, a.rectangular_footprint_wall_clearance_m,
      a.feasible ? "  [가능]" : "", a.rejection_reason.c_str());
  }
  return 0;
}
