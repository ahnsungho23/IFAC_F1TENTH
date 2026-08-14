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
RacelineSplineParameters operationalParameters()
{
  RacelineSplineParameters p;
  p.detection_lookahead_m = 12.0;
  p.obstacle_longitudinal_padding_m = 0.4149924657737441;
  p.vehicle_half_width_m = 0.1435;
  p.vehicle_length_m = 0.58;
  p.safety_margin_m = 0.0148;
  p.tracking_error_reserve_m = 0.14;
  p.tracking_error_lut_speed_bins_mps = {0.0, 1.5, 3.0, 4.5, 6.5};
  p.tracking_error_lut_curvature_bins_radpm = {0.0, 0.2, 0.5, 0.9, 1.316266519079011};
  p.tracking_error_lut_values_m = {
    0.200, 0.200, 0.200, 0.200, 0.200,
    0.260, 0.285, 0.285, 0.285, 0.285,
    0.275, 0.560, 0.560, 0.560, 0.560,
    0.275, 0.560, 0.560, 0.560, 0.560,
    0.275, 0.560, 0.560, 0.560, 0.560};
  p.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  p.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5};
  p.avoidance_minimum_speed_mps = 2.0;
  p.margin_pass_speed_cap_mps = 2.0;
  p.approach_feasibility_decel_mps2 = 2.0;
  p.commitment_retention_reserve_fraction = 0.5;
  p.localization_reserve_m = 0.12;
  p.wall_safety_margin_m = 0.04;
  p.fallback_track_half_width_m = 1.50;
  p.pre_apex_distances_m = {11.442, 7.628, 3.814};
  p.post_apex_distances_m = {2.0595099500051189, 4.1190199000102378, 6.1785298500153566};
  p.entry_transition_fractions = {0.5145, 0.75, 1.00};
  p.transition_distance_scales = {0.497, 0.699, 3.698};
  p.outside_line_transition_scale = 0.406;
  p.maximum_target_offset_m = 1.20;
  p.target_d_candidate_count = 3;
  p.maximum_lateral_slope = 0.8;
  p.maximum_curvature_radpm = 1.316266519079011;
  p.maximum_curvature_rate_radpm2 = 20.0;
  p.safe_stop_buffer_m = 1.20;
  p.safe_stop_deceleration_mps2 = 1.8;
  p.minimum_path_points = 8;
  return p;
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

  const auto result = planner.plan(ego, {obstacle});
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
  std::printf("결과: %s (margin_pass=%d, 점 %zu개)\n  이유: %s\n",
    kind, static_cast<int>(result.margin_pass), result.path.wpnts.size(),
    result.reason.c_str());
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
