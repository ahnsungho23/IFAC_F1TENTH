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

// Audit-only harness for immutable P3_ORACLE_EVENT_V1 snapshots. It invokes only the integrated
// production evaluator; no Oracle coordinates or outcomes enter the planner.

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/path_digest.hpp"
#include "local_planning/p3_r3_k12.hpp"
#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/research_instrumentation.hpp"

namespace
{

using local_planning::EgoFrenetState;
using local_planning::PlanningResearchCycle;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;

struct Event
{
  std::string id;
  EgoFrenetState ego;
  f110_msgs::msg::WpntArray reference;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> output;
  std::size_t begin = 0U;
  while (true) {
    const auto end = line.find('\t', begin);
    output.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      return output;
    }
    begin = end + 1U;
  }
}

double number(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite number: " + token);
  }
  return value;
}

std::int64_t integer(const std::string & token)
{
  std::size_t consumed = 0U;
  const auto value = std::stoll(token, &consumed);
  if (consumed != token.size()) {
    throw std::runtime_error("invalid integer: " + token);
  }
  return value;
}

Event readEvent(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "P3_ORACLE_EVENT_V1") {
    throw std::runtime_error("unsupported event: " + path.string());
  }
  Event event;
  event.reference.header.frame_id = "map";
  while (std::getline(input, line)) {
    const auto fields = split(line);
    if (fields[0] == "EVENT" && fields.size() == 11U) {
      event.id = fields[1];
      event.ego.s = number(fields[6]);
      event.ego.d = number(fields[7]);
      event.ego.speed = number(fields[8]);
    } else if (fields[0] == "W" && fields.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(integer(fields[1]));
      waypoint.s_m = number(fields[2]);
      waypoint.d_m = number(fields[3]);
      waypoint.x_m = number(fields[4]);
      waypoint.y_m = number(fields[5]);
      waypoint.d_right = number(fields[6]);
      waypoint.d_left = number(fields[7]);
      waypoint.psi_rad = number(fields[8]);
      waypoint.kappa_radpm = number(fields[9]);
      waypoint.vx_mps = number(fields[10]);
      waypoint.ax_mps2 = number(fields[11]);
      event.reference.wpnts.push_back(waypoint);
    } else if (fields[0] == "O" && fields.size() == 12U) {
      f110_msgs::msg::Obstacle obstacle;
      obstacle.id = static_cast<std::int32_t>(integer(fields[1]));
      obstacle.s_center = number(fields[2]);
      obstacle.s_start = number(fields[3]);
      obstacle.s_end = number(fields[4]);
      obstacle.d_right = number(fields[5]);
      obstacle.d_left = number(fields[6]);
      obstacle.size = number(fields[7]);
      obstacle.s_var = number(fields[8]);
      obstacle.d_var = number(fields[9]);
      obstacle.is_static = integer(fields[10]) != 0;
      obstacle.is_visible = integer(fields[11]) != 0;
      obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
      event.obstacles.push_back(obstacle);
    } else if (fields[0] == "END_EVENT") {
      break;
    }
  }
  if (event.id.empty() || event.reference.wpnts.empty()) {
    throw std::runtime_error("incomplete event: " + path.string());
  }
  return event;
}

RacelineSplineParameters frozenParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 15.0;
  parameters.obstacle_cluster_gap_m = 0.8;
  parameters.obstacle_longitudinal_padding_m = 0.0;
  parameters.vehicle_length_m = 0.56;
  parameters.vehicle_half_width_m = 0.15;
  parameters.safety_margin_m = 0.08;
  parameters.obstacle_reserve_from_lut = false;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0, 1, 1.6, 2.2, 2.9, 4.5, 7};
  parameters.tracking_error_lut_curvature_bins_radpm = {0, 0.1, 0.2, 0.35, 0.46};
  parameters.tracking_error_lut_values_m = {
    .095, .095, .095, .130, .130, .100, .165, .165, .165, .165,
    .100, .165, .165, .165, .165, .145, .165, .190, .230, .230,
    .275, .275, .390, .390, .390, .275, .275, .390, .390, .390,
    .275, .275, .390, .390, .390};
  parameters.avoidance_velocity_limit_speed_bins_mps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.6, 7.6, 7.6, 7.6, 7, 7, 7, 6.5, 6.5, 6.5};
  parameters.avoidance_velocity_limit_accel_mps2 =
  {3.7, 3.7, 3.7, 3.7, 3.7, 3.47, 3.33, 3, 3, 3};
  parameters.avoidance_velocity_limit_decel_mps2 = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
  parameters.longitudinal_launch_speed_floor_mps = 1.0;
  parameters.handoff_speed_shaping_enable = true;
  parameters.confirmed_obstacle_speed_envelope_enable = true;
  parameters.confirmed_speed_post_hold_distance_m = 1.0;
  parameters.confirmed_speed_response_delay_sec = 0.15;
  parameters.analytic_path_geometry_enable = true;
  parameters.avoidance_minimum_speed_mps = 1.0;
  parameters.wall_safety_margin_m = 0.04;
  parameters.fallback_track_half_width_m = 1.5;
  parameters.margin_pass_speed_cap_mps = 2.0;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  parameters.approach_feasibility_decel_max_mps2 = 3.5;
  parameters.profile_feasibility_decel_mps2 = 3.5;
  parameters.commitment_retention_reserve_fraction = 0.5;
  parameters.localization_reserve_m = 0.0;
  parameters.pre_apex_distances_m = {11.442220427651225, 7.628146951767484, 3.814073475883742};
  parameters.post_apex_distances_m = {2.059509950005119, 4.119019900010238, 6.178529850015357};
  parameters.entry_transition_fractions = {0.5145810930150512, 0.75, 1.0};
  parameters.transition_distance_scales = {0.4971684162574945, 0.6991537701867223,
    3.698773101198193};
  parameters.outside_line_transition_scale = 0.4060036444074003;
  parameters.maximum_exit_length_m = 0.0;
  parameters.post_merge_lookahead_m = 5.0;
  parameters.post_merge_min_time_sec = 1.0;
  parameters.merge_ramp_min_length_m = 0.0;
  parameters.merge_ramp_time_sec = 0.0;
  parameters.minimum_target_offset_m = 0.15;
  parameters.maximum_target_offset_m = 1.5;
  parameters.target_d_candidate_count = 5;
  parameters.maximum_lateral_slope = 0.8;
  parameters.entry_discontinuity_min_budget_m = 0.2;
  parameters.entry_continuity_baseline_m = 0.5;
  parameters.maximum_curvature_radpm = 1.316266519079011;
  parameters.maximum_curvature_rate_radpm2 = 20.0;
  parameters.control_wheelbase_m = 0.33;
  parameters.control_max_steering_left_rad = 0.410;
  parameters.control_max_steering_right_rad = 0.361;
  parameters.control_understeer_gradient_left_rad_per_mps2 = 0.014;
  parameters.control_understeer_gradient_right_rad_per_mps2 = 0.019;
  parameters.control_max_steering_rate_radps = 20.0;
  parameters.safe_stop_buffer_m = 2.6;
  parameters.safe_stop_deceleration_mps2 = 1.8;
  parameters.raw_slowdown_post_hold_distance_m = 1.0;
  parameters.minimum_path_points = 8;
  return parameters;
}

void emit(const Event & event, double event_read_us)
{
  using Clock = std::chrono::steady_clock;
  RacelineSplinePlanner planner(frozenParameters());
  std::string error;
  const auto reference_start = Clock::now();
  if (!planner.setReference(event.reference, &error)) {
    throw std::runtime_error(error);
  }
  const double reference_setup_us = std::chrono::duration<double, std::micro>(
    Clock::now() - reference_start).count();
  PlanningResearchCycle cycle;
  const bool instrumentation_enabled =
    std::getenv("LOCAL_PLANNING_RESEARCH_PARITY") != nullptr;
  if (instrumentation_enabled) {
    planner.setActiveResearchCycle(&cycle);
  }
  const auto evaluation_start = Clock::now();
  const auto result = planner.evaluateP3Shadow(event.ego, event.obstacles, 0, 0U, 1U, "R3_PARITY");
  const double evaluation_wall_us = std::chrono::duration<double, std::micro>(
    Clock::now() - evaluation_start).count();
  planner.setActiveResearchCycle(nullptr);
  local_planning::PlanningResearchSerializationProfile serialization;
  if (instrumentation_enabled && std::getenv("R3_PROFILE_SERIALIZE") != nullptr) {
    local_planning::PlanningResearchConfig config;
    serialization = local_planning::profilePlanningResearchSerialization(cycle, config);
  }
  std::cout << "SUMMARY\t" << event.id << '\t' << result.r3_invoked << '\t' <<
    result.would_recover << '\t' << result.r3_lateral_factor_count << '\t' <<
    result.r3_pair_priority_count << '\t' << result.r3_constructed_candidate_count << '\t' <<
    result.r3_path_digest_duplicate_count << '\t' << result.r3_validator_call_count << '\t' <<
    result.r3_hard_valid_count << '\t' << result.r3_usable_valid_count << '\t' <<
    result.selected_d_target << '\t' << result.selected_d_mid << '\t' <<
    result.selected_candidate_identity << '\t' << result.selected_path_digest << '\t' <<
    result.failure_classification << '\t' << result.r3_runtime_total_us << '\n';
  std::cout << "PROFILE\t" << event.id <<
    "\tinstrumentation_enabled=" << instrumentation_enabled <<
    "\tevent_read_us=" << event_read_us <<
    "\treference_setup_us=" << reference_setup_us <<
    "\tevaluation_wall_us=" << evaluation_wall_us <<
    "\tr3_total_us=" << result.r3_runtime_total_us <<
    "\tcontext_preparation_us=" << result.r3_runtime_context_preparation_us <<
    "\tgeometry_preparation_us=" << result.r3_runtime_geometry_preparation_us <<
    "\ttransition_generation_us=" << result.r3_runtime_transition_generation_us <<
    "\tlateral_factor_generation_us=" << result.r3_runtime_lateral_factor_generation_us <<
    "\tpair_priority_computation_us=" << result.r3_runtime_pair_priority_computation_us <<
    "\tlexicographic_ordering_us=" << result.r3_runtime_lexicographic_ordering_us <<
    "\tcoverage_ordering_us=" << result.r3_runtime_coverage_ordering_us <<
    "\tshape_deduplication_us=" << result.r3_runtime_shape_deduplication_us <<
    "\tcandidate_deduplication_us=" << result.r3_runtime_candidate_deduplication_us <<
    "\treconstruction_us=" << result.r3_runtime_reconstruction_us <<
    "\texact_validation_us=" << result.r3_runtime_validation_us <<
    "\tfinal_ranking_us=" << result.r3_runtime_final_ranking_us <<
    "\tresearch_lineage_us=" << cycle.runtime_research_lineage_us <<
    "\tresearch_capture_us=" << cycle.runtime_research_capture_us <<
    "\tresearch_serialization_us=" << serialization.runtime_us <<
    "\tresearch_serialized_bytes=" << serialization.serialized_byte_count <<
    "\ttransition_count=" << result.r3_transition_count <<
    "\tlateral_factor_count=" << result.r3_lateral_factor_count <<
    "\traw_combination_count=" << result.r3_raw_combination_count <<
    "\tproduction_excluded_count=" << result.r3_production_excluded_count <<
    "\tfactor_pool_count=" << result.r3_factor_pool_count <<
    "\tunique_profile_count=" << result.r3_unique_profile_count <<
    "\tproxy_metric_evaluation_count=" << result.r3_proxy_metric_evaluation_count <<
    "\tproxy_metric_cache_hit_count=" << result.r3_proxy_metric_cache_hit_count <<
    "\tpair_priority_count=" << result.r3_pair_priority_count <<
    "\tcorridor_sample_evaluation_count=" << result.r3_corridor_sample_evaluation_count <<
    "\treference_spacing_scan_count=" << result.r3_reference_spacing_scan_count <<
    "\treference_sample_count=" << result.r3_reference_sample_count <<
    "\tprofile_basis_build_count=" << result.r3_profile_basis_build_count <<
    "\tprofile_basis_cache_hit_count=" << result.r3_profile_basis_cache_hit_count <<
    "\tprofile_sample_basis_count=" << result.r3_profile_sample_basis_count <<
    "\tpair_metric_worker_count=" << result.r3_pair_metric_worker_count <<
    "\tconstructed_candidate_count=" << result.r3_constructed_candidate_count <<
    "\tvalidator_call_count=" << result.r3_validator_call_count << '\n';
  if (std::getenv("R3_PROFILE_ONLY") != nullptr) {
    return;
  }
  std::cout << "INTEGRATED_SELECTED\t" << event.id << '\t' << result.would_recover << '\t' <<
    result.candidate_count << '\t' << result.hard_validator_call_count << '\t' <<
    result.hard_valid_count << '\t' << result.selected_path_digest << '\t' <<
    result.selected_candidate_identity << '\t' << result.selected_d_target << '\t' <<
    result.selected_d_mid << '\n';
  for (const auto & candidate : result.candidates) {
    std::cout << "INTEGRATED_SEQUENCE\t" << event.id << '\t' << candidate.generation_index <<
      '\t' << candidate.generator_stage << '\t' << candidate.path_digest << '\t' <<
      candidate.hard_valid << '\n';
  }
  for (const auto & factor : result.r3_selected_factors) {
    std::cout << "FACTOR\t" << event.id << '\t' << factor.rank_stream << '\t' <<
      factor.stream_rank << '\t' << (factor.go_left ? "LEFT" : "RIGHT") << '\t' <<
      factor.d_target << '\t' << factor.d_mid << '\t' << factor.entry_scale << '\t' <<
      factor.exit_scale << '\t' << factor.target_source << '\t' << factor.mid_source << '\t' <<
      factor.path_digest << '\t' << factor.path_digest_duplicate << '\t' <<
      factor.validator_executed << '\t' << factor.hard_valid << '\t' << factor.usable_valid <<
      '\t' <<
      factor.exit_conflict_proxy << '\t' << factor.maximum_corridor_violation_m << '\t' <<
      factor.sum_corridor_violation_m << '\t' << factor.slope_excess << '\t' <<
      factor.curvature_proxy << '\t' << factor.center_error << '\t' <<
      factor.minimum_clearance_m << '\t' << factor.shape_energy << '\n';
  }
  const auto downstream = planner.plan(event.ego, event.obstacles);
  const std::string downstream_digest = downstream.path.wpnts.empty() ?
    "NONE" : local_planning::pathDigest(downstream.path);
  std::cout << "DOWNSTREAM_SELECTED\t" << event.id << '\t' <<
    static_cast<int>(downstream.kind) << '\t' << downstream_digest << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: p3_r3_k12_integration_harness EVENT [EVENT ...]\n";
    return 2;
  }
  try {
    std::cout << std::setprecision(17);
    for (int index = 1; index < argc; ++index) {
      const auto read_start = std::chrono::steady_clock::now();
      const Event event = readEvent(argv[index]);
      const double read_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - read_start).count();
      emit(event, read_us);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "R3-K12 integration harness failed: " << error.what() << '\n';
    return 1;
  }
}
