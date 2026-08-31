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

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
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

const local_planning::P3ShadowCandidateTrace * selectedTrace(
  const local_planning::P3ShadowResult & result)
{
  const auto selected = std::find_if(
    result.candidates.begin(), result.candidates.end(), [&result](const auto & candidate) {
      return candidate.selected ||
             candidate.candidate_identity == result.selected_candidate_identity;
    });
  return selected == result.candidates.end() ? nullptr : &*selected;
}

bool usable(const local_planning::P3ShadowCandidateTrace & candidate)
{
  return candidate.hard_valid && !candidate.exit_reaches_next_obstacle &&
         candidate.ego_braking_distance_deficit_m <= 1.0e-9;
}

bool anyUsable(const local_planning::P3ShadowResult & result)
{
  return std::any_of(result.candidates.begin(), result.candidates.end(), usable);
}

void emitStudyResult(
  const Event & event, const std::string & label,
  const local_planning::P3ShadowResult & result)
{
  const auto * selected = selectedTrace(result);
  std::cout << "STUDY_RESULT\t" << event.id << '\t' << label << '\t' << result.invoked << '\t' <<
    result.would_recover << '\t' << (selected != nullptr && usable(*selected)) << '\t' <<
    anyUsable(result) << '\t' << result.selected_generator_stage << '\t' <<
    result.selected_source << '\t' << result.selected_path_digest << '\t' <<
    result.selected_go_left << '\t' << result.selected_d_target << '\t' << result.selected_d_mid <<
    '\t' << (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->entry_scale) << '\t' <<
    (selected ==
  nullptr ? std::numeric_limits<double>::quiet_NaN() : selected->exit_scale) << '\t' <<
    result.candidate_count << '\t' << result.hard_validator_call_count << '\t' <<
    result.hard_valid_count << '\t' << result.m0_v1_candidate_count_actual << '\t' <<
    result.m0_v1_validator_call_count_actual << '\t' << result.m0_v1_hard_valid_count_actual <<
    '\t' << result.m0_v2_candidate_count << '\t' << result.m0_v2_validator_call_count << '\t' <<
    result.m0_v2_hard_valid_count << '\t' << result.m1_candidate_count << '\t' <<
    result.m1_validator_call_count << '\t' << result.m1_hard_valid_count << '\t' <<
    result.r3_constructed_candidate_count << '\t' << result.r3_validator_call_count << '\t' <<
    result.r3_hard_valid_count << '\t' << result.r3_usable_valid_count << '\t' <<
    result.runtime_context_preparation_us << '\t' << result.runtime_m0_v1_us << '\t' <<
    result.runtime_m0_v2_us << '\t' << result.runtime_m1_us << '\t' <<
    result.runtime_final_bookkeeping_us << '\t' << result.runtime_total_us << '\t' <<
    result.r3_runtime_total_us << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->minimum_normalized_safety_slack) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->minimum_track_margin_m) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->minimum_obstacle_margin_m) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->minimum_curvature_margin_radpm) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->curvature_rate_margin_radpm2) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->lateral_slope_margin) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->ego_braking_distance_deficit_m) << '\t' <<
    (selected == nullptr ? std::numeric_limits<double>::quiet_NaN() :
  selected->velocity_loss) << '\t' <<
    (selected != nullptr && selected->exit_reaches_next_obstacle) << '\t' <<
    result.failure_classification << '\n';
}

void emitGQSCDetail(
  const Event & event, const local_planning::P3ShadowResult & result,
  const std::string & prefix = "GQSC")
{
  for (std::size_t index = 0U; index < result.r3_production_seed_trace.size(); ++index) {
    const auto & row = result.r3_production_seed_trace[index];
    const auto & factor = row.factor;
    std::cout << prefix << "_LEGACY_SEED\t" << event.id << '\t' << index << '\t' <<
      (factor.go_left ? "LEFT" : "RIGHT") << '\t' << factor.d_target << '\t' <<
      factor.d_mid << '\t' << factor.entry_scale << '\t' << factor.exit_scale << '\t' <<
      row.generator_stage << '\t' <<
      row.candidate_template << '\t' << row.source_cell << '\t' << row.root_type << '\t' <<
      row.probe_location_rule << '\t' << row.probe_anchor_rule << '\n';
  }
  for (const auto & row : result.r3_side_geometry_trace) {
    std::cout << prefix << "_GEOMETRY\t" << event.id << '\t' <<
      (row.go_left ? "LEFT" : "RIGHT") <<
      '\t' << row.ego_d << '\t' << row.ego_speed << '\t' << row.cluster_start << '\t' <<
      row.cluster_end << '\t' << row.domain_lower << '\t' << row.domain_upper << '\t' <<
      row.bottleneck_center << '\t' << row.reference_spacing_m << '\t' <<
      row.entry_scale_min << '\t' << row.entry_scale_max << '\t' <<
      row.exit_scale_min << '\t' << row.exit_scale_max << '\t';
    for (std::size_t index = 0U; index < row.components.size(); ++index) {
      if (index != 0U) {std::cout << ';';}
      std::cout << row.components[index].lower << ':' << row.components[index].upper;
    }
    std::cout << '\t';
    for (std::size_t index = 0U; index < row.center_values.size(); ++index) {
      if (index != 0U) {std::cout << ';';}
      std::cout << row.center_values[index];
    }
    std::cout << '\n';
  }
  for (const auto & row : result.r3_rt_selection.proposed_laterals) {
    std::cout << prefix << "_LATERAL\t" << event.id << '\t' <<
      row.lateral_factor_index << '\t' <<
      (row.go_left ? "LEFT" : "RIGHT") << '\t' << row.d_target << '\t' << row.d_mid << '\t' <<
      row.target_source << '\t' << row.mid_source << '\t' << row.lateral_source_family << '\t' <<
      row.source_priority << '\t' << row.proposal_operator << '\n';
  }
  for (const auto & row : result.r3_rt_selection.proposed_transitions) {
    std::cout << prefix << "_TRANSITION\t" << event.id << '\t' <<
      row.transition_index << '\t' <<
      row.entry_scale << '\t' << row.exit_scale << '\t' << row.transition_family << '\n';
  }
  for (std::size_t index = 0U; index < result.r3_rt_selection.pair_pool.size(); ++index) {
    const auto & row = result.r3_rt_selection.pair_pool[index];
    std::cout << prefix << "_PAIR\t" << event.id << '\t' << index << '\t' <<
      (row.go_left ? "LEFT" : "RIGHT") << '\t' << row.d_target << '\t' << row.d_mid << '\t' <<
      row.entry_scale << '\t' << row.exit_scale << '\t' << row.configuration_key << '\t' <<
      row.proposal_operator << '\t' << row.lateral_source_family << '\t' <<
      row.transition_family << '\t' << row.source_priority << '\t' <<
      row.lateral_factor_index << '\t' << row.transition_index << '\t' <<
      row.construction_guard_proxy << '\t' << row.exit_conflict_proxy << '\t' <<
      row.maximum_corridor_violation_m << '\t' << row.sum_corridor_violation_m << '\t' <<
      row.slope_excess << '\t' << row.curvature_proxy << '\t' << row.center_error << '\t' <<
      row.minimum_clearance_m << '\t' << row.shape_energy << '\t' <<
      row.preconstruction_shape_key << '\t' << row.entry_normalized << '\t' <<
      row.exit_normalized << '\n';
  }
  const auto emit_top = [&](const std::string & stream,
    const std::vector<local_planning::P3R3K12Factor> & factors) {
      for (std::size_t index = 0U; index < factors.size(); ++index) {
        const auto & row = factors[index];
        std::cout << prefix << "_TOP\t" << event.id << '\t' << stream << '\t' <<
          index << '\t' <<
          (row.go_left ? "LEFT" : "RIGHT") << '\t' << row.d_target << '\t' << row.d_mid << '\t' <<
          row.entry_scale << '\t' << row.exit_scale << '\t' << row.configuration_key << '\n';
      }
    };
  emit_top("LEXICOGRAPHIC", result.r3_rt_selection.lexicographic);
  emit_top("COVERAGE", result.r3_rt_selection.coverage);
  for (const auto & row : result.candidates) {
    std::cout << prefix << "_CANDIDATE\t" << event.id << '\t' <<
      (row.go_left ? "LEFT" : "RIGHT") << '\t' << row.d_target << '\t' << row.d_mid << '\t' <<
      row.entry_scale << '\t' << row.exit_scale << '\t' << row.path_digest << '\t' <<
      row.hard_valid << '\t' << usable(row) << '\t' << row.minimum_normalized_safety_slack <<
      '\t' <<
      row.minimum_track_margin_m << '\t' << row.minimum_obstacle_margin_m << '\t' <<
      row.minimum_curvature_margin_radpm << '\t' << row.curvature_rate_margin_radpm2 << '\t' <<
      row.lateral_slope_margin << '\t' << row.ego_braking_distance_deficit_m << '\t' <<
      row.velocity_loss << '\t' << row.exit_reaches_next_obstacle << '\t' <<
      row.global_path_deviation_m << '\t' << row.peak_curvature_radpm << '\t' <<
      row.peak_curvature_rate_radpm2 << '\t' << row.point_count << '\t' <<
      (row.knot_stations.back() - row.knot_stations.front()) << '\n';
  }
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
  if (std::getenv("GQSC_STANDALONE_STUDY") != nullptr) {
    const int warmup = std::getenv("GQSC_STANDALONE_WARMUP") == nullptr ? 0 :
      static_cast<int>(integer(std::getenv("GQSC_STANDALONE_WARMUP")));
    const int repeats = std::getenv("GQSC_STANDALONE_REPEATS") == nullptr ? 1 :
      static_cast<int>(integer(std::getenv("GQSC_STANDALONE_REPEATS")));
    local_planning::P3R3RTStandaloneOptions standalone_options;
    const std::string policy = std::getenv("GQSC_V2_POLICY") == nullptr ?
      "V1_BASELINE" : std::getenv("GQSC_V2_POLICY");
    if (policy == "DISJOINT_COVERAGE") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::DISJOINT_COVERAGE;
    } else if (policy == "SHORT_SHORT_RESERVE") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::SHORT_SHORT_RESERVE;
    } else if (policy == "ZERO_INTERFACE_PROXY_SPLIT") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::ZERO_INTERFACE_PROXY_SPLIT;
    } else if (policy == "ZERO_INTERFACE_EQUAL_MIN_OUTWARD") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::ZERO_INTERFACE_EQUAL_MIN_OUTWARD;
    } else if (policy == "ZERO_INTERFACE_EQUAL_MAX_OUTWARD") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::ZERO_INTERFACE_EQUAL_MAX_OUTWARD;
    } else if (policy == "V3_SIDE_BALANCED_DISJOINT") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::V3_SIDE_BALANCED_DISJOINT;
    } else if (policy == "V3_NORMALIZED_MAXIMIN_DISJOINT") {
      standalone_options.diversity_policy =
        local_planning::P3R3RTStandaloneDiversityPolicy::V3_NORMALIZED_MAXIMIN_DISJOINT;
    } else if (policy != "V1_BASELINE") {
      throw std::runtime_error("unknown GQSC_V2_POLICY: " + policy);
    }
    standalone_options.add_component_half_far002_inward015 =
      std::getenv("GQSC_V2_COMPONENT_OPERATOR") != nullptr;
    constexpr std::size_t kBudget = 128U;
    const auto legacy = planner.evaluateP3LegacyOnlyShadow(event.ego, event.obstacles);
    const auto teacher = planner.evaluateP3R3RTForcedShadow(event.ego, event.obstacles, kBudget);
    const auto standalone = planner.evaluateP3R3RTStandaloneShadow(
      event.ego, event.obstacles, kBudget, standalone_options);
    emitStudyResult(event, "LEGACY_ONLY", legacy);
    emitStudyResult(event, "GQSC_LEGACY_SEEDED_TEACHER_B128", teacher);
    emitStudyResult(event, "GQSC_STANDALONE_DIRECT_SEED_B128", standalone);
    emitGQSCDetail(event, teacher, "TEACHER");
    emitGQSCDetail(event, standalone, "STANDALONE");

    const auto time_selector = [&](const std::string & label, auto evaluate) {
        for (int repeat = -warmup; repeat < repeats; ++repeat) {
          const auto start = Clock::now();
          const auto result = evaluate();
          const double wall_us = std::chrono::duration<double, std::micro>(
            Clock::now() - start).count();
          if (repeat >= 0) {
            std::cout << "STANDALONE_TIMING\t" << event.id << '\t' << label << '\t' <<
              repeat << '\t' << wall_us << '\t' << result.r3_runtime_total_us << '\t' <<
              result.r3_runtime_context_preparation_us << '\t' <<
              result.r3_runtime_geometry_preparation_us << '\t' <<
              result.r3_runtime_transition_generation_us << '\t' <<
              result.r3_runtime_lateral_factor_generation_us << '\t' <<
              result.r3_runtime_pair_priority_computation_us << '\t' <<
              result.r3_runtime_lexicographic_ordering_us << '\t' <<
              result.r3_runtime_coverage_ordering_us << '\t' <<
              result.r3_runtime_shape_deduplication_us << '\t' <<
              result.r3_runtime_reconstruction_us << '\t' <<
              result.r3_runtime_validation_us << '\t' <<
              result.r3_runtime_final_ranking_us << '\t' <<
              result.r3_lateral_factor_count << '\t' << result.r3_pair_priority_count << '\t' <<
              result.r3_constructed_candidate_count << '\t' << result.r3_validator_call_count <<
              '\n';
          }
        }
      };
    time_selector("LEGACY_SEEDED_TEACHER_B128", [&]() {
        return planner.evaluateP3R3RTForcedShadow(event.ego, event.obstacles, kBudget);
    });
    time_selector("STANDALONE_DIRECT_SEED_B128", [&]() {
        return planner.evaluateP3R3RTStandaloneShadow(
          event.ego, event.obstacles, kBudget, standalone_options);
    });
    planner.setActiveResearchCycle(nullptr);
    return;
  }
  if (std::getenv("GQSC_ARCHITECTURE_STUDY") != nullptr) {
    const int warmup = std::getenv("GQSC_STUDY_WARMUP") == nullptr ? 0 :
      static_cast<int>(integer(std::getenv("GQSC_STUDY_WARMUP")));
    const int repeats = std::getenv("GQSC_STUDY_REPEATS") == nullptr ? 1 :
      static_cast<int>(integer(std::getenv("GQSC_STUDY_REPEATS")));
    constexpr std::size_t kBudget = 128U;

    const auto strict = planner.evaluateP3LegacyPassShadow(event.ego, event.obstacles, false,
        false);
    emitStudyResult(event, "LEGACY_STRICT", strict);
    if (!strict.would_recover && strict.invoked && !strict.cluster_obstacle_ids.empty()) {
      const auto relaxed = planner.evaluateP3LegacyPassShadow(
        event.ego, event.obstacles, true, false);
      emitStudyResult(event, "LEGACY_RELAXED", relaxed);
    }
    const auto m0_v1 = planner.evaluateP3LegacyPassShadow(
      event.ego, event.obstacles, false, true);
    emitStudyResult(event, "M0_V1_ONLY", m0_v1);
    const auto legacy = planner.evaluateP3LegacyOnlyShadow(event.ego, event.obstacles);
    const auto sequential = planner.evaluateP3R3RTShadow(event.ego, event.obstacles, kBudget);
    const auto forced = planner.evaluateP3R3RTForcedShadow(event.ego, event.obstacles, kBudget);
    const auto prefix = planner.evaluateP3R3RTM0V1PrefixShadow(
      event.ego, event.obstacles, kBudget);
    emitStudyResult(event, "ARCH_A_LEGACY_ONLY", legacy);
    emitStudyResult(event, "ARCH_B_LEGACY_THEN_GQSC_B128", sequential);
    emitStudyResult(event, "ARCH_C_GQSC_FORCED_WITH_LEGACY_SEEDS", forced);
    emitStudyResult(event, "ARCH_D_M0V1_THEN_GQSC_B128", prefix);
    emitGQSCDetail(event, forced);

    const auto time_architecture = [&](const std::string & architecture, auto evaluate) {
        for (int repeat = -warmup; repeat < repeats; ++repeat) {
          const auto start = Clock::now();
          const auto result = evaluate();
          const double wall_us = std::chrono::duration<double, std::micro>(
            Clock::now() - start).count();
          if (repeat >= 0) {
            std::cout << "ARCH_TIMING\t" << event.id << '\t' << architecture << '\t' << repeat <<
              '\t' << wall_us << '\t' << result.runtime_total_us << '\t' <<
              result.r3_runtime_total_us << '\t' << result.runtime_context_preparation_us << '\t' <<
              result.runtime_m0_v1_us << '\t' << result.runtime_m0_v2_us << '\t' <<
              result.runtime_m1_us << '\t' << result.runtime_final_bookkeeping_us << '\n';
          }
        }
      };
    time_architecture("LEGACY_STRICT_PASS", [&]() {
        return planner.evaluateP3LegacyPassShadow(event.ego, event.obstacles, false, false);
    });
    if (!strict.would_recover && strict.invoked && !strict.cluster_obstacle_ids.empty()) {
      time_architecture("LEGACY_RELAXED_PASS", [&]() {
          return planner.evaluateP3LegacyPassShadow(event.ego, event.obstacles, true, false);
      });
    }
    time_architecture("M0_V1_ONLY_PASS", [&]() {
        return planner.evaluateP3LegacyPassShadow(event.ego, event.obstacles, false, true);
    });
    time_architecture("A_LEGACY_ONLY", [&]() {
        return planner.evaluateP3LegacyOnlyShadow(event.ego, event.obstacles);
    });
    time_architecture("B_LEGACY_THEN_GQSC_B128", [&]() {
        return planner.evaluateP3R3RTShadow(event.ego, event.obstacles, kBudget);
    });
    time_architecture("C_GQSC_FORCED_WITH_LEGACY_SEEDS", [&]() {
        return planner.evaluateP3R3RTForcedShadow(event.ego, event.obstacles, kBudget);
    });
    time_architecture("D_M0V1_THEN_GQSC_B128", [&]() {
        return planner.evaluateP3R3RTM0V1PrefixShadow(event.ego, event.obstacles, kBudget);
    });
    planner.setActiveResearchCycle(nullptr);
    return;
  }
  const char * native_budgets = std::getenv("R3_RT_NATIVE_BUDGETS");
  if (native_budgets != nullptr) {
    const int warmup = std::getenv("R3_RT_WARMUP") == nullptr ? 0 :
      static_cast<int>(integer(std::getenv("R3_RT_WARMUP")));
    const int repeats = std::getenv("R3_RT_REPEATS") == nullptr ? 1 :
      static_cast<int>(integer(std::getenv("R3_RT_REPEATS")));
    const bool detail = std::getenv("R3_RT_PARITY_DETAIL") != nullptr;
    std::stringstream budgets(native_budgets);
    std::string token;
    while (std::getline(budgets, token, ',')) {
      const std::size_t budget = static_cast<std::size_t>(integer(token));
      local_planning::P3ShadowResult result;
      for (int repeat = -warmup; repeat < repeats; ++repeat) {
        const auto start = Clock::now();
        result = planner.evaluateP3R3RTShadow(event.ego, event.obstacles, budget);
        const double wall_us = std::chrono::duration<double, std::micro>(
          Clock::now() - start).count();
        if (repeat >= 0) {
          std::cout << "NATIVE_TIMING\t" << event.id << '\t' << budget << '\t' << repeat << '\t' <<
            wall_us << '\t' << result.r3_runtime_context_preparation_us << '\t' <<
            result.r3_runtime_geometry_preparation_us << '\t' <<
            result.r3_runtime_transition_generation_us << '\t' <<
            result.r3_runtime_lateral_factor_generation_us << '\t' <<
            result.r3_runtime_pair_priority_computation_us << '\t' <<
            result.r3_runtime_lexicographic_ordering_us << '\t' <<
            result.r3_runtime_coverage_ordering_us << '\t' <<
            result.r3_runtime_shape_deduplication_us << '\t' <<
            result.r3_runtime_reconstruction_us << '\t' << result.r3_runtime_validation_us <<
            '\t' <<
            result.r3_runtime_final_ranking_us << '\t' << result.r3_runtime_total_us << '\n';
        }
      }
      std::cout << "NATIVE_SUMMARY\t" << event.id << '\t' << budget << '\t' <<
        result.r3_invoked << '\t' << result.would_recover << '\t' <<
        result.r3_rt_selection.raw_side_lateral_count << '\t' <<
        result.r3_rt_selection.proposed_laterals.size() << '\t' <<
        result.r3_rt_selection.proposed_transitions.size() << '\t' <<
        result.r3_rt_selection.pair_pool.size() << '\t' <<
        result.r3_constructed_candidate_count << '\t' << result.r3_validator_call_count << '\t' <<
        result.r3_hard_valid_count << '\t' << result.r3_usable_valid_count << '\t' <<
        result.selected_d_target << '\t' << result.selected_d_mid << '\t' <<
        result.selected_path_digest << '\t' << result.failure_classification << '\n';
      if (!detail) {
        continue;
      }
      for (const auto & row : result.r3_rt_selection.proposed_laterals) {
        std::cout << "NATIVE_LATERAL\t" << event.id << '\t' << budget << '\t' <<
          row.lateral_factor_index << '\t' << (row.go_left ? "LEFT" : "RIGHT") << '\t' <<
          row.d_target << '\t' << row.d_mid << '\t' << row.target_source << '\t' <<
          row.mid_source << '\t' << row.lateral_source_family << '\t' << row.source_priority <<
          '\t' <<
          row.proposal_operator << '\n';
      }
      for (const auto & row : result.r3_rt_selection.proposed_transitions) {
        std::cout << "NATIVE_TRANSITION\t" << event.id << '\t' << budget << '\t' <<
          row.transition_index << '\t' << row.entry_scale << '\t' << row.exit_scale << '\t' <<
          row.transition_family << '\n';
      }
      for (std::size_t index = 0U; index < result.r3_rt_selection.pair_pool.size(); ++index) {
        const auto & row = result.r3_rt_selection.pair_pool[index];
        std::cout << "NATIVE_PAIR\t" << event.id << '\t' << budget << '\t' << index << '\t' <<
          row.configuration_key << '\t' << row.preconstruction_shape_key << '\t' <<
          row.transition_family << '\t' << row.construction_guard_proxy << '\t' <<
          row.exit_conflict_proxy << '\t' <<
          row.maximum_corridor_violation_m << '\t' << row.sum_corridor_violation_m << '\t' <<
          row.slope_excess << '\t' << row.curvature_proxy << '\t' << row.center_error << '\t' <<
          row.minimum_clearance_m << '\t' << row.shape_energy << '\t' <<
          row.entry_normalized << '\t' << row.exit_normalized << '\n';
      }
      const auto emit_top = [&](const char * stream,
        const std::vector<local_planning::P3R3K12Factor> & factors) {
          for (std::size_t index = 0U; index < factors.size(); ++index) {
            const auto & row = factors[index];
            std::cout << "NATIVE_TOP_FACTOR\t" << event.id << '\t' << budget << '\t' <<
              stream << '\t' << index << '\t' << (row.go_left ? "LEFT" : "RIGHT") << '\t' <<
              row.d_target << '\t' << row.d_mid << '\t' << row.entry_scale << '\t' <<
              row.exit_scale << '\t' << row.configuration_key << '\n';
          }
        };
      emit_top("LEXICOGRAPHIC", result.r3_rt_selection.lexicographic);
      emit_top("COVERAGE", result.r3_rt_selection.coverage);
      for (const auto & row : result.r3_selected_factors) {
        std::cout << "NATIVE_SELECTED\t" << event.id << '\t' << budget << '\t' <<
          row.rank_stream << '\t' << row.stream_rank << '\t' <<
          (row.go_left ? "LEFT" : "RIGHT") << '\t' << row.d_target << '\t' << row.d_mid << '\t' <<
          row.entry_scale << '\t' << row.exit_scale << '\t' << row.path_digest << '\t' <<
          row.validator_executed << '\t' << row.hard_valid << '\t' << row.usable_valid << '\n';
      }
    }
    planner.setActiveResearchCycle(nullptr);
    return;
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
