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

#include "local_planning/research_instrumentation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace local_planning
{
namespace
{

std::string jsonEscape(const std::string & input)
{
  std::ostringstream output;
  for (const char character : input) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(static_cast<unsigned char>(character)) << std::dec;
        } else {
          output << character;
        }
        break;
    }
  }
  return output.str();
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

std::string jsonStrings(const std::vector<std::string> & values)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    output << '"' << jsonEscape(values[index]) << '"';
  }
  output << ']';
  return output.str();
}

std::string obstacleSnapshotsJson(
  const std::vector<P3ResearchObstacleSnapshotRecord> & obstacles)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0U; index < obstacles.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    const auto & obstacle = obstacles[index];
    output << "{\"id\":" << obstacle.id
           << ",\"s_start\":" << jsonNumber(obstacle.s_start)
           << ",\"s_end\":" << jsonNumber(obstacle.s_end)
           << ",\"s_center\":" << jsonNumber(obstacle.s_center)
           << ",\"d_right\":" << jsonNumber(obstacle.d_right)
           << ",\"d_left\":" << jsonNumber(obstacle.d_left)
           << ",\"d_center\":" << jsonNumber(obstacle.d_center)
           << ",\"is_static\":" << (obstacle.is_static ? "true" : "false")
           << ",\"is_visible\":" << (obstacle.is_visible ? "true" : "false") << '}';
  }
  output << ']';
  return output.str();
}

const char * validationFailureName(int value)
{
  switch (value) {
    case 0: return "NONE";
    case 1: return "INPUT";
    case 2: return "NO_FORWARD_PATH";
    case 3: return "TRACK_BOUNDARY";
    case 4: return "OBSTACLE_COLLISION";
    case 5: return "GEOMETRY";
    default: return "UNKNOWN";
  }
}

std::string r3SelectedFactorsJson(const std::vector<P3R3SelectedFactorTrace> & factors)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0U; index < factors.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    const auto & factor = factors[index];
    output << "{\"rank_stream\":\"" << jsonEscape(factor.rank_stream) << "\""
           << ",\"stream_rank\":" << factor.stream_rank
           << ",\"side\":\"" << (factor.go_left ? "LEFT" : "RIGHT") << "\""
           << ",\"d_target\":" << jsonNumber(factor.d_target)
           << ",\"d_mid\":" << jsonNumber(factor.d_mid)
           << ",\"entry_scale\":" << jsonNumber(factor.entry_scale)
           << ",\"exit_scale\":" << jsonNumber(factor.exit_scale)
           << ",\"target_source\":\"" << jsonEscape(factor.target_source) << "\""
           << ",\"mid_source\":\"" << jsonEscape(factor.mid_source) << "\""
           << ",\"lateral_factor_index\":" << factor.lateral_factor_index
           << ",\"transition_index\":" << factor.transition_index
           << ",\"exit_conflict_proxy\":"
           << (factor.exit_conflict_proxy ? "true" : "false")
           << ",\"maximum_corridor_violation_m\":"
           << jsonNumber(factor.maximum_corridor_violation_m)
           << ",\"sum_corridor_violation_m\":"
           << jsonNumber(factor.sum_corridor_violation_m)
           << ",\"slope_excess\":" << jsonNumber(factor.slope_excess)
           << ",\"curvature_proxy\":" << jsonNumber(factor.curvature_proxy)
           << ",\"center_error\":" << jsonNumber(factor.center_error)
           << ",\"minimum_clearance_m\":" << jsonNumber(factor.minimum_clearance_m)
           << ",\"shape_energy\":" << jsonNumber(factor.shape_energy)
           << ",\"constructed\":" << (factor.constructed ? "true" : "false")
           << ",\"path_digest_duplicate\":"
           << (factor.path_digest_duplicate ? "true" : "false")
           << ",\"validator_executed\":"
           << (factor.validator_executed ? "true" : "false")
           << ",\"hard_valid\":" << (factor.hard_valid ? "true" : "false")
           << ",\"usable_valid\":" << (factor.usable_valid ? "true" : "false")
           << ",\"path_digest\":\"" << jsonEscape(factor.path_digest) << "\""
           << ",\"candidate_identity\":\""
           << jsonEscape(factor.candidate_identity) << "\"}";
  }
  output << ']';
  return output.str();
}

std::string planningJson(
  const PlanningResearchCycle & cycle, const PlanningResearchConfig & config,
  std::uint64_t dropped_log_count)
{
  std::ostringstream output;
  output << std::setprecision(17)
         << "{\"schema_version\":\"" << kPlanningResearchSchemaVersion << "\""
         << ",\"event_type\":\"PLANNING_EVENT\""
         << ",\"run_id\":\"" << jsonEscape(cycle.run_id) << "\""
         << ",\"scenario_id\":\"" << jsonEscape(cycle.scenario_id) << "\""
         << ",\"callback_sequence\":" << cycle.callback_sequence
         << ",\"ros_time_ns\":" << cycle.ros_time_ns
         << ",\"obstacle_source_stamp_ns\":" << cycle.obstacle_source_stamp_ns
         << ",\"obstacle_sequence\":" << cycle.obstacle_sequence
         << ",\"source_epoch\":" << cycle.source_epoch
         << ",\"reference_generation\":" << cycle.reference_generation
         << ",\"git_commit\":\"" << jsonEscape(config.git_commit) << "\""
         << ",\"source_sha256\":\"" << jsonEscape(config.source_sha256) << "\""
         << ",\"config_sha256\":\"" << jsonEscape(config.config_sha256) << "\""
         << ",\"effective_parameter_snapshot_id\":\""
         << jsonEscape(config.effective_parameter_snapshot_id) << "\""
         << ",\"ego\":{\"x\":" << jsonNumber(cycle.ego_x)
         << ",\"y\":" << jsonNumber(cycle.ego_y)
         << ",\"yaw\":" << jsonNumber(cycle.ego_yaw)
         << ",\"pose_source\":\"" << jsonEscape(cycle.ego_pose_source) << "\""
         << ",\"s\":" << jsonNumber(cycle.ego_s)
         << ",\"d\":" << jsonNumber(cycle.ego_d)
         << ",\"speed_mps\":" << jsonNumber(cycle.measured_speed_mps) << '}'
         << ",\"p3_mode\":\"" << jsonEscape(cycle.p3_mode) << "\""
         << ",\"lifecycle_state\":\"" << jsonEscape(cycle.lifecycle_state) << "\""
         << ",\"lifecycle_owner\":\"" << jsonEscape(cycle.lifecycle_owner) << "\""
         << ",\"continuation_result\":\""
         << jsonEscape(cycle.continuation_result) << "\""
         << ",\"invalidation_reason\":\""
         << jsonEscape(cycle.invalidation_reason) << "\""
         << ",\"selected_side\":\"" << jsonEscape(cycle.selected_side) << "\""
         << ",\"selected_candidate_identity\":\""
         << jsonEscape(cycle.selected_candidate_identity) << "\""
         << ",\"selected_path_digest\":\""
         << jsonEscape(cycle.selected_path_digest) << "\""
         << ",\"fallback_reason\":\"" << jsonEscape(cycle.fallback_reason) << "\""
         << ",\"active\":" << (cycle.active ? "true" : "false")
         << ",\"continued\":" << (cycle.continued ? "true" : "false")
         << ",\"fresh\":" << (cycle.fresh ? "true" : "false")
         << ",\"backup\":" << (cycle.backup ? "true" : "false")
         << ",\"safe_stop\":" << (cycle.safe_stop ? "true" : "false")
         << ",\"global_handoff\":" << (cycle.global_handoff ? "true" : "false")
         << ",\"counters\":{\"strict_attempted\":" << cycle.strict_attempted
         << ",\"relaxed_attempted\":" << cycle.relaxed_attempted
         << ",\"m0_v1_constructed_right\":" << cycle.m0_v1_constructed_right
         << ",\"m0_v1_constructed_left\":" << cycle.m0_v1_constructed_left
         << ",\"discarded_side_candidate_count\":"
         << cycle.discarded_side_candidate_count
         << ",\"m0_v2_constructed\":" << cycle.m0_v2_constructed
         << ",\"m1_constructed\":" << cycle.m1_constructed
         << ",\"constructed_total_actual\":" << cycle.constructed_total_actual
         << ",\"validate_candidate_executed_total_actual\":"
         << cycle.validate_candidate_executed_total_actual
         << ",\"hard_valid_total_actual\":" << cycle.hard_valid_total_actual
         << ",\"returned_candidate_count\":" << cycle.returned_candidate_count
         << ",\"lifecycle_revalidation_count\":" << cycle.lifecycle_revalidation_count
         << ",\"safe_stop_escape_evaluator_count\":"
         << cycle.safe_stop_escape_evaluator_count
         << ",\"r3_invocation_count\":" << cycle.r3_invocation_count
         << ",\"r3_constructed_candidate_count\":"
         << cycle.r3_constructed_candidate_count
         << ",\"r3_path_digest_duplicate_count\":"
         << cycle.r3_path_digest_duplicate_count
         << ",\"r3_validator_call_count\":" << cycle.r3_validator_call_count
         << ",\"r3_hard_valid_count\":" << cycle.r3_hard_valid_count
         << ",\"r3_usable_valid_count\":" << cycle.r3_usable_valid_count << '}'
         << ",\"runtime_us\":{\"callback_total\":"
         << jsonNumber(cycle.runtime_callback_total_us)
         << ",\"planning_total\":" << jsonNumber(cycle.runtime_planning_total_us)
         << ",\"corridor_extraction\":" << jsonNumber(cycle.runtime_corridor_us)
         << ",\"probe_anchor_selection\":"
         << jsonNumber(cycle.runtime_probe_anchor_us)
         << ",\"analytic_root_solve\":" << jsonNumber(cycle.runtime_root_solve_us)
         << ",\"spline_reconstruction\":"
         << jsonNumber(cycle.runtime_spline_reconstruction_us)
         << ",\"geometry_recomputation\":"
         << jsonNumber(cycle.runtime_geometry_recompute_us)
         << ",\"velocity_shaping\":" << jsonNumber(cycle.runtime_velocity_shaping_us)
         << ",\"candidate_measurement\":"
         << jsonNumber(cycle.runtime_candidate_measurement_us)
         << ",\"hard_validation\":" << jsonNumber(cycle.runtime_hard_validation_us)
         << ",\"ranking\":" << jsonNumber(cycle.runtime_ranking_us)
         << ",\"lifecycle_revalidation\":"
         << jsonNumber(cycle.runtime_lifecycle_revalidation_us)
         << ",\"r3_total\":" << jsonNumber(cycle.runtime_r3_total_us)
         << ",\"research_lineage\":" << jsonNumber(cycle.runtime_research_lineage_us)
         << ",\"research_capture\":" << jsonNumber(cycle.runtime_research_capture_us) << '}'
         << ",\"evaluation_count\":" << cycle.evaluations.size()
         << ",\"dropped_log_count\":" << dropped_log_count << '}';
  return output.str();
}

std::string candidateJson(
  const PlanningResearchCycle & cycle,
  const P3ResearchEvaluationRecord & evaluation,
  const P3ResearchCandidateRecord & candidate)
{
  std::ostringstream output;
  output << std::setprecision(17)
         << "{\"schema_version\":\"" << kPlanningResearchSchemaVersion << "\""
         << ",\"event_type\":\"CANDIDATE_EVENT\""
         << ",\"run_id\":\"" << jsonEscape(cycle.run_id) << "\""
         << ",\"scenario_id\":\"" << jsonEscape(cycle.scenario_id) << "\""
         << ",\"callback_sequence\":" << cycle.callback_sequence
         << ",\"evaluation_sequence\":" << evaluation.lineage.evaluation_sequence
         << ",\"evaluation_role\":\""
         << jsonEscape(evaluation.lineage.evaluation_role) << "\""
         << ",\"input_snapshot_id\":\""
         << jsonEscape(evaluation.lineage.input_snapshot_id) << "\""
         << ",\"ego_snapshot_id\":\""
         << jsonEscape(evaluation.lineage.ego_snapshot_id) << "\""
         << ",\"obstacle_snapshot_id\":\""
         << jsonEscape(evaluation.lineage.obstacle_snapshot_id) << "\""
         << ",\"reference_snapshot_id\":\""
         << jsonEscape(evaluation.lineage.reference_snapshot_id) << "\""
         << ",\"source_stamp_ns\":" << evaluation.lineage.source_stamp_ns
         << ",\"obstacle_sequence\":" << evaluation.lineage.obstacle_sequence
         << ",\"source_epoch\":" << evaluation.lineage.source_epoch
         << ",\"reference_generation\":" << evaluation.lineage.reference_generation
         << ",\"clearance_pass\":\"" << jsonEscape(candidate.clearance_pass) << "\""
         << ",\"generator_stage\":\"" << jsonEscape(candidate.generator_stage) << "\""
         << ",\"template\":\"" << jsonEscape(candidate.candidate_template) << "\""
         << ",\"side\":\"" << jsonEscape(candidate.side) << "\""
         << ",\"generation_order\":" << candidate.generation_order
         << ",\"candidate_identity\":\""
         << jsonEscape(candidate.candidate_identity) << "\""
         << ",\"logical_identity\":\"" << jsonEscape(candidate.logical_identity) << "\""
         << ",\"path_digest\":\"" << jsonEscape(candidate.path_digest) << "\""
         << ",\"component\":\"" << jsonEscape(candidate.component_id) << "\""
         << ",\"mapping_source\":\"" << jsonEscape(candidate.mapping_source) << "\""
         << ",\"source_cell\":\"" << jsonEscape(candidate.source_cell) << "\""
         << ",\"analytic_branch_regime\":\""
         << jsonEscape(candidate.analytic_branch_regime) << "\""
         << ",\"r3\":{\"rank_stream\":\""
         << jsonEscape(candidate.r3_rank_stream) << "\""
         << ",\"target_source\":\"" << jsonEscape(candidate.r3_target_source) << "\""
         << ",\"mid_source\":\"" << jsonEscape(candidate.r3_mid_source) << "\""
         << ",\"lateral_factor_index\":" << candidate.r3_lateral_factor_index
         << ",\"transition_index\":" << candidate.r3_transition_index << '}'
         << ",\"returned_by_policy\":"
         << (candidate.returned_by_policy ? "true" : "false")
         << ",\"discarded_side\":" << (candidate.discarded_side ? "true" : "false")
         << ",\"d_target\":" << jsonNumber(candidate.d_target)
         << ",\"s_probe\":" << jsonNumber(candidate.s_probe)
         << ",\"d_probe\":" << jsonNumber(candidate.d_probe)
         << ",\"d_mid\":" << jsonNumber(candidate.d_mid)
         << ",\"root_index\":" << candidate.root_index
         << ",\"root_type\":\"" << jsonEscape(candidate.root_type) << "\""
         << ",\"probe_location_rule\":\""
         << jsonEscape(candidate.probe_location_rule) << "\""
         << ",\"probe_anchor_rule\":\""
         << jsonEscape(candidate.probe_anchor_rule) << "\""
         << ",\"entry_scale\":" << jsonNumber(candidate.entry_scale)
         << ",\"exit_scale\":" << jsonNumber(candidate.exit_scale)
         << ",\"z0\":" << jsonNumber(candidate.knots[0])
         << ",\"z1\":" << jsonNumber(candidate.knots[1])
         << ",\"z2\":" << jsonNumber(candidate.knots[2])
         << ",\"z3\":" << jsonNumber(candidate.knots[3])
         << ",\"z4\":" << jsonNumber(candidate.knots[4])
         << ",\"point_count\":" << candidate.point_count
         << ",\"waypoint0\":{\"s_m\":" << jsonNumber(candidate.waypoint0_s_m)
         << ",\"d_m\":" << jsonNumber(candidate.waypoint0_d_m)
         << ",\"x_m\":" << jsonNumber(candidate.waypoint0_x_m)
         << ",\"y_m\":" << jsonNumber(candidate.waypoint0_y_m)
         << ",\"yaw_rad\":" << jsonNumber(candidate.waypoint0_yaw_rad)
         << ",\"center_track_margin_m\":"
         << jsonNumber(candidate.waypoint0_center_track_margin_m)
         << ",\"footprint_track_margin_m\":"
         << jsonNumber(candidate.waypoint0_footprint_track_margin_m)
         << ",\"footprint_invalid\":"
         << (candidate.waypoint0_footprint_invalid ? "true" : "false") << '}'
         << ",\"validator_executed\":"
         << (candidate.validator_executed ? "true" : "false")
         << ",\"validation_authority\":\""
         << jsonEscape(candidate.validation_authority) << "\""
         << ",\"hard_valid\":" << (candidate.hard_valid ? "true" : "false")
         << ",\"usable_valid\":" << (candidate.usable_valid ? "true" : "false")
         << ",\"first_failure_enum\":" << candidate.first_failure_enum
         << ",\"first_failure_name\":\""
         << validationFailureName(candidate.first_failure_enum) << "\""
         << ",\"first_failure_reason\":\""
         << jsonEscape(candidate.first_failure_reason) << "\""
         << ",\"failure_waypoint_index\":" << candidate.failure_waypoint_index
         << ",\"failure_obstacle_id\":" << candidate.failure_obstacle_id
         << ",\"all_observed_violation_flags\":"
         << jsonStrings(candidate.all_observed_violation_flags)
         << ",\"margins\":{\"center_track_m\":"
         << jsonNumber(candidate.center_track_margin_m)
         << ",\"footprint_track_m\":" << jsonNumber(candidate.footprint_track_margin_m)
         << ",\"obstacle_m\":" << jsonNumber(candidate.obstacle_margin_m)
         << ",\"peak_lateral_slope\":" << jsonNumber(candidate.peak_lateral_slope)
         << ",\"lateral_slope_margin\":" << jsonNumber(candidate.lateral_slope_margin)
         << ",\"peak_positive_curvature_radpm\":"
         << jsonNumber(candidate.peak_positive_curvature_radpm)
         << ",\"peak_negative_curvature_radpm\":"
         << jsonNumber(candidate.peak_negative_curvature_radpm)
         << ",\"signed_curvature_margin_radpm\":"
         << jsonNumber(candidate.signed_curvature_margin_radpm)
         << ",\"peak_curvature_rate_radpm2\":"
         << jsonNumber(candidate.peak_curvature_rate_radpm2)
         << ",\"curvature_rate_margin_radpm2\":"
         << jsonNumber(candidate.curvature_rate_margin_radpm2) << '}'
         << ",\"rank_tuple\":{\"exit_reaches_next_obstacle\":"
         << (candidate.exit_reaches_next_obstacle ? "true" : "false")
         << ",\"braking_feasible\":"
         << (candidate.braking_feasible ? "true" : "false")
         << ",\"braking_deficit_m\":" << jsonNumber(candidate.braking_deficit_m)
         << ",\"velocity_loss\":" << jsonNumber(candidate.velocity_loss)
         << ",\"minimum_normalized_safety_slack\":"
         << jsonNumber(candidate.minimum_normalized_safety_slack)
         << ",\"global_path_deviation_m\":"
         << jsonNumber(candidate.global_path_deviation_m)
         << ",\"generation_index\":" << candidate.generation_order << '}'
         << ",\"final_rank\":" << candidate.final_rank
         << ",\"selected\":" << (candidate.selected ? "true" : "false")
         << ",\"runtime_us\":{\"probe_anchor\":"
         << jsonNumber(candidate.runtime_probe_anchor_us)
         << ",\"root_solve\":" << jsonNumber(candidate.runtime_root_solve_us)
         << ",\"spline_reconstruction\":"
         << jsonNumber(candidate.runtime_spline_reconstruction_us)
         << ",\"geometry_recompute\":"
         << jsonNumber(candidate.runtime_geometry_recompute_us)
         << ",\"velocity_shaping\":" << jsonNumber(candidate.runtime_velocity_shaping_us)
         << ",\"candidate_measurement\":"
         << jsonNumber(candidate.runtime_candidate_measurement_us)
         << ",\"hard_validation\":"
         << jsonNumber(candidate.runtime_hard_validation_us) << '}'
         << ",\"evaluation_selected\":" << (evaluation.selected ? "true" : "false")
         << '}';
  return output.str();
}

std::string evaluationJson(
  const PlanningResearchCycle & cycle,
  const P3ResearchEvaluationRecord & evaluation)
{
  std::vector<std::string> candidate_digests;
  std::vector<std::string> stages;
  std::vector<std::string> templates;
  candidate_digests.reserve(evaluation.candidates.size());
  for (const auto & candidate : evaluation.candidates) {
    candidate_digests.push_back(candidate.path_digest);
    if (std::find(stages.begin(), stages.end(), candidate.generator_stage) == stages.end()) {
      stages.push_back(candidate.generator_stage);
    }
    if (std::find(
        templates.begin(), templates.end(), candidate.candidate_template) == templates.end())
    {
      templates.push_back(candidate.candidate_template);
    }
  }

  const auto & lineage = evaluation.lineage;
  std::ostringstream output;
  output << std::setprecision(17)
         << "{\"schema_version\":\"" << kPlanningResearchSchemaVersion << "\""
         << ",\"event_type\":\"EVALUATION_EVENT\""
         << ",\"run_id\":\"" << jsonEscape(cycle.run_id) << "\""
         << ",\"scenario_id\":\"" << jsonEscape(cycle.scenario_id) << "\""
         << ",\"callback_sequence\":" << cycle.callback_sequence
         << ",\"evaluation_sequence\":" << lineage.evaluation_sequence
         << ",\"evaluation_role\":\"" << jsonEscape(lineage.evaluation_role) << "\""
         << ",\"clearance_pass\":\"" << jsonEscape(evaluation.clearance_pass) << "\""
         << ",\"input_snapshot_id\":\"" << jsonEscape(lineage.input_snapshot_id) << "\""
         << ",\"ego_snapshot_id\":\"" << jsonEscape(lineage.ego_snapshot_id) << "\""
         << ",\"obstacle_snapshot_id\":\""
         << jsonEscape(lineage.obstacle_snapshot_id) << "\""
         << ",\"reference_snapshot_id\":\""
         << jsonEscape(lineage.reference_snapshot_id) << "\""
         << ",\"source_stamp_ns\":" << lineage.source_stamp_ns
         << ",\"obstacle_sequence\":" << lineage.obstacle_sequence
         << ",\"source_epoch\":" << lineage.source_epoch
         << ",\"reference_generation\":" << lineage.reference_generation
         << ",\"ego\":{\"s\":" << jsonNumber(lineage.ego_s)
         << ",\"d\":" << jsonNumber(lineage.ego_d)
         << ",\"speed_mps\":" << jsonNumber(lineage.ego_speed_mps) << '}'
         << ",\"obstacles\":" << obstacleSnapshotsJson(lineage.obstacles)
         << ",\"invoked\":" << (evaluation.invoked ? "true" : "false")
         << ",\"selected\":" << (evaluation.selected ? "true" : "false")
         << ",\"failure_classification\":\""
         << jsonEscape(evaluation.failure_classification) << "\""
         << ",\"constructed_total_actual\":" << evaluation.constructed_total_actual
         << ",\"validate_candidate_executed_total_actual\":"
         << evaluation.validate_candidate_executed_total_actual
         << ",\"hard_valid_total_actual\":" << evaluation.hard_valid_total_actual
         << ",\"returned_candidate_count\":" << evaluation.returned_candidate_count
         << ",\"r3\":{\"invoked\":" << (evaluation.r3_invoked ? "true" : "false")
         << ",\"method_name\":\"" << jsonEscape(evaluation.r3_method_name) << "\""
         << ",\"method_sha256\":\"" << jsonEscape(evaluation.r3_method_sha256) << "\""
         << ",\"lateral_factor_count\":" << evaluation.r3_lateral_factor_count
         << ",\"pair_priority_count\":" << evaluation.r3_pair_priority_count
         << ",\"lexicographic_factor_count\":"
         << evaluation.r3_lexicographic_factor_count
         << ",\"coverage_factor_count\":" << evaluation.r3_coverage_factor_count
         << ",\"constructed_candidate_count\":"
         << evaluation.r3_constructed_candidate_count
         << ",\"path_digest_duplicate_count\":"
         << evaluation.r3_path_digest_duplicate_count
         << ",\"validator_call_count\":" << evaluation.r3_validator_call_count
         << ",\"hard_valid_count\":" << evaluation.r3_hard_valid_count
         << ",\"usable_valid_count\":" << evaluation.r3_usable_valid_count
         << ",\"transition_count\":" << evaluation.r3_transition_count
         << ",\"raw_combination_count\":" << evaluation.r3_raw_combination_count
         << ",\"production_excluded_count\":" << evaluation.r3_production_excluded_count
         << ",\"factor_pool_count\":" << evaluation.r3_factor_pool_count
         << ",\"unique_profile_count\":" << evaluation.r3_unique_profile_count
         << ",\"proxy_metric_evaluation_count\":"
         << evaluation.r3_proxy_metric_evaluation_count
         << ",\"proxy_metric_cache_hit_count\":"
         << evaluation.r3_proxy_metric_cache_hit_count
         << ",\"corridor_sample_evaluation_count\":"
         << evaluation.r3_corridor_sample_evaluation_count
         << ",\"reference_spacing_scan_count\":"
         << evaluation.r3_reference_spacing_scan_count
         << ",\"reference_sample_count\":" << evaluation.r3_reference_sample_count
         << ",\"profile_basis_build_count\":" << evaluation.r3_profile_basis_build_count
         << ",\"profile_basis_cache_hit_count\":"
         << evaluation.r3_profile_basis_cache_hit_count
         << ",\"profile_sample_basis_count\":"
         << evaluation.r3_profile_sample_basis_count
         << ",\"pair_metric_worker_count\":" << evaluation.r3_pair_metric_worker_count
         << ",\"fallback_after_failure\":\""
         << jsonEscape(evaluation.r3_fallback_after_failure) << "\""
         << ",\"runtime_context_preparation_us\":"
         << jsonNumber(evaluation.r3_runtime_context_preparation_us)
         << ",\"runtime_geometry_preparation_us\":"
         << jsonNumber(evaluation.r3_runtime_geometry_preparation_us)
         << ",\"runtime_transition_generation_us\":"
         << jsonNumber(evaluation.r3_runtime_transition_generation_us)
         << ",\"runtime_lateral_factor_generation_us\":"
         << jsonNumber(evaluation.r3_runtime_lateral_factor_generation_us)
         << ",\"runtime_pair_priority_computation_us\":"
         << jsonNumber(evaluation.r3_runtime_pair_priority_computation_us)
         << ",\"runtime_lexicographic_ordering_us\":"
         << jsonNumber(evaluation.r3_runtime_lexicographic_ordering_us)
         << ",\"runtime_coverage_ordering_us\":"
         << jsonNumber(evaluation.r3_runtime_coverage_ordering_us)
         << ",\"runtime_shape_deduplication_us\":"
         << jsonNumber(evaluation.r3_runtime_shape_deduplication_us)
         << ",\"runtime_candidate_deduplication_us\":"
         << jsonNumber(evaluation.r3_runtime_candidate_deduplication_us)
         << ",\"runtime_factor_generation_us\":"
         << jsonNumber(evaluation.r3_runtime_factor_generation_us)
         << ",\"runtime_reconstruction_us\":"
         << jsonNumber(evaluation.r3_runtime_reconstruction_us)
         << ",\"runtime_validation_us\":"
         << jsonNumber(evaluation.r3_runtime_validation_us)
         << ",\"runtime_final_ranking_us\":"
         << jsonNumber(evaluation.r3_runtime_final_ranking_us)
         << ",\"runtime_total_us\":" << jsonNumber(evaluation.r3_runtime_total_us)
         << ",\"selected_factors\":"
         << r3SelectedFactorsJson(evaluation.r3_selected_factors) << '}'
         << ",\"candidate_path_digests\":" << jsonStrings(candidate_digests)
         << ",\"generator_stages\":" << jsonStrings(stages)
         << ",\"templates\":" << jsonStrings(templates)
         << ",\"runtime_total_us\":" << jsonNumber(evaluation.runtime_total_us) << '}';
  return output.str();
}

}  // namespace

class PlanningResearchLogger::Impl
{
public:
  explicit Impl(PlanningResearchConfig config)
  : config_(std::move(config))
  {
    if (config_.output_root.empty() || config_.run_id.empty()) {
      throw std::invalid_argument("research output_root and run_id must be non-empty");
    }
    if (config_.git_commit.empty() || config_.source_sha256.empty() ||
      config_.config_sha256.empty())
    {
      throw std::invalid_argument(
              "research git/source/config provenance must be supplied when enabled");
    }
    config_.queue_capacity = std::max<std::size_t>(1U, config_.queue_capacity);
    const std::filesystem::path root(config_.output_root);
    std::filesystem::create_directories(root);
    const std::filesystem::path run_path = root / config_.run_id;
    if (!std::filesystem::create_directory(run_path)) {
      throw std::runtime_error("research run directory already exists: " + run_path.string());
    }
    run_directory_ = run_path.string();
    planning_output_.open(run_path / "planning_events.jsonl", std::ios::out);
    evaluation_output_.open(run_path / "evaluation_events.jsonl", std::ios::out);
    candidate_output_.open(run_path / "candidate_events.jsonl", std::ios::out);
    std::ofstream metadata(run_path / "metadata.json", std::ios::out);
    if (!planning_output_ || !evaluation_output_ || !candidate_output_ || !metadata) {
      throw std::runtime_error("failed to open research output files");
    }
    metadata << "{\n"
             << "  \"schema_version\": \"" << kPlanningResearchSchemaVersion << "\",\n"
             << "  \"run_id\": \"" << jsonEscape(config_.run_id) << "\",\n"
             << "  \"scenario_id\": \"" << jsonEscape(config_.scenario_id) << "\",\n"
             << "  \"git_commit\": \"" << jsonEscape(config_.git_commit) << "\",\n"
             << "  \"git_dirty_status\": \""
             << jsonEscape(config_.git_dirty_status) << "\",\n"
             << "  \"source_sha256\": \"" << jsonEscape(config_.source_sha256) << "\",\n"
             << "  \"config_sha256\": \"" << jsonEscape(config_.config_sha256) << "\",\n"
             << "  \"effective_parameter_snapshot_id\": \""
             << jsonEscape(config_.effective_parameter_snapshot_id) << "\",\n"
             << "  \"effective_parameters\": "
             << (config_.effective_parameters_json.empty() ? "{}" :
    config_.effective_parameters_json) << "\n}\n";
    metadata.close();
    writer_ = std::thread([this]() {writerLoop();});
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (writer_.joinable()) {
      writer_.join();
    }
  }

  bool submit(PlanningResearchCycle cycle)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || stopping_ || queue_.size() >= config_.queue_capacity) {
      ++dropped_count_;
      return false;
    }
    queue_.push_back(std::move(cycle));
    ++accepted_count_;
    lock.unlock();
    condition_.notify_one();
    return true;
  }

  void writerLoop()
  {
    while (true) {
      PlanningResearchCycle cycle;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
        if (queue_.empty() && stopping_) {
          break;
        }
        cycle = std::move(queue_.front());
        queue_.pop_front();
      }
      planning_output_ << planningJson(cycle, config_, dropped_count_.load()) << '\n';
      for (const auto & evaluation : cycle.evaluations) {
        evaluation_output_ << evaluationJson(cycle, evaluation) << '\n';
        for (const auto & candidate : evaluation.candidates) {
          candidate_output_ << candidateJson(cycle, evaluation, candidate) << '\n';
        }
      }
      ++written_count_;
    }
    planning_output_.flush();
    evaluation_output_.flush();
    candidate_output_.flush();
    std::ofstream summary(
      std::filesystem::path(run_directory_) / "run_summary.json", std::ios::out);
    if (summary) {
      summary << "{\"schema_version\":\"" << kPlanningResearchSchemaVersion
              << "\",\"clean_shutdown\":true,\"accepted_cycle_count\":"
              << accepted_count_.load() << ",\"written_cycle_count\":" << written_count_
              << ",\"dropped_log_count\":" << dropped_count_.load() << "}\n";
      summary.flush();
    }
  }

  PlanningResearchConfig config_;
  std::string run_directory_;
  std::ofstream planning_output_;
  std::ofstream evaluation_output_;
  std::ofstream candidate_output_;
  std::deque<PlanningResearchCycle> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread writer_;
  std::atomic<std::uint64_t> dropped_count_{0U};
  std::atomic<std::uint64_t> accepted_count_{0U};
  std::uint64_t written_count_{0U};
  bool stopping_{false};
};

PlanningResearchLogger::PlanningResearchLogger(PlanningResearchConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

PlanningResearchLogger::~PlanningResearchLogger() = default;

bool PlanningResearchLogger::submit(PlanningResearchCycle cycle)
{
  return impl_->submit(std::move(cycle));
}

std::uint64_t PlanningResearchLogger::droppedCount() const
{
  return impl_->dropped_count_.load();
}

const std::string & PlanningResearchLogger::runDirectory() const
{
  return impl_->run_directory_;
}

PlanningResearchSerializationProfile profilePlanningResearchSerialization(
  const PlanningResearchCycle & cycle,
  const PlanningResearchConfig & config)
{
  const auto start = std::chrono::steady_clock::now();
  PlanningResearchSerializationProfile profile;
  profile.serialized_byte_count += planningJson(cycle, config, 0U).size();
  ++profile.event_count;
  for (const auto & evaluation : cycle.evaluations) {
    profile.serialized_byte_count += evaluationJson(cycle, evaluation).size();
    ++profile.event_count;
    for (const auto & candidate : evaluation.candidates) {
      profile.serialized_byte_count += candidateJson(cycle, evaluation, candidate).size();
      ++profile.event_count;
    }
  }
  profile.runtime_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - start).count();
  return profile;
}

void captureP3ResearchEvaluation(
  PlanningResearchCycle & cycle,
  const std::string & clearance_pass,
  const P3ShadowResult & result)
{
  P3ResearchEvaluationRecord evaluation;
  if (cycle.active_evaluation_lineage != nullptr) {
    evaluation.lineage = *cycle.active_evaluation_lineage;
  }
  evaluation.clearance_pass = clearance_pass;
  evaluation.invoked = result.invoked;
  evaluation.selected = result.would_recover;
  evaluation.failure_classification = result.failure_classification;
  evaluation.m0_v1_constructed_right = result.research_m0_v1_constructed_right;
  evaluation.m0_v1_constructed_left = result.research_m0_v1_constructed_left;
  evaluation.discarded_side_candidate_count = result.research_discarded_side_candidate_count;
  evaluation.m0_v2_constructed = result.research_m0_v2_constructed;
  evaluation.m1_constructed = result.m1_candidate_count;
  evaluation.constructed_total_actual = result.research_constructed_total_actual;
  evaluation.validate_candidate_executed_total_actual =
    result.research_validate_candidate_executed_total_actual;
  evaluation.hard_valid_total_actual = result.research_hard_valid_total_actual;
  evaluation.returned_candidate_count = result.candidate_count;
  evaluation.r3_invoked = result.r3_invoked;
  evaluation.r3_method_name = result.r3_method_name;
  evaluation.r3_method_sha256 = result.r3_method_sha256;
  evaluation.r3_lateral_factor_count = result.r3_lateral_factor_count;
  evaluation.r3_pair_priority_count = result.r3_pair_priority_count;
  evaluation.r3_lexicographic_factor_count = result.r3_lexicographic_factor_count;
  evaluation.r3_coverage_factor_count = result.r3_coverage_factor_count;
  evaluation.r3_constructed_candidate_count = result.r3_constructed_candidate_count;
  evaluation.r3_path_digest_duplicate_count = result.r3_path_digest_duplicate_count;
  evaluation.r3_validator_call_count = result.r3_validator_call_count;
  evaluation.r3_hard_valid_count = result.r3_hard_valid_count;
  evaluation.r3_usable_valid_count = result.r3_usable_valid_count;
  evaluation.r3_transition_count = result.r3_transition_count;
  evaluation.r3_raw_combination_count = result.r3_raw_combination_count;
  evaluation.r3_production_excluded_count = result.r3_production_excluded_count;
  evaluation.r3_factor_pool_count = result.r3_factor_pool_count;
  evaluation.r3_unique_profile_count = result.r3_unique_profile_count;
  evaluation.r3_proxy_metric_evaluation_count = result.r3_proxy_metric_evaluation_count;
  evaluation.r3_proxy_metric_cache_hit_count = result.r3_proxy_metric_cache_hit_count;
  evaluation.r3_corridor_sample_evaluation_count =
    result.r3_corridor_sample_evaluation_count;
  evaluation.r3_reference_spacing_scan_count = result.r3_reference_spacing_scan_count;
  evaluation.r3_reference_sample_count = result.r3_reference_sample_count;
  evaluation.r3_profile_basis_build_count = result.r3_profile_basis_build_count;
  evaluation.r3_profile_basis_cache_hit_count = result.r3_profile_basis_cache_hit_count;
  evaluation.r3_profile_sample_basis_count = result.r3_profile_sample_basis_count;
  evaluation.r3_pair_metric_worker_count = result.r3_pair_metric_worker_count;
  evaluation.r3_fallback_after_failure = result.r3_fallback_after_failure;
  evaluation.r3_runtime_context_preparation_us = result.r3_runtime_context_preparation_us;
  evaluation.r3_runtime_geometry_preparation_us = result.r3_runtime_geometry_preparation_us;
  evaluation.r3_runtime_transition_generation_us = result.r3_runtime_transition_generation_us;
  evaluation.r3_runtime_lateral_factor_generation_us =
    result.r3_runtime_lateral_factor_generation_us;
  evaluation.r3_runtime_pair_priority_computation_us =
    result.r3_runtime_pair_priority_computation_us;
  evaluation.r3_runtime_lexicographic_ordering_us =
    result.r3_runtime_lexicographic_ordering_us;
  evaluation.r3_runtime_coverage_ordering_us = result.r3_runtime_coverage_ordering_us;
  evaluation.r3_runtime_shape_deduplication_us =
    result.r3_runtime_shape_deduplication_us;
  evaluation.r3_runtime_candidate_deduplication_us =
    result.r3_runtime_candidate_deduplication_us;
  evaluation.r3_runtime_factor_generation_us = result.r3_runtime_factor_generation_us;
  evaluation.r3_runtime_reconstruction_us = result.r3_runtime_reconstruction_us;
  evaluation.r3_runtime_validation_us = result.r3_runtime_validation_us;
  evaluation.r3_runtime_final_ranking_us = result.r3_runtime_final_ranking_us;
  evaluation.r3_runtime_total_us = result.r3_runtime_total_us;
  evaluation.r3_selected_factors = result.r3_selected_factors;
  evaluation.runtime_total_us = result.runtime_total_us;
  evaluation.runtime_corridor_us = result.research_runtime_corridor_actual_us;
  evaluation.runtime_probe_anchor_us = result.research_runtime_probe_anchor_us;
  evaluation.runtime_root_solve_us = result.research_runtime_root_solver_actual_us;
  evaluation.runtime_spline_reconstruction_us =
    result.research_runtime_reconstruction_actual_us;
  evaluation.runtime_geometry_recompute_us = result.research_runtime_geometry_recompute_us;
  evaluation.runtime_velocity_shaping_us = result.research_runtime_velocity_shaping_us;
  evaluation.runtime_candidate_measurement_us = result.research_runtime_candidate_measurement_us;
  evaluation.runtime_hard_validation_us =
    result.research_runtime_hard_validation_actual_us;
  evaluation.runtime_ranking_us = result.research_runtime_ranking_us;

  evaluation.candidates.reserve(result.research_all_candidates.size());
  for (const auto & trace : result.research_all_candidates) {
    P3ResearchCandidateRecord record;
    record.clearance_pass = clearance_pass;
    record.generator_stage = trace.generator_stage;
    record.candidate_template = trace.candidate_template;
    record.side = trace.go_left ? "LEFT" : "RIGHT";
    record.generation_order = trace.generation_index;
    record.candidate_identity = trace.candidate_identity;
    record.logical_identity = trace.logical_identity;
    record.path_digest = trace.path_digest;
    record.component_id = trace.component_id;
    record.mapping_source = trace.mapping_source;
    record.source_cell = trace.source_cell;
    record.analytic_branch_regime = trace.source_branch_regime;
    record.r3_rank_stream = trace.r3_rank_stream;
    record.r3_target_source = trace.r3_target_source;
    record.r3_mid_source = trace.r3_mid_source;
    record.r3_lateral_factor_index = trace.r3_lateral_factor_index;
    record.r3_transition_index = trace.r3_transition_index;
    record.returned_by_policy = trace.returned_by_policy;
    record.discarded_side = trace.discarded_side;
    record.d_target = trace.d_target;
    record.s_probe = trace.s_probe;
    record.d_probe = trace.d_probe;
    record.d_mid = trace.d_mid;
    record.root_index = trace.root_index;
    record.root_type = trace.root_type;
    record.probe_location_rule = trace.probe_location_rule;
    record.probe_anchor_rule = trace.probe_anchor_rule;
    record.entry_scale = trace.entry_scale;
    record.exit_scale = trace.exit_scale;
    record.knots = trace.knot_stations;
    record.point_count = trace.point_count;
    record.waypoint0_s_m = trace.waypoint0_s_m;
    record.waypoint0_d_m = trace.waypoint0_d_m;
    record.waypoint0_x_m = trace.waypoint0_x_m;
    record.waypoint0_y_m = trace.waypoint0_y_m;
    record.waypoint0_yaw_rad = trace.waypoint0_yaw_rad;
    record.waypoint0_center_track_margin_m = trace.waypoint0_center_track_margin_m;
    record.waypoint0_footprint_track_margin_m = trace.waypoint0_footprint_track_margin_m;
    record.waypoint0_footprint_invalid = trace.waypoint0_footprint_invalid;
    record.validator_executed = trace.validator_executed;
    record.validation_authority = trace.validation_authority;
    record.hard_valid = trace.hard_valid;
    record.usable_valid = trace.hard_valid && !trace.exit_reaches_next_obstacle &&
      trace.ego_braking_distance_deficit_m <= 1.0e-9;
    record.first_failure_enum = trace.validation.first_failure_kind;
    record.first_failure_reason = trace.rejection_reason;
    record.failure_waypoint_index = trace.validation.failure_waypoint_index;
    record.failure_obstacle_id = trace.validation.failure_obstacle_id;
    record.all_observed_violation_flags = trace.all_observed_violation_flags;
    record.center_track_margin_m = trace.validation.minimum_center_track_margin_m;
    record.footprint_track_margin_m = trace.minimum_track_margin_m;
    record.obstacle_margin_m = trace.minimum_obstacle_margin_m;
    record.peak_lateral_slope = trace.peak_lateral_slope;
    record.lateral_slope_margin = trace.lateral_slope_margin;
    record.peak_positive_curvature_radpm = trace.validation.peak_positive_curvature_radpm;
    record.peak_negative_curvature_radpm = trace.validation.peak_negative_curvature_radpm;
    record.signed_curvature_margin_radpm = trace.minimum_curvature_margin_radpm;
    record.peak_curvature_rate_radpm2 = trace.peak_curvature_rate_radpm2;
    record.curvature_rate_margin_radpm2 = trace.curvature_rate_margin_radpm2;
    record.exit_reaches_next_obstacle = trace.exit_reaches_next_obstacle;
    record.braking_feasible = trace.ego_braking_distance_deficit_m <= 1.0e-9;
    record.braking_deficit_m = trace.ego_braking_distance_deficit_m;
    record.velocity_loss = trace.velocity_loss;
    record.minimum_normalized_safety_slack = trace.minimum_normalized_safety_slack;
    record.global_path_deviation_m = trace.global_path_deviation_m;
    record.final_rank = trace.final_rank;
    record.selected = trace.selected;
    record.runtime_probe_anchor_us = trace.runtime_probe_anchor_us;
    record.runtime_root_solve_us = trace.runtime_root_solve_us;
    record.runtime_spline_reconstruction_us = trace.runtime_spline_reconstruction_us;
    record.runtime_geometry_recompute_us = trace.runtime_geometry_recompute_us;
    record.runtime_velocity_shaping_us = trace.runtime_velocity_shaping_us;
    record.runtime_candidate_measurement_us = trace.runtime_candidate_measurement_us;
    record.runtime_hard_validation_us = trace.runtime_hard_validation_us;
    evaluation.candidates.push_back(std::move(record));
  }

  if (clearance_pass == "STRICT") {
    ++cycle.strict_attempted;
  } else if (clearance_pass == "RELAXED") {
    ++cycle.relaxed_attempted;
  }
  cycle.m0_v1_constructed_right += evaluation.m0_v1_constructed_right;
  cycle.m0_v1_constructed_left += evaluation.m0_v1_constructed_left;
  cycle.discarded_side_candidate_count += evaluation.discarded_side_candidate_count;
  cycle.m0_v2_constructed += evaluation.m0_v2_constructed;
  cycle.m1_constructed += evaluation.m1_constructed;
  cycle.constructed_total_actual += evaluation.constructed_total_actual;
  // Callback-global validator totals are counted at validateCandidate() itself. Do not add the
  // evaluator-local totals here: plan(), lifecycle, retention, and safe-stop may invoke the same
  // authority again in the same callback, and summing both paths would double count P3 calls.
  cycle.returned_candidate_count += evaluation.returned_candidate_count;
  cycle.r3_invocation_count += evaluation.r3_invoked ? 1U : 0U;
  cycle.r3_constructed_candidate_count += evaluation.r3_constructed_candidate_count;
  cycle.r3_path_digest_duplicate_count += evaluation.r3_path_digest_duplicate_count;
  cycle.r3_validator_call_count += evaluation.r3_validator_call_count;
  cycle.r3_hard_valid_count += evaluation.r3_hard_valid_count;
  cycle.r3_usable_valid_count += evaluation.r3_usable_valid_count;
  cycle.runtime_planning_total_us += evaluation.runtime_total_us;
  cycle.runtime_corridor_us += evaluation.runtime_corridor_us;
  cycle.runtime_probe_anchor_us += evaluation.runtime_probe_anchor_us;
  cycle.runtime_root_solve_us += evaluation.runtime_root_solve_us;
  cycle.runtime_spline_reconstruction_us += evaluation.runtime_spline_reconstruction_us;
  cycle.runtime_geometry_recompute_us += evaluation.runtime_geometry_recompute_us;
  cycle.runtime_velocity_shaping_us += evaluation.runtime_velocity_shaping_us;
  cycle.runtime_candidate_measurement_us += evaluation.runtime_candidate_measurement_us;
  cycle.runtime_hard_validation_us += evaluation.runtime_hard_validation_us;
  cycle.runtime_ranking_us += evaluation.runtime_ranking_us;
  cycle.runtime_r3_total_us += evaluation.r3_runtime_total_us;
  cycle.evaluations.push_back(std::move(evaluation));
}

}  // namespace local_planning
