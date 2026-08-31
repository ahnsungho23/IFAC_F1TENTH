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

#ifndef LOCAL_PLANNING__RESEARCH_INSTRUMENTATION_HPP_
#define LOCAL_PLANNING__RESEARCH_INSTRUMENTATION_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "local_planning/p3_shadow.hpp"

namespace local_planning
{

inline constexpr const char * kPlanningResearchSchemaVersion =
  "local_planning_research/2";

struct P3ResearchObstacleSnapshotRecord
{
  int id{-1};
  double s_start{std::numeric_limits<double>::quiet_NaN()};
  double s_end{std::numeric_limits<double>::quiet_NaN()};
  double s_center{std::numeric_limits<double>::quiet_NaN()};
  double d_right{std::numeric_limits<double>::quiet_NaN()};
  double d_left{std::numeric_limits<double>::quiet_NaN()};
  double d_center{std::numeric_limits<double>::quiet_NaN()};
  bool is_static{false};
  bool is_visible{false};
};

// Immutable lineage for one top-level evaluateP3Shadow() invocation. STRICT and RELAXED are
// passes of the same invocation and therefore share evaluation_sequence and the snapshot IDs.
struct P3ResearchEvaluationLineage
{
  std::uint64_t evaluation_sequence{0U};
  std::string evaluation_role{"UNKNOWN"};
  std::string input_snapshot_id;
  std::string ego_snapshot_id;
  std::string obstacle_snapshot_id;
  std::string reference_snapshot_id;
  std::int64_t source_stamp_ns{0};
  std::uint64_t obstacle_sequence{0U};
  std::uint64_t source_epoch{0U};
  std::uint64_t reference_generation{0U};
  double ego_s{std::numeric_limits<double>::quiet_NaN()};
  double ego_d{std::numeric_limits<double>::quiet_NaN()};
  double ego_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  std::vector<P3ResearchObstacleSnapshotRecord> obstacles;
};

struct P3ResearchCandidateRecord
{
  std::string clearance_pass{"STRICT"};
  std::string generator_stage{"UNKNOWN"};
  std::string candidate_template;
  std::string side;
  std::size_t generation_order{0U};
  std::string candidate_identity;
  std::string logical_identity;
  std::string path_digest;
  std::string component_id;
  std::string mapping_source;
  std::string source_cell;
  std::string analytic_branch_regime;
  std::string r3_rank_stream;
  std::string r3_target_source;
  std::string r3_mid_source;
  std::size_t r3_lateral_factor_index{0U};
  std::size_t r3_transition_index{0U};
  bool returned_by_policy{false};
  bool discarded_side{false};

  double d_target{std::numeric_limits<double>::quiet_NaN()};
  double s_probe{std::numeric_limits<double>::quiet_NaN()};
  double d_probe{std::numeric_limits<double>::quiet_NaN()};
  double d_mid{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t root_index{-1};
  std::string root_type{"NONE"};
  std::string probe_location_rule{"NONE"};
  std::string probe_anchor_rule{"NONE"};
  double entry_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_scale{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 5> knots{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::size_t point_count{0U};
  double waypoint0_s_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_d_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_x_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_y_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_center_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double waypoint0_footprint_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  bool waypoint0_footprint_invalid{false};

  bool validator_executed{false};
  std::string validation_authority{"GUARD"};
  bool hard_valid{false};
  bool usable_valid{false};
  int first_failure_enum{0};
  std::string first_failure_reason;
  std::int64_t failure_waypoint_index{-1};
  int failure_obstacle_id{-1};
  std::vector<std::string> all_observed_violation_flags;

  double center_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_lateral_slope{std::numeric_limits<double>::quiet_NaN()};
  double lateral_slope_margin{std::numeric_limits<double>::quiet_NaN()};
  double peak_positive_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_negative_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double signed_curvature_margin_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double curvature_rate_margin_radpm2{std::numeric_limits<double>::quiet_NaN()};

  bool exit_reaches_next_obstacle{false};
  bool braking_feasible{false};
  double braking_deficit_m{0.0};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{
    -std::numeric_limits<double>::infinity()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  int final_rank{-1};
  bool selected{false};

  // Shared corridor/probe/root costs cannot be attributed to one candidate without double
  // counting; those fields remain null in CANDIDATE_EVENT and are authoritative per evaluation.
  double runtime_probe_anchor_us{std::numeric_limits<double>::quiet_NaN()};
  double runtime_root_solve_us{std::numeric_limits<double>::quiet_NaN()};
  double runtime_spline_reconstruction_us{0.0};
  double runtime_geometry_recompute_us{0.0};
  double runtime_velocity_shaping_us{0.0};
  double runtime_candidate_measurement_us{0.0};
  double runtime_hard_validation_us{0.0};
};

struct P3ResearchEvaluationRecord
{
  P3ResearchEvaluationLineage lineage;
  std::string clearance_pass{"STRICT"};
  bool invoked{false};
  bool selected{false};
  std::string failure_classification;
  std::size_t m0_v1_constructed_right{0U};
  std::size_t m0_v1_constructed_left{0U};
  std::size_t discarded_side_candidate_count{0U};
  std::size_t m0_v2_constructed{0U};
  std::size_t m1_constructed{0U};
  std::size_t constructed_total_actual{0U};
  std::size_t validate_candidate_executed_total_actual{0U};
  std::size_t hard_valid_total_actual{0U};
  std::size_t returned_candidate_count{0U};
  bool r3_invoked{false};
  std::string r3_method_name{"NONE"};
  std::string r3_method_sha256{"NONE"};
  std::size_t r3_lateral_factor_count{0U};
  std::size_t r3_pair_priority_count{0U};
  std::size_t r3_lexicographic_factor_count{0U};
  std::size_t r3_coverage_factor_count{0U};
  std::size_t r3_constructed_candidate_count{0U};
  std::size_t r3_path_digest_duplicate_count{0U};
  std::size_t r3_validator_call_count{0U};
  std::size_t r3_hard_valid_count{0U};
  std::size_t r3_usable_valid_count{0U};
  std::size_t r3_transition_count{0U};
  std::size_t r3_raw_combination_count{0U};
  std::size_t r3_production_excluded_count{0U};
  std::size_t r3_factor_pool_count{0U};
  std::size_t r3_unique_profile_count{0U};
  std::size_t r3_proxy_metric_evaluation_count{0U};
  std::size_t r3_proxy_metric_cache_hit_count{0U};
  std::size_t r3_corridor_sample_evaluation_count{0U};
  std::size_t r3_reference_spacing_scan_count{0U};
  std::size_t r3_reference_sample_count{0U};
  std::size_t r3_profile_basis_build_count{0U};
  std::size_t r3_profile_basis_cache_hit_count{0U};
  std::size_t r3_profile_sample_basis_count{0U};
  std::size_t r3_pair_metric_worker_count{0U};
  std::string r3_fallback_after_failure{"NOT_INVOKED"};
  double r3_runtime_context_preparation_us{0.0};
  double r3_runtime_geometry_preparation_us{0.0};
  double r3_runtime_transition_generation_us{0.0};
  double r3_runtime_lateral_factor_generation_us{0.0};
  double r3_runtime_pair_priority_computation_us{0.0};
  double r3_runtime_lexicographic_ordering_us{0.0};
  double r3_runtime_coverage_ordering_us{0.0};
  double r3_runtime_shape_deduplication_us{0.0};
  double r3_runtime_candidate_deduplication_us{0.0};
  double r3_runtime_factor_generation_us{0.0};
  double r3_runtime_reconstruction_us{0.0};
  double r3_runtime_validation_us{0.0};
  double r3_runtime_final_ranking_us{0.0};
  double r3_runtime_total_us{0.0};
  std::vector<P3R3SelectedFactorTrace> r3_selected_factors;
  double runtime_total_us{0.0};
  double runtime_corridor_us{0.0};
  double runtime_probe_anchor_us{0.0};
  double runtime_root_solve_us{0.0};
  double runtime_spline_reconstruction_us{0.0};
  double runtime_geometry_recompute_us{0.0};
  double runtime_velocity_shaping_us{0.0};
  double runtime_candidate_measurement_us{0.0};
  double runtime_hard_validation_us{0.0};
  double runtime_ranking_us{0.0};
  std::vector<P3ResearchCandidateRecord> candidates;
};

// One planning callback's passive research state. The planner points at this object only while
// instrumentation is enabled. No field is read by generation, validation, ranking, lifecycle, or
// publication code.
struct PlanningResearchCycle
{
  bool all_violation_audit_enabled{false};
  std::string run_id;
  std::string scenario_id;
  std::uint64_t callback_sequence{0U};
  std::int64_t ros_time_ns{0};
  std::int64_t obstacle_source_stamp_ns{0};
  std::uint64_t obstacle_sequence{0U};
  std::uint64_t source_epoch{0U};
  std::uint64_t reference_generation{0U};
  std::uint64_t next_evaluation_sequence{1U};
  // Populated only for the dynamic extent of one evaluator invocation. It is never read by the
  // planner's generation, validation, ranking, lifecycle, or publication decisions.
  std::unique_ptr<P3ResearchEvaluationLineage> active_evaluation_lineage;

  double ego_x{std::numeric_limits<double>::quiet_NaN()};
  double ego_y{std::numeric_limits<double>::quiet_NaN()};
  double ego_yaw{std::numeric_limits<double>::quiet_NaN()};
  std::string ego_pose_source{"UNAVAILABLE"};
  double ego_s{std::numeric_limits<double>::quiet_NaN()};
  double ego_d{std::numeric_limits<double>::quiet_NaN()};
  double measured_speed_mps{std::numeric_limits<double>::quiet_NaN()};

  std::string p3_mode{"OFF"};
  std::string lifecycle_state{"IDLE"};
  std::string lifecycle_owner{"NONE"};
  std::string continuation_result{"NOT_ATTEMPTED"};
  std::string invalidation_reason;
  std::string selected_side{"NONE"};
  std::string selected_candidate_identity{"NONE"};
  std::string selected_path_digest{"NONE"};
  std::string fallback_reason;
  bool active{false};
  bool continued{false};
  bool fresh{false};
  bool backup{false};
  bool safe_stop{false};
  bool global_handoff{false};

  std::size_t strict_attempted{0U};
  std::size_t relaxed_attempted{0U};
  std::size_t m0_v1_constructed_right{0U};
  std::size_t m0_v1_constructed_left{0U};
  std::size_t discarded_side_candidate_count{0U};
  std::size_t m0_v2_constructed{0U};
  std::size_t m1_constructed{0U};
  std::size_t constructed_total_actual{0U};
  std::size_t validate_candidate_executed_total_actual{0U};
  std::size_t hard_valid_total_actual{0U};
  std::size_t returned_candidate_count{0U};
  std::size_t lifecycle_revalidation_count{0U};
  std::size_t safe_stop_escape_evaluator_count{0U};
  std::size_t r3_invocation_count{0U};
  std::size_t r3_constructed_candidate_count{0U};
  std::size_t r3_path_digest_duplicate_count{0U};
  std::size_t r3_validator_call_count{0U};
  std::size_t r3_hard_valid_count{0U};
  std::size_t r3_usable_valid_count{0U};

  double runtime_callback_total_us{0.0};
  double runtime_planning_total_us{0.0};
  double runtime_corridor_us{0.0};
  double runtime_probe_anchor_us{0.0};
  double runtime_root_solve_us{0.0};
  double runtime_spline_reconstruction_us{0.0};
  double runtime_geometry_recompute_us{0.0};
  double runtime_velocity_shaping_us{0.0};
  double runtime_candidate_measurement_us{0.0};
  double runtime_hard_validation_us{0.0};
  double runtime_ranking_us{0.0};
  double runtime_lifecycle_revalidation_us{0.0};
  double runtime_r3_total_us{0.0};
  double runtime_research_lineage_us{0.0};
  double runtime_research_capture_us{0.0};

  std::vector<P3ResearchEvaluationRecord> evaluations;
};

struct PlanningResearchConfig
{
  std::string output_root;
  std::string run_id;
  std::string scenario_id;
  std::string git_commit;
  std::string git_dirty_status;
  std::string source_sha256;
  std::string config_sha256;
  std::string effective_parameter_snapshot_id;
  std::string effective_parameters_json;
  std::size_t queue_capacity{128U};
};

// A default-off, asynchronous JSONL sink. submit() uses try_lock and drops the cycle when the
// writer owns the queue or it is full; the planning thread never waits for disk or fsync.
class PlanningResearchLogger
{
public:
  explicit PlanningResearchLogger(PlanningResearchConfig config);
  ~PlanningResearchLogger();

  PlanningResearchLogger(const PlanningResearchLogger &) = delete;
  PlanningResearchLogger & operator=(const PlanningResearchLogger &) = delete;

  bool submit(PlanningResearchCycle cycle);
  std::uint64_t droppedCount() const;
  const std::string & runDirectory() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

struct PlanningResearchSerializationProfile
{
  std::size_t event_count{0U};
  std::size_t serialized_byte_count{0U};
  double runtime_us{0.0};
};

// Executes the same JSON string construction as the asynchronous writer without touching disk.
// This is an explicit research/profile call and is never reached by default-OFF production.
PlanningResearchSerializationProfile profilePlanningResearchSerialization(
  const PlanningResearchCycle & cycle,
  const PlanningResearchConfig & config);

void captureP3ResearchEvaluation(
  PlanningResearchCycle & cycle,
  const std::string & clearance_pass,
  const P3ShadowResult & result);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__RESEARCH_INSTRUMENTATION_HPP_
