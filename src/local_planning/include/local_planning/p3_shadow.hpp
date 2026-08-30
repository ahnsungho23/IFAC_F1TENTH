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

#ifndef LOCAL_PLANNING__P3_SHADOW_HPP_
#define LOCAL_PLANNING__P3_SHADOW_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <f110_msgs/msg/wpnt_array.hpp>

namespace local_planning
{

struct P3ShadowObstacleEnvelope
{
  int id{-1};
  double start{0.0};
  double end{0.0};
  double center{0.0};
  double d_right{0.0};
  double d_left{0.0};
};

struct P3ShadowSideDomain
{
  bool valid{false};
  bool go_left{false};
  double cluster_start{0.0};
  double cluster_end{0.0};
  double minimum_target{0.0};
  double maximum_target{0.0};
  std::string reason;
};

struct P3ShadowPlanningContext
{
  bool valid{false};
  bool outside_is_left{false};
  std::string reason;
  std::vector<P3ShadowObstacleEnvelope> visible;
  std::vector<int> cluster_ids;
  P3ShadowSideDomain right;
  P3ShadowSideDomain left;
};

struct P3ShadowPathEvaluation
{
  bool hard_valid{false};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double minimum_center_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_positive_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_negative_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double minimum_curvature_margin_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double ego_braking_distance_deficit_m{0.0};
  double runtime_candidate_measurement_us{0.0};
  double runtime_hard_validation_us{0.0};
  std::string rejection_reason;
  std::vector<std::string> all_observed_violation_flags;
  int first_failure_kind{0};
  std::int64_t failure_waypoint_index{-1};
  // 어떤 장애물과, 경로의 어느 지점에서 걸렸는가 (2026-08-16 진단).
  // 사유 문자열만으로는 "탈출 램프가 다음 장애물을 스쳤다"와 "라인으로 복귀하는 합류가
  // 라인 위 장애물을 관통했다"를 구분할 수 없고, 둘은 수리가 완전히 다르다.
  int failure_obstacle_id{-1};
  double failure_waypoint_s{std::numeric_limits<double>::quiet_NaN()};
  double failure_waypoint_d{std::numeric_limits<double>::quiet_NaN()};
  // footprint_track_bound 기각의 기하 (2026-08-23 진단). 이 사유가 후보 기각의 91%를
  // 차지하는데, 사유 문자열과 s 만으로는 "그 지점 트랙이 좁다"와 "경로가 비스듬해 회전한
  // 사각형의 모서리가 튀어나왔다"를 구분할 수 없다. 둘은 수리가 반대다 — 전자는 목표
  // 오프셋을, 후자는 램프 길이(=기울기)를 줄여야 한다.
  //   failure_footprint_side          : 어느 쪽 벽에 걸렸는가 ("left"/"right")
  //   failure_heading_relative_rad    : 그 지점 경로가 레퍼런스 대비 몇 rad 기울었는가
  //   failure_corner_protrusion_m     : 회전 때문에 반폭(0.15) 너머로 더 나간 양
  // protrusion 이 크면 기울기 문제, 0 에 가까우면 순수 폭 문제다.
  std::string failure_footprint_side;
  double failure_heading_relative_rad{std::numeric_limits<double>::quiet_NaN()};
  double failure_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
};

// Complete production P3 candidate trace. The evaluator ranks these internally; the node may
// publish only selected_path and only in explicit TEST_ACTIVE mode. SHADOW remains log-only.
struct P3ShadowCandidateTrace
{
  std::size_t generation_index{0U};
  bool go_left{false};
  std::string generator_stage{"UNKNOWN"};
  double entry_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_scale{std::numeric_limits<double>::quiet_NaN()};
  double d_target{std::numeric_limits<double>::quiet_NaN()};
  double d_mid{std::numeric_limits<double>::quiet_NaN()};
  double s_probe{std::numeric_limits<double>::quiet_NaN()};
  double d_probe{std::numeric_limits<double>::quiet_NaN()};
  std::string probe_location_rule{"NONE"};
  std::string probe_anchor_rule{"NONE"};
  std::int64_t root_index{-1};
  std::string root_type{"NONE"};
  std::array<double, 5> knot_stations{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::size_t point_count{0U};
  bool validator_executed{false};
  std::string validation_authority{"GUARD"};
  bool returned_by_policy{false};
  bool discarded_side{false};
  int final_rank{-1};
  bool selected{false};
  bool hard_valid{false};
  std::string mapping_source;
  std::string candidate_template;
  std::string source_cell;
  std::string component_id;
  std::string source_branch_regime;
  std::string candidate_identity;
  std::string logical_identity;
  std::string path_digest;
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double minimum_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double minimum_curvature_margin_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double peak_lateral_slope{std::numeric_limits<double>::quiet_NaN()};
  double lateral_slope_margin{std::numeric_limits<double>::quiet_NaN()};
  double curvature_rate_margin_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double ego_braking_distance_deficit_m{0.0};
  double minimum_commanded_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double maximum_commanded_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  // 확정 장애물 통과 envelope 진단. enabled일 때만 critical_speed가 finite다. Critical
  // 표본은 entry 시작~padded cluster end, hold는 padded start~speed-only post-hold 끝이다.
  // 뒤끝은 max(longitudinal padding, post-hold)로 합성해 이중 계상하지 않는다. 이 값들은
  // 후보 선택에는 참여하지 않는 수동 진단 메타데이터다.
  double confirmed_critical_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double confirmed_speed_hold_start_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double confirmed_speed_hold_end_forward_m{std::numeric_limits<double>::quiet_NaN()};
  std::string rejection_reason;
  // True when this candidate's post-cluster exit ramp still carries enough lateral offset to reach
  // a NOT-in-cluster obstacle's physical envelope. Such a candidate is legal — the maneuver-scope
  // collision horizon deliberately stops before it, and on tightly spaced obstacles carrying the
  // offset over is the intended behaviour — but it is the last resort, never the preference.
  bool exit_reaches_next_obstacle{false};
  std::vector<std::string> all_observed_violation_flags;
  double runtime_probe_anchor_us{std::numeric_limits<double>::quiet_NaN()};
  double runtime_root_solve_us{std::numeric_limits<double>::quiet_NaN()};
  double runtime_spline_reconstruction_us{0.0};
  double runtime_geometry_recompute_us{0.0};
  double runtime_velocity_shaping_us{0.0};
  double runtime_candidate_measurement_us{0.0};
  double runtime_hard_validation_us{0.0};
  // The exact-validator verdict this candidate was already measured with, kept verbatim so the
  // maneuver lifecycle can reuse it instead of re-running an identical validation. Valid only
  // together with the owning result's snapshot lineage (stamp/epoch/reference generation).
  P3ShadowPathEvaluation validation;
  f110_msgs::msg::WpntArray path;
};

struct P3ShadowResult
{
  bool enabled{false};
  bool invoked{false};
  std::int64_t snapshot_source_stamp_ns{0};
  std::uint64_t snapshot_epoch{0U};
  std::uint64_t global_reference_generation{0U};
  std::string p0_failure_reason;
  std::string mapping_semantics{"M1_BRANCH_COMPLETE_ACTIVE_SET_CLOSURE"};

  std::size_t raw_root_count{0U};
  std::size_t finite_root_count{0U};
  std::size_t branch_root_count{0U};
  std::size_t bounded_root_count{0U};
  std::size_t accepted_root_count{0U};
  std::size_t candidate_count{0U};
  std::size_t hard_validator_call_count{0U};
  std::size_t hard_valid_count{0U};

  std::size_t m0_candidate_count{0U};
  std::size_t m0_validator_call_count{0U};
  std::size_t m0_hard_valid_count{0U};
  bool m1_invoked{false};
  std::size_t m1_budget{0U};
  std::size_t m1_context_count{0U};
  std::size_t m1_candidate_count{0U};
  std::size_t m1_validator_call_count{0U};
  std::size_t m1_hard_valid_count{0U};
  std::size_t m1_active_outer_raw_root_count{0U};
  std::size_t m1_active_outer_accepted_root_count{0U};
  std::size_t m1_all_inactive_raw_root_count{0U};
  std::size_t m1_all_inactive_accepted_root_count{0U};
  std::size_t m1_exact_tuple_duplicate_count{0U};
  std::size_t m1_positive_segment_rejection_count{0U};
  std::size_t m1_boundary_handoff_unresolved_count{0U};

  // Blocking-cluster observation is independent of whether any candidate is selected. This keeps
  // SHADOW diagnostics complete on fail-closed and committed-suffix callbacks.
  std::vector<int> cluster_obstacle_ids;
  double cluster_start_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double cluster_end_forward_m{std::numeric_limits<double>::quiet_NaN()};

  bool would_recover{false};
  bool selected_go_left{false};
  std::vector<int> selected_obstacle_ids;
  double selected_cluster_start_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_cluster_end_forward_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_cluster_end_s{std::numeric_limits<double>::quiet_NaN()};
  std::string selected_source{"NONE"};
  std::string selected_candidate_template{"NONE"};
  std::string selected_source_cell{"NONE"};
  std::string selected_component_id{"NONE"};
  std::string selected_source_branch_regime{"NONE"};
  std::string selected_candidate_identity{"NONE"};
  std::string selected_logical_identity{"NONE"};
  std::string selected_path_digest{"NONE"};
  double selected_d_target{std::numeric_limits<double>::quiet_NaN()};
  double selected_d_mid{std::numeric_limits<double>::quiet_NaN()};
  double selected_probe_s{std::numeric_limits<double>::quiet_NaN()};
  double selected_probe_d{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_track_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_obstacle_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double selected_curvature_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_curvature_rate_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_slope_margin{std::numeric_limits<double>::quiet_NaN()};
  double selected_min_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double selected_max_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  // Guarded-geometry validation certificate for selected_path, produced during candidate
  // construction against exactly the ego/obstacles this result was evaluated with.
  bool selected_validation_available{false};
  P3ShadowPathEvaluation selected_validation;
  f110_msgs::msg::WpntArray selected_path;

  double runtime_total_us{0.0};
  double runtime_corridor_us{0.0};
  double runtime_root_solver_us{0.0};
  double runtime_reconstruction_us{0.0};
  double runtime_hard_validation_us{0.0};
  // Default-empty research-only fields. They are populated only while the node's explicit
  // research instrumentation gate is active and never participate in selection or publication.
  std::size_t research_m0_v1_constructed_right{0U};
  std::size_t research_m0_v1_constructed_left{0U};
  std::size_t research_discarded_side_candidate_count{0U};
  std::size_t research_m0_v2_constructed{0U};
  std::size_t research_constructed_total_actual{0U};
  std::size_t research_validate_candidate_executed_total_actual{0U};
  std::size_t research_hard_valid_total_actual{0U};
  double research_runtime_probe_anchor_us{0.0};
  double research_runtime_corridor_actual_us{0.0};
  double research_runtime_root_solver_actual_us{0.0};
  double research_runtime_reconstruction_actual_us{0.0};
  double research_runtime_geometry_recompute_us{0.0};
  double research_runtime_velocity_shaping_us{0.0};
  double research_runtime_candidate_measurement_us{0.0};
  double research_runtime_hard_validation_actual_us{0.0};
  double research_runtime_ranking_us{0.0};
  std::string failure_classification{"NOT_INVOKED"};
  std::vector<P3ShadowCandidateTrace> candidates;
  std::vector<P3ShadowCandidateTrace> research_all_candidates;
  // 진단 전용 (2026-08-16). 후보가 전멸했을 때 원인을 셋 중 하나로 좁히는 데 필요한
  // 최소 정보다:
  //   (a) 도메인이 실현 가능한 오프셋을 아예 포함하지 않았다  → 도메인 계산 문제
  //   (b) 포함했는데 그 깊이에 후보를 만들지 않았다            → 후보 배치 간격 문제
  //   (c) 만들었는데 탈락했다                                  → 검증 기준 문제
  // 이 셋은 발행된 candidate d_target 목록과 도메인 경계 없이는 구분할 수 없다.
  // 2026-08-16 16:06 백에서 P3가 후보 8개를 전부 기각한 순간, 같은 상태를 하니스에 넣으면
  // P0 quintic 계열이 target_d=-0.84로 실현 가능한 해를 찾았다. 어느 단계에서 갈렸는지
  // 알 수 없어 수리가 추측이 될 뻔했다.
  P3ShadowSideDomain left_domain;
  P3ShadowSideDomain right_domain;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__P3_SHADOW_HPP_
