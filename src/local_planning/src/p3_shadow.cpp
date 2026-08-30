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

#include "local_planning/p3_shadow.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "local_planning/candidate_rank.hpp"
#include "local_planning/p3_analytic_solver.hpp"
#include "local_planning/p3_r3_k12.hpp"
#include "local_planning/path_digest.hpp"
#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/research_instrumentation.hpp"

namespace local_planning
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashBytes(std::uint64_t & hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= static_cast<std::uint64_t>(bytes[index]);
    hash *= kFnvPrime;
  }
}

template<typename Value>
void hashValue(std::uint64_t & hash, const Value & value)
{
  hashBytes(hash, &value, sizeof(Value));
}

void hashString(std::uint64_t & hash, const std::string & value)
{
  const std::uint64_t size = value.size();
  hashValue(hash, size);
  hashBytes(hash, value.data(), value.size());
}

std::string hashId(const char * prefix, std::uint64_t hash)
{
  std::ostringstream output;
  output << prefix << '_' << std::hex << std::setw(16) << std::setfill('0') << hash;
  return output.str();
}

std::uint64_t hashEgo(const EgoFrenetState & ego)
{
  std::uint64_t hash = kFnvOffset;
  hashValue(hash, ego.s);
  hashValue(hash, ego.d);
  hashValue(hash, ego.speed);
  return hash;
}

std::uint64_t hashObstacles(const std::vector<f110_msgs::msg::Obstacle> & obstacles)
{
  std::uint64_t hash = kFnvOffset;
  const std::uint64_t count = obstacles.size();
  hashValue(hash, count);
  for (const auto & obstacle : obstacles) {
    hashValue(hash, obstacle.id);
    hashValue(hash, obstacle.has_cartesian);
    hashValue(hash, obstacle.x_center);
    hashValue(hash, obstacle.y_center);
    hashValue(hash, obstacle.radius);
    hashValue(hash, obstacle.x_min);
    hashValue(hash, obstacle.x_max);
    hashValue(hash, obstacle.y_min);
    hashValue(hash, obstacle.y_max);
    hashValue(hash, obstacle.x_var);
    hashValue(hash, obstacle.y_var);
    hashValue(hash, obstacle.s_start);
    hashValue(hash, obstacle.s_end);
    hashValue(hash, obstacle.d_right);
    hashValue(hash, obstacle.d_left);
    hashValue(hash, obstacle.is_actually_a_gap);
    hashValue(hash, obstacle.s_center);
    hashValue(hash, obstacle.d_center);
    hashValue(hash, obstacle.size);
    hashValue(hash, obstacle.vs);
    hashValue(hash, obstacle.vd);
    hashValue(hash, obstacle.s_var);
    hashValue(hash, obstacle.d_var);
    hashValue(hash, obstacle.vs_var);
    hashValue(hash, obstacle.vd_var);
    hashValue(hash, obstacle.s_vs_cov);
    hashValue(hash, obstacle.d_vd_cov);
    hashValue(hash, obstacle.is_static);
    hashValue(hash, obstacle.is_visible);
    hashValue(hash, obstacle.is_interfering);
  }
  return hash;
}

std::uint64_t hashReference(const f110_msgs::msg::WpntArray & reference)
{
  std::uint64_t hash = kFnvOffset;
  hashValue(hash, reference.header.stamp.sec);
  hashValue(hash, reference.header.stamp.nanosec);
  hashString(hash, reference.header.frame_id);
  const std::uint64_t count = reference.wpnts.size();
  hashValue(hash, count);
  for (const auto & waypoint : reference.wpnts) {
    hashValue(hash, waypoint.id);
    hashValue(hash, waypoint.s_m);
    hashValue(hash, waypoint.d_m);
    hashValue(hash, waypoint.x_m);
    hashValue(hash, waypoint.y_m);
    hashValue(hash, waypoint.d_right);
    hashValue(hash, waypoint.d_left);
    hashValue(hash, waypoint.psi_rad);
    hashValue(hash, waypoint.kappa_radpm);
    hashValue(hash, waypoint.vx_mps);
    hashValue(hash, waypoint.ax_mps2);
  }
  return hash;
}

std::unique_ptr<P3ResearchEvaluationLineage> makeEvaluationLineage(
  PlanningResearchCycle & cycle,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  const f110_msgs::msg::WpntArray & reference,
  std::int64_t source_stamp_ns,
  std::uint64_t source_epoch,
  std::uint64_t reference_generation,
  const std::string & role)
{
  auto lineage = std::make_unique<P3ResearchEvaluationLineage>();
  lineage->evaluation_sequence = cycle.next_evaluation_sequence++;
  lineage->evaluation_role = role.empty() ? "UNKNOWN" : role;
  lineage->source_stamp_ns = source_stamp_ns != 0 ?
    source_stamp_ns : cycle.obstacle_source_stamp_ns;
  lineage->obstacle_sequence = cycle.obstacle_sequence;
  lineage->source_epoch = source_epoch != 0U ? source_epoch : cycle.source_epoch;
  lineage->reference_generation = reference_generation != 0U ?
    reference_generation : cycle.reference_generation;
  lineage->ego_s = ego.s;
  lineage->ego_d = ego.d;
  lineage->ego_speed_mps = ego.speed;

  const std::uint64_t ego_hash = hashEgo(ego);
  const std::uint64_t obstacle_hash = hashObstacles(obstacles);
  const std::uint64_t reference_hash = hashReference(reference);
  lineage->ego_snapshot_id = hashId("ego", ego_hash);
  lineage->obstacle_snapshot_id = hashId("obs", obstacle_hash);
  lineage->reference_snapshot_id = hashId("ref", reference_hash);
  std::uint64_t input_hash = kFnvOffset;
  hashValue(input_hash, ego_hash);
  hashValue(input_hash, obstacle_hash);
  hashValue(input_hash, reference_hash);
  hashValue(input_hash, lineage->source_stamp_ns);
  hashValue(input_hash, lineage->obstacle_sequence);
  hashValue(input_hash, lineage->source_epoch);
  hashValue(input_hash, lineage->reference_generation);
  lineage->input_snapshot_id = hashId("input", input_hash);

  lineage->obstacles.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    P3ResearchObstacleSnapshotRecord record;
    record.id = obstacle.id;
    record.s_start = obstacle.s_start;
    record.s_end = obstacle.s_end;
    record.s_center = obstacle.s_center;
    record.d_right = obstacle.d_right;
    record.d_left = obstacle.d_left;
    record.d_center = obstacle.d_center;
    record.is_static = obstacle.is_static;
    record.is_visible = obstacle.is_visible;
    lineage->obstacles.push_back(record);
  }
  return lineage;
}

}  // namespace

// Production-owned runtime port of:
//   corridor_to_analytic_root_mapping_audit.cpp sha256 c3bdfc282d5c0b38342b763e0473d69e
//   p3_branch_analytic_solver.hpp sha256 a5accda81bc18dade6801dcb66973070b65d6f518
// The frozen CURVATURE_CONTINUITY semantic hash is
// b4282a44d50b2f4721d3edf8b7c32c3c8f4a1954a96a01887c2e01ee559eeb31.
//   p3_mapping_static_and_temporal_continuity_design_review.cpp (validated M1 closure).
// Container types are the only adaptation: equations, M0-first invocation, M1 round-robin
// templates, branch filtering, reconstruction, exact validation, ranking, and cap-24 remain
// literal. No external evaluator is linked or invoked at runtime.
class P3ShadowEvaluator
{
public:
  explicit P3ShadowEvaluator(const RacelineSplinePlanner & planner)
  : planner_(planner), parameters_(planner.parameters_)
  {
  }

  // 기준선 샘플 간격. 발행 경로는 기준선 격자 위에 생성되므로, 두 station이 이보다 가까우면
  // 서로 다른 점으로 나타나지 않는다. 튜닝값이 아니라 기준선의 성질이다.
  double referenceSpacing() const
  {
    const std::size_t count = planner_.reference_.wpnts.size();
    return count < 2U ? 0.25 : planner_.trackLength() / static_cast<double>(count);
  }

private:
  // 이 사이클이 책임지는 클러스터. buildCandidate가 검증 지평을 구할 때 필요한데 호출
  // 경로가 셋(M0/M0확장/M1)이라 인자로 흘리면 시그니처 셋이 다 바뀐다. 평가기는
  // evaluateP3Shadow 호출마다 새로 생성되므로(p3_shadow.cpp 하단) 사이클 상태로 두는 것이
  // 안전하다 — 사이클 간에 남지 않는다.
  mutable std::vector<int> cycle_cluster_ids_;

public:
  P3ShadowResult run(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::int64_t snapshot_source_stamp_ns,
    std::uint64_t snapshot_epoch,
    std::uint64_t global_reference_generation,
    const std::string & p0_failure_reason,
    bool relaxed_clearance_gate = false,
    std::vector<P3R3K12ProductionCandidate> * r3_production_factors = nullptr) const
  {
    P3ShadowResult result;
    result.enabled = true;
    result.invoked = true;
    result.snapshot_source_stamp_ns = snapshot_source_stamp_ns;
    result.snapshot_epoch = snapshot_epoch;
    result.global_reference_generation = global_reference_generation;
    result.p0_failure_reason = p0_failure_reason;
    const auto total_start = Clock::now();
    PlanningResearchCycle * research_cycle = planner_.activeResearchCycle();
    const double research_corridor_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_corridor_us;
    const double research_probe_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_probe_anchor_us;
    const double research_root_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_root_solve_us;
    const double research_spline_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_spline_reconstruction_us;
    const double research_geometry_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_geometry_recompute_us;
    const double research_velocity_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_velocity_shaping_us;
    const double research_measurement_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_candidate_measurement_us;
    const double research_validation_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_hard_validation_us;

    if (!planner_.ready()) {
      result.failure_classification = "REFERENCE_NOT_READY";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }
    if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.speed)) {
      result.failure_classification = "NONFINITE_EGO";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }

    const P3ShadowPlanningContext context = planner_.buildP3ShadowPlanningContext(
      ego, obstacles, relaxed_clearance_gate);
    if (!context.valid) {
      result.failure_classification = context.reason.empty() ?
        "NO_BLOCKING_CLUSTER" : context.reason;
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }
    result.cluster_obstacle_ids = context.cluster_ids;
    cycle_cluster_ids_ = context.cluster_ids;
    // computeSideTargetRange fills the same longitudinal cluster span for both sides before any
    // side-feasibility rejection. Keep this observation separate from selected-candidate fields.
    result.cluster_start_forward_m = context.right.cluster_start;
    result.cluster_end_forward_m = context.right.cluster_end;
    result.left_domain = context.left;
    result.right_domain = context.right;

    bool m0_nonpositive_abort = false;
    std::vector<SideResult> sides;
    std::vector<P3ShadowCandidateTrace> research_pre_abort_candidates;
    sides.reserve(2U);
    try {
      for (const bool go_left : {false, true}) {
        const auto & domain = go_left ? context.left : context.right;
        sides.push_back(evaluateSide(
            ego, obstacles, context.visible, domain, context.outside_is_left));
      }
    } catch (const std::runtime_error & error) {
      if (std::string(error.what()) != "non-positive quintic-Hermite segment") {
        throw;
      }
      // The frozen mapping review catches the whole M0 policy at this boundary. Preserve that
      // behavior so h0<=0 becomes a fail-closed no-M0 offer instead of escaping the callback.
      m0_nonpositive_abort = true;
      if (research_cycle != nullptr || r3_production_factors != nullptr) {
        for (const auto & side : sides) {
          if (research_cycle != nullptr) {
            if (side.go_left) {
              result.research_m0_v1_constructed_left += side.candidates.size();
            } else {
              result.research_m0_v1_constructed_right += side.candidates.size();
            }
            result.research_discarded_side_candidate_count += side.candidates.size();
          }
          for (const auto & trace : side.candidates) {
            if (r3_production_factors != nullptr) {
              r3_production_factors->push_back({
                  trace.go_left, trace.d_target, trace.d_mid,
                  trace.entry_scale, trace.exit_scale});
            }
            if (research_cycle != nullptr) {
              auto research_trace = trace;
              research_trace.returned_by_policy = false;
              research_trace.discarded_side = true;
              research_pre_abort_candidates.push_back(std::move(research_trace));
            }
          }
        }
      }
      sides.clear();
      SideResult aborted;
      aborted.domain_valid = true;
      aborted.failure = "NON_POSITIVE_SEGMENT_ABORT";
      sides.push_back(std::move(aborted));
    }

    // Frozen M0 runQuery semantics retain exactly one canonical side record: RIGHT first when
    // neither side has a hard-valid candidate, otherwise the globally ranked feasible side.
    const SideResult * baseline = nullptr;
    for (const auto & side : sides) {
      if (!side.domain_valid) {
        continue;
      }
      if (baseline == nullptr ||
        (side.best_index.has_value() && !baseline->best_index.has_value()) ||
        (side.best_index.has_value() && baseline->best_index.has_value() &&
        betterFeasible(
          side.candidates[*side.best_index], baseline->candidates[*baseline->best_index])))
      {
        baseline = &side;
      }
    }
    if (baseline == nullptr) {
      result.failure_classification = "NO_VALID_SIDE_DOMAIN";
      result.runtime_total_us = elapsedUs(total_start);
      return result;
    }

    std::vector<P3ShadowCandidateTrace> research_side_candidates =
      std::move(research_pre_abort_candidates);
    if (research_cycle != nullptr || r3_production_factors != nullptr) {
      for (const auto & side : sides) {
        const bool discarded = &side != baseline;
        if (research_cycle != nullptr) {
          if (side.go_left) {
            result.research_m0_v1_constructed_left += side.candidates.size();
          } else {
            result.research_m0_v1_constructed_right += side.candidates.size();
          }
          if (discarded) {
            result.research_discarded_side_candidate_count += side.candidates.size();
          }
        }
        for (const auto & trace : side.candidates) {
          if (r3_production_factors != nullptr) {
            r3_production_factors->push_back({
                trace.go_left, trace.d_target, trace.d_mid,
                trace.entry_scale, trace.exit_scale});
          }
          if (research_cycle != nullptr) {
            auto research_trace = trace;
            research_trace.returned_by_policy = !discarded;
            research_trace.discarded_side = discarded;
            research_side_candidates.push_back(std::move(research_trace));
          }
        }
      }
    }

    const auto append_side_metrics = [&](const SideResult & side) {
        result.raw_root_count += side.raw_root_count;
        result.finite_root_count += side.finite_root_count;
        result.branch_root_count += side.branch_root_count;
        result.bounded_root_count += side.bounded_root_count;
        result.accepted_root_count += side.accepted_root_count;
        result.runtime_corridor_us += side.runtime_corridor_us;
        result.runtime_root_solver_us += side.runtime_root_solver_us;
        result.runtime_reconstruction_us += side.runtime_reconstruction_us;
        result.runtime_hard_validation_us += side.runtime_hard_validation_us;
      };
    append_side_metrics(*baseline);
    result.selected_probe_s = baseline->probe.station;
    result.selected_probe_d = baseline->probe.desired;
    result.m0_candidate_count = baseline->candidates.size();
    result.m0_validator_call_count = baseline->validator_calls;
    result.m0_hard_valid_count = baseline->hard_valid_count;
    result.candidates.insert(
      result.candidates.end(), baseline->candidates.begin(), baseline->candidates.end());

    std::optional<P3ShadowCandidateTrace> selected;
    if (baseline->best_index.has_value()) {
      selected = baseline->candidates[*baseline->best_index];
    } else {
      ExtensionOutcome extension;
      if (!m0_nonpositive_abort) {
        try {
          // 🔴 2026-08-17: 확장 계열의 상한을 **남은 총예산**으로 제한한다.
          //
          // 종전 상한은 kFrozenCandidateCap(16) + kM0ExtensionCandidateCap(12) = 28 로
          // kTotalCandidateCap(24)을 넘을 수 있었고, 그러면 아래 불변식 검사가 throw 했다.
          // 오래 잠복해 있다가 후보 수를 늘리는 변경(진입 눈금 이분법)에서 실제로 터졌다
          // — 속성 테스트 546 배치 중 1건. 가드가 없었으면 그 자리에서 노드가 죽는다.
          extension = evaluateM0Extension(
            ego, obstacles, context,
            kTotalCandidateCap > baseline->candidates.size() ?
            kTotalCandidateCap - baseline->candidates.size() : 0U);
        } catch (const std::runtime_error & error) {
          if (std::string(error.what()) != "non-positive quintic-Hermite segment") {
            throw;
          }
          // The validated review treats a V2 constructor boundary abort as no M0 mapping offer;
          // M1 then receives the complete cap-24 budget and its strict segment guard is authority.
          m0_nonpositive_abort = true;
        }
      }
      if (m0_nonpositive_abort) {
        result.raw_root_count = 0U;
        result.finite_root_count = 0U;
        result.branch_root_count = 0U;
        result.bounded_root_count = 0U;
        result.accepted_root_count = 0U;
        result.m0_candidate_count = 0U;
        result.m0_validator_call_count = 0U;
        result.m0_hard_valid_count = 0U;
        result.candidates.clear();
      }
      if (!m0_nonpositive_abort) {
        result.raw_root_count += extension.raw_root_count;
        result.finite_root_count += extension.finite_root_count;
        result.branch_root_count += extension.branch_root_count;
        result.bounded_root_count += extension.bounded_root_count;
        result.accepted_root_count += extension.accepted_root_count;
        result.m0_candidate_count += extension.candidates.size();
        result.m0_validator_call_count += extension.validator_calls;
        result.m0_hard_valid_count += extension.hard_valid_count;
        result.runtime_root_solver_us += extension.runtime_root_solver_us;
        result.runtime_reconstruction_us += extension.runtime_reconstruction_us;
        result.runtime_hard_validation_us += extension.runtime_hard_validation_us;
        if (extension.best_index.has_value()) {
          selected = extension.candidates[*extension.best_index];
        }
        result.candidates.insert(
          result.candidates.end(), extension.candidates.begin(), extension.candidates.end());
      }
      if (!selected.has_value()) {
        if (result.m0_candidate_count > kTotalCandidateCap) {
          throw std::runtime_error("frozen M0 candidate count exceeds total predeclared cap");
        }
        result.m1_invoked = true;
        result.m1_budget = kTotalCandidateCap - result.m0_candidate_count;
        ExtensionOutcome m1 = evaluateM1(ego, obstacles, context, result);
        result.runtime_root_solver_us += m1.runtime_root_solver_us;
        result.runtime_reconstruction_us += m1.runtime_reconstruction_us;
        result.runtime_hard_validation_us += m1.runtime_hard_validation_us;
        if (m1.best_index.has_value()) {
          selected = m1.candidates[*m1.best_index];
        }
        result.candidates.insert(
          result.candidates.end(), m1.candidates.begin(), m1.candidates.end());
        if (result.m0_candidate_count + result.m1_candidate_count > kTotalCandidateCap ||
          result.m1_validator_call_count != result.m1_candidate_count)
        {
          throw std::runtime_error("M0+M1 constructed-candidate/validator-call bound violated");
        }
      }
    }

    result.candidate_count = result.m0_candidate_count + result.m1_candidate_count;
    result.hard_validator_call_count =
      result.m0_validator_call_count + result.m1_validator_call_count;
    result.hard_valid_count = result.m0_hard_valid_count + result.m1_hard_valid_count;
    if (selected.has_value()) {
      result.would_recover = true;
      result.selected_go_left = selected->go_left;
      result.selected_obstacle_ids = context.cluster_ids;
      const auto & selected_domain = selected->go_left ? context.left : context.right;
      result.selected_cluster_start_forward_m = selected_domain.cluster_start;
      result.selected_cluster_end_forward_m = selected_domain.cluster_end;
      result.selected_cluster_end_s = planner_.wrapS(ego.s + selected_domain.cluster_end);
      result.selected_source = selected->mapping_source;
      result.selected_candidate_template = selected->candidate_template;
      result.selected_source_cell = selected->source_cell;
      result.selected_component_id = selected->component_id;
      result.selected_source_branch_regime = selected->source_branch_regime;
      result.selected_candidate_identity = selected->candidate_identity;
      result.selected_logical_identity = selected->logical_identity;
      result.selected_path_digest = selected->path_digest;
      result.selected_d_target = selected->d_target;
      result.selected_d_mid = selected->d_mid;
      result.selected_min_track_margin_m = selected->minimum_track_margin_m;
      result.selected_min_obstacle_margin_m = selected->minimum_obstacle_margin_m;
      result.selected_curvature_margin = selected->minimum_curvature_margin_radpm;
      result.selected_curvature_rate_margin =
        parameters_.maximum_curvature_rate_radpm2 - selected->peak_curvature_rate_radpm2;
      result.selected_slope_margin =
        parameters_.maximum_lateral_slope - selected->peak_lateral_slope;
      result.selected_min_speed_mps = selected->minimum_commanded_speed_mps;
      result.selected_max_speed_mps = selected->maximum_commanded_speed_mps;
      result.selected_validation = selected->validation;
      result.selected_validation_available = true;
      result.selected_path = selected->path;
      result.failure_classification = "NONE";
    } else if (result.m1_boundary_handoff_unresolved_count > 0U) {
      result.failure_classification = "BOUNDARY_HANDOFF_UNRESOLVED";
    } else if (result.m1_invoked) {
      result.failure_classification = "NO_HARD_VALID_M1_CANDIDATE";
    } else {
      result.failure_classification = baseline->failure;
    }
    result.runtime_total_us = elapsedUs(total_start);
    if (r3_production_factors != nullptr) {
      for (const auto & trace : result.candidates) {
        if (trace.generator_stage != "M0_V1") {
          r3_production_factors->push_back({
              trace.go_left, trace.d_target, trace.d_mid,
              trace.entry_scale, trace.exit_scale});
        }
      }
    }
    if (research_cycle != nullptr) {
      const auto ranking_start = Clock::now();
      std::vector<std::size_t> feasible_order;
      for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
        if (result.candidates[index].hard_valid) {
          feasible_order.push_back(index);
        }
      }
      std::stable_sort(
        feasible_order.begin(), feasible_order.end(),
        [&result](std::size_t first, std::size_t second) {
          return betterFeasible(result.candidates[first], result.candidates[second]);
        });
      for (std::size_t rank = 0U; rank < feasible_order.size(); ++rank) {
        result.candidates[feasible_order[rank]].final_rank = static_cast<int>(rank + 1U);
      }
      for (auto & trace : result.candidates) {
        trace.returned_by_policy = true;
        trace.selected = result.would_recover &&
          trace.candidate_identity == result.selected_candidate_identity;
      }

      result.research_all_candidates = std::move(research_side_candidates);
      for (const auto & trace : result.candidates) {
        if (trace.generator_stage != "M0_V1") {
          result.research_all_candidates.push_back(trace);
        }
      }
      for (auto & trace : result.research_all_candidates) {
        const auto returned = std::find_if(
          result.candidates.begin(), result.candidates.end(), [&trace](const auto & candidate) {
            return candidate.candidate_identity == trace.candidate_identity;
          });
        if (returned != result.candidates.end()) {
          trace.returned_by_policy = true;
          trace.discarded_side = false;
          trace.final_rank = returned->final_rank;
          trace.selected = returned->selected;
        } else if (m0_nonpositive_abort) {
          trace.returned_by_policy = false;
        }
      }
      result.research_m0_v2_constructed = static_cast<std::size_t>(std::count_if(
          result.research_all_candidates.begin(), result.research_all_candidates.end(),
          [](const auto & trace) {return trace.generator_stage == "M0_V2";}));
      result.research_constructed_total_actual = result.research_all_candidates.size();
      result.research_validate_candidate_executed_total_actual =
        static_cast<std::size_t>(std::count_if(
          result.research_all_candidates.begin(), result.research_all_candidates.end(),
          [](const auto & trace) {return trace.validator_executed;}));
      result.research_hard_valid_total_actual = static_cast<std::size_t>(std::count_if(
          result.research_all_candidates.begin(), result.research_all_candidates.end(),
          [](const auto & trace) {return trace.hard_valid;}));
      result.research_runtime_ranking_us = elapsedUs(ranking_start);
      research_cycle->runtime_ranking_us += result.research_runtime_ranking_us;
      result.research_runtime_corridor_actual_us =
        research_cycle->runtime_corridor_us - research_corridor_before;
      result.research_runtime_probe_anchor_us =
        research_cycle->runtime_probe_anchor_us - research_probe_before;
      result.research_runtime_root_solver_actual_us =
        research_cycle->runtime_root_solve_us - research_root_before;
      result.research_runtime_reconstruction_actual_us =
        research_cycle->runtime_spline_reconstruction_us - research_spline_before;
      result.research_runtime_geometry_recompute_us =
        research_cycle->runtime_geometry_recompute_us - research_geometry_before;
      result.research_runtime_velocity_shaping_us =
        research_cycle->runtime_velocity_shaping_us - research_velocity_before;
      result.research_runtime_candidate_measurement_us =
        research_cycle->runtime_candidate_measurement_us - research_measurement_before;
      result.research_runtime_hard_validation_actual_us =
        research_cycle->runtime_hard_validation_us - research_validation_before;
    }
    return result;
  }

  P3ShadowResult runR3K12(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowResult & production_failure,
    const std::vector<P3R3K12ProductionCandidate> & production_candidates) const
  {
    const auto total_start = Clock::now();
    P3ShadowResult result = production_failure;
    result.r3_invoked = true;
    result.r3_method_name = kP3R3K12MethodName;
    result.r3_method_sha256 = kP3R3K12MethodSha256;
    result.mapping_semantics = kP3R3K12MethodName;
    result.r3_fallback_after_failure = production_failure.failure_classification;
    result.would_recover = false;
    result.candidates.clear();
    result.candidate_count = 0U;
    result.hard_validator_call_count = 0U;
    result.hard_valid_count = 0U;
    result.selected_source = "NONE";
    result.selected_candidate_template = "NONE";
    result.selected_candidate_identity = "NONE";
    result.selected_logical_identity = "NONE";
    result.selected_path_digest = "NONE";
    result.selected_probe_s = std::numeric_limits<double>::quiet_NaN();
    result.selected_probe_d = std::numeric_limits<double>::quiet_NaN();
    result.selected_validation_available = false;
    result.selected_path.wpnts.clear();

    const P3ShadowPlanningContext context = planner_.buildP3ShadowPlanningContext(
      ego, obstacles, false);
    if (!context.valid) {
      result.r3_runtime_total_us = elapsedUs(total_start);
      result.runtime_total_us = result.r3_runtime_total_us;
      return result;
    }
    cycle_cluster_ids_ = context.cluster_ids;

    const auto factor_start = Clock::now();
    const std::vector<P3R3K12SideGeometry> geometries = buildR3K12Geometry(
      ego, obstacles, context);
    const P3R3K12Selection selection = selectP3R3K12Factors(
      geometries, production_candidates);
    result.r3_runtime_factor_generation_us = elapsedUs(factor_start);
    result.r3_lateral_factor_count = selection.lateral_factor_count;
    result.r3_pair_priority_count = selection.pair_priority_count;

    std::set<std::string> validated_path_digests;
    std::optional<std::size_t> best_index;
    const auto consume_stream = [&](const std::string & stream_name,
      const std::vector<P3R3K12Factor> & factors, std::size_t quota) {
        std::size_t consumed = 0U;
        for (std::size_t stream_rank = 0U;
          stream_rank < factors.size() && consumed < quota; ++stream_rank)
        {
          const auto & factor = factors[stream_rank];
          const auto & domain = factor.go_left ? context.left : context.right;
          if (!domain.valid) {
            continue;
          }
          const std::array<double, 5> stations = stationsFor(
            parameters_, domain, context.outside_is_left, factor.entry_scale,
            factor.exit_scale, ego.d, factor.d_target, referenceSpacing());
          double minimum_length = std::numeric_limits<double>::quiet_NaN();
          if (!strictPositiveSegments(stations, minimum_length)) {
            continue;  // frozen construction guards do not consume K
          }

          const auto reconstruction_start = Clock::now();
          double reconstruction_us = 0.0;
          double validation_us = 0.0;
          const std::size_t generation_index = result.r3_constructed_candidate_count;
          P3ShadowCandidateTrace trace = buildCandidate(
            ego, obstacles, factor.go_left, context.outside_is_left,
            factor.d_target, factor.d_mid, factor.entry_scale, factor.exit_scale,
            stations, generation_index, reconstruction_us, validation_us, false);
          result.r3_runtime_reconstruction_us += elapsedUs(reconstruction_start);
          ++consumed;
          ++result.r3_constructed_candidate_count;
          if (stream_name == "LEXICOGRAPHIC") {
            ++result.r3_lexicographic_factor_count;
          } else {
            ++result.r3_coverage_factor_count;
          }

          P3R3SelectedFactorTrace factor_trace;
          factor_trace.rank_stream = stream_name;
          factor_trace.stream_rank = stream_rank;
          factor_trace.go_left = factor.go_left;
          factor_trace.d_target = factor.d_target;
          factor_trace.d_mid = factor.d_mid;
          factor_trace.entry_scale = factor.entry_scale;
          factor_trace.exit_scale = factor.exit_scale;
          factor_trace.target_source = factor.target_source;
          factor_trace.mid_source = factor.mid_source;
          factor_trace.lateral_factor_index = factor.lateral_factor_index;
          factor_trace.transition_index = factor.transition_index;
          factor_trace.exit_conflict_proxy = factor.exit_conflict_proxy;
          factor_trace.maximum_corridor_violation_m = factor.maximum_corridor_violation_m;
          factor_trace.sum_corridor_violation_m = factor.sum_corridor_violation_m;
          factor_trace.slope_excess = factor.slope_excess;
          factor_trace.curvature_proxy = factor.curvature_proxy;
          factor_trace.center_error = factor.center_error;
          factor_trace.minimum_clearance_m = factor.minimum_clearance_m;
          factor_trace.shape_energy = factor.shape_energy;
          factor_trace.constructed = true;
          factor_trace.path_digest = trace.path_digest;

          if (!validated_path_digests.insert(trace.path_digest).second) {
            factor_trace.path_digest_duplicate = true;
            const auto original = std::find_if(
              result.candidates.begin(), result.candidates.end(), [&trace](const auto & candidate) {
                return candidate.path_digest == trace.path_digest;
              });
            if (original != result.candidates.end()) {
              factor_trace.hard_valid = original->hard_valid;
              factor_trace.usable_valid = original->hard_valid &&
                !original->exit_reaches_next_obstacle &&
                original->ego_braking_distance_deficit_m <= kCandidateRankEpsilon;
              factor_trace.candidate_identity = original->candidate_identity;
            }
            ++result.r3_path_digest_duplicate_count;
            result.r3_selected_factors.push_back(std::move(factor_trace));
            continue;  // duplicate consumes K but not the logical exact-validator budget
          }

          const auto validation_start = Clock::now();
          validateReconstructedCandidate(ego, obstacles, trace, validation_us);
          result.r3_runtime_validation_us += elapsedUs(validation_start);
          if (trace.validator_executed) {
            ++result.r3_validator_call_count;
          }
          trace.mapping_source = kP3R3K12MethodName;
          trace.generator_stage = "R3_K12";
          trace.candidate_template = "DIRECT_FACTOR_" + stream_name;
          trace.source_cell = stream_name;
          trace.component_id = "DIRECT_FACTOR_POOL";
          trace.root_type = "DIRECT_D_MID_FACTOR";
          trace.probe_location_rule = "NOT_USED_DIRECT_FACTOR";
          trace.probe_anchor_rule = "NOT_USED_DIRECT_FACTOR";
          trace.r3_rank_stream = stream_name;
          trace.r3_target_source = factor.target_source;
          trace.r3_mid_source = factor.mid_source;
          trace.r3_lateral_factor_index = factor.lateral_factor_index;
          trace.r3_transition_index = factor.transition_index;
          const std::string side = factor.go_left ? "LEFT" : "RIGHT";
          trace.candidate_identity = "R3_K12_" + stream_name + "_" + side + "_" +
            trace.path_digest;
          trace.logical_identity = "R3_K12_" + stream_name + "_" + side + "_" +
            std::to_string(stream_rank);
          trace.returned_by_policy = true;

          factor_trace.validator_executed = trace.validator_executed;
          factor_trace.hard_valid = trace.hard_valid;
          factor_trace.usable_valid = trace.hard_valid &&
            !trace.exit_reaches_next_obstacle &&
            trace.ego_braking_distance_deficit_m <= kCandidateRankEpsilon;
          factor_trace.candidate_identity = trace.candidate_identity;
          if (trace.hard_valid) {
            ++result.r3_hard_valid_count;
            if (factor_trace.usable_valid) {
              ++result.r3_usable_valid_count;
            }
          }
          result.r3_selected_factors.push_back(std::move(factor_trace));
          result.candidates.push_back(std::move(trace));
        }
      };

    consume_stream("LEXICOGRAPHIC", selection.lexicographic, kP3R3K12LexicographicQuota);
    consume_stream("COVERAGE", selection.coverage, kP3R3K12CoverageQuota);

    if (result.r3_constructed_candidate_count > kP3R3K12CandidateBudget ||
      result.r3_validator_call_count > kP3R3K12CandidateBudget)
    {
      throw std::runtime_error("R3-K12 reconstructed-candidate/validator-call bound violated");
    }
    result.candidate_count = result.candidates.size();
    result.hard_validator_call_count = result.r3_validator_call_count;
    result.hard_valid_count = result.r3_hard_valid_count;

    std::vector<std::size_t> feasible_order;
    for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
      if (result.candidates[index].hard_valid) {
        feasible_order.push_back(index);
      }
    }
    std::stable_sort(
      feasible_order.begin(), feasible_order.end(),
      [&result](std::size_t first, std::size_t second) {
        return betterR3FrozenRank(result.candidates[first], result.candidates[second]);
      });
    for (std::size_t rank = 0U; rank < feasible_order.size(); ++rank) {
      result.candidates[feasible_order[rank]].final_rank = static_cast<int>(rank + 1U);
    }
    if (!feasible_order.empty()) {
      best_index = feasible_order.front();
    }

    if (best_index.has_value()) {
      auto & selected = result.candidates[*best_index];
      selected.selected = true;
      result.would_recover = true;
      result.selected_go_left = selected.go_left;
      result.selected_obstacle_ids = context.cluster_ids;
      const auto & selected_domain = selected.go_left ? context.left : context.right;
      result.selected_cluster_start_forward_m = selected_domain.cluster_start;
      result.selected_cluster_end_forward_m = selected_domain.cluster_end;
      result.selected_cluster_end_s = planner_.wrapS(ego.s + selected_domain.cluster_end);
      result.selected_source = kP3R3K12MethodName;
      result.selected_candidate_template = selected.candidate_template;
      result.selected_source_cell = selected.source_cell;
      result.selected_component_id = selected.component_id;
      result.selected_source_branch_regime = selected.source_branch_regime;
      result.selected_candidate_identity = selected.candidate_identity;
      result.selected_logical_identity = selected.logical_identity;
      result.selected_path_digest = selected.path_digest;
      result.selected_d_target = selected.d_target;
      result.selected_d_mid = selected.d_mid;
      result.selected_min_track_margin_m = selected.minimum_track_margin_m;
      result.selected_min_obstacle_margin_m = selected.minimum_obstacle_margin_m;
      result.selected_curvature_margin = selected.minimum_curvature_margin_radpm;
      result.selected_curvature_rate_margin =
        parameters_.maximum_curvature_rate_radpm2 - selected.peak_curvature_rate_radpm2;
      result.selected_slope_margin =
        parameters_.maximum_lateral_slope - selected.peak_lateral_slope;
      result.selected_min_speed_mps = selected.minimum_commanded_speed_mps;
      result.selected_max_speed_mps = selected.maximum_commanded_speed_mps;
      result.selected_validation = selected.validation;
      result.selected_validation_available = true;
      result.selected_path = selected.path;
      result.failure_classification = "NONE";
      result.r3_fallback_after_failure = "NONE";
    }

    if (planner_.activeResearchCycle() != nullptr) {
      result.research_all_candidates = result.candidates;
      result.research_constructed_total_actual = result.r3_constructed_candidate_count;
      result.research_validate_candidate_executed_total_actual = result.r3_validator_call_count;
      result.research_hard_valid_total_actual = result.r3_hard_valid_count;
    }
    result.r3_runtime_total_us = elapsedUs(total_start);
    result.runtime_total_us = result.r3_runtime_total_us;
    return result;
  }

  static void copyR3Audit(P3ShadowResult & destination, const P3ShadowResult & source)
  {
    destination.r3_invoked = source.r3_invoked;
    destination.r3_method_name = source.r3_method_name;
    destination.r3_method_sha256 = source.r3_method_sha256;
    destination.r3_lateral_factor_count = source.r3_lateral_factor_count;
    destination.r3_pair_priority_count = source.r3_pair_priority_count;
    destination.r3_lexicographic_factor_count = source.r3_lexicographic_factor_count;
    destination.r3_coverage_factor_count = source.r3_coverage_factor_count;
    destination.r3_constructed_candidate_count = source.r3_constructed_candidate_count;
    destination.r3_path_digest_duplicate_count = source.r3_path_digest_duplicate_count;
    destination.r3_validator_call_count = source.r3_validator_call_count;
    destination.r3_hard_valid_count = source.r3_hard_valid_count;
    destination.r3_usable_valid_count = source.r3_usable_valid_count;
    destination.r3_runtime_factor_generation_us = source.r3_runtime_factor_generation_us;
    destination.r3_runtime_reconstruction_us = source.r3_runtime_reconstruction_us;
    destination.r3_runtime_validation_us = source.r3_runtime_validation_us;
    destination.r3_runtime_total_us = source.r3_runtime_total_us;
    destination.r3_fallback_after_failure = source.r3_fallback_after_failure;
    destination.r3_selected_factors = source.r3_selected_factors;
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double kEpsilon = 1.0e-9;
  static constexpr std::size_t kFrozenCandidateCap = 16U;
  static constexpr std::size_t kM0ExtensionCandidateCap = 12U;
  static constexpr std::size_t kTotalCandidateCap = 24U;

  struct Interval
  {
    double lower{0.0};
    double upper{0.0};
  };

  struct CorridorSample
  {
    double station{0.0};
    double curvature{0.0};
    Interval track;
    std::vector<Interval> feasible;
    std::vector<int> active_obstacles;
  };

  struct BranchSample
  {
    double station{0.0};
    double lower{0.0};
    double upper{0.0};
    double center{0.0};
    double width{0.0};
    double curvature{0.0};
  };

  struct Corridor
  {
    std::vector<CorridorSample> samples;
    std::vector<BranchSample> branch;
    std::string branch_id;
    std::size_t branch_count{0U};
    bool connected{false};
  };

  struct Probe
  {
    double station{std::numeric_limits<double>::quiet_NaN()};
    double desired{std::numeric_limits<double>::quiet_NaN()};
    std::string location_rule;
    std::string anchor_rule;
  };

  struct QuinticSegment
  {
    double start{0.0};
    double end{0.0};
    std::array<double, 6> coefficients{};

    double evaluate(double station) const
    {
      const double t = std::clamp((station - start) / (end - start), 0.0, 1.0);
      double value = coefficients[5];
      for (int index = 4; index >= 0; --index) {
        value = value * t + coefficients[static_cast<std::size_t>(index)];
      }
      return value;
    }
  };

  struct KnotStates
  {
    std::array<double, 4> secants{};
    std::array<double, 5> derivatives{};
    std::array<double, 5> accelerations{};
    std::array<std::string, 3> branches{};
  };

  struct RootSolve
  {
    p3_analytic_solver::PolynomialRoots algebraic;
    std::vector<double> finite;
    std::vector<double> branch;
    std::vector<double> bounded;
    std::vector<double> accepted;
  };

  struct SideResult
  {
    bool domain_valid{false};
    bool go_left{false};
    P3ShadowSideDomain domain;
    Corridor corridor;
    Probe probe;
    std::size_t raw_root_count{0U};
    std::size_t finite_root_count{0U};
    std::size_t branch_root_count{0U};
    std::size_t bounded_root_count{0U};
    std::size_t accepted_root_count{0U};
    std::size_t validator_calls{0U};
    std::size_t hard_valid_count{0U};
    std::optional<std::size_t> best_index;
    std::string failure{"NO_ALGEBRAIC_ROOT"};
    double runtime_corridor_us{0.0};
    double runtime_root_solver_us{0.0};
    double runtime_reconstruction_us{0.0};
    double runtime_hard_validation_us{0.0};
    std::vector<P3ShadowCandidateTrace> candidates;
  };

  struct BranchRange
  {
    double lower{0.0};
    double upper{0.0};
    std::string id;
  };

  struct InactiveSolve
  {
    std::size_t raw_roots{0U};
    std::size_t finite_roots{0U};
    std::size_t branch_roots{0U};
    std::size_t bounded_roots{0U};
    std::vector<double> accepted;
  };

  struct M1Context
  {
    P3ShadowSideDomain domain;
    bool outside_is_left{false};
    Corridor corridor;
    BranchRange component;
    std::size_t component_index{0U};
    double boundary_inset{0.0};
    double branch_near{0.0};
    double branch_far{0.0};
    double entry_min{0.0};
    double entry_max{0.0};
    double entry_quarter{0.0};
    double exit_min{0.0};
    double exit_max{0.0};
    double exit_span{0.0};
  };

  struct ExtensionOutcome
  {
    // 이 계열이 만들 수 있는 후보 수. 자체 상한과 남은 총예산 중 작은 쪽으로, 호출부가 채운다.
    // 후보를 추가하는 함수들이 셋이라 인자로 흘리면 시그니처가 다 바뀌므로 결과에 싣는다.
    std::size_t candidate_cap{kM0ExtensionCandidateCap};
    std::vector<P3ShadowCandidateTrace> candidates;
    std::optional<std::size_t> best_index;
    std::size_t validator_calls{0U};
    std::size_t hard_valid_count{0U};
    std::size_t raw_root_count{0U};
    std::size_t finite_root_count{0U};
    std::size_t branch_root_count{0U};
    std::size_t bounded_root_count{0U};
    std::size_t accepted_root_count{0U};
    double runtime_root_solver_us{0.0};
    double runtime_reconstruction_us{0.0};
    double runtime_hard_validation_us{0.0};
  };

  std::vector<P3ShadowObstacleEnvelope> frozenR3Visible(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
  {
    constexpr double kFrozenVehicleHalfWidthM = 0.15;
    constexpr double kFrozenSafetyMarginM = 0.08;
    constexpr double kFrozenLookaheadM = 15.0;
    constexpr double kFrozenObstacleProjectionM =
      kFrozenVehicleHalfWidthM + kFrozenSafetyMarginM;
    std::vector<P3ShadowObstacleEnvelope> visible;
    for (const auto & obstacle : obstacles) {
      const double center = planner_.forwardDistance(ego.s, obstacle.s_center);
      double span = std::min(
        planner_.forwardDistance(obstacle.s_start, obstacle.s_end),
        planner_.forwardDistance(obstacle.s_end, obstacle.s_start));
      if (!(span > kEpsilon)) {
        span = std::max(0.05, std::abs(obstacle.size));
      }
      const double half = 0.5 * span;
      P3ShadowObstacleEnvelope expanded;
      expanded.id = obstacle.id;
      expanded.start = center - half;
      expanded.end = center + half;
      expanded.center = center;
      expanded.d_right =
        std::min(obstacle.d_right, obstacle.d_left) - kFrozenObstacleProjectionM;
      expanded.d_left =
        std::max(obstacle.d_right, obstacle.d_left) + kFrozenObstacleProjectionM;
      if (expanded.end >= 0.0 && expanded.start <= kFrozenLookaheadM) {
        visible.push_back(expanded);
      }
    }
    std::sort(visible.begin(), visible.end(), [](const auto & first, const auto & second) {
        return first.start < second.start;
      });
    return visible;
  }

  CorridorSample frozenR3CorridorSample(
    const EgoFrenetState & ego,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    double station) const
  {
    constexpr double kFrozenVehicleHalfWidthM = 0.15;
    constexpr double kFrozenWallSafetyMarginM = 0.04;
    constexpr double kFrozenFallbackTrackHalfWidthM = 1.5;
    const auto & reference = planner_.reference_.wpnts[
      planner_.nearestReferenceIndex(planner_.wrapS(ego.s + station))];
    const double left = reference.d_left > 0.05 ?
      reference.d_left : kFrozenFallbackTrackHalfWidthM;
    const double right = reference.d_right > 0.05 ?
      reference.d_right : kFrozenFallbackTrackHalfWidthM;
    CorridorSample sample;
    sample.station = station;
    sample.curvature = reference.kappa_radpm;
    // Preserve the frozen Python expression order exactly. Grouping the two margins changes
    // some binary64 corridor endpoints by one ulp and therefore changes exact factor dedup.
    sample.track = {
      -right + kFrozenVehicleHalfWidthM + kFrozenWallSafetyMarginM,
      left - kFrozenVehicleHalfWidthM - kFrozenWallSafetyMarginM};
    if (sample.track.upper > sample.track.lower + kEpsilon) {
      sample.feasible.push_back(sample.track);
    }
    for (const auto & obstacle : visible) {
      if (station + kEpsilon < obstacle.start || station - kEpsilon > obstacle.end) {
        continue;
      }
      sample.active_obstacles.push_back(obstacle.id);
      sample.feasible = subtractInterval(
        sample.feasible, {obstacle.d_right, obstacle.d_left});
    }
    return sample;
  }

  Corridor makeFrozenR3Corridor(
    const EgoFrenetState & ego,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    double cluster_start,
    double cluster_end,
    bool go_left,
    bool outside_is_left) const
  {
    constexpr double kFrozenPostApexFarM = 6.178529850015357;
    constexpr double kFrozenMaximumExitScale = 3.698773101198193;
    constexpr double kFrozenOutsideMultiplier = 0.4060036444074003;
    const double multiplier = go_left == outside_is_left ? kFrozenOutsideMultiplier : 1.0;
    const double horizon = cluster_end +
      kFrozenPostApexFarM * kFrozenMaximumExitScale * multiplier;
    std::vector<double> stations{
      0.0, cluster_start, cluster_end, 0.5 * (cluster_start + cluster_end)};
    for (const auto & obstacle : visible) {
      if (obstacle.end < -kEpsilon || obstacle.start > horizon + kEpsilon) {
        continue;
      }
      stations.push_back(std::max(0.0, obstacle.start));
      stations.push_back(std::clamp(obstacle.center, 0.0, horizon));
      stations.push_back(std::min(horizon, obstacle.end));
    }
    const std::size_t first = planner_.nextReferenceIndex(ego.s);
    for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
      const auto & waypoint =
        planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
      const double station = planner_.forwardDistance(ego.s, waypoint.s_m);
      if (station > horizon + kEpsilon) {
        break;
      }
      stations.push_back(station);
    }
    stations = uniqueSorted(std::move(stations));

    Corridor result;
    result.connected = true;
    result.branch_count = 1U;
    Interval previous{};
    bool have_previous = false;
    for (const double station : stations) {
      CorridorSample sample = frozenR3CorridorSample(ego, visible, station);
      result.branch_count = std::max(result.branch_count, sample.feasible.size());
      if (sample.feasible.empty()) {
        result.connected = false;
        result.samples.push_back(std::move(sample));
        continue;
      }
      const Interval selected = go_left ? sample.feasible.back() : sample.feasible.front();
      if (have_previous && !overlaps(previous, selected)) {
        result.connected = false;
      }
      previous = selected;
      have_previous = true;
      result.branch.push_back({
          sample.station, selected.lower, selected.upper,
          0.5 * (selected.lower + selected.upper), selected.upper - selected.lower,
          sample.curvature});
      result.samples.push_back(std::move(sample));
    }
    return result;
  }

  double frozenR3ReferenceSpacing() const
  {
    std::vector<double> spacing;
    for (std::size_t index = 1U; index < planner_.reference_.wpnts.size(); ++index) {
      spacing.push_back(
        planner_.reference_.wpnts[index].s_m - planner_.reference_.wpnts[index - 1U].s_m);
    }
    if (spacing.empty()) {
      return 0.25;
    }
    std::sort(spacing.begin(), spacing.end());
    const std::size_t middle = spacing.size() / 2U;
    return spacing.size() % 2U == 0U ?
           0.5 * (spacing[middle - 1U] + spacing[middle]) : spacing[middle];
  }

  std::vector<P3R3K12SideGeometry> buildR3K12Geometry(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowPlanningContext & context) const
  {
    constexpr double kFrozenFeatureHorizonM = 32.0;
    const auto visible = frozenR3Visible(ego, obstacles);
    const std::set<int> cluster_ids(context.cluster_ids.begin(), context.cluster_ids.end());
    std::vector<P3R3K12SideGeometry> output;
    for (const bool go_left : {false, true}) {
      const auto & domain = go_left ? context.left : context.right;
      if (!domain.valid) {
        continue;
      }
      const Corridor corridor = makeFrozenR3Corridor(
        ego, visible, domain.cluster_start, domain.cluster_end,
        go_left, context.outside_is_left);
      if (corridor.branch.empty()) {
        continue;
      }
      const auto ranges = connectedConstantRanges(corridor, domain);
      std::vector<const BranchSample *> span;
      for (const auto & sample : corridor.branch) {
        if (sample.station > domain.cluster_start + kEpsilon &&
          sample.station < domain.cluster_end - kEpsilon)
        {
          span.push_back(&sample);
        }
      }
      if (span.empty()) {
        span.push_back(&nearestBranchSample(
            corridor, 0.5 * (domain.cluster_start + domain.cluster_end)));
      }
      const BranchSample * bottleneck = *std::min_element(
        span.begin(), span.end(), [](const auto * first, const auto * second) {
          return std::tie(first->width, first->station) <
                 std::tie(second->width, second->station);
        });
      const CorridorSample midpoint_sample = frozenR3CorridorSample(
        ego, visible, 0.5 * (domain.cluster_start + domain.cluster_end));
      if (midpoint_sample.feasible.empty()) {
        continue;
      }
      const Interval midpoint_interval =
        go_left ? midpoint_sample.feasible.back() : midpoint_sample.feasible.front();

      std::set<double> sample_stations{
        0.0, std::max(0.0, domain.cluster_start), std::max(0.0, domain.cluster_end),
        std::max(0.0, 0.5 * (domain.cluster_start + domain.cluster_end))};
      std::vector<double> reference_stations;
      const std::size_t first = planner_.nextReferenceIndex(ego.s);
      for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
        const auto & waypoint =
          planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
        const double station = planner_.forwardDistance(ego.s, waypoint.s_m);
        if (station > kFrozenFeatureHorizonM + kEpsilon) {
          break;
        }
        sample_stations.insert(station);
        reference_stations.push_back(station);
      }
      for (const auto & obstacle : visible) {
        for (const double station : {obstacle.start, obstacle.center, obstacle.end}) {
          if (station >= -kEpsilon && station <= kFrozenFeatureHorizonM + kEpsilon) {
            sample_stations.insert(std::max(0.0, station));
          }
        }
      }

      P3R3K12SideGeometry geometry;
      geometry.go_left = go_left;
      geometry.outside_is_left = context.outside_is_left;
      geometry.ego_d = ego.d;
      geometry.ego_speed = ego.speed;
      geometry.cluster_start = domain.cluster_start;
      geometry.cluster_end = domain.cluster_end;
      geometry.domain_lower = std::min(domain.minimum_target, domain.maximum_target);
      geometry.domain_upper = std::max(domain.minimum_target, domain.maximum_target);
      geometry.reference_spacing_m = frozenR3ReferenceSpacing();
      for (const auto & range : ranges) {
        geometry.components.push_back({range.lower, range.upper});
      }
      std::vector<double> center_values{
        bottleneck->center, 0.5 * (midpoint_interval.lower + midpoint_interval.upper)};
      for (const double station : sample_stations) {
        const CorridorSample sample = frozenR3CorridorSample(ego, visible, station);
        if (sample.feasible.empty()) {
          continue;
        }
        const Interval selected = go_left ? sample.feasible.back() : sample.feasible.front();
        if (station >= domain.cluster_start - kEpsilon &&
          station <= domain.cluster_end + kEpsilon)
        {
          center_values.push_back(0.5 * (selected.lower + selected.upper));
        }
      }
      std::sort(center_values.begin(), center_values.end());
      center_values.erase(
        std::unique(center_values.begin(), center_values.end()), center_values.end());
      geometry.center_values = std::move(center_values);
      for (const double station : reference_stations) {
        const CorridorSample sample = frozenR3CorridorSample(ego, visible, station);
        P3R3K12CorridorSample row;
        row.station = station;
        if (!sample.feasible.empty()) {
          const Interval selected = go_left ? sample.feasible.back() : sample.feasible.front();
          row.lower = selected.lower;
          row.upper = selected.upper;
          row.center = 0.5 * (selected.lower + selected.upper);
          row.width = selected.upper - selected.lower;
          row.finite = std::isfinite(row.lower) && std::isfinite(row.upper);
        }
        row.later_obstacle_active = std::any_of(
          sample.active_obstacles.begin(), sample.active_obstacles.end(),
          [&cluster_ids](int id) {return cluster_ids.find(id) == cluster_ids.end();});
        geometry.reference_samples.push_back(row);
      }
      output.push_back(std::move(geometry));
    }
    return output;
  }

  static double elapsedUs(const Clock::time_point & start)
  {
    return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
  }

  struct ResearchTimer
  {
    explicit ResearchTimer(double * destination)
    : total(destination), start(destination == nullptr ? Clock::time_point() : Clock::now()) {}

    double * total{nullptr};
    Clock::time_point start;

    ~ResearchTimer()
    {
      if (total != nullptr) {
        *total += elapsedUs(start);
      }
    }
  };

  static std::uint64_t fnvAppend(std::uint64_t hash, const void * data, std::size_t size)
  {
    const auto * bytes = static_cast<const unsigned char *>(data);
    for (std::size_t index = 0U; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ULL;
    }
    return hash;
  }

  static std::string hexHash(std::uint64_t hash)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
  }

  static std::string fnvHex(const std::string & value)
  {
    return hexHash(fnvAppend(1469598103934665603ULL, value.data(), value.size()));
  }

  // 기동의 5개 지점. 오프셋은 {ego_d, target, middle, target, 0} 순으로 걸린다.
  //
  // 🔴 2026-08-17: 자차가 클러스터에 이미 닿았을 때를 표현할 수 있게 한다.
  //
  // 종전에는 진입 램프 길이가 cluster_start에 **비례**했다:
  //     entry_length = cluster_start * pre_apex.front() * entry / lookahead
  // cluster_start가 음수(자차가 이미 클러스터 앞단을 지나 옆에 나란히 있음)면 entry_length도
  // 음수가 되어 첫 구간 길이가 음수가 되고, strictPositiveSegments가 후보를 전부 기각했다.
  // 작은 양수여도 램프가 지나치게 짧아져 곡률 한계에서 전멸했다. 어느 쪽이든 **표현 가능한
  // 형상이 아예 없어서**, 통과 가능한 상황에서도 안전정지로 떨어졌다.
  //
  // 실해 — 2026-08-17 01:55 백. 잔여 정지 5건 전부 이 자리에서 죽었다:
  //     cluster_start = -0.541 / -0.151 / -0.018 / +0.174 / +0.275 / +0.756 / +1.095
  // 안전정지는 스스로 해제 조건(유효 회피 8사이클)을 막으므로 2.3 s씩 갇혔다.
  //
  // 물리적으로 옳은 답은 둘 중 하나다:
  //   지금 d가 이미 장애물을 비켜 있다 → "이 오프셋을 클러스터 끝까지 유지하고 빠져나가기"
  //   지금 d가 안 비켜 있다           → 전진으로는 해결 불가. 정지가 맞다.
  // 종전에는 이 둘을 구분하지 못하고 **무조건 두 번째로** 처리했다.
  //
  // 그래서 진입 램프가 클러스터 **앞에** 들어갈 자리가 없으면, 램프를 자차에서 시작시키고
  // 목표 도달 지점을 "물리적으로 가능한 가장 이른 곳"으로 잡는다:
  //     required = |target - ego_d| / maximum_lateral_slope
  // 새 상수는 없다 — 이미 있는 기울기 한계가 정한다. 그 지점이 클러스터 안이면 그 구간에서
  // 경로는 아직 목표에 못 미치는데, 그건 하드 검증의 장애물 충돌 검사가 판정한다. 비켜
  // 있으면 통과하고, 아니면 기각된다 — 위 두 경우가 정확히 갈린다.
  //
  // minimum_station_gap_m은 기준선 샘플 간격이다. 두 지점이 그보다 가까우면 발행 경로에
  // 서로 다른 점으로 나타나지 않아 구분이 의미를 잃는다. 튜닝값이 아니라 기준선의 성질이다.
  static std::array<double, 5> stationsFor(
    const RacelineSplineParameters & parameters,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    double entry,
    double exit,
    double ego_d,
    double target,
    double minimum_station_gap_m)
  {
    const double outside_multiplier = domain.go_left == outside_is_left ?
      parameters.outside_line_transition_scale : 1.0;
    const double exit_length = parameters.post_apex_distances_m.back() *
      parameters.cappedCombinedExitScale(exit * outside_multiplier);

    const double slope = std::max(parameters.maximum_lateral_slope, kEpsilon);
    const double required_transition_m = std::abs(target - ego_d) / slope;
    double apex = domain.cluster_start;
    double start = apex - apex *
      parameters.pre_apex_distances_m.front() * entry / parameters.detection_lookahead_m;
    if (!(apex >= required_transition_m)) {
      apex = std::max(required_transition_m, minimum_station_gap_m);
      start = 0.0;                                  // 램프가 자차에서 시작한다
    }
    return {
      start,
      apex,
      0.5 * (apex + domain.cluster_end),
      domain.cluster_end,
      domain.cluster_end + exit_length};
  }

  static std::vector<double> uniqueSorted(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    values.erase(
      std::unique(values.begin(), values.end(), [](double first, double second) {
        return std::abs(first - second) <= 1.0e-12;
      }), values.end());
    return values;
  }

  static std::vector<Interval> subtractInterval(
    const std::vector<Interval> & source, const Interval & forbidden)
  {
    std::vector<Interval> result;
    for (const auto & interval : source) {
      if (forbidden.upper <= interval.lower + kEpsilon ||
        forbidden.lower >= interval.upper - kEpsilon)
      {
        result.push_back(interval);
        continue;
      }
      if (forbidden.lower > interval.lower + kEpsilon) {
        result.push_back({interval.lower, std::min(interval.upper, forbidden.lower)});
      }
      if (forbidden.upper < interval.upper - kEpsilon) {
        result.push_back({std::max(interval.lower, forbidden.upper), interval.upper});
      }
    }
    return result;
  }

  static bool overlaps(const Interval & first, const Interval & second)
  {
    return std::min(first.upper, second.upper) >=
           std::max(first.lower, second.lower) - kEpsilon;
  }

  static QuinticSegment quinticHermite(
    double start, double end,
    double d0, double v0, double a0,
    double d1, double v1, double a1)
  {
    const double h = end - start;
    if (!(h > kEpsilon)) {
      throw std::runtime_error("non-positive quintic-Hermite segment");
    }
    QuinticSegment segment;
    segment.start = start;
    segment.end = end;
    auto & c = segment.coefficients;
    c[0] = d0;
    c[1] = h * v0;
    c[2] = 0.5 * h * h * a0;
    const double r0 = d1 - c[0] - c[1] - c[2];
    const double r1 = h * v1 - c[1] - 2.0 * c[2];
    const double r2 = h * h * a1 - 2.0 * c[2];
    c[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    c[4] = -15.0 * r0 + 7.0 * r1 - r2;
    c[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
    return segment;
  }

  static std::vector<QuinticSegment> makeC2Profile(
    const std::array<double, 5> & stations,
    const std::array<double, 5> & offsets)
  {
    std::array<double, 5> derivative{};
    std::array<double, 5> acceleration{};
    for (std::size_t index = 1U; index + 1U < stations.size(); ++index) {
      const double h_previous = stations[index] - stations[index - 1U];
      const double h_next = stations[index + 1U] - stations[index];
      const double slope_previous = (offsets[index] - offsets[index - 1U]) / h_previous;
      const double slope_next = (offsets[index + 1U] - offsets[index]) / h_next;
      derivative[index] = p3_analytic_solver::sourceHarmonicDerivative(
        h_previous, h_next, slope_previous, slope_next);
      acceleration[index] = 2.0 * (slope_next - slope_previous) /
        (h_previous + h_next);
    }
    std::vector<QuinticSegment> segments;
    segments.reserve(4U);
    for (std::size_t index = 0U; index + 1U < stations.size(); ++index) {
      segments.push_back(quinticHermite(
          stations[index], stations[index + 1U], offsets[index], derivative[index],
          acceleration[index], offsets[index + 1U], derivative[index + 1U],
          acceleration[index + 1U]));
    }
    return segments;
  }

  static double evaluateProfile(
    const std::vector<QuinticSegment> & segments, double station, double ego_d)
  {
    if (station <= segments.front().start) {
      return ego_d;
    }
    for (const auto & segment : segments) {
      if (station <= segment.end) {
        return segment.evaluate(station);
      }
    }
    return 0.0;
  }

  static KnotStates sourceRuleStates(
    const std::array<double, 5> & stations,
    const std::array<double, 5> & offsets)
  {
    KnotStates result;
    for (std::size_t index = 0U; index < 4U; ++index) {
      result.secants[index] = (offsets[index + 1U] - offsets[index]) /
        (stations[index + 1U] - stations[index]);
    }
    for (std::size_t index = 1U; index < 4U; ++index) {
      const double h_left = stations[index] - stations[index - 1U];
      const double h_right = stations[index + 1U] - stations[index];
      const double left = result.secants[index - 1U];
      const double right = result.secants[index];
      result.branches[index - 1U] = p3_analytic_solver::sourceBranch(left, right);
      result.derivatives[index] = p3_analytic_solver::sourceHarmonicDerivative(
        h_left, h_right, left, right);
      result.accelerations[index] = 2.0 * (right - left) / (h_left + h_right);
    }
    return result;
  }

  static std::string sourceBranchRegime(
    const std::array<double, 5> & stations, double ego_d, double target, double middle)
  {
    const auto states = sourceRuleStates(stations, {ego_d, target, middle, target, 0.0});
    return states.branches[0] + "__" + states.branches[1] + "__" + states.branches[2];
  }

  // Which wall an entry-scale rejection hit, so the bisection knows which way to move.
  //
  // 진입 램프 길이는 entry에 비례하므로(stationsFor), 두 실패군은 서로 반대 방향을 가리킨다.
  //   램프가 짧아서 나는 실패 — 곡률·곡률변화율·횡기울기 초과 → 더 긴 램프가 필요
  //   램프가 길어서 일찍 시작해 나는 실패 — 트랙 경계·발자국 침범 → 더 짧은 램프가 필요
  // 장애물 충돌·진입 불연속 등은 exit이나 목표 오프셋에서 오므로 방향을 알려주지 않는다.
  // 그때는 kUnknown을 돌려주고 이분법을 중단한다 — 잘못된 방향으로 계속 좁히느니
  // 종전과 동일하게 실패하는 편이 안전하다.
  enum class EntrySteer
  {
    kUnknown,
    kNeedsLongerRamp,
    kNeedsShorterRamp,
  };

  static EntrySteer classifyEntrySteer(const std::string & rejection_reason)
  {
    if (rejection_reason.find("maximum_curvature_radpm") != std::string::npos ||
      rejection_reason.find("control steering curvature") != std::string::npos ||
      rejection_reason.find("maximum_curvature_rate_radpm2") != std::string::npos ||
      rejection_reason.find("maximum_lateral_slope") != std::string::npos)
    {
      return EntrySteer::kNeedsLongerRamp;
    }
    if (rejection_reason.find("footprint_track_bound") != std::string::npos ||
      rejection_reason.find("track bounds") != std::string::npos)
    {
      return EntrySteer::kNeedsShorterRamp;
    }
    return EntrySteer::kUnknown;
  }

  static bool strictPositiveSegments(
    const std::array<double, 5> & stations, double & minimum_length)
  {
    minimum_length = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < stations.size(); ++index) {
      const double length = stations[index + 1U] - stations[index];
      minimum_length = std::min(minimum_length, length);
      if (!std::isfinite(length) || !(length > kEpsilon)) {
        return false;
      }
    }
    return true;
  }

  static double segmentD1(const QuinticSegment & segment, double t)
  {
    const auto & c = segment.coefficients;
    const double h = segment.end - segment.start;
    return (c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t * t +
           4.0 * c[4] * t * t * t + 5.0 * c[5] * t * t * t * t) / h;
  }

  static double linearSampleWithoutSideDerivative(
    const std::array<double, 5> & stations, double ego_d, double target,
    double middle, std::size_t segment, double normalized_t)
  {
    const std::array<double, 5> offsets{ego_d, target, middle, target, 0.0};
    KnotStates states = sourceRuleStates(stations, offsets);
    states.derivatives.fill(0.0);
    std::vector<QuinticSegment> segments;
    for (std::size_t index = 0U; index < 4U; ++index) {
      segments.push_back(quinticHermite(
          stations[index], stations[index + 1U], offsets[index], states.derivatives[index],
          states.accelerations[index], offsets[index + 1U], states.derivatives[index + 1U],
          states.accelerations[index + 1U]));
    }
    const double station = stations[segment] + normalized_t *
      (stations[segment + 1U] - stations[segment]);
    return segments[segment].evaluate(station);
  }

  static double sideDerivativeWeight(
    const std::array<double, 5> & stations, std::size_t segment, double t)
  {
    const double h = stations[segment + 1U] - stations[segment];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    if (segment == 0U || segment == 2U) {
      return h * (-4.0 * t3 + 7.0 * t4 - 3.0 * t5);
    }
    return h * (t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5);
  }

  static bool branchCombinationMatches(
    const std::array<double, 5> & stations, double ego_d, double target,
    double middle, const std::array<std::string, 3> & assumed)
  {
    return sourceRuleStates(
      stations, {ego_d, target, middle, target, 0.0}).branches == assumed;
  }

  static p3_analytic_solver::ActiveBranch activeBranch(const std::string & name)
  {
    return name == "SAME_SIGN_POSITIVE" ?
           p3_analytic_solver::ActiveBranch::SameSignPositive :
           p3_analytic_solver::ActiveBranch::SameSignNegative;
  }

  Corridor makeCorridor(
    const EgoFrenetState & ego,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    double cluster_start, double cluster_end, bool go_left, bool outside_is_left) const
  {
    PlanningResearchCycle * research_cycle = planner_.activeResearchCycle();
    const auto research_start = research_cycle == nullptr ? Clock::time_point() : Clock::now();
    Corridor result;
    std::vector<double> stations{
      0.0, cluster_start, cluster_end, 0.5 * (cluster_start + cluster_end)};
    double horizon = cluster_end + parameters_.post_apex_distances_m.back() *
      parameters_.cappedCombinedExitScale(
      *std::max_element(
        parameters_.transition_distance_scales.begin(),
        parameters_.transition_distance_scales.end()) *
      (go_left == outside_is_left ?
      parameters_.outside_line_transition_scale : 1.0));
    for (const auto & obstacle : visible) {
      if (obstacle.end < -kEpsilon || obstacle.start > horizon + kEpsilon) {
        continue;
      }
      stations.push_back(std::max(0.0, obstacle.start));
      stations.push_back(std::clamp(obstacle.center, 0.0, horizon));
      stations.push_back(std::min(horizon, obstacle.end));
    }
    const std::size_t first = planner_.nextReferenceIndex(ego.s);
    for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
      const auto & waypoint =
        planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
      const double station = planner_.forwardDistance(ego.s, waypoint.s_m);
      if (station > horizon + kEpsilon) {
        break;
      }
      stations.push_back(station);
    }
    stations = uniqueSorted(std::move(stations));

    const double projection = parameters_.vehicle_half_width_m +
      parameters_.wall_safety_margin_m;
    result.samples.reserve(stations.size());
    for (const double station : stations) {
      const auto & reference = planner_.reference_.wpnts[
        planner_.nearestReferenceIndex(planner_.wrapS(ego.s + station))];
      const double left = reference.d_left > 0.05 ?
        reference.d_left : parameters_.fallback_track_half_width_m;
      const double right = reference.d_right > 0.05 ?
        reference.d_right : parameters_.fallback_track_half_width_m;
      CorridorSample sample;
      sample.station = station;
      sample.curvature = reference.kappa_radpm;
      sample.track = {-right + projection, left - projection};
      if (sample.track.upper > sample.track.lower + kEpsilon) {
        sample.feasible.push_back(sample.track);
      }
      for (const auto & obstacle : visible) {
        if (station + kEpsilon < obstacle.start || station - kEpsilon > obstacle.end) {
          continue;
        }
        sample.active_obstacles.push_back(obstacle.id);
        sample.feasible = subtractInterval(
          sample.feasible, {obstacle.d_right, obstacle.d_left});
      }
      result.samples.push_back(std::move(sample));
    }

    result.branch_count = 1U;
    result.connected = true;
    Interval previous{};
    bool have_previous = false;
    for (const auto & sample : result.samples) {
      result.branch_count = std::max(result.branch_count, sample.feasible.size());
      if (sample.feasible.empty()) {
        result.connected = false;
        continue;
      }
      const Interval selected = go_left ? sample.feasible.back() : sample.feasible.front();
      if (have_previous && !overlaps(previous, selected)) {
        result.connected = false;
      }
      previous = selected;
      have_previous = true;
      result.branch.push_back({
          sample.station, selected.lower, selected.upper,
          0.5 * (selected.lower + selected.upper), selected.upper - selected.lower,
          sample.curvature});
    }
    std::ostringstream digest;
    digest << std::setprecision(17) << (go_left ? "L" : "R");
    for (const auto & sample : result.branch) {
      digest << ':' << sample.station << ',' << sample.lower << ',' << sample.upper;
    }
    result.branch_id = fnvHex(digest.str());
    if (research_cycle != nullptr) {
      research_cycle->runtime_corridor_us += elapsedUs(research_start);
    }
    return result;
  }

  static const BranchSample & nearestBranchSample(
    const Corridor & corridor, double station)
  {
    return *std::min_element(
      corridor.branch.begin(), corridor.branch.end(),
      [station](const auto & first, const auto & second) {
        return std::abs(first.station - station) < std::abs(second.station - station);
      });
  }

  Probe chooseProbe(
    const Corridor & corridor, double cluster_start, double cluster_end,
    double target, const std::string & policy = "CURVATURE_CONTINUITY") const
  {
    PlanningResearchCycle * research_cycle = planner_.activeResearchCycle();
    const auto research_start = research_cycle == nullptr ? Clock::time_point() : Clock::now();
    std::vector<const BranchSample *> span;
    for (const auto & sample : corridor.branch) {
      if (sample.station > cluster_start + kEpsilon &&
        sample.station < cluster_end - kEpsilon)
      {
        span.push_back(&sample);
      }
    }
    if (span.empty()) {
      span.push_back(&nearestBranchSample(corridor, 0.5 * (cluster_start + cluster_end)));
    }
    const BranchSample * selected = span.front();
    Probe probe;
    if (policy == "BOTTLENECK_CENTER") {
      selected = *std::min_element(span.begin(), span.end(),
          [](const auto * first, const auto * second) {
            return std::tie(first->width, first->station) <
                   std::tie(second->width, second->station);
          });
      probe.location_rule = "CORRIDOR_BOTTLENECK";
      probe.anchor_rule = "CORRIDOR_CENTER";
    } else if (policy == "MAX_DISPLACEMENT_CENTER") {
      selected = *std::max_element(
        span.begin(), span.end(), [target](const auto * first, const auto * second) {
          return std::make_tuple(std::abs(first->center - target), first->station) <
                 std::make_tuple(std::abs(second->center - target), second->station);
        });
      probe.location_rule = "MAX_CENTER_DISPLACEMENT";
      probe.anchor_rule = "MAX_CLEARANCE_ANCHOR";
    } else if (policy == "CURVATURE_CONTINUITY") {
      selected = *std::max_element(
        span.begin(), span.end(), [](const auto * first, const auto * second) {
          return std::make_tuple(std::abs(first->curvature), first->station) <
                 std::make_tuple(std::abs(second->curvature), second->station);
        });
      probe.location_rule = "CURVATURE_CORRIDOR_CRITICAL";
      probe.anchor_rule = "CONTINUITY_BIASED_SAFE_ANCHOR";
    } else {
      throw std::runtime_error("unknown P3 mapping policy");
    }
    probe.station = selected->station;
    if (policy == "CURVATURE_CONTINUITY") {
      const double lower = selected->lower + parameters_.safety_margin_m;
      const double upper = selected->upper - parameters_.safety_margin_m;
      probe.desired = lower <= upper ? std::clamp(target, lower, upper) : selected->center;
    } else {
      probe.desired = selected->center;
    }
    if (research_cycle != nullptr) {
      research_cycle->runtime_probe_anchor_us += elapsedUs(research_start);
    }
    return probe;
  }

  double existingTargetAnchor(double minimum_target, double maximum_target) const
  {
    const int count = std::max(1, parameters_.target_d_candidate_count);
    const double low = std::min(minimum_target, maximum_target);
    const double high = std::max(minimum_target, maximum_target);
    std::vector<double> candidates;
    for (int index = 0; index < count; ++index) {
      const double ratio = count == 1 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(count - 1);
      candidates.push_back(low + ratio * (high - low));
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](double first, double second) {
        return std::abs(first) < std::abs(second);
      });
    return candidates.front();
  }

  RootSolve solvePosition(
    const std::array<double, 5> & stations, double ego_d, double target,
    double station, double desired, double lower, double upper) const
  {
    PlanningResearchCycle * research_cycle = planner_.activeResearchCycle();
    const auto research_start = research_cycle == nullptr ? Clock::time_point() : Clock::now();
    RootSolve result;
    std::size_t segment = 0U;
    while (segment + 1U < 4U && station > stations[segment + 1U]) {
      ++segment;
    }
    const double h = stations[segment + 1U] - stations[segment];
    const double normalized_t = std::clamp(
      (station - stations[segment]) / h, 0.0, 1.0);
    const bool left = target > 0.0;
    const std::array<std::string, 3> assumed{
      left ? "SAME_SIGN_POSITIVE" : "SAME_SIGN_NEGATIVE",
      "SIGN_CHANGE",
      left ? "SAME_SIGN_NEGATIVE" : "SAME_SIGN_POSITIVE"};
    const bool entry = segment <= 1U;
    const auto active = activeBranch(entry ? assumed[0] : assumed[2]);
    const auto map = entry ? p3_analytic_solver::makeEntryMap(
      stations[1] - stations[0], stations[2] - stations[1], ego_d, target, active) :
      p3_analytic_solver::makeExitMap(
      stations[3] - stations[2], stations[4] - stations[3], target, 0.0, active);
    const double intercept = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 0.0, segment, normalized_t);
    const double slope = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 1.0, segment, normalized_t) - intercept;
    const double weight = sideDerivativeWeight(stations, segment, normalized_t);
    const auto polynomial = p3_analytic_solver::samplePolynomial(
      slope, intercept, weight, map, desired);
    result.algebraic = p3_analytic_solver::solvePolynomialStable(polynomial);
    for (const double root : result.algebraic.raw_roots) {
      if (!std::isfinite(root)) {
        continue;
      }
      result.finite.push_back(root);
      if (!branchCombinationMatches(stations, ego_d, target, root, assumed)) {
        continue;
      }
      result.branch.push_back(root);
      if (root < lower - kEpsilon || root > upper + kEpsilon) {
        continue;
      }
      result.bounded.push_back(root);
      const auto segments = makeC2Profile(
        stations, {ego_d, target, root, target, 0.0});
      const double residual = evaluateProfile(segments, station, ego_d) - desired;
      if (std::abs(residual) <= 1.0e-12 * std::max(1.0, std::abs(desired))) {
        result.accepted.push_back(root);
      }
    }
    p3_analytic_solver::sortAndDeduplicate(result.finite);
    p3_analytic_solver::sortAndDeduplicate(result.branch);
    p3_analytic_solver::sortAndDeduplicate(result.bounded);
    p3_analytic_solver::sortAndDeduplicate(result.accepted);
    if (research_cycle != nullptr) {
      research_cycle->runtime_root_solve_us += elapsedUs(research_start);
    }
    return result;
  }

  double peakSlope(
    const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path) const
  {
    double peak = 0.0;
    if (path.wpnts.size() < 2U) {
      return peak;
    }
    double previous_forward = planner_.forwardDistance(ego.s, path.wpnts.front().s_m);
    double previous_d = path.wpnts.front().d_m;
    for (std::size_t index = 1U; index < path.wpnts.size(); ++index) {
      const double forward = planner_.forwardDistance(ego.s, path.wpnts[index].s_m);
      const double ds = forward - previous_forward;
      if (ds > kEpsilon) {
        peak = std::max(peak, std::abs(path.wpnts[index].d_m - previous_d) / ds);
      }
      previous_forward = forward;
      previous_d = path.wpnts[index].d_m;
    }
    return peak;
  }

  P3ShadowCandidateTrace buildCandidate(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    bool go_left, bool outside_is_left, double target, double middle,
    double entry_scale, double exit_scale, const std::array<double, 5> & stations,
    std::size_t generation, double & reconstruction_us, double & hard_validation_us,
    bool run_exact_validation = true) const
  {
    const auto reconstruction_start = Clock::now();
    PlanningResearchCycle * research_cycle = planner_.activeResearchCycle();
    const double geometry_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_geometry_recompute_us;
    const double velocity_before = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_velocity_shaping_us;
    const auto segments = makeC2Profile(
      stations, {ego.d, target, middle, target, 0.0});
    (void)outside_is_left;
    const double tail = std::max(
      parameters_.post_merge_lookahead_m,
      std::abs(ego.speed) * parameters_.post_merge_min_time_sec);
    const double path_end = stations.back() + tail;
    const std::size_t first = planner_.nextReferenceIndex(ego.s);
    f110_msgs::msg::WpntArray path;
    double confirmed_critical_speed_mps = std::numeric_limits<double>::quiet_NaN();
    path.header = planner_.reference_.header;
    for (std::size_t count = 0U; count < planner_.reference_.wpnts.size(); ++count) {
      const auto & global =
        planner_.reference_.wpnts[(first + count) % planner_.reference_.wpnts.size()];
      const double forward = planner_.forwardDistance(ego.s, global.s_m);
      if (forward > path_end + kEpsilon) {
        break;
      }
      auto waypoint = global;
      waypoint.id = static_cast<std::int32_t>(path.wpnts.size());
      waypoint.d_m = evaluateProfile(segments, forward, ego.d);
      waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
      waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
      path.wpnts.push_back(waypoint);
    }
    const double spline_reconstruction_us = elapsedUs(reconstruction_start);
    if (research_cycle != nullptr) {
      research_cycle->runtime_spline_reconstruction_us += spline_reconstruction_us;
    }
    if (path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points)) {
      confirmed_critical_speed_mps = planner_.finalizeP3ShadowPath(
        path, ego, obstacles, stations);
    }
    std::optional<RacelineSplinePlanner::FootprintTrackBoundSample> waypoint0_sample;
    if (research_cycle != nullptr && !path.wpnts.empty()) {
      waypoint0_sample = planner_.measureFootprintTrackBound(path.wpnts.front(), 0U);
    }
    reconstruction_us += elapsedUs(reconstruction_start);

    P3ShadowPathEvaluation evaluation;
    if (run_exact_validation &&
      path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points))
    {
      const auto validation_start = Clock::now();
      // 후보 검증의 장애물 범위는 이 기동이 책임지는 구간(클러스터 끝 + post_merge_lookahead)
      // 까지다. `generateP3Candidates`가 P0 선택 경로에서 이미 같은 horizon으로 재검증하고
      // 있었는데(2026-08-15 run18: horizon 없이는 12 m 밖 꼬리 충돌로 앞 장애물의 회피
      // 후보가 전멸 → kNoSafePath → 영구 크립), 정작 P3 자신의 후보 인증서는 horizon 없이
      // 만들어져 두 경로의 판정이 갈렸다. 2026-08-16 백에서 P3는 s=31.7 장애물에 대해
      // 3 m 이내 245 콜백 전부 NO_HARD_VALID_M1_CANDIDATE였고, 같은 순간 P0의 plan()은
      // 6개 중 3개를 feasible로 통과시켰다. 트랙 경계·기하 검사는 여전히 경로 전체다.
      // 다음 클러스터 앞에서 자른다 — 근거는 RacelineSplinePlanner::maneuverScopeEnd 주석.
      const std::optional<double> collision_horizon(
        planner_.maneuverScopeEnd(ego, obstacles, cycle_cluster_ids_, stations[3]));
      evaluation = planner_.validateP3ShadowPath(ego, path, obstacles, 1.0, collision_horizon);
      hard_validation_us += elapsedUs(validation_start);
    } else if (path.wpnts.size() < static_cast<std::size_t>(parameters_.minimum_path_points)) {
      evaluation.rejection_reason = "spline segment has too few global race-line samples";
    }

    P3ShadowCandidateTrace trace;
    trace.generation_index = generation;
    trace.go_left = go_left;
    trace.entry_scale = entry_scale;
    trace.exit_scale = exit_scale;
    trace.d_target = target;
    trace.d_mid = middle;
    trace.knot_stations = stations;
    trace.point_count = path.wpnts.size();
    if (waypoint0_sample.has_value()) {
      trace.waypoint0_s_m = waypoint0_sample->waypoint_s_m;
      trace.waypoint0_d_m = path.wpnts.front().d_m;
      trace.waypoint0_x_m = waypoint0_sample->waypoint_x_m;
      trace.waypoint0_y_m = waypoint0_sample->waypoint_y_m;
      trace.waypoint0_yaw_rad = waypoint0_sample->waypoint_yaw_rad;
      trace.waypoint0_center_track_margin_m = waypoint0_sample->centerline_clearance_m;
      trace.waypoint0_footprint_track_margin_m = waypoint0_sample->footprint_clearance_m;
      trace.waypoint0_footprint_invalid = waypoint0_sample->invalid;
    }
    trace.validator_executed = run_exact_validation &&
      path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points);
    trace.hard_valid = evaluation.hard_valid;
    trace.minimum_normalized_safety_slack = evaluation.minimum_normalized_safety_slack;
    trace.minimum_track_margin_m = evaluation.minimum_track_margin_m;
    trace.minimum_obstacle_margin_m = evaluation.minimum_obstacle_margin_m;
    trace.peak_curvature_radpm = evaluation.peak_curvature_radpm;
    trace.minimum_curvature_margin_radpm = evaluation.minimum_curvature_margin_radpm;
    trace.peak_curvature_rate_radpm2 = evaluation.peak_curvature_rate_radpm2;
    trace.peak_lateral_slope = peakSlope(ego, path);
    trace.lateral_slope_margin = parameters_.maximum_lateral_slope - trace.peak_lateral_slope;
    trace.curvature_rate_margin_radpm2 =
      parameters_.maximum_curvature_rate_radpm2 - trace.peak_curvature_rate_radpm2;
    trace.velocity_loss = evaluation.velocity_loss;
    trace.global_path_deviation_m = evaluation.global_path_deviation_m;
    trace.ego_braking_distance_deficit_m = evaluation.ego_braking_distance_deficit_m;
    trace.minimum_commanded_speed_mps = std::numeric_limits<double>::infinity();
    trace.maximum_commanded_speed_mps = 0.0;
    trace.confirmed_critical_speed_mps = confirmed_critical_speed_mps;
    trace.confirmed_speed_hold_start_forward_m = stations[1];
    trace.confirmed_speed_hold_end_forward_m =
      parameters_.confirmedSpeedHoldEndForwardM(stations[3]);
    for (const auto & waypoint : path.wpnts) {
      trace.minimum_commanded_speed_mps = std::min(
        trace.minimum_commanded_speed_mps, waypoint.vx_mps);
      trace.maximum_commanded_speed_mps = std::max(
        trace.maximum_commanded_speed_mps, waypoint.vx_mps);
    }
    trace.rejection_reason = evaluation.rejection_reason;
    trace.exit_reaches_next_obstacle = exitReachesNextObstacle(
      ego, path, obstacles, stations[3], stations[4]);
    trace.validation = evaluation;
    trace.all_observed_violation_flags = evaluation.all_observed_violation_flags;
    trace.path_digest = pathDigest(path);
    trace.source_branch_regime = sourceBranchRegime(stations, ego.d, target, middle);
    trace.runtime_spline_reconstruction_us = spline_reconstruction_us;
    trace.runtime_geometry_recompute_us = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_geometry_recompute_us - geometry_before;
    trace.runtime_velocity_shaping_us = research_cycle == nullptr ? 0.0 :
      research_cycle->runtime_velocity_shaping_us - velocity_before;
    trace.runtime_candidate_measurement_us = evaluation.runtime_candidate_measurement_us;
    trace.runtime_hard_validation_us = evaluation.runtime_hard_validation_us;
    trace.path = std::move(path);
    return trace;
  }

  void validateReconstructedCandidate(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    P3ShadowCandidateTrace & trace,
    double & hard_validation_us) const
  {
    if (trace.validator_executed) {
      return;
    }
    P3ShadowPathEvaluation evaluation;
    if (trace.path.wpnts.size() >= static_cast<std::size_t>(parameters_.minimum_path_points)) {
      const auto validation_start = Clock::now();
      const std::optional<double> collision_horizon(
        planner_.maneuverScopeEnd(
          ego, obstacles, cycle_cluster_ids_, trace.knot_stations[3]));
      evaluation = planner_.validateP3ShadowPath(
        ego, trace.path, obstacles, 1.0, collision_horizon);
      hard_validation_us += elapsedUs(validation_start);
      trace.validator_executed = true;
    } else {
      evaluation.rejection_reason = "spline segment has too few global race-line samples";
    }
    trace.validation = evaluation;
    trace.hard_valid = evaluation.hard_valid;
    trace.minimum_normalized_safety_slack = evaluation.minimum_normalized_safety_slack;
    trace.minimum_track_margin_m = evaluation.minimum_track_margin_m;
    trace.minimum_obstacle_margin_m = evaluation.minimum_obstacle_margin_m;
    trace.peak_curvature_radpm = evaluation.peak_curvature_radpm;
    trace.minimum_curvature_margin_radpm = evaluation.minimum_curvature_margin_radpm;
    trace.peak_curvature_rate_radpm2 = evaluation.peak_curvature_rate_radpm2;
    trace.curvature_rate_margin_radpm2 =
      parameters_.maximum_curvature_rate_radpm2 - trace.peak_curvature_rate_radpm2;
    trace.velocity_loss = evaluation.velocity_loss;
    trace.global_path_deviation_m = evaluation.global_path_deviation_m;
    trace.ego_braking_distance_deficit_m = evaluation.ego_braking_distance_deficit_m;
    trace.rejection_reason = evaluation.rejection_reason;
    trace.all_observed_violation_flags = evaluation.all_observed_violation_flags;
    trace.runtime_candidate_measurement_us = evaluation.runtime_candidate_measurement_us;
    trace.runtime_hard_validation_us = evaluation.runtime_hard_validation_us;
  }

  // 클러스터를 지난 뒤(exit 램프 + merge 뒤 꼬리)에도 오프셋이 남아 다음 장애물의 물리
  // 엔벨로프에 닿는가. 닿는다고 후보를 버리지는 않는다 — 장애물 간격이 좁으면 오프셋을
  // 그대로 넘겨주는 것이 설계된 동작이고(AGENTS의 maximum_exit_length 비활성 사유), 여기서
  // 거부하면 2026-08-12/08-15의 "후보 전멸 → 영구 크립" 회귀가 그대로 돌아온다. 대신
  // 순위에서만 뒤로 민다: 다음 장애물을 건드리지 않는 exit이 하나라도 있으면 그쪽을 쓴다.
  // 검사 구간은 **exit 램프뿐**이다: 클러스터 끝 이후 ~ merge 지점까지. merge 뒤 꼬리는
  // 정의상 d=0인 글로벌 라인이라, 다음 장애물이 라인 위에 있으면(이번 백의 s=40.6이 정확히
  // 그렇다) 모든 후보가 무조건 참이 되어 이 우선순위 자체가 무력해진다. 꼬리가 장애물을
  // 지나가는 것은 이 기동의 문제가 아니라 연쇄 기동이 교체할 몫이다.
  bool exitReachesNextObstacle(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double cluster_end,
    double merge_station) const
  {
    if (!(merge_station > cluster_end + kEpsilon)) {
      return false;
    }
    const double clearance = parameters_.obstacleBaseClearance();
    for (const auto & obstacle : obstacles) {
      const double start = planner_.forwardDistance(ego.s, obstacle.s_start);
      const double end = planner_.forwardDistance(ego.s, obstacle.s_end);
      if (!(start > cluster_end + kEpsilon) || end < start || start > merge_station + kEpsilon) {
        continue;   // 이 기동이 피하는 클러스터이거나, 뒤에 있거나, exit 구간 밖이다.
      }
      for (const auto & waypoint : path.wpnts) {
        const double forward = planner_.forwardDistance(ego.s, waypoint.s_m);
        if (forward + kEpsilon < start || forward > end + kEpsilon ||
          forward > merge_station + kEpsilon)
        {
          continue;
        }
        if (waypoint.d_m > obstacle.d_right - clearance - kEpsilon &&
          waypoint.d_m < obstacle.d_left + clearance + kEpsilon)
        {
          return true;
        }
      }
    }
    return false;
  }

  // 순위 정의는 candidate_rank.hpp 한 곳에만 둔다 (2026-08-21 통합). 여기서는 트레이스를
  // 공통 키로 옮기기만 한다 — plan() 의 better_candidate 와 갈릴 여지를 없애기 위해서다.
  static bool betterFeasible(
    const P3ShadowCandidateTrace & first, const P3ShadowCandidateTrace & second)
  {
    return betterCandidateRank(rankKey(first), rankKey(second));
  }

  // The frozen offline method used this exact tuple without epsilon collapsing. It is isolated
  // to the new R3 recovery stage; M0/M1 and the shared production comparator remain unchanged.
  static bool betterR3FrozenRank(
    const P3ShadowCandidateTrace & first, const P3ShadowCandidateTrace & second)
  {
    return std::make_tuple(
      first.exit_reaches_next_obstacle,
      first.ego_braking_distance_deficit_m > kCandidateRankEpsilon,
      first.ego_braking_distance_deficit_m,
      first.velocity_loss,
      -first.minimum_normalized_safety_slack,
      first.global_path_deviation_m,
      first.generation_index) <
           std::make_tuple(
      second.exit_reaches_next_obstacle,
      second.ego_braking_distance_deficit_m > kCandidateRankEpsilon,
      second.ego_braking_distance_deficit_m,
      second.velocity_loss,
      -second.minimum_normalized_safety_slack,
      second.global_path_deviation_m,
      second.generation_index);
  }

  static CandidateRankKey rankKey(const P3ShadowCandidateTrace & trace)
  {
    CandidateRankKey key;
    key.exit_reaches_next_obstacle = trace.exit_reaches_next_obstacle;
    key.ego_braking_distance_deficit_m = trace.ego_braking_distance_deficit_m;
    key.velocity_loss = trace.velocity_loss;
    key.minimum_normalized_safety_slack = trace.minimum_normalized_safety_slack;
    key.global_path_deviation_m = trace.global_path_deviation_m;
    key.tiebreak_index = trace.generation_index;
    return key;
  }

  std::vector<double> entryScales(bool extended = true) const
  {
    auto values = parameters_.entry_transition_fractions;
    if (extended) {
      values.push_back(
        parameters_.detection_lookahead_m / parameters_.pre_apex_distances_m.front());
    }
    return uniqueSorted(std::move(values));
  }

  std::vector<double> exitScales(
    const EgoFrenetState & ego, double cluster_end,
    bool go_left, bool outside_is_left, bool extended = true) const
  {
    auto values = parameters_.transition_distance_scales;
    if (!extended || values.empty()) {
      return uniqueSorted(std::move(values));
    }
    const double outside_multiplier = go_left == outside_is_left ?
      parameters_.outside_line_transition_scale : 1.0;
    const double nominal_post_far = parameters_.post_apex_distances_m.back();
    const double current_max_raw = *std::max_element(values.begin(), values.end());
    const double current_effective_length =
      nominal_post_far * current_max_raw * outside_multiplier;
    const double tail = std::max(
      parameters_.post_merge_lookahead_m,
      std::abs(ego.speed) * parameters_.post_merge_min_time_sec);
    const double horizon_limit = std::max(
      0.0, planner_.trackLength() - cluster_end - tail - 1.0e-3);
    for (const double factor : {1.5, 2.0}) {
      const double requested = current_effective_length * factor;
      const double effective = std::min(requested, horizon_limit);
      if (effective > current_effective_length + kEpsilon &&
        nominal_post_far * outside_multiplier > kEpsilon)
      {
        values.push_back(effective / (nominal_post_far * outside_multiplier));
      }
    }
    return uniqueSorted(std::move(values));
  }

  static std::pair<double, double> nearFar(double first, double second)
  {
    const auto key = [](double value) {return std::make_pair(std::abs(value), value);};
    return key(first) <= key(second) ? std::make_pair(first, second) :
           std::make_pair(second, first);
  }

  static std::vector<BranchRange> connectedConstantRanges(
    const Corridor & corridor, const P3ShadowSideDomain & domain)
  {
    std::vector<Interval> active;
    bool initialized = false;
    for (const auto & sample : corridor.samples) {
      if (sample.station < domain.cluster_start - kEpsilon ||
        sample.station > domain.cluster_end + kEpsilon)
      {
        continue;
      }
      if (!initialized) {
        active = sample.feasible;
        initialized = true;
        continue;
      }
      std::vector<Interval> next;
      for (const auto & previous : active) {
        for (const auto & current : sample.feasible) {
          const Interval intersection{
            std::max(previous.lower, current.lower),
            std::min(previous.upper, current.upper)};
          if (intersection.upper >= intersection.lower - kEpsilon) {
            next.push_back(intersection);
          }
        }
      }
      std::sort(next.begin(), next.end(), [](const auto & first, const auto & second) {
          return std::tie(first.lower, first.upper) < std::tie(second.lower, second.upper);
        });
      next.erase(
        std::unique(next.begin(), next.end(), [](const auto & first, const auto & second) {
          return std::abs(first.lower - second.lower) <= 1.0e-12 &&
                 std::abs(first.upper - second.upper) <= 1.0e-12;
        }), next.end());
      active = std::move(next);
      if (active.empty()) {
        break;
      }
    }

    const double domain_low = std::min(domain.minimum_target, domain.maximum_target);
    const double domain_high = std::max(domain.minimum_target, domain.maximum_target);
    std::vector<BranchRange> ranges;
    for (const auto & interval : active) {
      const double lower = std::max(interval.lower, domain_low);
      const double upper = std::min(interval.upper, domain_high);
      if (upper < lower - kEpsilon) {
        continue;
      }
      std::ostringstream identity;
      identity << std::setprecision(17) << (domain.go_left ? "LEFT" : "RIGHT") << ':' <<
        lower << ':' << upper;
      ranges.push_back({lower, upper, fnvHex(identity.str())});
    }
    if (ranges.empty()) {
      std::ostringstream identity;
      identity << std::setprecision(17) <<
        (domain.go_left ? "LEFT_DOMAIN" : "RIGHT_DOMAIN") << ':' <<
        domain_low << ':' << domain_high;
      ranges.push_back({domain_low, domain_high, fnvHex(identity.str())});
    }
    std::stable_sort(ranges.begin(), ranges.end(), [](const auto & first, const auto & second) {
        const double first_width = first.upper - first.lower;
        const double second_width = second.upper - second.lower;
        if (std::abs(first_width - second_width) > 1.0e-12) {
          return first_width > second_width;
        }
        return std::tie(first.lower, first.upper) < std::tie(second.lower, second.upper);
      });
    if (ranges.size() > 2U) {
      ranges.resize(2U);
    }
    return ranges;
  }

  static std::vector<double> rankedTargets(const BranchRange & branch)
  {
    std::vector<double> targets;
    for (const double ratio : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      targets.push_back(branch.lower + ratio * (branch.upper - branch.lower));
    }
    targets = uniqueSorted(std::move(targets));
    std::stable_sort(targets.begin(), targets.end(), [](double first, double second) {
        if (std::abs(std::abs(first) - std::abs(second)) > 1.0e-12) {
          return std::abs(first) < std::abs(second);
        }
        return first < second;
      });
    return targets;
  }

  InactiveSolve solveAllInactive(
    const std::array<double, 5> & stations, double ego_d, double target,
    double station, double desired, double lower, double upper) const
  {
    PlanningResearchCycle * cycle = planner_.activeResearchCycle();
    ResearchTimer research_timer(
      cycle == nullptr ? nullptr : &cycle->runtime_root_solve_us);
    InactiveSolve result;
    std::size_t segment = 0U;
    while (segment + 1U < 4U && station > stations[segment + 1U]) {
      ++segment;
    }
    const double length = stations[segment + 1U] - stations[segment];
    const double normalized = std::clamp(
      (station - stations[segment]) / length, 0.0, 1.0);
    const double intercept = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 0.0, segment, normalized);
    const double slope = linearSampleWithoutSideDerivative(
      stations, ego_d, target, 1.0, segment, normalized) - intercept;
    const double coefficient_scale = std::max({
        1.0, std::abs(intercept), std::abs(slope), std::abs(desired)});
    const double zero_tolerance =
      p3_analytic_solver::kCoefficientRelativeTolerance * coefficient_scale;
    if (std::abs(slope) <= zero_tolerance) {
      return result;
    }
    const double root = (desired - intercept) / slope;
    ++result.raw_roots;
    if (!std::isfinite(root)) {
      return result;
    }
    ++result.finite_roots;
    const auto states = sourceRuleStates(stations, {ego_d, target, root, target, 0.0});
    const bool all_inactive = std::none_of(
      states.branches.begin(), states.branches.end(), [](const std::string & branch) {
        return branch.rfind("SAME_SIGN", 0U) == 0U;
      });
    if (!all_inactive) {
      return result;
    }
    ++result.branch_roots;
    if (root < lower - kEpsilon || root > upper + kEpsilon) {
      return result;
    }
    ++result.bounded_roots;
    const auto segments = makeC2Profile(stations, {ego_d, target, root, target, 0.0});
    const double residual = evaluateProfile(segments, station, ego_d) - desired;
    if (std::abs(residual) <= 1.0e-12 * std::max(1.0, std::abs(desired))) {
      result.accepted.push_back(root);
    }
    return result;
  }

  // 이미 만들어진 후보들에서 진입 눈금 브래킷을 읽는다.
  //
  // 두 제약은 entry에 대해 서로 반대 방향으로 단조다(stationsFor의 구조에서 따라온다):
  //   entry ↑ → 램프가 길어져 곡률 ↓, 동시에 시작점이 자차 쪽으로 당겨져 좁은 구간을 깊이 지남
  // 그래서 "짧은 쪽은 곡률로, 긴 쪽은 경계로" 죽었다면 실현 구간이 그 사이에 있다.
  // 그 조건이 아니면 이분법의 전제가 없으므로 false를 돌려주고 아무것도 하지 않는다.
  static bool entryBracketFrom(
    const std::vector<P3ShadowCandidateTrace> & candidates,
    double low_entry, double high_entry)
  {
    bool low_needs_longer = false;
    bool high_needs_shorter = false;
    for (const auto & trace : candidates) {
      if (trace.hard_valid) {
        continue;
      }
      const EntrySteer steer = classifyEntrySteer(trace.rejection_reason);
      if (std::abs(trace.entry_scale - low_entry) <= 1.0e-9 &&
        steer == EntrySteer::kNeedsLongerRamp)
      {
        low_needs_longer = true;
      }
      if (std::abs(trace.entry_scale - high_entry) <= 1.0e-9 &&
        steer == EntrySteer::kNeedsShorterRamp)
      {
        high_needs_shorter = true;
      }
    }
    return low_needs_longer && high_needs_shorter && high_entry > low_entry + kEpsilon;
  }

  // 후보 전체에 "더 긴 램프가 필요한 실패"와 "더 짧은 램프가 필요한 실패"가 모두 있는가.
  //
  // entryBracketFrom은 눈금 값이 정확히 일치하는 후보만 본다. M1 템플릿은 목표 오프셋이
  // 서로 달라 그 조건이 잘 성립하지 않는다. 실현 구간의 존재를 시사하는 신호로는 "양쪽 벽에
  // 부딪힌 후보가 모두 있다"로 충분하고, 방향이 틀리면 이분법이 스스로 멈춘다.
  static bool entryBracketPresent(const std::vector<P3ShadowCandidateTrace> & candidates)
  {
    bool longer = false;
    bool shorter = false;
    for (const auto & trace : candidates) {
      if (trace.hard_valid) {
        continue;
      }
      const EntrySteer steer = classifyEntrySteer(trace.rejection_reason);
      longer = longer || steer == EntrySteer::kNeedsLongerRamp;
      shorter = shorter || steer == EntrySteer::kNeedsShorterRamp;
    }
    return longer && shorter;
  }

  // 새로 추가된 후보들이 가리키는 방향. 하나라도 통과했으면 kUnknown(이분법 종료)이다.
  static EntrySteer steerOfNewCandidates(
    const std::vector<P3ShadowCandidateTrace> & candidates, std::size_t from)
  {
    EntrySteer steer = EntrySteer::kUnknown;
    for (std::size_t index = from; index < candidates.size(); ++index) {
      if (candidates[index].hard_valid) {
        return EntrySteer::kUnknown;
      }
      if (steer == EntrySteer::kUnknown) {
        steer = classifyEntrySteer(candidates[index].rejection_reason);
      }
    }
    return steer;
  }

  void addM0ExtensionCandidate(
    ExtensionOutcome & outcome,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    const BranchRange & component,
    const std::string & candidate_template,
    const std::string & source_cell,
    double target,
    double middle,
    double entry,
    double exit,
    std::size_t & generation,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (outcome.candidates.size() >= outcome.candidate_cap) {
      return;
    }
    const auto key = std::make_tuple(domain.go_left, target, middle, entry, exit);
    if (!seen.insert(key).second) {
      return;
    }
    const auto stations = stationsFor(
      parameters_, domain, outside_is_left, entry, exit, ego.d, target, referenceSpacing());
    ++outcome.validator_calls;
    auto trace = buildCandidate(
      ego, obstacles, domain.go_left, outside_is_left, target, middle, entry, exit,
      stations, generation++, outcome.runtime_reconstruction_us,
      outcome.runtime_hard_validation_us);
    const std::string side = domain.go_left ? "LEFT" : "RIGHT";
    trace.mapping_source = "FROZEN_V2_EXTENSION";
    trace.generator_stage = "M0_V2";
    trace.candidate_template = candidate_template;
    trace.source_cell = source_cell;
    trace.root_type = source_cell;
    trace.component_id = component.id;
    trace.candidate_identity = "M0_V2_" + side + "_" + candidate_template + "_" +
      trace.path_digest;
    trace.logical_identity = "M0_V2_" + side + "_" + candidate_template;
    const std::size_t index = outcome.candidates.size();
    if (trace.hard_valid) {
      ++outcome.hard_valid_count;
      if (!outcome.best_index.has_value() ||
        betterFeasible(trace, outcome.candidates[*outcome.best_index]))
      {
        outcome.best_index = index;
      }
    }
    outcome.candidates.push_back(std::move(trace));
  }

  void addM0AnalyticCandidate(
    ExtensionOutcome & outcome,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowSideDomain & domain,
    bool outside_is_left,
    const Corridor & corridor,
    const BranchRange & component,
    double target,
    double entry,
    double exit,
    std::size_t & generation,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (outcome.candidates.size() >= outcome.candidate_cap || corridor.branch.empty()) {
      return;
    }
    const Probe probe = chooseProbe(
      corridor, domain.cluster_start, domain.cluster_end, target, "BOTTLENECK_CENTER");
    const auto stations = stationsFor(
      parameters_, domain, outside_is_left, entry, exit, ego.d, target, referenceSpacing());
    if (probe.station < stations.front() - kEpsilon ||
      probe.station > stations.back() + kEpsilon)
    {
      return;
    }
    const auto root_start = Clock::now();
    const RootSolve roots = solvePosition(
      stations, ego.d, target, probe.station, probe.desired,
      component.lower, component.upper);
    outcome.runtime_root_solver_us += elapsedUs(root_start);
    outcome.raw_root_count += roots.algebraic.raw_roots.size();
    outcome.finite_root_count += roots.finite.size();
    outcome.branch_root_count += roots.branch.size();
    outcome.bounded_root_count += roots.bounded.size();
    outcome.accepted_root_count += roots.accepted.size();
    for (std::size_t root_index = 0U; root_index < roots.accepted.size(); ++root_index) {
      const double root = roots.accepted[root_index];
      const std::size_t before = outcome.candidates.size();
      addM0ExtensionCandidate(
        outcome, ego, obstacles, domain, outside_is_left, component,
        "BRANCH_AWARE_ANALYTIC_ROOT", "ACTIVE_OUTER", target, root, entry, exit,
        generation, seen);
      if (outcome.candidates.size() > before) {
        auto & trace = outcome.candidates.back();
        trace.s_probe = probe.station;
        trace.d_probe = probe.desired;
        trace.probe_location_rule = probe.location_rule;
        trace.probe_anchor_rule = probe.anchor_rule;
        trace.root_index = static_cast<std::int64_t>(root_index);
        trace.root_type = "ACTIVE_OUTER";
      }
    }
  }

  ExtensionOutcome evaluateM0Extension(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowPlanningContext & context,
    std::size_t remaining_total_budget) const
  {
    // 이 계열이 쓸 수 있는 후보 수. 자체 상한과 남은 총예산 중 작은 쪽이다.
    ExtensionOutcome outcome;
    outcome.candidate_cap = std::min(kM0ExtensionCandidateCap, remaining_total_budget);
    const std::size_t extension_cap = outcome.candidate_cap;
    std::size_t generation = 1000U;
    std::set<std::tuple<bool, double, double, double, double>> seen;
    for (const bool go_left : {false, true}) {
      const auto & domain = go_left ? context.left : context.right;
      if (!domain.valid) {
        continue;
      }
      const Corridor corridor = makeCorridor(
        ego, context.visible, domain.cluster_start, domain.cluster_end,
        go_left, context.outside_is_left);
      const auto components = connectedConstantRanges(corridor, domain);
      const auto entries = entryScales(true);
      const auto exits = exitScales(
        ego, domain.cluster_end, go_left, context.outside_is_left, true);
      if (entries.empty() || exits.empty()) {
        continue;
      }
      const double entry_full = entries.back();
      const double exit_short = exits.front() + 0.015625 * (exits.back() - exits.front());
      const double entry_quarter = entries.front() + 0.25 * (entries.back() - entries.front());
      const double exit_quarter = exits.front() + 0.25 * (exits.back() - exits.front());
      const double exit_long = exits.back();
      for (const auto & component : components) {
        const auto targets = rankedTargets(component);
        if (targets.empty()) {
          continue;
        }
        const BranchRange side_domain{
          std::min(domain.minimum_target, domain.maximum_target),
          std::max(domain.minimum_target, domain.maximum_target), "SIDE_DOMAIN"};
        const auto domain_targets = rankedTargets(side_domain);
        const double near = targets.front();
        const double center = 0.5 * (component.lower + component.upper);
        const double far = targets.back();
        const double quarter = near + 0.25 * (far - near);
        const double domain_near = domain_targets.front();
        const double domain_far = domain_targets.back();
        const double domain_boundary_inset =
          domain_near + 0.015625 * (domain_far - domain_near);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_BOUNDARY_INSET", "ZERO_INTERFACE",
          domain_boundary_inset, domain_boundary_inset, entry_full, exit_short,
          generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_QUARTER_LONG_EXIT", "ZERO_INTERFACE",
          quarter, quarter, entry_full, exit_long, generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_CENTER_LONG_EXIT", "ZERO_INTERFACE",
          center, center, entry_full, exit_long, generation, seen);
        addM0ExtensionCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, component,
          "ZERO_INTERFACE_QUARTER_SCALE", "ZERO_INTERFACE",
          quarter, quarter, entry_quarter, exit_quarter, generation, seen);
        addM0AnalyticCandidate(
          outcome, ego, obstacles, domain, context.outside_is_left, corridor,
          component, near, entry_quarter, exit_quarter, generation, seen);

        // 🔴 2026-08-17: 고정 눈금이 실현 구간을 건너뛰면 이분법으로 찾는다.
        //
        // 이 계열은 entry를 entries.back()과 그 1/4점 둘만 썼다. baseline과 같은 병(病)이다 —
        // 실현 구간이 두 눈금 사이에 끼면 통과 가능한 갭에서도 후보가 전멸한다. 눈금을 더
        // 촘촘히 박는 대신 구간을 직접 찾는다. 기각 사유가 어느 벽인지 알려주므로 신탁은
        // 공짜다. 템플릿이 이미 해를 냈으면 이 경로는 돌지 않는다(평소 비용 0).
        if (!outcome.best_index.has_value() &&
          entryBracketFrom(outcome.candidates, entry_quarter, entry_full))
        {
          double low_entry = entry_quarter;
          double high_entry = entry_full;
          while (!outcome.best_index.has_value() &&
            outcome.candidates.size() < extension_cap &&
            high_entry - low_entry > kEpsilon)
          {
            const double middle_entry = 0.5 * (low_entry + high_entry);
            const std::size_t before = outcome.candidates.size();
            addM0ExtensionCandidate(
              outcome, ego, obstacles, domain, context.outside_is_left, component,
              "ZERO_INTERFACE_BISECTED", "ZERO_INTERFACE",
              quarter, quarter, middle_entry, exit_quarter, generation, seen);
            if (outcome.candidates.size() == before) {
              break;   // 중복으로 걸러졌다면 더 좁혀도 같은 후보만 나온다.
            }
            const EntrySteer steer = steerOfNewCandidates(outcome.candidates, before);
            if (steer == EntrySteer::kNeedsLongerRamp) {
              low_entry = middle_entry;
            } else if (steer == EntrySteer::kNeedsShorterRamp) {
              high_entry = middle_entry;
            } else {
              break;
            }
          }
        }
        if (outcome.candidates.size() >= outcome.candidate_cap) {
          break;
        }
      }
      if (outcome.candidates.size() >= outcome.candidate_cap) {
        break;
      }
    }
    return outcome;
  }

  std::vector<M1Context> buildM1Contexts(
    const EgoFrenetState & ego,
    const P3ShadowPlanningContext & planning_context,
    P3ShadowResult & result) const
  {
    std::vector<M1Context> contexts;
    for (const bool go_left : {false, true}) {
      const auto & domain = go_left ? planning_context.left : planning_context.right;
      if (!domain.valid) {
        continue;
      }
      const Corridor corridor = makeCorridor(
        ego, planning_context.visible, domain.cluster_start, domain.cluster_end,
        go_left, planning_context.outside_is_left);
      const auto components = connectedConstantRanges(corridor, domain);
      const auto entries = entryScales(true);
      const auto frozen_exits = exitScales(
        ego, domain.cluster_end, go_left, planning_context.outside_is_left, false);
      const auto exits = exitScales(
        ego, domain.cluster_end, go_left, planning_context.outside_is_left, true);
      if (entries.empty() || exits.empty()) {
        continue;
      }
      if (frozen_exits.empty() || exits.back() + 1.0e-12 < frozen_exits.back()) {
        throw std::runtime_error("extended exit interval does not contain frozen exit interval");
      }
      for (const double frozen_exit : frozen_exits) {
        const bool retained = std::any_of(
          exits.begin(), exits.end(), [frozen_exit](double value) {
            return std::abs(value - frozen_exit) <= 1.0e-12;
          });
        if (!retained) {
          throw std::runtime_error("extended exit interval dropped a frozen exit scale");
        }
      }
      const double entry_min = entries.front();
      const double entry_max = entries.back();
      const double exit_min = exits.front();
      const double exit_max = exits.back();
      const double outside_multiplier = go_left == planning_context.outside_is_left ?
        parameters_.outside_line_transition_scale : 1.0;
      const double cluster_width = domain.cluster_end - domain.cluster_start;
      const double exit_span = std::clamp(
        cluster_width / (parameters_.post_apex_distances_m.back() * outside_multiplier),
        exit_min, exit_max);
      const auto domain_ends = nearFar(
        std::min(domain.minimum_target, domain.maximum_target),
        std::max(domain.minimum_target, domain.maximum_target));
      const double boundary_inset =
        domain_ends.first + (domain_ends.second - domain_ends.first) / 64.0;
      for (std::size_t index = 0U; index < components.size(); ++index) {
        const auto component_ends = nearFar(components[index].lower, components[index].upper);
        contexts.push_back({
            domain, planning_context.outside_is_left, corridor, components[index], index,
            boundary_inset, component_ends.first, component_ends.second,
            entry_min, entry_max, entry_min + (entry_max - entry_min) / 4.0,
            exit_min, exit_max, exit_span});
      }
    }
    result.m1_context_count = contexts.size();
    return contexts;
  }

  void addM1Candidate(
    ExtensionOutcome & outcome,
    P3ShadowResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const M1Context & context,
    const std::string & candidate_template,
    const std::string & source_cell,
    std::size_t source_root_index,
    const Probe * probe,
    double target,
    double middle,
    double entry,
    double exit,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (result.m1_candidate_count >= result.m1_budget) {
      return;
    }
    const auto key = std::make_tuple(
      context.domain.go_left, target, middle, entry, exit);
    if (!seen.insert(key).second) {
      ++result.m1_exact_tuple_duplicate_count;
      return;
    }
    const auto stations = stationsFor(
      parameters_, context.domain, context.outside_is_left, entry, exit,
      ego.d, target, referenceSpacing());
    double minimum_length = std::numeric_limits<double>::quiet_NaN();
    if (!strictPositiveSegments(stations, minimum_length)) {
      ++result.m1_positive_segment_rejection_count;
      ++result.m1_boundary_handoff_unresolved_count;
      return;
    }

    const std::size_t generation = 2000U + result.m1_candidate_count;
    ++result.m1_candidate_count;
    ++result.m1_validator_call_count;
    ++outcome.validator_calls;
    auto trace = buildCandidate(
      ego, obstacles, context.domain.go_left, context.outside_is_left,
      target, middle, entry, exit, stations, generation,
      outcome.runtime_reconstruction_us, outcome.runtime_hard_validation_us);
    const std::string side = context.domain.go_left ? "LEFT" : "RIGHT";
    std::ostringstream tuple_identity;
    tuple_identity << std::setprecision(17) << context.domain.go_left << ':' <<
      candidate_template << ':' << source_cell << ':' << context.component_index << ':' <<
      source_root_index << ':' << target << ':' << middle << ':' << entry << ':' << exit;
    trace.mapping_source = "M1_BRANCH_COMPLETE_ACTIVE_SET_CLOSURE";
    trace.generator_stage = "M1";
    trace.candidate_template = candidate_template;
    trace.source_cell = source_cell;
    trace.component_id = context.component.id;
    trace.candidate_identity = "M1_" + side + "_" + candidate_template + "_" +
      source_cell + "_" + fnvHex(tuple_identity.str());
    trace.logical_identity = "M1_" + side + "_" + candidate_template + "_" +
      source_cell + "_c" + std::to_string(context.component_index) + "_r" +
      std::to_string(source_root_index);
    trace.root_index = static_cast<std::int64_t>(source_root_index);
    trace.root_type = source_cell;
    if (probe != nullptr) {
      trace.s_probe = probe->station;
      trace.d_probe = probe->desired;
      trace.probe_location_rule = probe->location_rule;
      trace.probe_anchor_rule = probe->anchor_rule;
    }
    const std::size_t index = outcome.candidates.size();
    if (trace.hard_valid) {
      ++result.m1_hard_valid_count;
      ++outcome.hard_valid_count;
      if (!outcome.best_index.has_value() ||
        betterFeasible(trace, outcome.candidates[*outcome.best_index]))
      {
        outcome.best_index = index;
      }
    }
    outcome.candidates.push_back(std::move(trace));
  }

  void offerM1AnalyticTemplate(
    ExtensionOutcome & outcome,
    P3ShadowResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const M1Context & context,
    const std::string & candidate_template,
    double target,
    double entry,
    double exit,
    std::set<std::tuple<bool, double, double, double, double>> & seen) const
  {
    if (result.m1_candidate_count >= result.m1_budget) {
      return;
    }
    const Probe probe = chooseProbe(
      context.corridor, context.domain.cluster_start, context.domain.cluster_end,
      target, "BOTTLENECK_CENTER");
    const auto stations = stationsFor(
      parameters_, context.domain, context.outside_is_left, entry, exit,
      ego.d, target, referenceSpacing());
    double minimum_length = std::numeric_limits<double>::quiet_NaN();
    if (!strictPositiveSegments(stations, minimum_length)) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ACTIVE_OUTER", 0U, &probe, target, target, entry, exit, seen);
      return;
    }
    if (probe.station < stations.front() - kEpsilon ||
      probe.station > stations.back() + kEpsilon)
    {
      return;
    }

    const auto active_start = Clock::now();
    const RootSolve active = solvePosition(
      stations, ego.d, target, probe.station, probe.desired,
      context.component.lower, context.component.upper);
    outcome.runtime_root_solver_us += elapsedUs(active_start);
    result.m1_active_outer_raw_root_count += active.algebraic.raw_roots.size();
    result.m1_active_outer_accepted_root_count += active.accepted.size();
    for (std::size_t index = 0U; index < active.accepted.size(); ++index) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ACTIVE_OUTER", index, &probe, target, active.accepted[index], entry, exit, seen);
    }

    const auto inactive_start = Clock::now();
    const InactiveSolve inactive = solveAllInactive(
      stations, ego.d, target, probe.station, probe.desired,
      context.component.lower, context.component.upper);
    outcome.runtime_root_solver_us += elapsedUs(inactive_start);
    result.m1_all_inactive_raw_root_count += inactive.raw_roots;
    result.m1_all_inactive_accepted_root_count += inactive.accepted.size();
    for (std::size_t index = 0U; index < inactive.accepted.size(); ++index) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, candidate_template,
        "ALL_INACTIVE", index, &probe, target, inactive.accepted[index], entry, exit, seen);
    }
  }

  ExtensionOutcome evaluateM1(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const P3ShadowPlanningContext & planning_context,
    P3ShadowResult & result) const
  {
    ExtensionOutcome outcome;
    const auto contexts = buildM1Contexts(ego, planning_context, result);
    std::set<std::tuple<bool, double, double, double, double>> seen;
    for (const auto & context : contexts) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, "ZERO_BOUNDARY_SHORT",
        "ZERO_INTERFACE", 0U, nullptr, context.boundary_inset, context.boundary_inset,
        context.entry_min, context.exit_min, seen);
    }
    for (const auto & context : contexts) {
      addM1Candidate(
        outcome, result, ego, obstacles, context, "ZERO_BOUNDARY_SPAN",
        "ZERO_INTERFACE", 0U, nullptr, context.boundary_inset, context.boundary_inset,
        context.entry_quarter, context.exit_span, seen);
    }
    for (const auto & context : contexts) {
      offerM1AnalyticTemplate(
        outcome, result, ego, obstacles, context, "NEAR_LONG",
        context.branch_near, context.entry_max, context.exit_max, seen);
    }
    for (const auto & context : contexts) {
      offerM1AnalyticTemplate(
        outcome, result, ego, obstacles, context, "FAR_SPAN",
        context.branch_far, context.entry_quarter, context.exit_span, seen);
    }

    // 🔴 2026-08-17: 고정 눈금이 실현 구간을 건너뛰면 이분법으로 찾는다.
    //
    // 이 계열은 entry를 entry_min / entry_quarter / entry_max 셋만 썼다. M0 baseline과 같은
    // 병(病)이다 — 실현 구간이 눈금 사이에 끼면 통과 가능한 갭에서도 후보가 전멸한다.
    // 두 제약(곡률·트랙 경계)이 entry에 대해 서로 반대 방향으로 단조이므로 실현 집합은 항상
    // 구간이고, 기각 사유가 어느 벽인지 알려주므로 이분법의 신탁은 공짜다.
    //
    // 템플릿이 이미 해를 냈으면 이 경로는 돌지 않는다(평소 비용 0). 예산은 m1_budget이
    // 그대로 강제하므로(addM1Candidate가 먼저 검사한다) 총 후보 상한을 넘지 않는다.
    if (!outcome.best_index.has_value() && entryBracketPresent(outcome.candidates)) {
      for (const auto & context : contexts) {
        if (outcome.best_index.has_value()) {
          break;
        }
        double low_entry = context.entry_min;
        double high_entry = context.entry_max;
        while (!outcome.best_index.has_value() &&
          result.m1_candidate_count < result.m1_budget &&
          high_entry - low_entry > kEpsilon)
        {
          const double middle_entry = 0.5 * (low_entry + high_entry);
          const std::size_t before = outcome.candidates.size();
          offerM1AnalyticTemplate(
            outcome, result, ego, obstacles, context, "NEAR_BISECTED",
            context.branch_near, middle_entry, context.exit_max, seen);
          if (outcome.candidates.size() == before) {
            break;   // 중복으로 걸러졌다면 더 좁혀도 같은 후보만 나온다.
          }
          const EntrySteer steer = steerOfNewCandidates(outcome.candidates, before);
          if (steer == EntrySteer::kNeedsLongerRamp) {
            low_entry = middle_entry;
          } else if (steer == EntrySteer::kNeedsShorterRamp) {
            high_entry = middle_entry;
          } else {
            break;
          }
        }
      }
    }
    return outcome;
  }

  SideResult evaluateSide(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<P3ShadowObstacleEnvelope> & visible,
    const P3ShadowSideDomain & domain,
    bool outside_is_left) const
  {
    SideResult result;
    result.go_left = domain.go_left;
    result.domain = domain;
    if (!domain.valid) {
      result.failure = domain.reason;
      return result;
    }
    result.domain_valid = true;
    const bool go_left = domain.go_left;
    const double cluster_start = domain.cluster_start;
    const double cluster_end = domain.cluster_end;
    const double minimum_target = domain.minimum_target;
    const double maximum_target = domain.maximum_target;

    const auto corridor_start = Clock::now();
    result.corridor = makeCorridor(
      ego, visible, cluster_start, cluster_end, go_left, outside_is_left);
    result.runtime_corridor_us += elapsedUs(corridor_start);
    if (!result.corridor.connected || result.corridor.branch.empty()) {
      result.failure = "CORRIDOR_BRANCH_WRONG";
      return result;
    }

    const double target = existingTargetAnchor(minimum_target, maximum_target);
    result.probe = chooseProbe(result.corridor, cluster_start, cluster_end, target);
    const auto & mid_interval = nearestBranchSample(
      result.corridor, 0.5 * (cluster_start + cluster_end));
    double lower = std::max(-parameters_.maximum_target_offset_m, mid_interval.lower);
    double upper = std::min(parameters_.maximum_target_offset_m, mid_interval.upper);
    if (go_left) {
      lower = std::max(lower, target);
    } else {
      upper = std::min(upper, target);
    }

    const auto all_entries = entryScales();
    const auto all_exits = exitScales(ego, cluster_end, go_left, outside_is_left);
    const std::array<double, 3> exit_ratios{0.015625, 0.5, 1.0};
    std::vector<double> exits;
    for (const double ratio : exit_ratios) {
      exits.push_back(all_exits.front() + ratio * (all_exits.back() - all_exits.front()));
    }
    exits = uniqueSorted(std::move(exits));

    std::size_t generation = 0U;
    bool cap_exceeded = false;
    // Build every candidate for one entry scale over the given exit set, and report which wall the
    // rejections hit so the caller can steer. Returns kUnknown when nothing conclusive was seen.
    const auto offer_entry =
      [&](double entry, const std::vector<double> & exit_set) -> EntrySteer {
        EntrySteer steer = EntrySteer::kUnknown;
        for (const double exit : exit_set) {
          // stationsFor와 같은 식을 여기 복사해 두면 한쪽만 고쳤을 때 조용히 갈라진다.
          // 실제로 cluster_start <= 0 수리(2026-08-17)가 이 복사본을 비껴갈 뻔했다.
          const std::array<double, 5> stations = stationsFor(
            parameters_, domain, outside_is_left, entry, exit,
            ego.d, target, referenceSpacing());
          if (result.probe.station < stations.front() - kEpsilon ||
            result.probe.station > stations.back() + kEpsilon)
          {
            continue;
          }
          const auto root_start = Clock::now();
          const RootSolve roots = solvePosition(
            stations, ego.d, target, result.probe.station, result.probe.desired, lower, upper);
          result.runtime_root_solver_us += elapsedUs(root_start);
          result.raw_root_count += roots.algebraic.raw_roots.size();
          result.finite_root_count += roots.finite.size();
          result.branch_root_count += roots.branch.size();
          result.bounded_root_count += roots.bounded.size();
          result.accepted_root_count += roots.accepted.size();

          for (std::size_t root_index = 0U; root_index < roots.accepted.size(); ++root_index) {
            const double root = roots.accepted[root_index];
            if (result.candidates.size() >= kFrozenCandidateCap) {
              result.failure = "CANDIDATE_CAP_EXCEEDED";
              cap_exceeded = true;
              return steer;
            }
            ++result.validator_calls;
            auto trace = buildCandidate(
              ego, obstacles, go_left, outside_is_left, target, root, entry, exit, stations,
              generation++, result.runtime_reconstruction_us,
              result.runtime_hard_validation_us);
            trace.mapping_source = "FROZEN_V1";
            trace.generator_stage = "M0_V1";
            trace.candidate_template = "FROZEN_V1_CURVATURE_CONTINUITY";
            trace.source_cell = "ACTIVE_OUTER";
            trace.component_id = result.corridor.branch_id;
            trace.s_probe = result.probe.station;
            trace.d_probe = result.probe.desired;
            trace.probe_location_rule = result.probe.location_rule;
            trace.probe_anchor_rule = result.probe.anchor_rule;
            trace.root_index = static_cast<std::int64_t>(root_index);
            trace.root_type = "ACTIVE_OUTER";
            const std::string side = go_left ? "LEFT" : "RIGHT";
            trace.candidate_identity = "M0_V1_" + side + "_" + trace.path_digest;
            trace.logical_identity = "M0_V1_" + side;
            const std::size_t index = result.candidates.size();
            if (trace.hard_valid) {
              ++result.hard_valid_count;
              if (!result.best_index.has_value() ||
                betterFeasible(trace, result.candidates[*result.best_index]))
              {
                result.best_index = index;
              }
            } else {
              result.failure = "P3_CANDIDATE_HARD_INVALID";
              if (steer == EntrySteer::kUnknown) {
                steer = classifyEntrySteer(trace.rejection_reason);
              }
            }
            result.candidates.push_back(std::move(trace));
          }
        }
        return steer;
      };

    const double entry_short = all_entries.front();
    const double entry_long = all_entries.back();
    const EntrySteer short_steer = offer_entry(entry_short, exits);
    const EntrySteer long_steer = cap_exceeded ?
      EntrySteer::kUnknown : offer_entry(entry_long, exits);

    // 🔴 2026-08-17: 진입 눈금을 상수 집합에서 뽑지 않고 제약에서 찾는다.
    //
    // 종전에는 entryScales()가 돌려주는 눈금 중 **양 끝만** 썼다
    // (entries{all_entries.front(), all_entries.back()}). 그래서 YAML의 중간값
    // (entry_transition_fractions의 0.75, 1.00)은 한 번도 시도되지 않았다.
    //
    // 실해 — 2026-08-17 01:05 백, 매 랩 같은 자리에서 9.9 s 정지 (안전망 pinch_failure):
    //   entry=0.5146(front)  peakK=1.976  곡률 초과 (한계 1.316)
    //   entry=1.311 (back)   peakK=0.808  곡률 OK, 그러나 회랑 병목 침범
    //   ── 실현 구간 [0.75, 1.05]가 두 눈금 사이에 통째로 끼어 건너뛰어졌다 ──
    // 통과 가능한 갭인데 후보가 0개가 되어 안전정지가 걸렸고, 안전정지는 스스로 해제
    // 조건(유효 회피 8사이클)을 막아 사람이 차를 옮겨야 풀렸다.
    //
    // 두 제약은 entry에 대해 서로 **반대 방향으로 단조**다. 이것은 관측이 아니라
    // stationsFor의 구조에서 따라온다:
    //   entry ↑ → entry_length ↑ → 램프가 길어져 곡률 ↓, 동시에 시작점(stations[0])이
    //             자차 쪽으로 당겨져 좁은 구간을 더 깊이 지난다
    // 따라서 실현 집합은 항상 하나의 구간이고, 기각 사유가 어느 벽인지 알려주므로
    // 이분법의 신탁은 공짜다. 눈금을 더 촘촘히 박는 대신 구간을 직접 찾는다 —
    // 새 상수도, 새 튜닝값도 없다.
    //
    // 고정 눈금이 이미 해를 냈으면 이 경로는 돌지 않는다(평소 비용 0). 탐색 중에는
    // exit을 하나로 고정한다 — 위 실측처럼 진입 쪽 기각은 exit과 무관하게 같으므로
    // exit을 3개로 늘리면 후보 예산만 3배로 쓴다. 구간을 찾은 뒤에 exit을 펼친다.
    if (!cap_exceeded && !result.best_index.has_value() &&
      short_steer == EntrySteer::kNeedsLongerRamp &&
      long_steer == EntrySteer::kNeedsShorterRamp &&
      entry_long > entry_short + kEpsilon)
    {
      const std::vector<double> probe_exits{exits.front()};
      double lower_entry = entry_short;
      double upper_entry = entry_long;
      while (!cap_exceeded && !result.best_index.has_value() &&
        result.candidates.size() + exits.size() <= kFrozenCandidateCap)
      {
        const double middle_entry = 0.5 * (lower_entry + upper_entry);
        if (!(upper_entry - lower_entry > kEpsilon)) {
          break;
        }
        const EntrySteer steer = offer_entry(middle_entry, probe_exits);
        if (steer == EntrySteer::kNeedsLongerRamp) {
          lower_entry = middle_entry;
        } else if (steer == EntrySteer::kNeedsShorterRamp) {
          upper_entry = middle_entry;
        } else {
          break;   // 진입 눈금과 무관한 기각이면 이분법의 전제가 깨진다.
        }
      }
      // 구간을 찾았으면 그 눈금에서 exit을 펼쳐 나머지 후보를 준다. 못 찾았으면
      // 아무것도 추가하지 않고 종전과 동일하게 실패한다.
      if (!cap_exceeded && result.best_index.has_value()) {
        const double solved_entry =
          result.candidates[*result.best_index].entry_scale;
        std::vector<double> remaining;
        for (const double exit : exits) {
          if (std::abs(exit - probe_exits.front()) > kEpsilon) {
            remaining.push_back(exit);
          }
        }
        if (!remaining.empty()) {
          (void)offer_entry(solved_entry, remaining);
        }
      }
    }
    if (cap_exceeded) {
      return result;
    }
    if (result.accepted_root_count == 0U) {
      result.failure = result.raw_root_count == 0U ? "NO_ALGEBRAIC_ROOT" :
        (result.branch_root_count == 0U ? "ROOT_BRANCH_MISMATCH" :
        (result.bounded_root_count == 0U ? "ROOT_LATERAL_BOUND" :
        "ANALYTIC_ROOT_EXISTS_BUT_REJECTED"));
    } else if (result.best_index.has_value()) {
      result.failure = "NONE";
    }
    return result;
  }

  const RacelineSplinePlanner & planner_;
  const RacelineSplineParameters & parameters_;
};

P3ShadowResult RacelineSplinePlanner::evaluateP3Shadow(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::int64_t snapshot_source_stamp_ns,
  std::uint64_t snapshot_epoch,
  std::uint64_t global_reference_generation,
  const std::string & p0_failure_reason,
  const std::string & research_evaluation_role) const
{
  PlanningResearchCycle * research_cycle = activeResearchCycle();
  if (research_cycle != nullptr) {
    const std::string & evaluation_role = research_evaluation_role.empty() ?
      p0_failure_reason : research_evaluation_role;
    research_cycle->active_evaluation_lineage = makeEvaluationLineage(
      *research_cycle, ego, obstacles, reference_, snapshot_source_stamp_ns,
      snapshot_epoch, global_reference_generation, evaluation_role);
  }
  // 🔴 2026-08-17: 내부 불변식 위반이 **노드를 죽이지 않게** 한다.
  //
  // 이 파일에는 후보 예산·구간 포함관계 같은 불변식을 지키는 throw std::runtime_error가
  // 있다. 그런데 이 함수는 (a) local_planner_node의 타이머 콜백에서, (b) plan() 안에서
  // 불린다. ROS 2 콜백을 넘어간 예외는 executor를 타고 나가 **local_planner_node를 통째로
  // 종료시킨다** — 주행 중이면 회피도 안전정지도 남지 않는다. 종전에는 이 경계에 catch가
  // 없었고, 노드 전체를 통틀어 catch는 진단용 std::stoll 하나뿐이었다.
  //
  // 불변식 위반은 버그이므로 조용히 삼키지 않는다 — 호출자가 볼 수 있도록 실패 분류를
  // 남기고, 노드는 ERROR 로그를 던진다. 다만 그 대가가 "노드 사망"이어서는 안 된다.
  // 평가 실패로 떨어지면 상위는 후보를 못 찾았을 때와 **같은 경로**(안전정지 사다리)를
  // 탄다. 이것이 이미 검증된 실패 경로다.
  try {
    P3ShadowResult result = evaluateP3ShadowUnguarded(
      ego, obstacles, snapshot_source_stamp_ns, snapshot_epoch,
      global_reference_generation, p0_failure_reason);
    if (research_cycle != nullptr) {
      research_cycle->active_evaluation_lineage.reset();
    }
    return result;
  } catch (const std::exception & error) {
    P3ShadowResult failed;
    failed.enabled = true;
    failed.snapshot_source_stamp_ns = snapshot_source_stamp_ns;
    failed.snapshot_epoch = snapshot_epoch;
    failed.global_reference_generation = global_reference_generation;
    failed.p0_failure_reason = p0_failure_reason;
    failed.failure_classification =
      std::string("EVALUATOR_INVARIANT_VIOLATION: ") + error.what();
    if (research_cycle != nullptr) {
      captureP3ResearchEvaluation(*research_cycle, "INVARIANT", failed);
      research_cycle->active_evaluation_lineage.reset();
    }
    return failed;
  }
}

P3ShadowResult RacelineSplinePlanner::evaluateP3ShadowUnguarded(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles,
  std::int64_t snapshot_source_stamp_ns,
  std::uint64_t snapshot_epoch,
  std::uint64_t global_reference_generation,
  const std::string & p0_failure_reason) const
{
  const P3ShadowEvaluator evaluator(*this);
  std::vector<P3R3K12ProductionCandidate> strict_production_factors;
  P3ShadowResult strict = evaluator.run(
    ego, obstacles, snapshot_source_stamp_ns, snapshot_epoch,
    global_reference_generation, p0_failure_reason, false,
    &strict_production_factors);
  if (activeResearchCycle() != nullptr) {
    captureP3ResearchEvaluation(*activeResearchCycle(), "STRICT", strict);
  }
  if (strict.would_recover || !strict.invoked || strict.cluster_obstacle_ids.empty()) {
    return strict;
  }
  // strict(레이스 속도 예약) 게이트가 후보를 하나도 통과시키지 못했다. localization_reserve
  // 인상(0.06→0.12, 2026-08-15) 이후 strict 최소 target이 벽 캡 바로 앞까지 밀려 후보 전체가
  // footprint 검사에서 죽는 구간이 실측됐다(map s=16.5: 우측 여유 0.945 m에서 전멸). 기존
  // 감속-게이트 재시도는 "strict가 트랙에 안 들어갈 때"만 발동해 이 경우를 놓친다. 여기서
  // avoidance_minimum_speed_mps 게이트로 고려 범위만 넓혀 한 번 더 돈다 — 수용 기준(정확
  // 검증, gap 기반 속도 상한)은 동일하므로 "느리지만 가능한" 통로만 추가로 살아난다.
  P3ShadowResult relaxed = evaluator.run(
    ego, obstacles, snapshot_source_stamp_ns, snapshot_epoch,
    global_reference_generation, p0_failure_reason, true);
  if (activeResearchCycle() != nullptr) {
    captureP3ResearchEvaluation(*activeResearchCycle(), "RELAXED", relaxed);
  }
  if (std::getenv("P3_DEBUG_RELAXED") != nullptr) {
    std::fprintf(stderr, "[RELAXED] recover=%d fail=%s candidates=%zu\n",
      relaxed.would_recover ? 1 : 0, relaxed.failure_classification.c_str(),
      relaxed.candidates.size());
    for (const auto & trace : relaxed.candidates) {
      std::fprintf(stderr, "[RELAXED]  gen=%zu left=%d target=%.3f hard=%d rej=%s\n",
        trace.generation_index, trace.go_left ? 1 : 0, trace.d_target,
        trace.hard_valid ? 1 : 0, trace.rejection_reason.c_str());
    }
  }
  if (relaxed.would_recover) {
    relaxed.selected_source += "+RELAXED_CLEARANCE_GATE";
    return relaxed;
  }
  P3ShadowResult r3 = evaluator.runR3K12(
    ego, obstacles, relaxed, strict_production_factors);
  if (activeResearchCycle() != nullptr) {
    captureP3ResearchEvaluation(*activeResearchCycle(), "R3_RECOVERY", r3);
  }
  if (r3.would_recover) {
    return r3;
  }
  P3ShadowEvaluator::copyR3Audit(relaxed, r3);
  return relaxed;
}

}  // namespace local_planning
