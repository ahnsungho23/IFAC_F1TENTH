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

#include "local_planning/safe_stop_lifecycle.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace local_planning
{
namespace
{

constexpr double kLifecycleEpsilon = 1.0e-9;

double wrapS(double s, double track_length_m)
{
  if (!(track_length_m > kLifecycleEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length_m);
  return s < 0.0 ? s + track_length_m : s;
}

double forwardDistance(double from_s, double to_s, double track_length_m)
{
  return wrapS(to_s - from_s, track_length_m);
}

}  // namespace

void SafeStopLifecycle::activate(SafeStopActivation activation)
{
  std::sort(activation.obstacle_ids.begin(), activation.obstacle_ids.end());
  activation.obstacle_ids.erase(
    std::unique(activation.obstacle_ids.begin(), activation.obstacle_ids.end()),
    activation.obstacle_ids.end());
  activation_ = std::move(activation);
  active_ = true;
  feasible_avoidance_count_ = 0;
  stopped_clear_count_ = 0;
  last_counted_clear_sequence_ = 0;
}

void SafeStopLifecycle::reset()
{
  active_ = false;
  activation_ = SafeStopActivation();
  feasible_avoidance_count_ = 0;
  stopped_clear_count_ = 0;
  last_counted_clear_sequence_ = 0;
}

bool SafeStopLifecycle::active() const
{
  return active_;
}

const SafeStopActivation & SafeStopLifecycle::activation() const
{
  return activation_;
}

SafeStopCycleDecision SafeStopLifecycle::evaluate(
  const SafeStopCycleInput & input,
  double track_length_m,
  double obstacle_pass_margin_m,
  double stopped_speed_threshold_mps,
  int release_confirmation_cycles)
{
  SafeStopCycleDecision decision;
  if (!active_) {
    decision.raceline_global_handoff_allowed = true;
    return decision;
  }

  const double driven_distance = forwardDistance(
    activation_.activation_ego_s, input.ego_s, track_length_m);
  const bool before_one_lap_guard =
    std::isfinite(driven_distance) && driven_distance < 0.5 * track_length_m;
  decision.obstacle_passed = before_one_lap_guard &&
    driven_distance + kLifecycleEpsilon >=
    activation_.danger_end_distance_m + std::max(0.0, obstacle_pass_margin_m);
  decision.vehicle_stopped = std::isfinite(input.ego_speed_mps) &&
    std::abs(input.ego_speed_mps) <= std::max(0.0, stopped_speed_threshold_mps);

  // The streak counts plan validity only. While the hold path is degenerate the state machine
  // can flap AVOID<->GLOBAL every tick (2026-08-13 real-car deadlock: run_0813_220641 —
  // hard-valid escape existed each cycle but GLOBAL ticks kept resetting this counter, so
  // condition B never released without a manual push). FSM selectability is enforced at the
  // release decision below instead, so a release still only fires on a tick the state machine
  // would actually forward the avoidance path.
  if (input.hard_valid_avoidance_for_latched_obstacle) {
    ++feasible_avoidance_count_;
  } else {
    feasible_avoidance_count_ = 0;
  }

  // 빈 프레임 취급 (2026-08-21, run_052119 t=62~69 실차 교착):
  // "빈 배열은 clear 증거가 아니다"를 무조건 적용하면, 래치 장애물 곁을 지나치며 멈춰
  // 목록 전체가 비는 순간 A(전진 필요)·B(kAvoidance 결과 필요)·C(비면 불가)가 전부
  // 봉쇄되는 영구 교착이 된다 — 실차에서 사람이 estop 으로 구출해야 했다. 빈 프레임은
  // ① 래치 이후에 도착한 신선한 프레임이고(검출기 생존 증거 — 시퀀스는 빈 메시지에도
  // 증가한다) ② 래치가 기억한 위험구간 끝을 ego 가 이미 지났을 때에 한해 clear 증거로
  // 센다. ②가 근접 사각 보호의 본체다: 기억 속 상자가 아직 전방이면 빈 프레임으로는
  // 절대 풀리지 않는다(그 경우 눈이 아니라 기억을 믿는 게 맞다).
  const bool latched_danger_behind = before_one_lap_guard &&
    driven_distance + kLifecycleEpsilon >= activation_.danger_end_distance_m;
  const bool fresh_post_latch_frame =
    input.obstacle_sequence > activation_.obstacle_sequence;
  const bool corridor_clear_evidence = input.static_obstacles_empty ?
    (latched_danger_behind && fresh_post_latch_frame) :
    input.explicit_forward_corridor_clear;
  if (decision.vehicle_stopped && corridor_clear_evidence) {
    if (input.obstacle_sequence != last_counted_clear_sequence_) {
      ++stopped_clear_count_;
      last_counted_clear_sequence_ = input.obstacle_sequence;
    }
  } else {
    stopped_clear_count_ = 0;
    last_counted_clear_sequence_ = 0;
  }

  const int required_cycles = std::max(1, release_confirmation_cycles);
  decision.feasible_avoidance_count = feasible_avoidance_count_;
  decision.stopped_clear_count = stopped_clear_count_;
  decision.release_condition_a = decision.obstacle_passed;
  decision.release_condition_b = feasible_avoidance_count_ >= required_cycles &&
    input.state_can_select_avoidance;
  decision.release_condition_c = stopped_clear_count_ >= required_cycles;

  if (decision.release_condition_a) {
    decision.release_authorized = true;
    decision.release_reason = SafeStopReleaseReason::kObstaclePassed;
    decision.raceline_global_handoff_allowed = true;
  } else if (decision.release_condition_b) {
    decision.release_authorized = true;
    decision.release_reason = SafeStopReleaseReason::kValidAvoidance;
    // Condition B selects the avoidance path. It does not authorize a global-raceline handoff.
    decision.raceline_global_handoff_allowed = false;
  } else if (decision.release_condition_c) {
    decision.release_authorized = true;
    decision.release_reason = SafeStopReleaseReason::kStoppedCorridorClear;
    decision.raceline_global_handoff_allowed = true;
  }
  return decision;
}

const char * safeStopReleaseReasonName(SafeStopReleaseReason reason)
{
  switch (reason) {
    case SafeStopReleaseReason::kObstaclePassed:
      return "obstacle_passed_with_margin";
    case SafeStopReleaseReason::kValidAvoidance:
      return "hard_valid_avoidance_selectable";
    case SafeStopReleaseReason::kStoppedCorridorClear:
      return "stopped_and_forward_corridor_persistently_clear";
    case SafeStopReleaseReason::kNone:
    default:
      return "none";
  }
}

}  // namespace local_planning
