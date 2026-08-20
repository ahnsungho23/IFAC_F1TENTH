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

#ifndef LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
#define LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include "local_planning/p3_shadow.hpp"

namespace local_planning
{

class P3ShadowEvaluator;

struct RacelineSplineParameters
{
  double detection_lookahead_m{12.0};
  double obstacle_cluster_gap_m{0.8};
  double obstacle_longitudinal_padding_m{0.35};
  double vehicle_length_m{0.56};
  double vehicle_half_width_m{0.1435};
  // Obstacle bounds are raw detector geometry, so vehicle size, physical margin, and measured
  // closed-loop tracking error are applied exactly once. Global d_left/d_right are distances to
  // physical track boundaries; the rotated vehicle footprint and independent wall reserve are
  // therefore checked against those boundaries in the hard validator.
  double safety_margin_m{0.03};
  // Fallback used when the LUT arrays below are all empty. A configured LUT is row-major with
  // speed as the outer axis and absolute curvature as the inner axis.
  double tracking_error_reserve_m{0.14};
  std::vector<double> tracking_error_lut_speed_bins_mps;
  std::vector<double> tracking_error_lut_curvature_bins_radpm;
  std::vector<double> tracking_error_lut_values_m;
  // Speed-dependent lateral-acceleration table copied from jazzy_main velocity_limits.csv.
  // Avoidance waypoint speeds are capped so v^2 * |kappa| stays within the interpolated limit.
  std::vector<double> avoidance_velocity_limit_speed_bins_mps;
  std::vector<double> avoidance_velocity_limit_lateral_accel_mps2;
  // Speed-dependent LONGITUDINAL limits, taken from the max_accel / max_decel columns of
  // offline_trajectory_generator/config/velocity_limits.csv — the very table the raceline
  // generator uses, so planner and generator finally share one vehicle model (2026-08-19).
  // Both share avoidance_velocity_limit_speed_bins_mps as their speed axis.
  //
  // 🔴 이게 왜 들어왔나 (2026-08-19 실차 백 5개, 자율주행 구간만 계측):
  //  - 가속: 이 패스가 생기기 전에는 **전진 제약이 아예 없었다**. applyLongitudinalFeasibility는
  //    후방(감속) 패스 하나뿐이라, 캡이 한 점을 눌러도 다음 점이 라인 속도로 되튀는 것을
  //    아무도 막지 않았다. 발행 경로의 가속 요구가 csv 한계를 넘은 비율이 전체 55%,
  //    v 5~9 m/s 구간에서 93.5%였고, 정지 후 출발 구간(v 2~3)은 요구 p90 이 30 m/s² 였다
  //    (= 경로 첫 점이 자차 실측 속도가 아니라 라인 속도를 그대로 실어서 생긴 계단).
  //  - 감속: 상수 3.5 하나로 전 속도를 덮고 있었는데 csv 는 v>=4 에서 2.0 이다. 그 결과
  //    감속 요구가 csv 를 넘은 비율이 v 4~5 에서 55.2%, v 5~9 에서 84.2% 였다.
  // 비어 있으면 표가 없는 것으로 보고 종전 스칼라 거동으로 폴백한다(파라미터 파일이 옛것이어도
  // 노드가 죽지 않게).
  std::vector<double> avoidance_velocity_limit_accel_mps2;
  std::vector<double> avoidance_velocity_limit_decel_mps2;
  // 전진 패스의 시드 하한 [m/s]. 시드는 자차의 **실측** 속도인데, 정지 상태(0)에서 그대로
  // 시작하면 경로 첫 점이 0 근처로 눌려 컨트롤러의 lookahead 가 그 점을 집고 출발을 못 한다.
  // 전진 패스는 속도를 낮추기만 하므로(min) 이 하한이 안전정지 0 프로파일을 되살리지는 않는다.
  double longitudinal_launch_speed_floor_mps{1.0};
  // The lateral-acceleration cap above only binds in curves, so an obstacle sitting on a straight
  // is planned at full race-line speed and reserves the widest tracking error in the LUT -- which
  // is what makes an otherwise passable gap unusable. Slow down for the gap itself instead: the
  // reserve a waypoint may spend is whatever lateral room is left between the path and the
  // obstacle face, and speed is reduced only until the LUT reserve fits inside it. A waypoint that
  // is not passing an obstacle is never slowed, so obstacle-free laps keep their race-line speed.
  // Below this floor the maneuver is treated as infeasible rather than crawled through.
  double avoidance_minimum_speed_mps{2.0};
  double wall_safety_margin_m{0.0};
  double fallback_track_half_width_m{1.50};
  // Speed cap for the margin-only slow pass: a cluster that blocks the race line only through
  // the inflated tracking/uncertainty margin (its raw envelope plus the physical base clearance
  // never reaches d = 0) is physically passable on the line, so avoidance failure degrades to a
  // capped-speed lane hold instead of a safe stop or a zero-speed hold.
  double margin_pass_speed_cap_mps{2.0};
  // Approach feasibility ramp: a slow section that begins abruptly (margin pass flat cap from
  // ego, gap/curvature-capped obstacle span) is a speed STEP the vehicle cannot track — the
  // service brake saturates, the tires slip past the friction limit and steering authority is
  // lost (2026-08-14 real-car wall crashes: 4.4 m/s ego vs flat 2.0 m/s plan). Two shapes, one
  // rate, both only active when the ego/profile is faster than the slow section:
  //  - margin slow pass: pre-cluster waypoints may keep a profile that decelerates from the
  //    MEASURED ego speed at this rate, steepened only as much as needed to still reach the cap
  //    by the cluster start (degrades to the old flat cap when there is no room).
  //  - avoidance spline: pre-span waypoints are LOWERED onto the backward braking profile that
  //    reaches the span-start speed at this rate, so braking starts well before the span
  //    boundary instead of as a step at it. Span speeds themselves are never touched.
  // <= 0 disables both (old step behavior).
  double approach_feasibility_decel_mps2{2.0};
  // Adaptive upper bound for the avoidance-span approach ramp (2026-08-16). The ramp slope is
  // no longer fixed: per span it uses the GENTLEST decel that still reaches the span speed
  // within the available run-up, clamped to [approach_feasibility_decel_mps2, this]. Obstacles
  // with a generous approach keep the comfortable base rate unchanged; only geometrically tight
  // gaps (e.g. re-acceleration to 6+ m/s between obstacles 8 m apart) steepen, and never beyond
  // this cap — sized at ~70% of the raceline's own braking limit (5.0). A gap that needs more
  // than the cap keeps the capped ramp and falls to the safe-stop ladder as before.
  double approach_feasibility_decel_max_mps2{3.5};
  // Longitudinal-feasibility backward pass (2026-08-16). The per-waypoint curvature and gap caps
  // are computed independently, so a curvature dip (the inflection of an S-transition) lets the
  // profile snap back to raceline speed for a point or two and then fall again — 14:30 백에서
  // 6.63 → 4.58 m/s in 0.25 m, i.e. a 46 m/s² braking request. This is the deceleration used to
  // make every drop in the published profile actually brakeable.
  //
  // 기본값이 접근 램프의 comfort base(2.0)가 아니라 그 상한(3.5)인 이유: 이 패스의 목적은
  // "물리적으로 불가능한 계단"을 없애는 것이고, 그 기준은 comfort 목표가 아니라 차가 실제로
  // 낼 수 있는 제동이다. 2.0으로 두면 곡률이 조금만 출렁여도 프로파일 전체가 끌려 내려가
  // 근거 없이 속도를 잃는다. 접근 램프의 comfort base는 그대로 유지된다.
  double profile_feasibility_decel_mps2{3.5};
  // Committed-path retention band: while re-validating an ALREADY COMMITTED path (P3
  // continuation, P0 commitment hold), the tracking-error reserve portion of the obstacle
  // clearance is scaled by this fraction, so envelope growth/jitter inside the released band
  // freezes the path instead of reshaping it every callback. The physical base clearance is
  // never reduced, and fresh planning always uses the full reserve. 1.0 disables the band.
  double commitment_retention_reserve_fraction{0.5};
  // Localization (MCL vs ground-truth) lateral error reserve, added as a constant floor inside
  // trackingErrorReserve() so every consumer (envelope expansion, hard validation, gap-limited
  // speed inversion) sees the same total. Participates in the retention scaling above like the
  // rest of the reserve. Sized from sustained error (per-speed-bin P95), NOT transient MCL
  // correction spikes — those are single-cycle events the retention band absorbs. 0 disables.
  double localization_reserve_m{0.0};

  std::vector<double> pre_apex_distances_m{6.0, 4.0, 2.0};
  std::vector<double> post_apex_distances_m{1.0, 2.0, 3.0};
  // Entry and exit intentionally use separate parameter families. Entry fractions scale the
  // available-distance ratio pre_apex_far/detection_lookahead; transition scales are exit-only.
  std::vector<double> entry_transition_fractions{0.50, 0.75, 1.00};
  std::vector<double> transition_distance_scales{1.0, 1.25, 1.50};
  double outside_line_transition_scale{1.35};
  // Absolute cap on the exit segment length past the obstacle cluster. Disabled (non-positive)
  // by default: capping the exit forces the path back to the race line early, and when a second
  // obstacle sits a few metres downstream the shortened exit dives straight into its inflated
  // box the moment the detector reveals it, invalidating the committed maneuver at a range where
  // no fresh plan exists yet (regression observed 2026-08-12 with obstacles 3.6 m apart). The
  // long exit deliberately keeps the offset through closely-following obstacles; merge latency
  // is solved by the completion handback to the closed global handoff loop instead.
  double maximum_exit_length_m{0.0};
  double post_merge_lookahead_m{2.0};
  double post_merge_min_time_sec{1.0};
  // 완료 핸드오프 복귀 램프: 길이 = max(min_length, |v| * time). 너무 짧으면 램프의
  // 추가 곡률(최대 6|d0|/L^2)이 커지므로 min_length가 하한을 지킨다.
  // 🔴 기본 0 = 비활성 (2026-08-13). 시뮬 회귀에서 램프가 벽 협착부(s≈23) 최소 벽 여유를
  // 0.117→0.082 m로 깎았고, 벽 클램프는 웨이포인트 d_left/d_right가 실제 벽보다 낙관적이라
  // (제어팀 실측 0.16~0.23 m) 물리지 않았다. 제어팀 섹터별 벽 여유 실측 테이블로 경계를
  // 보정한 뒤에만 활성화할 것 — 낙관 경계로 켜면 협착부 벽 여유를 그대로 깎는다.
  double merge_ramp_min_length_m{0.0};
  double merge_ramp_time_sec{0.0};
  double minimum_target_offset_m{0.20};
  double maximum_target_offset_m{1.50};
  int target_d_candidate_count{5};
  double maximum_lateral_slope{0.65};
  // 진입 불연속 검사(validatePath)의 **추종오차 예산 하한** [m] (2026-08-20 신설).
  //
  // 그 검사는 두 조건의 AND 다:
  //   (1) |d_진입점 − d_자차| > 예산            ← 오탐 방지 장치
  //   (2) |d_진입점 − d_자차| / 전방거리 > maximum_lateral_slope
  // 예산은 trackingErrorReserve() = localization_reserve_m + tracking_error_lut 인데,
  // 그 둘은 **장애물 마진 정책**의 값이다. 마진을 0 으로 두는 구성(프로토타입)에서는
  // 예산이 0 이 되어 (1) 이 "간격 > 0" 즉 상시 참이 되고, AND 가 (2) 하나로 무너진다.
  // 진입점은 자차 앞 3~14 cm 에 놓이므로 분모가 작아 기울기가 쉽게 문턱을 넘는다.
  //
  // 🔴 2026-08-19 run_001453 실측(자율 327.3 s): P3 기동 무효화 36 건 중 **24 건(67%)**
  //    이 이 검사였고, 기동 생성 후 무효화까지 p50 **110 ms**(97% 가 0.5 s 미만)였다.
  //    그 왕복이 안전정지 자율 시간의 24%, 0.8 초에 커밋 5 회 교체, 경로 d 부호 전환
  //    43 회(차가 1 초에 0.29 m 좌우로 끌림)로 이어졌다.
  //
  // 이 하한은 마진이 아니라 **"이 정도 간격은 정상 추종오차다"라는 검사의 기준선**이므로
  // 마진 정책과 분리해 둔다.
  //
  // 🔑 값을 정하는 기준은 "추종오차가 얼마나 큰가"가 아니라 **"진입점이 얼마나 가까운가"**다.
  //    기각은 lateral > 0.8·forward 일 때 나므로, 하한 F 가 그 오탐을 덮으려면
  //        0.8·forward < lateral <= F      즉      forward < F / 0.8
  //    이어야 한다. 즉 F 는 **덮고 싶은 진입거리 × 0.8** 로 정해진다.
  //    2026-08-19 run_001453 자율 발행 경로 8938 건의 진입거리(entry_forward) 분포:
  //        p10 0.031  p25 0.069  p50 0.138  p75 0.205  p90 0.243 m
  //    하한별로 덮이는 비율:
  //        F=0.10 → forward<0.125 → 45.2%
  //        F=0.15 → forward<0.187 → 67.8%
  //        F=0.20 → forward<0.250 → 92.8%   ← 무릎. 여기서 포화한다
  //        F=0.25 → forward<0.312 → 92.9%
  //        F=0.30 → forward<0.375 → 93.0%
  //    0.20 위로는 덮이는 양이 사실상 안 늘고 진짜 불연속 탐지만 무뎌지므로 0.20 을 쓴다.
  //    (초안은 추종오차 p50 대역을 보고 0.15 로 잡았는데, 그건 **덮어야 할 대상의 크기**이지
  //     **덮는 데 필요한 크기**가 아니었다 — 필요한 값은 0.8·진입거리다.)
  //
  // 진짜 불연속은 그대로 걸린다: 코드가 잡아야 한다고 명시한 실해 사례가 간격 0.447 m 로
  // 이 하한의 2.2 배다. 0 으로 두면 하한이 꺼지고 종전(마진 종속) 거동으로 돌아간다.
  double entry_discontinuity_min_budget_m{0.20};
  double maximum_curvature_radpm{3.20};
  double maximum_curvature_rate_radpm2{20.0};

  double safe_stop_buffer_m{0.40};
  double safe_stop_deceleration_mps2{2.5};
  int minimum_path_points{8};

  // 안전정지 정지점 탈출 검증. safe_stop_buffer_m은 손으로 맞춘 상수라, 기하에 따라
  // "정지는 했는데 그 자리에서 회피 곡선을 만들 진입 거리가 없는" 영구 교착이 생긴다
  // (2026-08-14 실차: 임계 2.0~2.5 m vs 버퍼 1.20 m — 모든 안전정지가 교착이었다).
  // 켜면 정지점을 확정하기 전에 그 지점에서 v=0으로 회피가 생성되는지 확인하고,
  // 안 되면 safe_stop_escape_retreat_step_m씩 뒤로 물린다.
  bool safe_stop_escape_check_enable{true};
  // 한 스텝 후퇴 거리와 최대 후퇴 횟수. step × max_steps가 버퍼에 더해질 수 있는 최대
  // 후퇴량이다. 후보 생성을 그만큼 반복하므로 무한정 키우면 안 된다.
  double safe_stop_escape_retreat_step_m{0.30};
  int safe_stop_escape_max_retreats{8};

  bool hasTrackingErrorLut() const;
  bool trackingErrorLutValid() const;
  bool avoidanceVelocityLimitValid() const;
  // 종방향 표(가속/감속)가 쓸 수 있는 상태인가. 둘 다 비면 true(표 없음 = 스칼라 폴백).
  bool longitudinalVelocityLimitValid() const;
  // 표 보간. 표가 없거나 무효면 0.0 을 돌려주며, 호출부는 그때 스칼라로 폴백한다.
  // 횡가속 표와 달리 단조성을 요구하지 않는다 — 이분법이 아니라 단순 보간에만 쓰인다.
  double accelLimitAt(double speed_mps) const;
  double decelLimitAt(double speed_mps) const;
  double limitedAvoidanceSpeed(double requested_speed_mps, double curvature_radpm) const;
  // Largest speed not above `requested_speed_mps` whose tracking-error reserve still fits inside
  // `admissible_reserve_m`, never going below avoidance_minimum_speed_mps. The reserve is
  // monotonically non-decreasing in speed, so this is a plain monotone inversion of the LUT.
  double gapLimitedAvoidanceSpeed(
    double requested_speed_mps, double curvature_radpm, double admissible_reserve_m) const;
  // Clamp a combined (outside-multiplier-applied) exit transition scale so that
  // post_apex_distances_m.back() * scale never exceeds maximum_exit_length_m.
  double cappedCombinedExitScale(double combined_exit_scale) const;
  double trackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double avoidanceTrackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double obstacleBaseClearance() const;
  double obstacleSafetyClearance(
    double speed_mps, double curvature_radpm, double reserve_scale = 1.0) const;
  double trackBoundaryReserve(double speed_mps, double curvature_radpm) const;
  // Deceleration used by the longitudinal-feasibility backward pass. Falls back to the approach
  // ramp's adaptive cap when unset so an out-of-date parameter file cannot silently disable it.
  double profileFeasibilityDecel() const
  {
    return profile_feasibility_decel_mps2 > 0.0 ?
           profile_feasibility_decel_mps2 : approach_feasibility_decel_max_mps2;
  }
};

struct EgoFrenetState
{
  double s{0.0};
  double d{0.0};
  double speed{0.0};
};

enum class SplinePlanKind
{
  kNoObstacle,
  kPreparation,
  kAvoidance,
  kSafeStop,
  kNoSafePath
};

struct SplineControlPoint
{
  double forward_s{0.0};
  double d{0.0};
};

// Complete, passive record of one generated candidate. Metrics are calculated before hard
// validation so rejected candidates remain auditable. final_rank is one-based for feasible
// candidates and -1 for rejected candidates.
struct SplineCandidateAudit
{
  std::size_t generation_index{0U};
  bool feasible{false};
  bool selected{false};
  bool go_left{false};
  int final_rank{-1};
  // exit 램프가 다음(비클러스터) 장애물의 물리 엔벨로프에 닿는 후보인가 — 순위 강등의
  // 원인 플래그. rank_without_exit_demotion은 그 강등 항을 뺀 순수 slack 순위로,
  // final_rank와 다르면 강등이 이 사이클의 선택을 실제로 바꿨다는 뜻이다. 감사 전용.
  bool exit_reaches_next_obstacle{false};
  int rank_without_exit_demotion{-1};
  double target_d{std::numeric_limits<double>::quiet_NaN()};
  double entry_fraction{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance_m{
    std::numeric_limits<double>::quiet_NaN()};
  bool footprint_invalid{false};
  std::string footprint_violation_side;
  std::int64_t footprint_violation_waypoint_index{-1};
  double footprint_violation_s_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_x_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_y_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_violation_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double footprint_heading_relative_to_reference_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion_m{std::numeric_limits<double>::quiet_NaN()};
  // Compatibility metric used by the existing candidate ranker. It remains the centerline
  // headroom so this hard-validator change does not silently alter candidate ranking semantics.
  double wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_radpm{std::numeric_limits<double>::quiet_NaN()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::quiet_NaN()};
  double velocity_loss{std::numeric_limits<double>::quiet_NaN()};
  double global_path_deviation_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  std::string rejection_reason;
};

struct RacelineSplineResult
{
  SplinePlanKind kind{SplinePlanKind::kNoObstacle};
  f110_msgs::msg::WpntArray path;
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  // Passive decision metadata used by deterministic replay diagnostics. These values do not
  // participate in candidate selection or path generation.
  double entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double effective_exit_transition_scale{std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  int obstacle_id{-1};
  std::vector<int> obstacle_ids;
  std::vector<SplineControlPoint> control_points;
  std::vector<SplineCandidateAudit> candidate_audits;
  // True for a margin-only slow pass: the path intentionally rides inside the inflated margin
  // band, so margin-based commitment validation must not replace it; it stays valid until an
  // obstacle's raw envelope (plus the physical base clearance) actually reaches the race line.
  bool margin_pass{false};
  // 안전정지 진단. escape_verified가 false면 정지점(그리고 자차 위치까지의 모든 후퇴
  // 지점)에서 회피 후보가 하나도 생성되지 않는다 — 전진 계획으로는 재출발할 수 없는
  // 상태이므로 노드가 이를 로그로 드러내야 한다. forward_m은 자차 기준 정지점 거리다.
  bool safe_stop_escape_verified{true};
  double safe_stop_forward_m{std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
};

enum class PathValidationFailureKind
{
  kNone,
  kInput,
  kNoForwardPath,
  kTrackBoundary,
  kObstacleCollision,
  kGeometry
};

struct PathValidationFailure
{
  PathValidationFailureKind kind{PathValidationFailureKind::kNone};
  std::string reason;
  int obstacle_id{-1};
  std::size_t waypoint_index{std::numeric_limits<std::size_t>::max()};
  double waypoint_s{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_d{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_start{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_end{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance{std::numeric_limits<double>::quiet_NaN()};
  double centerline_wall_clearance{std::numeric_limits<double>::quiet_NaN()};
  double rectangular_footprint_wall_clearance{
    std::numeric_limits<double>::quiet_NaN()};
  std::string footprint_violation_side;
  double waypoint_x{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_y{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_yaw{std::numeric_limits<double>::quiet_NaN()};
  double heading_relative_to_reference{std::numeric_limits<double>::quiet_NaN()};
  double wallward_corner_protrusion{std::numeric_limits<double>::quiet_NaN()};
};

// Static-obstacle planner whose only geometric reference is the ordered global race line.
// A candidate never searches the map for a shortcut: it keeps every selected global waypoint's
// s/order and changes only its local Frenet d offset before converting it back to map coordinates.
class RacelineSplinePlanner
{
public:
  explicit RacelineSplinePlanner(
    RacelineSplineParameters parameters = RacelineSplineParameters());

  void setParameters(const RacelineSplineParameters & parameters);
  bool setReference(const f110_msgs::msg::WpntArray & reference, std::string * error = nullptr);
  bool ready() const;
  double trackLength() const;
  double forwardDistance(double from_s, double to_s) const;
  std::vector<int> blockingClusterIds(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  // True when any visible obstacle's RAW envelope plus the physical base clearance
  // (vehicle half width + safety margin, no tracking-error reserve) reaches the race line —
  // i.e. the vehicle could not physically follow d = 0 without touching it. Margin-only
  // blocking (inflated interval touches the line but the raw one stays clear) returns false.
  bool obstaclesPhysicallyBlockRaceline(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  // 완료 핸드오프용 글로벌 루프. ego의 현재 횡오프셋 d에서 d=0까지 smoothstep 램프로
  // 내려가는 계획된 복귀 구간을 앞머리에 접붙인다 — 램프 없이 d=0 라인을 그대로 주면
  // 복귀가 컨트롤러의 자연 수렴에 맡겨져 실측 0.055 m/m로 느리고(2026-08-12), 연속
  // 장애물에서 오프셋이 누적된다. FSM 합류 판정(|ego_d| <= threshold 지속)은 램프와
  // 무관하게 물리적 합류를 계속 게이트한다.
  f110_msgs::msg::WpntArray buildGlobalHandoffPath(
    const EgoFrenetState & ego, double state_tail_distance_m, double speed_cap_mps) const;
  // B1 raw 감속 힌트 경로 (2026-08-20). confirmed 승격 전의 raw(/static_obs) 장애물을
  // 향해 접근할 때, 라인 기하는 그대로 두고 속도만 낮춘 글로벌 루프를 만든다:
  // 전방 obstacle_front_m 지점에서 cap_mps 에 닿도록 decel_mps2 의 실현 가능 램프로
  // 줄이고, 장애물 스팬+1 m 동안 cap 을 유지한 뒤 글로벌 속도로 되돌린다. 회피 기하·
  // 커밋·정지는 만들지 않는다 — 이 경로의 역할은 confirmed 승격이 끝나기 전에 접근
  // 속도를 미리 깎아 (a) 승격 지연이 잡아먹는 거리를 줄이고 (b) 접근 계단 감속을
  // 없애는 것뿐이다.
  f110_msgs::msg::WpntArray buildRawSlowdownPath(
    const EgoFrenetState & ego, double state_tail_distance_m,
    double obstacle_front_m, double obstacle_span_m,
    double cap_mps, double decel_mps2) const;
  f110_msgs::msg::WpntArray buildEmergencyStopPath(const EgoFrenetState & ego) const;
  // Truncate `path` from the waypoint nearest ahead of ego and apply a braking profile. Used as
  // the last-resort stop geometry when no collision-free stop prefix exists: braking along the
  // most recent vetted path beats an instantaneous zero-speed hold on a moving vehicle.
  //
  // 🔴 2026-08-16: 이 함수는 종전에 장애물을 전혀 보지 않고 stop_at = v²/(2·a)만 썼다.
  // 그래서 장애물이 그 제동거리보다 가까우면 정지 목표가 **장애물 뒤**에 놓였고, 차는 그
  // 경로를 충실히 따라 장애물로 들어갔다. 15:37 백 실측: v=3.26에서 정지목표 2.95 m 뒤,
  // 접촉점은 1.37 m 앞 → 접촉 시점 명령속도 √(2·1.8·(2.95−1.37)) = 2.39 m/s, 실측 충돌
  // 속도 2.39와 일치. 랩마다 같은 자리에서 4회 충돌했다.
  //
  // 이제 obstacles를 받아 정지 목표를 첫 충돌 지점 이전으로 자른다. 그렇게 하면 요구
  // 감속이 safe_stop_deceleration_mps2를 넘을 수 있는데, 그것이 의도다 — "설정 감속으로는
  // 못 선다"는 "서면 안 된다"가 아니다. 못 서면 한계를 넘겨서라도 최대한 일찍 세워야 하고,
  // 명령이 0에 닿지 않는 것보다 급제동이 언제나 낫다.
  f110_msgs::msg::WpntArray buildLastPathBrake(
    const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  RacelineSplineResult buildCommittedPathStop(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & committed_path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  RacelineSplineResult buildPreparationStop(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  RacelineSplineResult plan(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::optional<bool> & preferred_left = std::nullopt,
    bool allow_side_switch = true) const;

  // Production-owned analytic P3/M1 path-family evaluation. Runtime ownership is decided by
  // LocalPlannerNode's explicit mode; this const adapter cannot alter P0 planner state.
  P3ShadowResult evaluateP3Shadow(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::int64_t snapshot_source_stamp_ns,
    std::uint64_t snapshot_epoch,
    std::uint64_t global_reference_generation,
    const std::string & p0_failure_reason) const;

  // 위 함수가 try로 감싸는 실제 구현. 예외를 그대로 던지므로 직접 부르지 말 것.
  P3ShadowResult evaluateP3ShadowUnguarded(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::int64_t snapshot_source_stamp_ns,
    std::uint64_t snapshot_epoch,
    std::uint64_t global_reference_generation,
    const std::string & p0_failure_reason) const;

  // Revalidate a committed P3 suffix against the current immutable planning snapshot using the
  // same production hard validator and configured minimum-path contract as fresh P3 candidates.
  // `obstacle_reserve_scale` scales only the tracking-error reserve portion of the obstacle
  // clearance (the physical base clearance is never reduced); values < 1 form the committed-path
  // retention band. Fresh planning must always validate with the default full reserve.
  // `collision_horizon` bounds the OBSTACLE check to this maneuver's responsibility range exactly
  // as `generateP3Candidates` does at selection time; track-bound and geometry checks always cover
  // the whole path. Selection and revalidation MUST pass the same horizon: an obstacle that lies
  // inside one's range and outside the other's makes every cycle select a path the next cycle
  // condemns, which is an unbreakable replan loop, not a safety check.
  P3ShadowPathEvaluation evaluateP3PathCurrent(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double obstacle_reserve_scale = 1.0,
    const std::optional<double> & collision_horizon = std::nullopt) const;

  bool validatePath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::string * error = nullptr,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt,
    double obstacle_reserve_scale = 1.0) const;

  double commitmentRetentionReserveFraction() const
  {
    return parameters_.commitment_retention_reserve_fraction;
  }

  // Controller tail appended after the merge. It is also the single margin every maneuver-scope
  // collision horizon adds to its cluster end, so selection and revalidation stay identical.
  // 이 기동이 책임지는 구간의 끝 [자차 기준 전방 m].
  //
  // 기본은 `cluster_end + post_merge_lookahead_m`이다 — 장애물을 지난 뒤에도 차는 아직
  // 옆으로 나가 있으므로 탈출·합류 램프의 꼬리까지 봐야 한다.
  //
  // 🔴 2026-08-16: 그 고정 거리(5.0 m)가 **다음 클러스터를 삼키면**, 기동 N이 자기 책임이
  // 아닌 장애물 N+1로 심판받는다. 두 장애물이 반대편 통과를 요구하면 그 순간 기동 N은
  // 원천적으로 불가능해진다. 17:02/17:59 백 실측: obs9(s=32.7, 우측 필수)와
  // obs10(s=37.4, 좌측 필수)가 4.2 m 간격이고 지평은 5.0 m라, obs9 기동의 **모든** 후보가
  // obs10에서 걸렸다(1179건). 특히 짧은 탈출 후보는 s=36.907에서 d=+0.0000 — 즉 라인으로
  // 합류를 마친 상태에서 걸렸다. obs10의 박스가 d=0을 물고 있어 **라인 복귀 자체가 충돌**
  // 이었던 것이다. 깊이·후보 수로는 절대 풀리지 않는 형태다.
  //
  // 그래서 다음 클러스터의 확장 앞면에서 자른다. 그 장애물은 체인 기동이 맡는다 —
  // beginChainedManeuverIfNeeded가 정확히 그 용도로 있다. 잘린 뒤 꼬리가 다음 장애물에
  // 닿더라도 안전정지 사다리(buildCommittedPathStop/buildLastPathBrake)는 지평 없이 전
  // 장애물을 보므로 최후 보루는 유지된다.
  //
  // ⚠️ 후보 선택과 커밋 재검증이 **반드시 같은 값**을 써야 한다. 두 범위가 어긋나면 더
  // 엄격한 검사가 되는 게 아니라 수렴하지 않는 루프가 된다(2026-08-15 run18: 25 ms마다
  // 같은 후보를 다시 고르는 무한 재계획, 랩당 hard collision 41회).
  double maneuverScopeEnd(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<int> & cluster_ids,
    double cluster_end_forward_m) const;
  double postMergeLookaheadM() const
  {
    return parameters_.post_merge_lookahead_m;
  }

  void toCartesian(double s, double d, double & x, double & y, double & yaw) const;

private:
  friend class P3ShadowEvaluator;

  struct ExpandedObstacle;
  // Forward distance from ego to the first waypoint of `path` whose footprint intersects an
  // obstacle, or infinity when the path is clear. Shared by every stop-geometry builder so they
  // cannot disagree about where contact begins.
  double firstCollisionForward(
    const EgoFrenetState & ego, const f110_msgs::msg::WpntArray & path,
    const std::vector<ExpandedObstacle> & visible, int * obstacle_id = nullptr) const;
  struct Candidate;
  struct FootprintTrackBoundSample;

  double wrapS(double s) const;
  std::size_t nextReferenceIndex(double s) const;
  std::size_t nearestReferenceIndex(double s) const;
  double maximumReferenceTrackingErrorReserve(
    const EgoFrenetState & ego, double start, double end,
    double speed_cap_mps = std::numeric_limits<double>::infinity()) const;
  std::vector<ExpandedObstacle> expandVisibleObstacles(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool isBlockingRaceline(const ExpandedObstacle & obstacle) const;
  bool physicallyBlocksRaceline(const ExpandedObstacle & obstacle) const;
  bool clusterPhysicallyBlocksRaceline(
    const std::vector<ExpandedObstacle> & cluster) const;
  std::vector<ExpandedObstacle> nearestCluster(
    const std::vector<ExpandedObstacle> & obstacles) const;
  bool outsideIsLeft(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;
  // relaxed_clearance_gate=true는 strict 게이트의 전 후보가 exact validator에서 기각된 뒤의
  // 2차 시도 전용: 최소 clearance target을 avoidance_minimum_speed_mps 기준 게이트로 낮춘다.
  bool computeSideTargetRange(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster,
    bool go_left,
    double & cluster_start,
    double & cluster_end,
    double & minimum_clearance_target_d,
    double & maximum_track_target_d,
    std::string & reason,
    bool relaxed_clearance_gate = false) const;
  bool targetFitsTrackBounds(
    const EgoFrenetState & ego,
    double cluster_start,
    double cluster_end,
    bool go_left,
    double target_d,
    std::string & reason,
    double * min_headroom = nullptr) const;
  void measureCandidate(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    Candidate & candidate) const;
  FootprintTrackBoundSample measureFootprintTrackBound(
    const f110_msgs::msg::Wpnt & waypoint,
    std::size_t waypoint_index) const;
  // 이 패키지의 유일한 회피 후보 생성기 — P0 quintic 격자는 2026-08-15에 제거됐다
  // (실차 시험에서 P0가 통과 가능한 모든 곳을 P3도 통과함이 확인됨). P3(analytic
  // corridor)가 이 상태에서 만들어 낸 후보들을 Candidate로 변환해 append한다. plan()과
  // 안전정지 탈출 검증이 **같은 코드**로 후보를 만들어야 "정지점에서 회피 가능"이라는
  // 판정이 실제 재계획과 일치한다. 두 벌로 나뉘면 조용히 어긋난다. 안전 계층
  // (expandVisibleObstacles / measureCandidate / validateCandidate)과 안전정지 사다리는
  // 그대로이며, 여기서 만든 후보도 동일한 measureCandidate로 재측정한다.
  // 반환값은 이번 호출에서 생성된 feasible 후보 수.
  // stop_on_first_feasible=true면 첫 통과 후보에서 즉시 멈춘다(탈출 가능성만 물을 때).
  std::size_t generateP3Candidates(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<ExpandedObstacle> & visible,
    const std::optional<bool> & preferred_left,
    bool allow_side_switch,
    bool stop_on_first_feasible,
    std::vector<Candidate> & candidates,
    std::string & reason) const;
  // 주어진 자차 상태에서 회피 경로가 하나라도 생성되는가. 경로는 만들지 않고 가능성만 본다.
  bool anyFeasibleCandidateFrom(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::vector<ExpandedObstacle> & visible,
    const std::vector<ExpandedObstacle> & cluster) const;
  // 경로 점 수가 minimum_path_points에 못 미치면 최장 구간을 반복 이등분해 채운다.
  void densifyPath(f110_msgs::msg::WpntArray & path, std::size_t minimum_points) const;
  // raw_obstacles는 **절대 s**를 담은 원본이다. ExpandedObstacle은 자차 상대거리를
  // 담으므로, 가상의 정지점 기준으로 탈출 가능성을 물으려면 그 지점 기준으로 다시
  // 확장해야 한다. 원본 없이 기존 visible/cluster를 재사용하면 장애물이 정지점에서도
  // 같은 거리에 있는 것으로 보여 검증이 통째로 무의미해진다.
  RacelineSplineResult buildSafeStop(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    const std::vector<ExpandedObstacle> & cluster,
    const std::vector<f110_msgs::msg::Obstacle> & raw_obstacles,
    const ExpandedObstacle & blocking) const;
  RacelineSplineResult buildMarginSlowPass(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;
  void applyAvoidanceVelocityLimit(
    f110_msgs::msg::WpntArray & path,
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible) const;
  void applyApproachFeasibilityRamp(
    f110_msgs::msg::WpntArray & path,
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible) const;
  void applyLongitudinalFeasibility(
    f110_msgs::msg::WpntArray & path, const EgoFrenetState & ego) const;
  void updateGeometryAndAcceleration(f110_msgs::msg::WpntArray & path) const;
  bool validateCandidate(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<ExpandedObstacle> & visible,
    std::string & reason,
    std::size_t start_index = 0U,
    std::size_t minimum_points = 0U,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt,
    double obstacle_reserve_scale = 1.0) const;


  P3ShadowPlanningContext buildP3ShadowPlanningContext(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    bool relaxed_clearance_gate = false) const;
  void finalizeP3ShadowPath(
    f110_msgs::msg::WpntArray & path,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  P3ShadowPathEvaluation validateP3ShadowPath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double obstacle_reserve_scale = 1.0,
    const std::optional<double> & collision_horizon = std::nullopt) const;

  RacelineSplineParameters parameters_;
  f110_msgs::msg::WpntArray reference_;
  double track_length_{0.0};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
