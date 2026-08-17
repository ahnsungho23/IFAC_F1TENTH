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

#ifndef LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/state_machine.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "local_planning/obstacle_guard.hpp"
#include "local_planning/p3_maneuver_lifecycle.hpp"
#include "local_planning/raceline_spline_planner.hpp"
#include "local_planning/safe_stop_lifecycle.hpp"

namespace local_planning
{

enum class P3RuntimeMode
{
  kOff,
  kShadow,
  kTestActive,
};

const char * p3RuntimeModeName(P3RuntimeMode mode);

struct P3CallbackSnapshot
{
  bool ready{false};
  bool has_odometry{false};
  std::string not_ready_reason;
  nav_msgs::msg::Odometry odometry;
  rclcpp::Time odometry_receipt_time{0, 0, RCL_ROS_TIME};
  P3ManeuverSnapshot maneuver;
  std::int64_t frenet_source_stamp_ns{0};
  // Frenet 소스 스탬프가 직전 콜백보다 과거로 후퇴한 콜백. 상태(epoch/lifecycle/envelope)는
  // 이미 리셋됐지만 이 샘플 자체가 의심스러우므로, 이 사이클은 계획을 건너뛰고 직전 출력을
  // 유지해야 한다. 2026-08-15 run9: 리셋 직후 같은 콜백이 오염된 상태로 계획을 강행해
  // "no collision-free stop prefix" 비상 홀드를 래치했고(정상 기하에서 plan()은 회피를
  // 반환함이 하네스로 입증됨), 그 래치는 해제 조건이 영영 충족되지 않아 영구 정지가 됐다.
  bool source_stamp_regressed{false};
};

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initializeParameters();
  void initializeInterfaces();
  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr message);
  void onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void onFrenetOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onState(const f110_msgs::msg::StateMachine::SharedPtr message);
  void onPlanningTimer();
  void runP0PlanningCycle(const P3CallbackSnapshot * snapshot = nullptr);
  void tryRunLockstepCycle();
  rclcpp::Time eventNow() const;

  P3CallbackSnapshot captureP3CallbackSnapshot();
  bool prepareP3InitialSelectionSnapshot(P3CallbackSnapshot & snapshot);
  void resetP3SelectionEnvelope();
  P3ShadowResult evaluateP3Snapshot(
    const P3CallbackSnapshot & snapshot,
    const std::string & p0_context) const;
  // Continuation-first is a computation order, not only an output priority: `evaluate` is pulled
  // ONLY when the recorded maneuver fails to continue. A held frozen suffix therefore costs no
  // candidate generation and no hard validation at all. Do not take a materialized result here.
  P3ManeuverLifecycleDecision advanceP3Lifecycle(
    const P3CallbackSnapshot & snapshot,
    const std::function<const P3ShadowResult &()> & evaluate);
  RacelineSplineResult makeP3ActiveResult(
    const P3ManeuverLifecycleDecision & decision) const;
  void publishP3CycleDiagnostic(
    const P3CallbackSnapshot & snapshot,
    const P3ShadowResult & evaluation,
    const P3ManeuverLifecycleDecision & lifecycle,
    const std::string & path_owner,
    bool p0_backup_only);

  bool sameReference(const f110_msgs::msg::WpntArray & message) const;
  void clearCommitment();
  void commitAvoidance(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  void resetInitialStabilization();
  std::vector<f110_msgs::msg::Obstacle> buildInitialStabilizationInput() const;
  std::vector<f110_msgs::msg::Obstacle> buildGuardedObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool updateInitialStabilization(
    const std::vector<int> & cluster_ids,
    const std::vector<f110_msgs::msg::Obstacle> & conservative_obstacles,
    const rclcpp::Time & update_time);
  std::vector<f110_msgs::msg::Obstacle> buildNextManeuverInput() const;
  std::vector<f110_msgs::msg::Obstacle> buildCurrentManeuverInput(
    const EgoFrenetState & ego,
    bool apply_uncertainty_guard = true) const;
  void resetNextManeuverStabilization();
  bool updateNextManeuverStabilization(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
    const rclcpp::Time & update_time);
  void promoteNextManeuverStabilization();
  bool activeManeuverObstacleCleared(const EgoFrenetState & ego) const;
  bool tryEarlyChainedManeuver(
    const EgoFrenetState & ego,
    std::vector<f110_msgs::msg::Obstacle> & next_obstacles);
  double remainingDistanceToMerge(const EgoFrenetState & ego) const;
  // Obstacle-check range for re-validating the committed path. Identical definition to the one
  // `generateP3Candidates` uses when selecting it; see the .cpp for why they must not diverge.
  double maneuverCollisionHorizon(const EgoFrenetState & ego) const;
  bool beginChainedManeuverIfNeeded(
    const EgoFrenetState & ego,
    std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
    const std::string & phase);
  void resetForChainedManeuver();
  bool commitmentSideLocked(const EgoFrenetState & ego) const;
  bool activateGlobalHandoff(
    const EgoFrenetState & ego,
    SafeStopReleaseReason safe_stop_release_reason = SafeStopReleaseReason::kNone);
  void clearSafeStopLatch();
  void latchSafeStop(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  SafeStopCycleDecision evaluateSafeStopLifecycle(
    const EgoFrenetState & ego,
    const RacelineSplineResult & replanned_result,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  bool resultTargetsLatchedObstacle(const RacelineSplineResult & result) const;
  bool explicitForwardCorridorClear(const EgoFrenetState & ego) const;
  void publishSafeStopLifecycleAudit(
    const SafeStopCycleInput & input,
    const SafeStopCycleDecision & decision);
  void handleSafeStopLatch(const EgoFrenetState & ego);
  bool commitmentComplete(const EgoFrenetState & ego);
  void resetCommitmentViolationConfirmation();
  void logObstacleCollision(
    const std::string & severity,
    const PathValidationFailure & failure,
    int confirmation_count = 0) const;
  void publishResult(const RacelineSplineResult & result);
  void publishEmpty(const std::string & reason);
  void publishTimingEvent(const std::string & event, const std::string & fields);
  void publishReplayEvent(const std::string & event, const std::string & fields);
  void publishCandidateAudit(
    const RacelineSplineResult & result, const std::string & decision);
  nav_msgs::msg::Path makePath(
    const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    const std_msgs::msg::Header & header) const;

  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odometry_sub_;
  rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr state_sub_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr replay_diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p3_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  RacelineSplineParameters planner_parameters_;
  ObstacleGuardParameters guard_parameters_;
  RacelineSplinePlanner planner_;
  P3ManeuverLifecycle p3_maneuver_lifecycle_;
  f110_msgs::msg::WpntArray global_waypoints_;
  std::vector<f110_msgs::msg::Obstacle> static_obstacles_;
  nav_msgs::msg::Odometry latest_odometry_;
  mutable std::mutex odometry_mutex_;
  mutable std::mutex lockstep_mutex_;
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacles_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_stabilization_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_stabilization_start_{0, 0, RCL_ROS_TIME};
  std::uint64_t obstacles_message_sequence_{0};
  std::int64_t latest_obstacle_source_stamp_ns_{0};
  std::int64_t last_p3_frenet_source_stamp_ns_{0};
  std::uint64_t initial_last_counted_sequence_{0};
  std::uint64_t p3_selection_last_sequence_{0};
  std::uint64_t next_last_counted_sequence_{0};
  std::int64_t lockstep_obstacle_stamp_ns_{0};
  std::int64_t lockstep_odometry_stamp_ns_{0};
  std::int64_t lockstep_state_stamp_ns_{0};
  std::int64_t lockstep_last_processed_stamp_ns_{0};
  rclcpp::Time lockstep_event_time_{0, 0, RCL_ROS_TIME};

  bool has_global_waypoints_{false};
  bool has_obstacles_message_{false};
  bool obstacle_perception_degraded_{false};
  bool has_odometry_{false};
  bool has_commitment_{false};
  RacelineSplineResult committed_result_;
  SafeStopLifecycle safe_stop_lifecycle_;
  RacelineSplineResult safe_stop_result_;
  // Most recent published guidance geometry (kAvoidance: spline, P3 maneuver, handoff loop,
  // margin slow pass). Survives commitment resets so the safe-stop latch can brake along the
  // last vetted line instead of publishing an in-place zero-speed hold when no collision-free
  // stop prefix exists.
  f110_msgs::msg::WpntArray last_valid_guidance_path_;
  int commitment_soft_violation_count_{0};
  double commitment_start_s_{0.0};
  int merge_complete_count_{0};
  bool merge_geometry_confirmed_{false};
  bool handoff_active_{false};
  bool avoid_state_observed_{false};
  // Set once a pre-engagement replan flips the committed side; blocks further flip-flops
  // (centred-obstacle ties) until lateral engagement or the next maneuver.
  bool pre_engagement_side_switched_{false};
  bool has_state_{false};
  bool initial_stabilization_active_{false};
  bool initial_prepare_published_{false};
  // 직전 발행이 non-empty였는지. state_machine의 AVOID 진입은 "경로가 비어 있지 않다"
  // 하나로 결정되므로(준비감속·안정화 중 조기회피 포함), 커밋이 없는 상태에서 트랙이
  // 비었을 때 글로벌 핸드오프 루프로 되돌려줘야 하는지를 이 플래그로 판단한다.
  // initial_prepare_published_만으로는 안정화 중 조기회피 분기가 그것을 false로 지워
  // 핸드오프 구제 경로를 건너뛴다.
  bool last_publication_non_empty_{false};
  bool initial_has_counted_sequence_{false};
  bool p3_selection_has_sequence_{false};
  rclcpp::Time p3_selection_start_{0, 0, RCL_ROS_TIME};
  bool next_stabilization_active_{false};
  bool next_has_counted_sequence_{false};
  uint8_t current_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  std::optional<bool> last_published_side_;
  std::map<int, f110_msgs::msg::Obstacle> initial_cluster_union_;
  std::map<int, int> initial_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> p3_selection_envelope_union_;
  std::map<int, int> p3_selection_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> next_cluster_union_;
  std::map<int, int> next_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> committed_obstacle_guards_;
  std::set<int> completed_obstacle_ids_;

  // 동일 ID 장애물의 최근 관측 창(면별). 가드 팽창을 상수가 아니라 이 면이 실제로 얼마나
  // 흔들렸는지로 정하기 위한 것이다 — 근거는 ObstacleFaceUncertainty 주석 참조.
  //
  // 창(window)인 이유: 박스는 접근하면서 정당하게 자란다(14:30 백, 장애물 2의 라인 쪽 면이
  // 7 m에서 -0.156, 6 m 안쪽에서 -0.278로 수렴). 전체 이력 분산은 그 성장을 영원히 기억해
  // 수렴한 뒤에도 크게 남는다. 최근 창은 성장 중에는 크고 수렴 후에는 작아져, "지금 이 면을
  // 얼마나 믿을 수 있나"를 그대로 나타낸다.
  struct FaceObservationWindow
  {
    std::deque<double> right;
    std::deque<double> left;
  };
  std::map<int, FaceObservationWindow> face_observation_windows_;

  // ── 확정 정적 장애물 기억 ───────────────────────────────────────────────────────────
  //
  // 왜 (2026-08-17): 오늘 실패의 전부가 "짧은 지평에서 현재 위치로부터 급하게 계획하기"가
  // 뿌리였다 — cluster_start <= 0, 램프가 병목 관통, 격자가 실현 구간을 건너뜀, 가림 때문에
  // 폭이 마지막 순간에 3.5배로 뜀. 한 랩 전에 장애물 위치를 알면 이 넷이 통째로 사라진다.
  //
  // 대회 규정: 20랩 중 선두 차량이 10랩을 완주하면 장애물이 제거된다. 1~2랩 학습 후
  // 3~10랩이 이득 구간이다(최대 8랩, 10랩 경기면 8/10).
  //
  // 🔴 제거 시점은 **상대차 진행**에 달려 있다. 우리 랩 카운터로는 맞출 수 없다. 그래서
  // 인지로 감지한다 — 그 자리를 시야 확보한 채 지났는데 검출기가 확정하지 못하면 제거다.
  //
  // 좌표 변환이 없다: /confirmed_static_obs 를 이미 Frenet(s/d)으로 받으므로 그대로 기억한다.
  // (static_obstacle_map 은 map 좌표 x/y AABB만 다루므로 이 용도에는 재투영이 필요하고,
  //  지금 스택에서 실행되지도 않는다 — 그래서 쓰지 않는다.)
  struct RememberedObstacle
  {
    f110_msgs::msg::Obstacle obstacle;
    std::uint64_t last_confirmed_sequence{0};
    // 시야가 확보된 채 지나쳤는데 확정하지 못한 횟수. 임계에 닿으면 제거로 본다.
    int unconfirmed_passes{0};
    // 이번 통과에서 이미 판정했는지. 한 번 지날 때 여러 번 세지 않기 위한 것.
    bool pass_pending{false};
  };
  std::vector<RememberedObstacle> remembered_obstacles_;
  bool remembered_obstacle_enable_{true};
  int remembered_obstacle_removal_passes_{1};
  double remembered_obstacle_visibility_margin_m_{2.0};
  double remembered_obstacle_match_tolerance_m_{0.60};
  void updateRememberedObstacles();
  std::vector<f110_msgs::msg::Obstacle> obstaclesWithMemory() const;
  std::size_t face_observation_window_size_{40};
  std::size_t face_observation_min_samples_{12};
  void updateFaceObservationWindows();
  ObstacleFaceUncertainty faceUncertaintyFor(int obstacle_id) const;

  bool require_obstacles_message_{true};
  double obstacle_stale_timeout_sec_{0.75};
  double odometry_stale_timeout_sec_{0.50};
  double merge_lateral_tolerance_m_{0.15};
  int merge_confirm_cycles_{15};
  int safe_stop_release_cycles_{8};
  int planning_period_ms_{50};
  double state_handoff_tail_distance_m_{6.0};
  double state_handoff_speed_cap_mps_{6.0};
  int initial_observation_count_{3};
  double initial_observation_min_duration_sec_{0.15};
  double initial_observation_max_wait_sec_{0.35};
  int commitment_soft_violation_confirm_cycles_{3};
  double chain_release_distance_m_{0.20};
  double commitment_lock_lateral_threshold_m_{0.10};
  double commitment_lock_longitudinal_m_{0.50};

  std::string global_waypoints_topic_{"/global_waypoints"};
  std::string obstacles_topic_{"/confirmed_static_obs"};
  std::string frenet_odom_topic_{"/car_state/frenet/odom"};
  std::string state_topic_{"/state"};
  std::string ot_waypoints_topic_{"/avoid_waypoints"};
  std::string local_path_topic_{"/local_planning/path"};
  std::string frame_id_{"map"};
  std::string timing_diagnostics_topic_{"/cma_timing/events"};
  std::string replay_diagnostics_topic_{"/cma_replay/planner_events"};
  bool timing_diagnostics_enable_{false};
  bool replay_diagnostics_enable_{false};
  bool lockstep_mode_{false};
  bool timing_t0_published_{false};
  bool timing_t1_published_{false};

  P3RuntimeMode p3_mode_{P3RuntimeMode::kOff};
  std::string p3_diagnostics_topic_{"/local_planning/p3_shadow"};
  std::uint64_t p3_source_epoch_{1U};
  std::uint64_t global_reference_generation_{0U};
  std::uint64_t p3_callback_sequence_{0U};
  // P3가 출력을 못 내 P0 백업으로 내려간 콜백 수. "P3 단독으로 충분한가"를 재는 유일한
  // 숫자라, P0 격자를 껐을 때도(그때는 곧바로 안전정지) 계속 센다.
  std::uint64_t p3_backup_fallback_count_{0U};
  // Last owner|lifecycle|backup triple actually logged, so P3_PATH_OWNERSHIP reports transitions
  // instead of repeating the steady state at the planning rate.
  std::string last_logged_ownership_state_;
  std::string current_path_owner_{"P0"};
  std::string last_selected_path_family_{"NONE"};
  std::string last_selected_path_digest_{"NONE"};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
