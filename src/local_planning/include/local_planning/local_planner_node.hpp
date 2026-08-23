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
#include "local_planning/detail/obstacle_ingress_buffer.hpp"
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

// P3 사이클 진단(/local_planning/p3_shadow)의 상세도 (2026-08-21 다이어트).
//  kOff    발행 안 함(퍼블리셔도 만들지 않음)
//  kEvents 의미 있는 상태 변화 + 하트비트에서만 발행. 변화 직전 프레임(preroll)을 함께
//          내보내므로 "무엇이 바뀌기 직전에 무엇을 보고 있었나"가 백에 남는다.
//  kFull   매 계획 사이클 발행(종전 거동). 실차 부검용.
enum class P3DiagnosticsDetail
{
  kOff,
  kEvents,
  kFull,
};


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
  void onObstacleIngress(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void acceptObstacles(
    const f110_msgs::msg::ObstacleArray::SharedPtr message,
    const rclcpp::Time & receipt_time);
  void drainLatestObstacleIngress();
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
  // 커밋만 지우는 좁은 리셋 (A1, 2026-08-20). clearCommitment() 와 달리
  // completed_obstacle_ids_ / maneuver_obstacle_rear_s_ / completion_deferred_since_ /
  // safe-stop 래치는 보존한다 — P3 무효화 직후에 쓴다.
  void invalidateCommitment();
  void commitAvoidance(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  // B1 raw 감속 힌트 (2026-08-20). confirmed 관점에서 트랙이 비었을 때, raw(/static_obs)
  // 장애물이 전방에서 라인을 물고 있으면 감속 힌트 경로를 발행하고 true 를 돌려준다.
  void onRawObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  // obstacle_id 는 고른 raw 장애물의 트랙 id (깜빡임 브리지로 대체된 경우 기억한 id).
  // 커밋 skip 판정이 이 id 로 confirmed·커밋 목록과 대조한다.
  bool selectRawSlowdownTarget(
    const EgoFrenetState & ego, double * front_m, double * span_m,
    int * obstacle_id = nullptr);
  bool maybePublishRawSlowdownHint(const EgoFrenetState & ego);
  // 이 raw 장애물이 이미 confirmed 이고 **지금 발행하는 경로**가 책임지는가
  // (= 승격 대기용 캡을 더 씌울 이유가 없는가). 판정은 raw_slowdown_gate.hpp 의 순수 함수.
  // ⚠️ committed_result_ 가 아니라 result 를 보는 이유는 정의부 주석 참고 (P3 경로).
  bool rawSlowdownHandledByCommitment(
    const RacelineSplineResult & result, int raw_obstacle_id) const;
  // 시각화 경로 발행 가부 (2026-08-21): 구독자 게이트 + 시간 간축. 구독 중일 때의
  // 40 Hz Path(165점) 는 원격 RViz 링크에서 수백 KB/s 라 화면이 밀린다.
  bool localPathVisualizationDue();
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
  // 완료 기동의 장애물을 재회피 차단 목록에 넣기 전 안전성 검사 (2026-08-21,
  // run_102718 t=155.3 / run_102438 t=123.7): 앞끝이 아직 명확히 전방인 장애물을
  // 차단하면 계획 입력에서 지워져 (a) 그 박스를 관통하는 체이닝 커밋 (b) 재계획
  // "막힘 없음" vs 회랑검사 "불청정" 모순 교착이 된다. 지났거나 ego 가 스팬 안일
  // 때만 true — 후자가 이 차단의 원래 회귀 보호(옆 박스 재회피 정지) 케이스다.
  bool safeToBlacklistCompletedObstacle(const EgoFrenetState & ego, int id) const;
  void resetForChainedManeuver(const EgoFrenetState & ego);
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
  void topUpStopGeometry(const EgoFrenetState & ego);
  bool commitmentComplete(const EgoFrenetState & ego);
  // 커밋한 장애물이 전방 handoff_latch_commit_distance_m_ 안에 남아 있는가.
  // true 면 이번 프레임에 그 장애물이 안 보여도 핸드오프로 넘어가지 않는다.
  bool committedObstacleWithinLatch(const EgoFrenetState & ego) const;

  // 활성 기동이 참조하는 장애물들의 뒤끝 s 를 최신 관측으로 갱신한다(미검출은 유지).
  void rememberManeuverObstacleRears(const std::vector<int> & obstacle_ids);
  // 기동 기억을 "전방"으로 인정할 상한 [m] = min(반 바퀴, maneuver_memory_max_ahead_m).
  // 이 기억을 읽는 두 곳이 반드시 공유해야 하는 값이라 함수로 뺐다 (2026-08-22).
  double maneuverMemoryMaxAhead() const;
  // 기억해 둔 뒤끝 중 아직 자차 앞에 남아 있는 것의 최대 전방거리. 없으면 음수.
  double maneuverObstacleRearAhead(const EgoFrenetState & ego) const;
  // 기동 장애물을 아직 안 지났으면 이번 콜백을 붙잡는다 (2026-08-20 확장).
  // true 를 돌려주면 이번 콜백은 여기서 끝난다 — 경로를 발행했거나 안전정지를 걸었다.
  //
  // ⚠️ complete 뿐 아니라 IDLE/무효화까지 덮는다. 앞선 판(597e6f8)은 lifecycle.complete
  //    분기에만 걸려 있었는데, 실차에서는 보류가 걸린 바로 다음 콜백에 lifecycle 이
  //    IDLE 로 떨어져 기동이 통째로 사라졌다 (run_062020 t=537.80 보류 → 537.83 IDLE →
  //    538.00 옛 경로 재발행 → 538.5 벽). 한 콜백만 막은 셈이었다.
  bool holdForManeuverObstacleAhead(
    const P3CallbackSnapshot & snapshot,
    const P3ShadowResult & evaluation,
    const P3ManeuverLifecycleDecision & lifecycle);
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
  rclcpp::CallbackGroup::SharedPtr obstacle_ingress_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr raw_obstacles_sub_;
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
  detail::ObstacleIngressBuffer obstacle_ingress_buffer_;
  std::uint64_t processed_obstacle_ingress_sequence_{0};
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
  // Most recent published guidance RESULT (kAvoidance: spline, P3 maneuver, handoff loop,
  // margin slow pass). Survives commitment resets so the safe-stop latch can brake along the
  // last vetted line instead of publishing an in-place zero-speed hold when no collision-free
  // stop prefix exists.
  //
  // 🔴 2026-08-22: 경로만이 아니라 result 전체를 들고 있는다. 보류 게이트가 이것을 **다시
  //    조향에 쓰기** 때문에 go_left / obstacle_ids / merge_s 가 같이 필요하다 — 경로만
  //    복원하면 publishResult 가 ot_side 와 side_switch 를 잘못 채워 상태머신이 없는
  //    좌우 전환을 본다. 근거는 holdForManeuverObstacleAhead 본문 주석 참고.
  RacelineSplineResult last_valid_guidance_result_;
  // 보류 게이트가 마지막 유효 기하를 재검증해 다시 발행한 횟수 (2026-08-22).
  // 종전에는 이 자리가 전부 안전정지였다 — A/B 로 그 감소를 직접 세려고 남긴다.
  unsigned int hold_guidance_republish_count_{0U};
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
  // 근접 사각 타임아웃 해제 (kStoppedBlindTimeout): 정지 + 기억 위험구간 전방 상태에서
  // 신선한 빈 프레임이 이 시간만큼 이어지면 크립 캡 핸드오프로 해제. 0 이하 = 비활성.
  double safe_stop_blind_release_sec_{4.0};
  double safe_stop_blind_creep_speed_mps_{0.7};
  int planning_period_ms_{50};
  double state_handoff_tail_distance_m_{6.0};
  double state_handoff_speed_cap_mps_{6.0};
  // B1 raw 감속 힌트 상태. latest_raw_obstacles_ 는 planning 콜백 그룹에서만 접근한다
  // (onRawObstacles 와 계획 사이클이 같은 그룹으로 직렬화됨 — onObstacles 와 동일 계약).
  bool raw_slowdown_enable_{true};
  std::string raw_slowdown_topic_{"/static_obs"};
  double raw_slowdown_trigger_distance_m_{12.0};
  double raw_slowdown_speed_cap_mps_{2.8};
  double raw_slowdown_hold_sec_{1.0};
  double raw_slowdown_lateral_margin_m_{0.25};
  // 🔴 raw 캡을 "승격 전"으로 되돌린다 (2026-08-22). 이 힌트의 선언된 목적은
  //    confirmed 승격 지연을 메우는 것인데(선언부 주석), 지금은 승격이 끝나고 회피 기동을
  //    커밋한 뒤에도 같은 장애물에 계속 2.8 캡을 씌운다.
  //    run_20260822_015820 실측: 스팬 안에서 가장 낮은 캡을 만든 주체는 갭 사다리 69.5%,
  //    raw 7.3% 였다. 갭 사다리가 예약 게이트로 꺼지면 raw 가 **유일하게 남는** 장애물
  //    관련 캡이 되어 통과 속도를 2.80 에 고정한다 (같은 백 id12 의 8~12 m 발행값이
  //    정확히 2.80 이다).
  //    켜면 "현재 confirmed 이면서 현재 커밋된 기동이 책임지는" 장애물에만 캡을 건너뛴다.
  //    미승격 raw·커밋 밖의 다음 장애물·승격이 무산된 물체는 그대로 캡이 걸리므로,
  //    run_102718 s5.5 충돌이 도입 근거였던 보호 경로는 유지된다.
  //    기본값 false = 종전 거동(운영 YAML 에서 켠다).
  bool raw_slowdown_skip_committed_{false};
  // 🔵 2026-08-23: raw 캡을 sqrt(2·a·d) 로 스케일. false 면 종전 상수 거동.
  bool raw_slowdown_distance_scaled_{true};
  // F1 (2026-08-22): 보류 게이트가 마지막 유효 회피 기하를 재검증해 다시 쓸지.
  // false 면 종전 거동(커밋 경로만 보고, 없으면 안전정지)으로 돌아간다 — A/B 와 롤백용.
  bool hold_republish_last_guidance_{true};
  // 컨트롤러가 L1 목표를 고를 수 있으려면 경로가 ego 앞으로 뻗어야 하는 거리 [m].
  // L1 = l1_offset + v*l1_speed_gain = 0.6 + 0.32*v 이므로 v=5.9 m/s 까지 덮는 값이다.
  // F1(재발행)과 F3(정지 기하)이 같은 질문을 하므로 상수를 공유한다. 0 이면 둘 다 하한이
  // 꺼진다.
  double controller_lookahead_floor_m_{2.5};
  // F3 (2026-08-22): 접촉점에서 잘려 나간 정지 경로의 **조향 기하**를 원본으로 되돌릴지.
  // false 면 종전 거동(2~8 점짜리 정지 경로)으로 돌아간다 — A/B 와 롤백용.
  // F5 (2026-08-22): 보류가 이 시간 안에 물리적으로 해소되면 safe-stop 래치로 키우지
  // 않고, 이미 타고 있던 회피 기하를 진입 불연속 검사만 면제해 몇 사이클 더 유지한다.
  // 0 이면 끈다 — A/B 와 롤백용. 실측 근거는 hold_recovery_gate.hpp 주석 참고.
  double hold_min_clear_time_sec_{0.0};   // 실차 미검증 — 기본 꺼짐 (2026-08-22)
  // 그 완화의 누적 노출 상한 [s]. 넘으면 종전대로 래치한다.
  double hold_brief_max_sec_{0.6};
  unsigned int hold_brief_count_{0U};
  bool stop_geometry_extend_{true};
  // F3: 정지 접두부를 잘라 낸 **원본** 기하. 래치 때 보관해 두고, 차가 정지 경로를
  // 갉아먹을 때마다 발행 직전에 그 뒤를 이어 붙여 L1 예견거리를 유지한다.
  // 래치 시점에만 늘리면 소용이 없다 — 하나의 래치가 100 회 넘게 재발행되는 동안
  // 전방 스팬이 계속 줄기 때문이다 (재생 실측: 발행의 89.6% 가 하한 미달).
  f110_msgs::msg::WpntArray stop_geometry_source_;
  unsigned int stop_geometry_extend_count_{0U};
  double path_cover_max_gap_m_{1.0};
  struct RawHintObstacle
  {
    int id{-1};
    double s_start{0.0};
    double s_end{0.0};
    double d_left{0.0};
    double d_right{0.0};
  };
  std::vector<RawHintObstacle> latest_raw_obstacles_;
  std::optional<rclcpp::Time> raw_hint_hold_until_;
  double raw_hint_s_start_{0.0};
  double raw_hint_s_end_{0.0};
  int raw_hint_id_{-1};
  int initial_observation_count_{3};
  double initial_observation_min_duration_sec_{0.15};
  double initial_observation_max_wait_sec_{0.35};
  int commitment_soft_violation_confirm_cycles_{3};
  double chain_release_distance_m_{0.20};
  // 🔴 커밋 래치 거리 [m] (2026-08-20 신설). 커밋한 장애물이 전방 이 거리 안에 있으면
  // "이번 프레임 미검출"을 이유로 글로벌 핸드오프(= d 오프셋 0 인 라인)로 넘어가지 않는다.
  //
  // 근거 — 2026-08-19 run_034444 실차(자율 343 s):
  //   원거리 LiDAR 각해상도의 물리적 한계로 클러스터가 min_cluster_points(5) 문턱을
  //   넘나든다. 폭 0.45 m 물체가 10 m 에서 각폭 2.6° = 약 10 빔뿐이고, 코너 차체 롤
  //   6~12° 가 그중 일부를 빗나가게 한다. 실측 미검출률(직전 1 s 내 관측 기준):
  //     3~5 m 42.2%   5~7 m 47.7%   7~10 m 65.9%   (3~10 m 전체 50.9%)
  //   깜빡임 자체는 못 없앤다. 문제는 플래너가 그 한 프레임을 "장애물 없음"으로 읽고
  //   d=0 핸드오프를 발행하는 것이고, 재검출돼도 그 id 는 buildNextManeuverInput() 의
  //   excluded 에 있어 재회피가 막힌다 — 그대로 정면 충돌한다.
  //   실제로 장애물 0.35~1.35 m 앞까지 오프셋 0 을 유지한 정면 충돌이 5 건이었다
  //   (t=244.1 / 276.6 / 278.3 / 341.2 / 545.2, 3.8~5.7 m/s).
  //
  // 값 8.0: 미검출이 시작되는 거리대(3~10 m)를 덮고, 5 m/s 정지거리(6~7 m)보다 크다.
  // edge_test 의 avoid_planner 는 같은 개념을 latch_commit_distance_m 5.0 으로 넣었는데,
  // 그쪽 미검출률은 7.9% 로 우리(50.9%)의 1/6 이라 더 크게 잡는다.
  // 0 이면 래치가 꺼지고 종전 거동(미검출 즉시 핸드오프)으로 돌아간다.
  double handoff_latch_commit_distance_m_{8.0};

  // 🔴 기동 장애물의 **마지막으로 관측된 뒤끝 s** [id -> s_end] (2026-08-20).
  // committed_obstacle_guards_ 는 commitAvoidance() 에서만 채워지므로 P3 가 경로를
  // 소유하는 동안에는 비어 있다 — 그래서 완료 판정에 쓸 수 없다. 이 맵은 P3/P0 구분 없이
  // 활성 기동이 참조하는 id 를 매 콜백 갱신하고, 미검출 프레임에는 마지막 값을 유지한다.
  // 완료·핸드오프 판정이 "장애물을 실제로 지났는가"를 물으려면 이 기억이 필요하다.
  std::map<int, double> maneuver_obstacle_rear_s_;
  // 🔴 기억 누수 차단 (2026-08-22). rememberManeuverObstacleRears 는 빈 id 목록에서
  // 조기 반환해 **정리 루프까지 건너뛰었다**. 그 목록은 lifecycle record 에서 오므로
  // "비어 있다 = 활성 기동이 없다"인데, 그때 옛 entry 가 영구히 남았다.
  // 그 잔존 entry 는 자차가 약 반 바퀴를 더 달리면 원형거리 wrap 으로 "전방 20.6 m"
  // 로 되읽혀 보류를 발동시키고, 실제 위험이 0 인데 안전정지로 이어졌다
  // (run_20260822_002336 실측: 22.67 s 정지 구간의 99.8% 프레임에 장애물 0 개).
  // 다만 한 프레임의 과도 IDLE 로 지우면 2026-08-20 이 막으려던 "한 콜백 구멍"이
  // 되살아나므로(run_062020: 다음 콜백 IDLE → 옛 경로 재발행 → 0.5 s 뒤 벽),
  // 빈 목록이 이 횟수만큼 **연속**됐을 때만 지운다. 0 이면 종전 거동(누수 포함).
  int maneuver_memory_clear_frames_{3};
  int empty_maneuver_frames_{0};
  // 🔴 wrap 모호성 차단 (2026-08-22). "반 바퀴 넘으면 이미 지나쳤다" heuristic 은
  // 41 m 트랙에서 half_track=20.67 m 라, 21 m 전에 지나친 물체가 20.63 m 로 읽히며
  // **여유 4 cm 로 뒤집힌다**. 기억을 "계획 지평 안" 으로 잘라 그 구간을 없앤다.
  // 기본 15.0 = detection_lookahead_m: 그 밖의 물체에는 보류의 근거("지금 복귀하면
  // 오프셋 0 으로 들이받는다")가 성립하지 않는다 — 그 전에 재계획이 몇 번이나 돈다.
  // ⚠️ 소비처가 둘이다: maneuverObstacleRearAhead() 와 safeToBlacklistCompletedObstacle()
  //    의 **뒤끝 기억 분기**. 둘 다 maneuverMemoryMaxAhead() 를 거치게 해 두었으니
  //    직접 상수를 다시 쓰지 말 것 — 한쪽만 고치면 두 판정이 조용히 갈라진다.
  //    (같은 함수의 s_start 분기는 실측/커밋 스냅샷이라 종전 half_track 을 유지한다.)
  double maneuver_memory_max_ahead_m_{15.0};
  // 완료 보류가 시작된 시각. 보류가 이 시간을 넘으면 강제로 완료시킨다 — 차가 멈춰 있으면
  // 장애물을 영영 못 지나므로 보류가 풀리지 않고 FSM 이 AVOID 에 갇힌다.
  std::optional<rclcpp::Time> completion_deferred_since_;
  double completion_defer_max_sec_{3.0};
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
  // p3_shadow 진단 상세도 상태 (2026-08-21). last_p3_event_key_ 는 EVENTS 모드의 발행
  // 트리거이고, previous_p3_cycle_json_ 은 변화 직전 프레임 preroll 용 버퍼다.
  P3DiagnosticsDetail p3_diagnostics_detail_{P3DiagnosticsDetail::kEvents};
  double p3_diagnostics_heartbeat_sec_{0.5};
  int p3_diagnostics_queue_depth_{100};
  std::string last_p3_event_key_;
  std::string previous_p3_cycle_json_;
  std::optional<rclcpp::Time> last_p3_diagnostic_publish_time_;
  // 시각화 경로 발행 주기 상한 (2026-08-21). 구독자가 있을 때만 발행하는 게이트는 이미
  // 있으므로, 구독 중일 때의 40 Hz 를 이 주기로 간축한다.
  double local_path_publish_period_sec_{0.1};
  std::optional<rclcpp::Time> last_local_path_publish_time_;
  std::string current_path_owner_{"P0"};
  std::string last_selected_path_family_{"NONE"};
  std::string last_selected_path_digest_{"NONE"};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
