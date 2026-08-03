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
#include <visualization_msgs/msg/marker_array.hpp>

#include "local_planning/obstacle_guard.hpp"
#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{

// ============================================================================
// LocalPlannerNode — 정적 장애물 회피 계획의 ROS 2 입출력·상태 관리 노드
// ============================================================================
// 입력:
//   - /global_waypoints       : 순서가 고정된 글로벌 레이스 라인
//   - /static_obs             : obstacle_detector가 계산한 Frenet 장애물 경계
//   - /car_state/frenet/odom  : 차량의 현재 (s, d)와 속도
//   - /state                  : AVOID/GLOBAL 전환 확인용 상태
// 출력:
//   - /avoid_waypoints        : 회피, 감속 준비, 안전 정지 또는 글로벌 인계 경로
//   - /local_planning/path, /local_path, /local_planning/markers : 시각화·호환 출력
//
// 실제 spline 생성과 충돌 검사는 RacelineSplinePlanner가 담당한다. 이 클래스는 센서
// 스냅샷의 수명, 장애물 관측 안정화, 확정 경로(commitment), safe-stop latch 및
// state_machine과의 인계 절차를 관리한다.
// ============================================================================
class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 초기화와 ROS 콜백 ─────────────────────────────────────────────────────
  void initializeParameters();
  void initializeInterfaces();
  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr message);
  void onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void onFrenetOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onState(const f110_msgs::msg::StateMachine::SharedPtr message);
  void onPlanningTimer();

  // ── 기준 경로와 commitment 생명주기 ───────────────────────────────────────
  bool sameReference(const f110_msgs::msg::WpntArray & message) const;
  void clearCommitment();
  void commitAvoidance(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);

  // ── 최초 장애물 군집 안정화와 uncertainty Guard ──────────────────────────
  // 같은 ID를 여러 실제 /static_obs 메시지에서 관측하고 보수적 envelope 합집합을 만든다.
  void resetInitialStabilization();
  std::vector<f110_msgs::msg::Obstacle> buildInitialStabilizationInput() const;
  std::vector<f110_msgs::msg::Obstacle> buildGuardedObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool updateInitialStabilization(
    const std::vector<int> & cluster_ids,
    const std::vector<f110_msgs::msg::Obstacle> & conservative_obstacles,
    const rclcpp::Time & update_time);

  // ── 연속 장애물 maneuver 연결 ─────────────────────────────────────────────
  // 현재 회피의 controller tail 때문에 다음 장애물이 현재 경로를 불필요하게 무효화하지
  // 않도록, 현 maneuver와 다음 maneuver의 장애물 입력·관측 상태를 분리한다.
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
  bool beginChainedManeuverIfNeeded(
    const EgoFrenetState & ego,
    std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
    const std::string & phase);
  void resetForChainedManeuver();

  // ── 방향 잠금, 글로벌 인계, 안전 정지 ─────────────────────────────────────
  bool commitmentSideLocked(const EgoFrenetState & ego) const;
  bool activateGlobalHandoff(const EgoFrenetState & ego);
  void latchSafeStop(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  void handleSafeStopLatch(const EgoFrenetState & ego);
  bool commitmentComplete(const EgoFrenetState & ego);
  void resetCommitmentViolationConfirmation();
  void logObstacleCollision(
    const std::string & severity,
    const PathValidationFailure & failure,
    int confirmation_count = 0) const;

  // ── 메시지 및 RViz 출력 변환 ──────────────────────────────────────────────
  void publishResult(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles);
  void publishEmpty(const std::string & reason);
  nav_msgs::msg::Path makePath(
    const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    const std_msgs::msg::Header & header) const;
  visualization_msgs::msg::MarkerArray makeMarkers(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  // 계획 관련 콜백은 서로 직렬화하고 odometry는 별도 그룹에서 갱신한다. 따라서 느린 계획
  // 주기 중에도 최신 위치를 받을 수 있으며, 공유 odometry에는 mutex를 적용한다.
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;

  // 구독 인터페이스
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odometry_sub_;
  rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr state_sub_;

  // 제어용 회피 경로와 디버그/호환용 출력 인터페이스
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr compatibility_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // 계획 알고리즘, 기준 경로와 가장 최근 입력 스냅샷
  RacelineSplineParameters planner_parameters_;
  ObstacleGuardParameters guard_parameters_;
  RacelineSplinePlanner planner_;
  f110_msgs::msg::WpntArray global_waypoints_;
  std::vector<f110_msgs::msg::Obstacle> static_obstacles_;
  nav_msgs::msg::Odometry latest_odometry_;
  mutable std::mutex odometry_mutex_;

  // 입력 freshness와 관측 안정화 시간을 판정하는 ROS 시간
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacles_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_stabilization_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_stabilization_start_{0, 0, RCL_ROS_TIME};
  std::uint64_t obstacles_message_sequence_{0};
  std::uint64_t initial_last_counted_sequence_{0};
  std::uint64_t next_last_counted_sequence_{0};

  // 입력 준비 및 perception degraded 상태
  bool has_global_waypoints_{false};
  bool has_obstacles_message_{false};
  bool obstacle_perception_degraded_{false};
  bool has_odometry_{false};

  // 현재 확정 회피 경로와 safe-stop latch 상태
  bool has_commitment_{false};
  RacelineSplineResult committed_result_;
  bool safe_stop_latched_{false};
  RacelineSplineResult safe_stop_result_;
  int safe_stop_release_count_{0};
  int commitment_soft_violation_count_{0};
  double commitment_start_s_{0.0};

  // spline의 d=0 합류와 state_machine의 GLOBAL 확인은 별도 단계로 관리한다.
  int merge_complete_count_{0};
  bool merge_geometry_confirmed_{false};
  bool handoff_active_{false};
  bool avoid_state_observed_{false};
  bool has_state_{false};

  // 최초/다음 장애물 군집의 실제 메시지별 관측 계수
  bool initial_stabilization_active_{false};
  bool initial_prepare_published_{false};
  bool initial_has_counted_sequence_{false};
  bool next_stabilization_active_{false};
  bool next_has_counted_sequence_{false};
  uint8_t current_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  std::optional<bool> last_published_side_;

  // ID별 envelope 합집합, 관측 횟수, commitment 시점의 동결 Guard
  std::map<int, f110_msgs::msg::Obstacle> initial_cluster_union_;
  std::map<int, int> initial_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> next_cluster_union_;
  std::map<int, int> next_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> committed_obstacle_guards_;
  std::set<int> completed_obstacle_ids_;

  // 노드 상태 전이와 안전 판정 파라미터
  bool require_obstacles_message_{true};
  double obstacle_stale_timeout_sec_{0.75};
  double odometry_stale_timeout_sec_{0.50};
  double merge_lateral_tolerance_m_{0.15};
  int merge_confirm_cycles_{15};
  int safe_stop_release_cycles_{8};
  int planning_period_ms_{50};
  double state_handoff_tail_ratio_{0.10};
  double state_handoff_speed_cap_mps_{6.0};
  int initial_observation_count_{3};
  double initial_observation_min_duration_sec_{0.15};
  double initial_observation_max_wait_sec_{0.35};
  int commitment_soft_violation_confirm_cycles_{3};
  double hard_collision_margin_m_{0.03};
  double chain_release_margin_m_{0.20};
  double commitment_lock_lateral_threshold_m_{0.10};
  double commitment_lock_longitudinal_m_{0.50};
  double obstacle_marker_scale_m_{0.35};
  double path_marker_width_m_{0.06};

  // 모든 토픽과 좌표계 이름은 YAML에서 덮어쓸 수 있다.
  std::string global_waypoints_topic_{"/global_waypoints"};
  std::string obstacles_topic_{"/static_obs"};
  std::string frenet_odom_topic_{"/car_state/frenet/odom"};
  std::string state_topic_{"/state"};
  std::string ot_waypoints_topic_{"/avoid_waypoints"};
  std::string local_path_topic_{"/local_planning/path"};
  std::string compatibility_path_topic_{"/local_path"};
  std::string markers_topic_{"/local_planning/markers"};
  std::string frame_id_{"map"};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
