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

#include "local_planning/local_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace local_planning
{
namespace
{

// ============================================================================
// local_planner_node 내부 보조 함수
// ============================================================================
// 이 파일의 큰 흐름은 다음과 같다.
//   입력 검증 → 관측 안정화/Guard 생성 → 기존 commitment 재검증 → 새 spline 계획
//   → safe-stop 또는 state_machine 인계 → 제어·시각화 토픽 발행
// ============================================================================

// Frenet 구간과 부동소수점 비교에서 퇴화 형상을 거르는 허용 오차
constexpr double kGeometryEpsilon = 1.0e-9;

// RViz nav_msgs/Path에 넣을 평면 yaw를 z/w quaternion으로 변환한다.
geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

// planner가 직접 사용하는 Frenet s/d와 종방향 속도가 모두 유효한지 검사한다.
bool finiteOdometry(const nav_msgs::msg::Odometry & odometry)
{
  return std::isfinite(odometry.pose.pose.position.x) &&
         std::isfinite(odometry.pose.pose.position.y) &&
         std::isfinite(odometry.twist.twist.linear.x);
}

// 여러 관측의 envelope를 합칠 때 유효한 분산 중 가장 큰 값을 보존한다.
// 잘못된 음수/NaN 분산은 0으로 취급해 Guard 계산을 오염시키지 않는다.
double conservativeVariance(double first, double second)
{
  const double finite_first = std::isfinite(first) && first >= 0.0 ? first : 0.0;
  const double finite_second = std::isfinite(second) && second >= 0.0 ? second : 0.0;
  return std::max(finite_first, finite_second);
}

// 위치·속도 각 축의 분산을 가장 보수적인 값으로 병합한다.
void mergeObstacleVariances(
  f110_msgs::msg::Obstacle & target,
  const f110_msgs::msg::Obstacle & other)
{
  target.x_var = conservativeVariance(target.x_var, other.x_var);
  target.y_var = conservativeVariance(target.y_var, other.y_var);
  target.s_var = conservativeVariance(target.s_var, other.s_var);
  target.d_var = conservativeVariance(target.d_var, other.d_var);
  target.vs_var = conservativeVariance(target.vs_var, other.vs_var);
  target.vd_var = conservativeVariance(target.vd_var, other.vd_var);
}

// 폐곡선 Frenet s를 [0, track_length)로 정규화한다.
double wrapS(double s, double track_length)
{
  if (!(track_length > kGeometryEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

// wrap을 포함하는 from_s → to_s의 양수 전방 거리
double forwardDistance(double from_s, double to_s, double track_length)
{
  return wrapS(to_s - from_s, track_length);
}

// detector가 s=0을 걸친 장애물을 s_start > s_end로 표현해도 실제 짧은 span을 선택한다.
double obstacleLongitudinalSpan(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  const double forward = forwardDistance(obstacle.s_start, obstacle.s_end, track_length);
  const double reverse = forwardDistance(obstacle.s_end, obstacle.s_start, track_length);
  const double span = std::min(forward, reverse);
  if (std::isfinite(span) && span > kGeometryEpsilon) {
    return span;
  }
  return std::max(0.0, std::abs(obstacle.size));
}

// 폐곡선에서 두 중심 사이의 최단 거리에 진행 방향 부호를 붙인다.
// 첫 관측을 기준으로 여러 envelope를 하나의 펼친 구간에 합칠 때 사용한다.
double signedTrackDelta(double from_s, double to_s, double track_length)
{
  double delta = forwardDistance(from_s, to_s, track_length);
  if (delta > 0.5 * track_length) {
    delta -= track_length;
  }
  return delta;
}

// local_planning이 신뢰하는 detector Frenet 계약을 검사한다.
// Cartesian AABB는 선택적 진단 정보이므로 여기서 필수로 요구하지 않는다.
bool validFrenetObstacle(
  const f110_msgs::msg::Obstacle & obstacle,
  double track_length)
{
  if (!(track_length > kGeometryEpsilon) ||
    !std::isfinite(obstacle.s_center) ||
    !std::isfinite(obstacle.s_start) ||
    !std::isfinite(obstacle.s_end) ||
    !std::isfinite(obstacle.d_center) ||
    !std::isfinite(obstacle.d_right) ||
    !std::isfinite(obstacle.d_left) ||
    !std::isfinite(obstacle.size) ||
    obstacle.size < 0.0 ||
    obstacle.d_right > obstacle.d_left + kGeometryEpsilon)
  {
    return false;
  }
  const double longitudinal_span = obstacleLongitudinalSpan(obstacle, track_length);
  const double lateral_span = obstacle.d_left - obstacle.d_right;
  return longitudinal_span > kGeometryEpsilon || lateral_span > kGeometryEpsilon;
}

// RViz CUBE marker를 만들 수 있는 선택적 map-frame AABB인지 확인한다.
bool validCartesianAabb(const f110_msgs::msg::Obstacle & obstacle)
{
  return obstacle.has_cartesian &&
         std::isfinite(obstacle.x_min) &&
         std::isfinite(obstacle.x_max) &&
         std::isfinite(obstacle.y_min) &&
         std::isfinite(obstacle.y_max) &&
         obstacle.x_min <= obstacle.x_max &&
         obstacle.y_min <= obstacle.y_max;
}

// 같은 장애물 ID의 여러 관측을 모두 포함하는 보수적 Frenet/Cartesian envelope로 합친다.
// 중심 평균이 아니라 전체 경계의 합집합을 쓰므로 관측 jitter로 물체 크기가 줄어들지 않는다.
f110_msgs::msg::Obstacle mergeObstacleEnvelopes(
  const f110_msgs::msg::Obstacle & first,
  const f110_msgs::msg::Obstacle & second,
  double track_length)
{
  auto merged = first;
  const double first_half_span =
    0.5 * obstacleLongitudinalSpan(first, track_length);
  const double second_half_span =
    0.5 * obstacleLongitudinalSpan(second, track_length);
  const double second_center =
    signedTrackDelta(first.s_center, second.s_center, track_length);
  const double lower = std::min(
    -first_half_span, second_center - second_half_span);
  const double upper = std::max(
    first_half_span, second_center + second_half_span);

  // first 중심을 기준으로 펼친 lower/upper를 다시 폐곡선 s로 감싼다.
  merged.s_start = wrapS(first.s_center + lower, track_length);
  merged.s_end = wrapS(first.s_center + upper, track_length);
  merged.s_center = wrapS(
    first.s_center + 0.5 * (lower + upper), track_length);
  merged.d_right = std::min(first.d_right, second.d_right);
  merged.d_left = std::max(first.d_left, second.d_left);
  merged.d_center = 0.5 * (merged.d_right + merged.d_left);
  merged.size = std::hypot(upper - lower, merged.d_left - merged.d_right);
  mergeObstacleVariances(merged, second);

  const bool first_has_aabb = validCartesianAabb(first);
  const bool second_has_aabb = validCartesianAabb(second);
  if (first_has_aabb || second_has_aabb) {
    // 한 관측만 Cartesian 정보를 가져도 해당 AABB를 보존하고, 둘 다 있으면 합집합을 만든다.
    const auto & seed = first_has_aabb ? first : second;
    merged.has_cartesian = true;
    merged.x_min = seed.x_min;
    merged.x_max = seed.x_max;
    merged.y_min = seed.y_min;
    merged.y_max = seed.y_max;
    if (first_has_aabb && second_has_aabb) {
      merged.x_min = std::min(first.x_min, second.x_min);
      merged.x_max = std::max(first.x_max, second.x_max);
      merged.y_min = std::min(first.y_min, second.y_min);
      merged.y_max = std::max(first.y_max, second.y_max);
    }
    merged.x_center = 0.5 * (merged.x_min + merged.x_max);
    merged.y_center = 0.5 * (merged.y_min + merged.y_max);
    merged.radius = 0.5 * std::hypot(
      merged.x_max - merged.x_min, merged.y_max - merged.y_min);
  } else {
    merged.has_cartesian = false;
  }
  return merged;
}

}  // namespace

// 파라미터를 먼저 선언·검증하고 planner에 전달한 뒤 ROS 인터페이스를 생성한다.
LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options), planner_(planner_parameters_)
{
  initializeParameters();
  planner_.setParameters(planner_parameters_);
  initializeInterfaces();
  last_side_switch_time_ = now();
  RCLCPP_INFO(
    get_logger(),
    "Race-line-locked static planner started: lookahead=%.1f m, obstacle topic=%s",
    planner_parameters_.detection_lookahead_m, obstacles_topic_.c_str());
}

// ============================================================================
// ROS 파라미터 선언과 상호 제약 검증
// ============================================================================
// C++ 기본값은 launch 없이 ros2 run으로 실행해도 안전한 값이며, 운영값은
// config/local_planning.yaml에서 덮어쓴다.
void LocalPlannerNode::initializeParameters()
{
  // ── 장애물 탐색·군집·충돌 여유 ───────────────────────────────────────────
  planner_parameters_.detection_lookahead_m =
    declare_parameter<double>("detection_lookahead_m", 12.0);
  planner_parameters_.obstacle_cluster_gap_m =
    declare_parameter<double>("obstacle_cluster_gap_m", 0.8);
  planner_parameters_.obstacle_longitudinal_padding_m =
    declare_parameter<double>("obstacle_longitudinal_padding_m", 0.35);
  planner_parameters_.obstacle_clearance_m =
    declare_parameter<double>("obstacle_clearance_m", 0.35);
  planner_parameters_.blocking_margin_m =
    declare_parameter<double>("blocking_margin_m", 0.10);
  planner_parameters_.vehicle_half_width_m =
    declare_parameter<double>("vehicle_half_width_m", 0.121);
  planner_parameters_.boundary_margin_m =
    declare_parameter<double>("boundary_margin_m", 0.13);
  planner_parameters_.fallback_track_half_width_m =
    declare_parameter<double>("fallback_track_half_width_m", 1.50);

  // ── spline 제어점, 전환 길이와 목표 d ─────────────────────────────────────
  planner_parameters_.pre_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "pre_apex_distances_m", std::vector<double>{4.0, 3.0, 1.5});
  planner_parameters_.post_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "post_apex_distances_m", std::vector<double>{1.5, 3.0, 4.0});
  planner_parameters_.transition_distance_scales =
    declare_parameter<std::vector<double>>(
    "transition_distance_scales", std::vector<double>{1.0, 1.25, 1.50});
  planner_parameters_.outside_line_transition_scale =
    declare_parameter<double>("outside_line_transition_scale", 1.35);
  planner_parameters_.post_merge_lookahead_m =
    declare_parameter<double>("post_merge_lookahead_m", 2.0);
  planner_parameters_.post_merge_min_time_sec =
    declare_parameter<double>("post_merge_min_time_sec", 1.0);
  planner_parameters_.minimum_target_offset_m =
    declare_parameter<double>("minimum_target_offset_m", 0.20);
  planner_parameters_.maximum_target_offset_m =
    declare_parameter<double>("maximum_target_offset_m", 1.50);
  planner_parameters_.commitment_clearance_reserve_m =
    declare_parameter<double>("commitment_clearance_reserve_m", 0.05);
  planner_parameters_.maximum_lateral_slope =
    declare_parameter<double>("maximum_lateral_slope", 0.65);
  planner_parameters_.maximum_curvature_radpm =
    declare_parameter<double>("maximum_curvature_radpm", 3.20);
  planner_parameters_.maximum_curvature_rate_radpm2 =
    declare_parameter<double>("maximum_curvature_rate_radpm2", 20.0);

  // ── 회피 불가능 시 감속 경로 ───────────────────────────────────────────────
  planner_parameters_.safe_stop_buffer_m =
    declare_parameter<double>("safe_stop_buffer_m", 0.80);
  planner_parameters_.safe_stop_deceleration_mps2 =
    declare_parameter<double>("safe_stop_deceleration_mps2", 2.5);
  planner_parameters_.minimum_path_points =
    declare_parameter<int>("minimum_path_points", 8);

  // ── 입력 freshness와 상태 전이 ─────────────────────────────────────────────
  require_obstacles_message_ = declare_parameter<bool>("require_obstacles_message", true);
  obstacle_stale_timeout_sec_ = declare_parameter<double>("obstacle_stale_timeout_sec", 0.75);
  odometry_stale_timeout_sec_ = declare_parameter<double>("odometry_stale_timeout_sec", 0.50);
  merge_lateral_tolerance_m_ = declare_parameter<double>("merge_lateral_tolerance_m", 0.15);
  merge_confirm_cycles_ = declare_parameter<int>("merge_confirm_cycles", 15);
  safe_stop_release_cycles_ = declare_parameter<int>("safe_stop_release_cycles", 8);
  planning_period_ms_ = declare_parameter<int>("planning_period_ms", 50);
  state_handoff_tail_ratio_ = declare_parameter<double>("state_handoff_tail_ratio", 0.10);
  state_handoff_speed_cap_mps_ =
    declare_parameter<double>("state_handoff_speed_cap_mps", 6.0);
  initial_observation_count_ =
    declare_parameter<int>("initial_observation_count", 3);
  initial_observation_min_duration_sec_ =
    declare_parameter<double>("initial_observation_min_duration_sec", 0.15);
  initial_observation_max_wait_sec_ =
    declare_parameter<double>("initial_observation_max_wait_sec", 0.35);
  commitment_soft_violation_confirm_cycles_ =
    declare_parameter<int>("commitment_soft_violation_confirm_cycles", 3);
  hard_collision_margin_m_ =
    declare_parameter<double>("hard_collision_margin_m", 0.03);
  chain_release_margin_m_ =
    declare_parameter<double>("chain_release_margin_m", 0.20);
  guard_parameters_.uncertainty_sigma_scale =
    declare_parameter<double>("uncertainty_sigma_scale", 3.0);
  guard_parameters_.minimum_longitudinal_margin_m =
    declare_parameter<double>("uncertainty_min_longitudinal_margin_m", 0.05);
  guard_parameters_.minimum_lateral_margin_m =
    declare_parameter<double>("uncertainty_min_lateral_margin_m", 0.03);
  commitment_lock_lateral_threshold_m_ =
    declare_parameter<double>("commitment_lock_lateral_threshold_m", 0.10);
  commitment_lock_longitudinal_m_ =
    declare_parameter<double>("commitment_lock_longitudinal_m", 0.50);
  obstacle_marker_scale_m_ = declare_parameter<double>("obstacle_marker_scale_m", 0.35);
  path_marker_width_m_ = declare_parameter<double>("path_marker_width_m", 0.06);

  // ── ROS 토픽 및 frame 이름 ─────────────────────────────────────────────────
  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  obstacles_topic_ =
    declare_parameter<std::string>(
    "obstacles_topic", "/static_obs");
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  state_topic_ =
    declare_parameter<std::string>("state_topic", "/state");
  ot_waypoints_topic_ =
    declare_parameter<std::string>("ot_waypoints_topic", "/avoid_waypoints");
  local_path_topic_ =
    declare_parameter<std::string>("local_path_topic", "/local_planning/path");
  compatibility_path_topic_ =
    declare_parameter<std::string>("compatibility_path_topic", "/local_path");
  markers_topic_ =
    declare_parameter<std::string>("markers_topic", "/local_planning/markers");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  // 제어점은 [먼 점 > 중간 점 > 가까운 점 > 0] 순서여야 하고, 전환 배율은 짧은
  // spline부터 평가하도록 양수 오름차순이어야 한다.
  const bool control_points_valid =
    planner_parameters_.pre_apex_distances_m.size() == 3U &&
    planner_parameters_.post_apex_distances_m.size() == 3U &&
    planner_parameters_.pre_apex_distances_m[0] > planner_parameters_.pre_apex_distances_m[1] &&
    planner_parameters_.pre_apex_distances_m[1] > planner_parameters_.pre_apex_distances_m[2] &&
    planner_parameters_.pre_apex_distances_m[2] > 0.0 &&
    planner_parameters_.post_apex_distances_m[0] > 0.0 &&
    planner_parameters_.post_apex_distances_m[1] > planner_parameters_.post_apex_distances_m[0] &&
    planner_parameters_.post_apex_distances_m[2] > planner_parameters_.post_apex_distances_m[1];
  const bool scales_valid =
    !planner_parameters_.transition_distance_scales.empty() &&
    std::all_of(
    planner_parameters_.transition_distance_scales.begin(),
    planner_parameters_.transition_distance_scales.end(),
    [](double scale) {return std::isfinite(scale) && scale > 0.0;}) &&
    std::adjacent_find(
    planner_parameters_.transition_distance_scales.begin(),
    planner_parameters_.transition_distance_scales.end(),
    std::greater_equal<double>()) == planner_parameters_.transition_distance_scales.end();
  if (!control_points_valid || !scales_valid) {
    throw std::invalid_argument(
            "pre/post apex arrays must contain three ordered positive values and transition "
            "scales must be positive");
  }

  // 주기·횟수·거리의 범위뿐 아니라 hard clearance가 일반 obstacle clearance보다
  // 커지지 않는지도 함께 검사해 hard/soft 충돌 분류가 뒤집히는 설정을 막는다.
  if (planning_period_ms_ <= 0 || merge_confirm_cycles_ <= 0 ||
    safe_stop_release_cycles_ <= 0 ||
    planner_parameters_.commitment_clearance_reserve_m < 0.0 ||
    planner_parameters_.post_merge_lookahead_m < 0.0 ||
    planner_parameters_.post_merge_min_time_sec < 0.0 ||
    !(state_handoff_tail_ratio_ > 0.0) || state_handoff_tail_ratio_ > 1.0 ||
    !(state_handoff_speed_cap_mps_ > 0.0) ||
    initial_observation_count_ <= 0 ||
    !std::isfinite(initial_observation_min_duration_sec_) ||
    initial_observation_min_duration_sec_ < 0.0 ||
    !std::isfinite(initial_observation_max_wait_sec_) ||
    initial_observation_max_wait_sec_ < 0.0 ||
    initial_observation_max_wait_sec_ < initial_observation_min_duration_sec_ ||
    commitment_soft_violation_confirm_cycles_ <= 0 ||
    !std::isfinite(hard_collision_margin_m_) ||
    hard_collision_margin_m_ < 0.0 ||
    planner_parameters_.vehicle_half_width_m + hard_collision_margin_m_ >
    planner_parameters_.obstacle_clearance_m ||
    !std::isfinite(chain_release_margin_m_) ||
    chain_release_margin_m_ < 0.0 ||
    !std::isfinite(guard_parameters_.uncertainty_sigma_scale) ||
    guard_parameters_.uncertainty_sigma_scale < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_longitudinal_margin_m) ||
    guard_parameters_.minimum_longitudinal_margin_m < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_lateral_margin_m) ||
    guard_parameters_.minimum_lateral_margin_m < 0.0 ||
    commitment_lock_lateral_threshold_m_ < 0.0 ||
    commitment_lock_longitudinal_m_ < 0.0 ||
    planner_parameters_.minimum_path_points < 2)
  {
    throw std::invalid_argument(
            "planning periods, confirmation counts, clearance reserve, handoff settings, "
            "observation/uncertainty guard settings, hard/soft collision thresholds, commitment "
            "chain release, commitment locks, and point counts must be valid");
  }
}

// ============================================================================
// ROS 구독·발행·timer 배선
// ============================================================================
void LocalPlannerNode::initializeInterfaces()
{
  // planning 입력과 timer는 한 그룹에서 직렬 처리해 상태 전이를 보호한다. odometry만 별도
  // 그룹으로 분리하고 mutex로 스냅샷을 복사해 계획 중에도 최신 위치 수신을 허용한다.
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 고주기 입력/출력은 최신 1개만 사용한다. 기준 경로와 state는 늦게 참가한 노드도 마지막
  // 값을 받도록 transient_local을 사용한다.
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions odometry_options;
  odometry_options.callback_group = odometry_callback_group_;

  // 입력 구독
  global_waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    global_waypoints_topic_, global_qos,
    std::bind(&LocalPlannerNode::onGlobalWaypoints, this, std::placeholders::_1), planning_options);
  obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    obstacles_topic_, volatile_qos,
    std::bind(&LocalPlannerNode::onObstacles, this, std::placeholders::_1), planning_options);
  frenet_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, volatile_qos,
    std::bind(&LocalPlannerNode::onFrenetOdometry, this, std::placeholders::_1), odometry_options);
  state_sub_ = create_subscription<f110_msgs::msg::StateMachine>(
    state_topic_, global_qos,
    std::bind(&LocalPlannerNode::onState, this, std::placeholders::_1), planning_options);

  // 제어 경로와 시각화 출력
  avoid_waypoints_pub_ =
    create_publisher<f110_msgs::msg::OTWpntArray>(ot_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, volatile_qos);
  compatibility_path_pub_ =
    create_publisher<nav_msgs::msg::Path>(compatibility_path_topic_, volatile_qos);
  markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, volatile_qos);

  // 계획 주기는 wall timer이며 use_sim_time과 무관하게 안전 판정을 계속 수행한다.
  planning_timer_ = create_wall_timer(
    std::chrono::milliseconds(planning_period_ms_),
    std::bind(&LocalPlannerNode::onPlanningTimer, this), planning_callback_group_);
}

// 기준 경로의 핵심 기하(s/x/y)가 완전히 같은지 확인해 latched 재발행에 따른 불필요한
// commitment 초기화를 막는다.
bool LocalPlannerNode::sameReference(const f110_msgs::msg::WpntArray & message) const
{
  if (message.wpnts.size() != global_waypoints_.wpnts.size() || message.wpnts.empty()) {
    return false;
  }
  for (std::size_t i = 0; i < message.wpnts.size(); ++i) {
    const auto & first = message.wpnts[i];
    const auto & second = global_waypoints_.wpnts[i];
    if (std::abs(first.s_m - second.s_m) > 1.0e-9 ||
      std::abs(first.x_m - second.x_m) > 1.0e-9 ||
      std::abs(first.y_m - second.y_m) > 1.0e-9)
    {
      return false;
    }
  }
  return true;
}

// 새 글로벌 경로는 planner의 단조 s/유한값 검증을 통과한 뒤에만 적용한다.
// 기준 기하가 바뀌면 이전 경로의 안전성을 보장할 수 없으므로 commitment를 모두 해제한다.
void LocalPlannerNode::onGlobalWaypoints(
  const f110_msgs::msg::WpntArray::SharedPtr message)
{
  if (sameReference(*message)) {
    return;
  }
  std::string error;
  if (!planner_.setReference(*message, &error)) {
    has_global_waypoints_ = false;
    clearCommitment();
    RCLCPP_ERROR(get_logger(), "Rejected /global_waypoints: %s", error.c_str());
    return;
  }
  global_waypoints_ = *message;
  has_global_waypoints_ = true;
  clearCommitment();
  RCLCPP_INFO(
    get_logger(), "Loaded %zu ordered global race-line waypoints (track %.2f m).",
    global_waypoints_.wpnts.size(), planner_.trackLength());
}

// detector가 소유하는 Frenet 경계만 선별해 최신 장애물 스냅샷으로 교체한다.
// 잘못된 frame 메시지는 마지막 정상 스냅샷을 지우지 않는다.
void LocalPlannerNode::onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring obstacle array in frame '%s'; expected '%s'. Retaining the last valid snapshot.",
      message->header.frame_id.c_str(), frame_id_.c_str());
    return;
  }

  std::vector<f110_msgs::msg::Obstacle> accepted_obstacles;
  accepted_obstacles.reserve(message->obstacles.size());
  std::size_t rejected = 0;
  for (const auto & obstacle : message->obstacles) {
    if (validFrenetObstacle(obstacle, planner_.trackLength())) {
      accepted_obstacles.push_back(obstacle);
    } else {
      ++rejected;
    }
  }
  if (rejected > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected %zu static obstacles with invalid detector-provided Frenet bounds.",
      rejected);
  }
  static_obstacles_ = std::move(accepted_obstacles);
  has_obstacles_message_ = true;
  last_obstacles_time_ = now();

  // timer tick이 아니라 실제 /static_obs 메시지를 안정화 횟수로 세기 위한 sequence
  ++obstacles_message_sequence_;
  if (obstacle_perception_degraded_) {
    obstacle_perception_degraded_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Static-obstacle perception recovered; replaced retained memory with a fresh snapshot.");
  }
}

// 유한한 Frenet odometry만 별도 callback group에서 원자적인 스냅샷으로 갱신한다.
void LocalPlannerNode::onFrenetOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!finiteOdometry(*message)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Ignoring non-finite Frenet odometry.");
    return;
  }
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  latest_odometry_ = *message;
  last_odometry_time_ = now();
  has_odometry_ = true;
}

// state_machine이 이번 회피에서 실제 STATE_AVOID를 거쳤는지 기억한다.
// 기하 합류 후 STATE_GLOBAL을 확인할 때 이 이력이 있어야 경로 발행을 끝낼 수 있다.
void LocalPlannerNode::onState(const f110_msgs::msg::StateMachine::SharedPtr message)
{
  current_state_ = message->state;
  has_state_ = true;
  if (has_commitment_ &&
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID)
  {
    avoid_state_observed_ = true;
  }
}

// 기준 경로 변경 또는 정상 GLOBAL 복귀 시 모든 회피·정지·안정화 상태를 초기화한다.
void LocalPlannerNode::clearCommitment()
{
  resetInitialStabilization();
  resetNextManeuverStabilization();
  resetCommitmentViolationConfirmation();
  completed_obstacle_ids_.clear();
  has_commitment_ = false;
  committed_result_ = RacelineSplineResult();
  safe_stop_latched_ = false;
  safe_stop_result_ = RacelineSplineResult();
  safe_stop_release_count_ = 0;
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = false;
  committed_obstacle_guards_.clear();
}

// 최초 blocking cluster의 준비 경로, envelope 합집합과 메시지별 횟수를 초기화한다.
void LocalPlannerNode::resetInitialStabilization()
{
  initial_stabilization_active_ = false;
  initial_prepare_published_ = false;
  initial_has_counted_sequence_ = false;
  initial_cluster_union_.clear();
  initial_observation_counts_.clear();
}

// 현재 maneuver 뒤에서 동시에 모으는 다음 cluster 안정화 상태를 초기화한다.
void LocalPlannerNode::resetNextManeuverStabilization()
{
  next_stabilization_active_ = false;
  next_has_counted_sequence_ = false;
  next_cluster_union_.clear();
  next_observation_counts_.clear();
}

// uncertainty 여유만 침범한 soft collision의 연속 확인 횟수를 초기화한다.
void LocalPlannerNode::resetCommitmentViolationConfirmation()
{
  commitment_soft_violation_count_ = 0;
}

// 완료한 장애물은 제외하고 현재 detector 스냅샷과 안정화 중 누적 envelope를 ID별로 합친다.
std::vector<f110_msgs::msg::Obstacle>
LocalPlannerNode::buildInitialStabilizationInput() const
{
  std::map<int, f110_msgs::msg::Obstacle> conservative;
  for (const auto & obstacle : static_obstacles_) {
    if (completed_obstacle_ids_.count(obstacle.id) > 0U) {
      continue;
    }
    conservative[obstacle.id] = obstacle;
  }
  for (const auto & entry : initial_cluster_union_) {
    const int id = entry.first;
    const auto current = conservative.find(id);
    if (current == conservative.end()) {
      conservative[id] = entry.second;
      continue;
    }
    conservative[id] = mergeObstacleEnvelopes(
      current->second, entry.second, planner_.trackLength());
  }

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(conservative.size());
  for (const auto & entry : conservative) {
    result.push_back(entry.second);
  }
  return result;
}

// detector envelope 각각에 위치 분산과 고정 크기 오차를 반영한 Guard를 만든다.
std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildGuardedObstacles(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  std::vector<f110_msgs::msg::Obstacle> guarded;
  guarded.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    guarded.push_back(
      buildUncertaintyGuard(obstacle, planner_.trackLength(), guard_parameters_));
  }
  return guarded;
}

// ============================================================================
// 최초 장애물 군집 관측 안정화
// ============================================================================
// 동일 timer에서 여러 번 불려도 실제 새 /static_obs 메시지가 들어왔을 때만 ID별 횟수를
// 올린다. 군집이 완전히 바뀌면 누적 envelope가 다른 물체에 섞이지 않도록 다시 시작한다.
bool LocalPlannerNode::updateInitialStabilization(
  const std::vector<int> & cluster_ids,
  const std::vector<f110_msgs::msg::Obstacle> & conservative_obstacles,
  const rclcpp::Time & update_time)
{
  if (cluster_ids.empty()) {
    resetInitialStabilization();
    return false;
  }

  const auto contains_id = [](const std::vector<int> & ids, int id) {
      return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
  bool restart = !initial_stabilization_active_;
  if (!restart && !initial_cluster_union_.empty()) {
    restart = std::none_of(
      initial_cluster_union_.begin(), initial_cluster_union_.end(),
      [&cluster_ids, &contains_id](const auto & entry) {
        return contains_id(cluster_ids, entry.first);
      });
  }
  if (restart) {
    initial_stabilization_active_ = true;
    initial_stabilization_start_ = update_time;
    initial_has_counted_sequence_ = false;
    initial_cluster_union_.clear();
    initial_observation_counts_.clear();
  }

  // obstacles_message_sequence_가 달라진 경우만 새 관측으로 인정한다.
  const bool new_obstacle_message =
    !initial_has_counted_sequence_ ||
    initial_last_counted_sequence_ != obstacles_message_sequence_;
  if (new_obstacle_message) {
    initial_has_counted_sequence_ = true;
    initial_last_counted_sequence_ = obstacles_message_sequence_;
    for (const int id : cluster_ids) {
      const auto current = std::find_if(
        static_obstacles_.begin(), static_obstacles_.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      if (current == static_obstacles_.end()) {
        continue;
      }
      const auto conservative = std::find_if(
        conservative_obstacles.begin(), conservative_obstacles.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      initial_cluster_union_[id] =
        conservative != conservative_obstacles.end() ? *conservative : *current;
      ++initial_observation_counts_[id];
    }
  }

  // 군집을 구성하는 모든 ID가 요구 횟수와 최소 시간을 만족해야 정상적으로 안정화된다.
  // 단, 최대 대기시간을 넘으면 장애물 앞에서 무기한 준비 상태에 머무르지 않고 진행한다.
  const bool observation_count_reached = std::all_of(
    cluster_ids.begin(), cluster_ids.end(),
    [this](int id) {
      const auto count = initial_observation_counts_.find(id);
      return count != initial_observation_counts_.end() &&
             count->second >= initial_observation_count_;
    });
  const double total_duration = (update_time - initial_stabilization_start_).seconds();
  const bool minimum_duration_reached =
    total_duration >= initial_observation_min_duration_sec_;
  return (observation_count_reached && minimum_duration_reached) ||
         total_duration >= initial_observation_max_wait_sec_;
}

std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildNextManeuverInput() const
{
  // 완료된 ID와 현재 commitment의 ID는 다음 maneuver 후보에서 제외한다.
  std::set<int> excluded = completed_obstacle_ids_;
  if (has_commitment_) {
    excluded.insert(
      committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
    if (committed_result_.obstacle_id >= 0) {
      excluded.insert(committed_result_.obstacle_id);
    }
  }

  // 최신 스냅샷이 일부 관측을 놓치더라도 이미 누적한 다음 cluster envelope를 합쳐 보존한다.
  std::map<int, f110_msgs::msg::Obstacle> conservative;
  for (const auto & obstacle : static_obstacles_) {
    if (excluded.count(obstacle.id) == 0U) {
      conservative[obstacle.id] = obstacle;
    }
  }
  for (const auto & entry : next_cluster_union_) {
    if (excluded.count(entry.first) > 0U) {
      continue;
    }
    const auto current = conservative.find(entry.first);
    if (current == conservative.end()) {
      conservative[entry.first] = entry.second;
    } else {
      current->second = mergeObstacleEnvelopes(
        current->second, entry.second, planner_.trackLength());
    }
  }

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(conservative.size());
  for (const auto & entry : conservative) {
    result.push_back(entry.second);
  }
  return buildGuardedObstacles(result);
}

// ============================================================================
// 다음 maneuver 사전 안정화
// ============================================================================
// 현재 spline의 merge 지점에 차량이 있다고 가정하고, 그 뒤에서 처음 만날 blocking cluster를
// 현재 maneuver 진행 중에 미리 누적한다. 그래야 첫 maneuver 종료 직후 빈 경로 없이 연결된다.
bool LocalPlannerNode::updateNextManeuverStabilization(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
  const rclcpp::Time & update_time)
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    resetNextManeuverStabilization();
    return false;
  }

  const EgoFrenetState merge_ego{committed_result_.merge_s, 0.0, ego.speed};
  const auto cluster_ids = planner_.blockingClusterIds(merge_ego, next_obstacles);
  if (cluster_ids.empty()) {
    resetNextManeuverStabilization();
    return false;
  }
  const auto contains_id = [&cluster_ids](int id) {
      return std::find(cluster_ids.begin(), cluster_ids.end(), id) != cluster_ids.end();
    };
  bool restart = !next_stabilization_active_;
  if (!restart && !next_cluster_union_.empty()) {
    restart = std::none_of(
      next_cluster_union_.begin(), next_cluster_union_.end(),
      [&contains_id](const auto & entry) {return contains_id(entry.first);});
  }
  if (restart) {
    next_stabilization_active_ = true;
    next_stabilization_start_ = update_time;
    next_has_counted_sequence_ = false;
    next_cluster_union_.clear();
    next_observation_counts_.clear();
  }

  // 최초 안정화와 마찬가지로 timer 호출 수가 아닌 실제 obstacle 메시지 수만 센다.
  const bool new_obstacle_message =
    !next_has_counted_sequence_ ||
    next_last_counted_sequence_ != obstacles_message_sequence_;
  if (new_obstacle_message) {
    next_has_counted_sequence_ = true;
    next_last_counted_sequence_ = obstacles_message_sequence_;
    for (const int id : cluster_ids) {
      const auto current = std::find_if(
        static_obstacles_.begin(), static_obstacles_.end(),
        [id](const auto & candidate) {return candidate.id == id;});
      if (current == static_obstacles_.end()) {
        continue;
      }
      const auto previous = next_cluster_union_.find(id);
      next_cluster_union_[id] = previous == next_cluster_union_.end() ?
        *current :
        mergeObstacleEnvelopes(previous->second, *current, planner_.trackLength());
      ++next_observation_counts_[id];
    }
  }

  // perception이 stale이면 새 표본이 올 수 없으므로 보존한 마지막 정상 스냅샷을 즉시 사용한다.
  const bool observation_count_reached = std::all_of(
    cluster_ids.begin(), cluster_ids.end(),
    [this](int id) {
      const auto count = next_observation_counts_.find(id);
      return count != next_observation_counts_.end() &&
             count->second >= initial_observation_count_;
    });
  const double duration = (update_time - next_stabilization_start_).seconds();
  return obstacle_perception_degraded_ ||
         (observation_count_reached && duration >= initial_observation_min_duration_sec_) ||
         duration >= initial_observation_max_wait_sec_;
}

// 다음 cluster를 실제 새 maneuver로 넘길 때 누적 상태를 최초 안정화 슬롯으로 이동한다.
void LocalPlannerNode::promoteNextManeuverStabilization()
{
  if (next_stabilization_active_) {
    initial_stabilization_active_ = true;
    initial_stabilization_start_ = next_stabilization_start_;
    initial_has_counted_sequence_ = next_has_counted_sequence_;
    initial_last_counted_sequence_ = next_last_counted_sequence_;
    initial_cluster_union_ = next_cluster_union_;
    initial_observation_counts_ = next_observation_counts_;
  }
  resetNextManeuverStabilization();
}

// 현재 ego부터 spline이 실제 d=0으로 복귀하는 merge_s까지의 남은 전방 거리
double LocalPlannerNode::remainingDistanceToMerge(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return 0.0;
  }
  const double planned_distance =
    planner_.forwardDistance(commitment_start_s_, committed_result_.merge_s);
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  if (driven_distance >= 0.5 * planner_.trackLength() ||
    driven_distance + 1.0e-6 >= planned_distance)
  {
    return 0.0;
  }
  return planned_distance - driven_distance;
}

// 현재 maneuver의 모든 동결 Guard 뒤쪽과 chain_release_margin_m을 완전히 지났는지 확인한다.
// 이를 만족해야 다음 장애물이 기존 spline merge 전에 새 경로로 선점할 수 있다.
bool LocalPlannerNode::activeManeuverObstacleCleared(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_obstacle_guards_.empty()) {
    return false;
  }
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  if (driven_distance >= 0.5 * planner_.trackLength()) {
    return false;
  }
  double active_rear_distance = 0.0;
  for (const auto & entry : committed_obstacle_guards_) {
    active_rear_distance = std::max(
      active_rear_distance,
      planner_.forwardDistance(commitment_start_s_, entry.second.s_end) +
      planner_parameters_.obstacle_longitudinal_padding_m);
  }
  return driven_distance + 1.0e-6 >= active_rear_distance + chain_release_margin_m_;
}

// 현재 장애물은 통과했지만 아직 d=0 merge 전일 때, 다음 장애물 spline을 현재 ego.d에서
// 직접 시작한다. AVOID를 끊지 않고 다음 commitment로 교체하는 연속 maneuver 경로다.
bool LocalPlannerNode::tryEarlyChainedManeuver(
  const EgoFrenetState & ego,
  std::vector<f110_msgs::msg::Obstacle> & next_obstacles)
{
  if (!has_commitment_ || merge_geometry_confirmed_) {
    return false;
  }
  next_obstacles = buildNextManeuverInput();
  const bool next_stable = updateNextManeuverStabilization(ego, next_obstacles, now());
  if (!next_stable || !activeManeuverObstacleCleared(ego)) {
    return false;
  }

  // 안정화됐더라도 현재 ego.d에서 실제 안전한 avoidance가 만들어질 때만 선점한다.
  RacelineSplineResult next_result = planner_.plan(ego, next_obstacles);
  if (next_result.kind != SplinePlanKind::kAvoidance) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Next static maneuver is stabilized but not yet feasible from ego "
      "(ego_s=%.3f ego_d=%.3f current_merge_s=%.3f): %s",
      ego.s, ego.d, committed_result_.merge_s, next_result.reason.c_str());
    return false;
  }

  const int completed_id = committed_result_.obstacle_id;
  const double previous_merge_s = committed_result_.merge_s;
  resetForChainedManeuver();
  commitAvoidance(std::move(next_result), ego, next_obstacles);
  resetNextManeuverStabilization();
  RCLCPP_INFO(
    get_logger(),
    "Preemptively chained completed obstacle %d to obstacle %d before the old merge "
    "(ego_s=%.3f ego_d=%.3f old_merge_s=%.3f); STATE_AVOID remains active.",
    completed_id, committed_result_.obstacle_id, ego.s, ego.d, previous_merge_s);
  return true;
}

// ============================================================================
// 현재 commitment 검증용 장애물 입력 구성
// ============================================================================
// 현재 maneuver의 obstacle은 항상 포함하고, 새 obstacle은 팽창된 앞면이 실제 spline merge
// 이전에 시작할 때만 포함한다. merge 뒤 controller tail과의 충돌은 다음 maneuver가 처리한다.
std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::buildCurrentManeuverInput(
  const EgoFrenetState & ego,
  bool apply_uncertainty_guard) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    const auto initial = buildInitialStabilizationInput();
    return apply_uncertainty_guard ? buildGuardedObstacles(initial) : initial;
  }

  std::set<int> active_ids(
    committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
  if (committed_result_.obstacle_id >= 0) {
    active_ids.insert(committed_result_.obstacle_id);
  }
  const double merge_forward = planner_.forwardDistance(ego.s, committed_result_.merge_s);
  const bool merge_is_ahead = merge_forward < 0.5 * planner_.trackLength();

  std::vector<f110_msgs::msg::Obstacle> result;
  result.reserve(static_obstacles_.size() + committed_obstacle_guards_.size());
  std::set<int> observed_active_ids;
  for (const auto & obstacle : static_obstacles_) {
    if (completed_obstacle_ids_.count(obstacle.id) > 0U) {
      continue;
    }
    auto validation_obstacle = apply_uncertainty_guard ?
      buildUncertaintyGuard(obstacle, planner_.trackLength(), guard_parameters_) :
      obstacle;
    if (active_ids.count(obstacle.id) > 0U) {
      observed_active_ids.insert(obstacle.id);
      const auto committed_guard = committed_obstacle_guards_.find(obstacle.id);

      // 최신 같은-ID envelope 전체가 동결 Guard 안이면 경로가 측정 jitter를 따라 움직이지
      // 않도록 commitment 시점 Guard를 계속 사용한다. Guard를 벗어난 관측은 그대로 검증한다.
      if (apply_uncertainty_guard &&
        committed_guard != committed_obstacle_guards_.end() &&
        obstacleEnvelopeContained(
          validation_obstacle, committed_guard->second, planner_.trackLength()))
      {
        validation_obstacle = committed_guard->second;
      }
    }

    // merge가 이미 뒤에 있거나 현재 commitment obstacle이면 필터링 없이 검증한다.
    if (!merge_is_ahead || active_ids.count(obstacle.id) > 0U) {
      result.push_back(validation_obstacle);
      continue;
    }

    const double center_forward = planner_.forwardDistance(ego.s, validation_obstacle.s_center);
    const double span_forward =
      planner_.forwardDistance(validation_obstacle.s_start, validation_obstacle.s_end);
    const double span_reverse =
      planner_.forwardDistance(validation_obstacle.s_end, validation_obstacle.s_start);
    double span = std::min(span_forward, span_reverse);
    if (!(span > 1.0e-6)) {
      span = std::max(0.05, std::abs(validation_obstacle.size));
    }
    const double start_forward = center_forward - 0.5 * span -
      planner_parameters_.obstacle_longitudinal_padding_m;
    if (start_forward <= merge_forward + 1.0e-6) {
      result.push_back(validation_obstacle);
    }
  }

  // detector가 통과 중인 장애물을 잠시 누락해도 동결 Guard로 현재 경로 검증을 계속한다.
  if (apply_uncertainty_guard) {
    for (const int id : active_ids) {
      if (observed_active_ids.count(id) > 0U) {
        continue;
      }
      const auto committed_guard = committed_obstacle_guards_.find(id);
      if (committed_guard != committed_obstacle_guards_.end()) {
        result.push_back(committed_guard->second);
      }
    }
  }
  return result;
}

// 완료한 obstacle ID는 기억하되 이전 방향 잠금과 경로 기하만 해제한다.
// state_machine의 AVOID 관측 이력은 보존해 maneuver 사이에 GLOBAL로 잘못 떨어지지 않게 한다.
void LocalPlannerNode::resetForChainedManeuver()
{
  completed_obstacle_ids_.insert(
    committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
  if (committed_result_.obstacle_id >= 0) {
    completed_obstacle_ids_.insert(committed_result_.obstacle_id);
  }
  const bool avoid_was_observed = has_state_ ?
    current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID :
    avoid_state_observed_;
  resetInitialStabilization();
  resetCommitmentViolationConfirmation();
  has_commitment_ = false;
  committed_result_ = RacelineSplineResult();
  safe_stop_latched_ = false;
  safe_stop_result_ = RacelineSplineResult();
  safe_stop_release_count_ = 0;
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = avoid_was_observed;
  committed_obstacle_guards_.clear();
}

// merge 완료 또는 global handoff 중 아직 막는 다음 cluster가 있으면 현재 maneuver를 종료하고
// 다음 안정화 상태를 승격한다. 새 방향은 이전 commitment와 독립적으로 다시 선택한다.
bool LocalPlannerNode::beginChainedManeuverIfNeeded(
  const EgoFrenetState & ego,
  std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
  const std::string & phase)
{
  if (!has_commitment_) {
    return false;
  }
  next_obstacles = buildNextManeuverInput();
  const auto next_cluster_ids = planner_.blockingClusterIds(ego, next_obstacles);
  if (next_cluster_ids.empty()) {
    return false;
  }

  std::string ids;
  for (const int id : next_cluster_ids) {
    ids += ids.empty() ? std::to_string(id) : "," + std::to_string(id);
  }
  RCLCPP_INFO(
    get_logger(),
    "Chaining static avoidance during %s for new blocking cluster [%s]; "
    "releasing the completed maneuver's side lock.",
    phase.c_str(), ids.c_str());
  resetForChainedManeuver();
  promoteNextManeuverStabilization();
  return true;
}

// 차량이 선택 방향으로 실제 이동했거나 시작점에서 충분히 전진하면 방향을 잠근다.
// 진입 전에는 새 장애물로 기존 쪽이 막힐 경우 반대편 재평가를 한 번 허용한다.
bool LocalPlannerNode::commitmentSideLocked(const EgoFrenetState & ego) const
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return false;
  }
  const bool lateral_engaged = committed_result_.go_left ?
    ego.d >= commitment_lock_lateral_threshold_m_ :
    ego.d <= -commitment_lock_lateral_threshold_m_;
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  const bool longitudinal_engaged =
    driven_distance >= commitment_lock_longitudinal_m_ &&
    driven_distance < 0.5 * planner_.trackLength();
  return lateral_engaged || longitudinal_engaged;
}

// 검증을 통과한 회피 결과를 commitment로 확정하고 관련 장애물 Guard를 동결한다.
void LocalPlannerNode::commitAvoidance(
  RacelineSplineResult result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  const bool replacing = has_commitment_;
  const bool avoid_was_observed = avoid_state_observed_;
  std::set<int> committed_ids(result.obstacle_ids.begin(), result.obstacle_ids.end());
  if (result.obstacle_id >= 0) {
    committed_ids.insert(result.obstacle_id);
  }

  // planning_obstacles에는 안정화 합집합과 uncertainty 확장이 이미 반영되어 있으므로
  // 해당 ID의 envelope를 그대로 동결 기준으로 저장한다.
  committed_obstacle_guards_.clear();
  for (const auto & obstacle : planning_obstacles) {
    if (committed_ids.count(obstacle.id) > 0U) {
      committed_obstacle_guards_[obstacle.id] = obstacle;
    }
  }
  resetInitialStabilization();
  resetCommitmentViolationConfirmation();
  committed_result_ = std::move(result);
  has_commitment_ = true;
  safe_stop_latched_ = false;
  safe_stop_result_ = RacelineSplineResult();
  safe_stop_release_count_ = 0;
  commitment_start_s_ = ego.s;
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  avoid_state_observed_ = avoid_was_observed ||
    (has_state_ && current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID);
  if (!replacing) {
    resetNextManeuverStabilization();
  }
  RCLCPP_INFO(
    get_logger(), "%s %s d-offset spline around static obstacle %d (target d=%.2f).",
    replacing ? "Replaced commitment with" : "Committed",
    committed_result_.go_left ? "left" : "right",
    committed_result_.obstacle_id, committed_result_.target_d);
}

// 기하 합류 후 글로벌 d=0 경로 전체를 회전해 non-empty handoff 경로로 바꾼다.
// state_machine이 STATE_GLOBAL을 확인할 때까지 commitment 자체는 유지한다.
bool LocalPlannerNode::activateGlobalHandoff(const EgoFrenetState & ego)
{
  auto handoff_path = planner_.buildGlobalHandoffPath(
    ego.s, state_handoff_tail_ratio_, state_handoff_speed_cap_mps_);
  if (handoff_path.wpnts.empty()) {
    return false;
  }
  if (!has_commitment_) {
    committed_result_ = RacelineSplineResult();
    committed_result_.kind = SplinePlanKind::kAvoidance;
    has_commitment_ = true;
    commitment_start_s_ = ego.s;
  }
  committed_result_.path = std::move(handoff_path);
  committed_result_.merge_s = ego.s;
  committed_result_.control_points.clear();
  safe_stop_latched_ = false;
  resetCommitmentViolationConfirmation();
  safe_stop_result_ = RacelineSplineResult();
  safe_stop_release_count_ = 0;
  merge_geometry_confirmed_ = true;
  handoff_active_ = true;
  avoid_state_observed_ = avoid_state_observed_ ||
    (has_state_ && current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID);
  return true;
}

// ============================================================================
// 안전 정지 latch
// ============================================================================
// 활성 회피 중이면 글로벌 d=0으로 즉시 복귀하지 않고 기존 commitment의 남은 기하를 따라
// 정지한다. 그 prefix도 만들 수 없을 때만 현재 d에서 zero-speed hold를 사용한다.
void LocalPlannerNode::latchSafeStop(
  RacelineSplineResult result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  resetCommitmentViolationConfirmation();
  if (has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance) {
    auto committed_stop = planner_.buildCommittedPathStop(
      ego, committed_result_.path, planning_obstacles);
    if (committed_stop.kind == SplinePlanKind::kSafeStop) {
      committed_stop.reason += "; trigger: " + result.reason;
      result = std::move(committed_stop);
    }
  }
  if (result.path.wpnts.empty()) {
    // 빈 /avoid_waypoints는 wpnt_publisher가 global 경로로 fallback할 수 있으므로 금지한다.
    result.kind = SplinePlanKind::kSafeStop;
    result.path = planner_.buildEmergencyStopPath(ego);
    result.reason = "no collision-free stop prefix; publishing a zero-speed emergency hold";
  }
  safe_stop_result_ = std::move(result);
  safe_stop_latched_ = true;
  safe_stop_release_count_ = 0;
  merge_complete_count_ = 0;
  merge_geometry_confirmed_ = false;
  handoff_active_ = false;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Static safe-stop latched: %s", safe_stop_result_.reason.c_str());
}

// latch 진입은 즉시지만 해제는 연속 safe_stop_release_cycles번의 안전한 계획을 요구한다.
// 일시적인 detector 흔들림이 정지/주행 상태를 빠르게 왕복시키는 것을 막는다.
void LocalPlannerNode::handleSafeStopLatch(const EgoFrenetState & ego)
{
  const auto planning_obstacles = has_commitment_ ?
    buildCurrentManeuverInput(ego) :
    buildGuardedObstacles(buildInitialStabilizationInput());
  const std::optional<bool> locked_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  const bool allow_side_switch =
    !locked_side.has_value() || !commitmentSideLocked(ego);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, locked_side, allow_side_switch);

  if (result.kind == SplinePlanKind::kAvoidance) {
    // 같은 조건에서 연속으로 회피 가능해야 새 commitment로 복귀한다.
    ++safe_stop_release_count_;
    if (safe_stop_release_count_ >= safe_stop_release_cycles_) {
      RCLCPP_INFO(
        get_logger(),
        "Safe-stop released after %d consecutive feasible avoidance plans.",
        safe_stop_release_count_);
      commitAvoidance(std::move(result), ego, planning_obstacles);
      publishResult(committed_result_, ego, static_obstacles_);
      return;
    }
  } else if (result.kind == SplinePlanKind::kNoObstacle) {
    // 연속으로 장애물이 없으면 빈 경로 대신 글로벌 handoff를 시작한다.
    ++safe_stop_release_count_;
    if (safe_stop_release_count_ >= safe_stop_release_cycles_) {
      if (activateGlobalHandoff(ego)) {
        RCLCPP_INFO(
          get_logger(),
          "Safe-stop released after %d clear cycles; publishing global handoff.",
          safe_stop_release_count_);
        publishResult(committed_result_, ego, static_obstacles_);
        return;
      }
      safe_stop_release_count_ = 0;
      RCLCPP_ERROR(
        get_logger(), "Failed to build global handoff while releasing safe-stop.");
    }
  } else if (result.kind == SplinePlanKind::kSafeStop) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
  } else {
    safe_stop_release_count_ = 0;
  }

  publishResult(safe_stop_result_, ego, static_obstacles_);
}

// commitment 시작점부터 merge_s까지의 진행량과 실제 |ego.d|를 함께 확인한다.
// 한 번의 위치 노이즈로 합류 처리되지 않도록 연속 merge_confirm_cycles번을 요구한다.
bool LocalPlannerNode::commitmentComplete(const EgoFrenetState & ego)
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    return false;
  }
  const double planned_distance =
    planner_.forwardDistance(commitment_start_s_, committed_result_.merge_s);
  const double driven_distance = planner_.forwardDistance(commitment_start_s_, ego.s);
  const bool reached_tail = driven_distance + 0.20 >= planned_distance &&
    driven_distance < 0.5 * planner_.trackLength();
  if (reached_tail && std::abs(ego.d) <= merge_lateral_tolerance_m_) {
    ++merge_complete_count_;
  } else {
    merge_complete_count_ = 0;
  }
  return merge_complete_count_ >= merge_confirm_cycles_;
}

// 경로를 무효화한 정확한 waypoint와 detector/검증 envelope를 한 줄에 기록한다.
void LocalPlannerNode::logObstacleCollision(
  const std::string & severity,
  const PathValidationFailure & failure,
  int confirmation_count) const
{
  const std::string confirmation = confirmation_count > 0 ?
    " confirmation=" + std::to_string(confirmation_count) + "/" +
    std::to_string(commitment_soft_violation_confirm_cycles_) :
    "";
  RCLCPP_WARN(
    get_logger(),
    "%s%s: obstacle_id=%d waypoint[%zu]=(s=%.3f,d=%.3f) "
    "obstacle_s=[%.3f,%.3f] source_d=[%.3f,%.3f] tested_d=[%.3f,%.3f] "
    "clearance=%.3f",
    severity.c_str(), confirmation.c_str(), failure.obstacle_id,
    failure.waypoint_index, failure.waypoint_s, failure.waypoint_d,
    failure.obstacle_s_start, failure.obstacle_s_end,
    failure.obstacle_source_d_right, failure.obstacle_source_d_left,
    failure.obstacle_test_d_right, failure.obstacle_test_d_left,
    failure.obstacle_clearance);
}

void LocalPlannerNode::onPlanningTimer()
{
  // 별도 callback group이 쓰는 odometry를 짧게 lock해 로컬 스냅샷으로 복사한다.
  nav_msgs::msg::Odometry odometry;
  rclcpp::Time odometry_time(0, 0, RCL_ROS_TIME);
  bool has_odometry = false;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    odometry = latest_odometry_;
    odometry_time = last_odometry_time_;
    has_odometry = has_odometry_;
  }
  if (!has_global_waypoints_ || !has_odometry) {
    publishEmpty("waiting for global race line and Frenet odometry");
    return;
  }

  // 메시지 계약: position.x=s, position.y=d, linear.x=종방향 속도
  EgoFrenetState ego;
  ego.s = odometry.pose.pose.position.x;
  ego.d = odometry.pose.pose.position.y;
  ego.speed = std::abs(odometry.twist.twist.linear.x);

  // 위치가 stale이면 장애물 유무와 무관하게 마지막 위치에서 즉시 정지한다. 이전 commitment는
  // 지우지 않으므로 odometry 회복 후 정상 재검증을 이어갈 수 있다.
  if ((now() - odometry_time).seconds() > odometry_stale_timeout_sec_) {
    RacelineSplineResult emergency_hold;
    emergency_hold.kind = SplinePlanKind::kSafeStop;
    emergency_hold.path = planner_.buildEmergencyStopPath(ego);
    emergency_hold.reason =
      "Frenet odometry is stale; publishing a zero-speed hold at the last known pose";
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s", emergency_hold.reason.c_str());
    publishResult(emergency_hold, ego, static_obstacles_);
    return;
  }
  if (require_obstacles_message_ && !has_obstacles_message_) {
    publishEmpty("waiting for static-obstacle perception");
    return;
  }

  // 장애물 입력 stale은 "트랙이 비었다"는 뜻이 아니다. 마지막 정상 스냅샷과 commitment를
  // 유지하는 degraded mode로 들어가며, 빈 배열을 포함한 다음 정상 메시지만 기억을 교체한다.
  if (has_obstacles_message_ &&
    (now() - last_obstacles_time_).seconds() > obstacle_stale_timeout_sec_)
  {
    if (!obstacle_perception_degraded_) {
      obstacle_perception_degraded_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Static-obstacle perception is stale; retaining the committed path and last valid "
        "obstacle snapshot until fresh perception arrives.");
    }
  }

  std::vector<f110_msgs::msg::Obstacle> planning_obstacles = static_obstacles_;
  bool chained_maneuver_started = false;

  // local planner가 먼저 빈 경로를 보내면 state_machine은 merge 판단에 사용할 tail을 잃는다.
  // 이번 commitment에서 STATE_AVOID를 실제로 관측했고, spline 합류가 확인된 뒤
  // state_machine이 STATE_GLOBAL을 발행한 경우에만 non-empty 경로 발행을 종료한다.
  if (has_commitment_ && merge_geometry_confirmed_) {
    // global handoff 중에도 새 blocking cluster가 나타나면 빈 구간 없이 AVOID 계획으로 복귀한다.
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "global handoff");
    if (!chained_maneuver_started) {
      if (avoid_state_observed_ && has_state_ &&
        current_state_ == f110_msgs::msg::StateMachine::STATE_GLOBAL)
      {
        // AVOID를 실제 거친 뒤 GLOBAL 확인까지 받았으므로 이제 non-empty tail을 해제해도 된다.
        RCLCPP_INFO(
          get_logger(),
          "State machine confirmed GLOBAL after all static obstacles cleared; "
          "releasing committed tail.");
        clearCommitment();
        publishEmpty("state machine confirmed global handoff");
        return;
      }
      publishResult(committed_result_, ego, static_obstacles_);
      return;
    }
  }
  if (safe_stop_latched_) {
    // latch가 활성화된 동안에는 일반 계획 흐름 대신 연속 안전 판정과 정지 경로 재발행을 수행한다.
    handleSafeStopLatch(ego);
    return;
  }
  if (has_commitment_ && !merge_geometry_confirmed_) {
    // 현재 obstacle을 이미 통과했다면 기존 merge까지 기다리지 않고 다음 maneuver 선점을 시도한다.
    std::vector<f110_msgs::msg::Obstacle> early_next_obstacles;
    if (tryEarlyChainedManeuver(ego, early_next_obstacles)) {
      publishResult(committed_result_, ego, static_obstacles_);
      return;
    }
  }
  if (has_commitment_ && !merge_geometry_confirmed_ && commitmentComplete(ego)) {
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "merge completion");
    if (chained_maneuver_started) {
      // 일반 최초 안정화 흐름으로 계속 내려간다. 다음 장애물은 선호 방향 없이 새로 시작하고,
      // non-empty 준비 경로를 발행해 STATE_AVOID는 유지한다.
    } else if (!activateGlobalHandoff(ego)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build the global handoff loop; retaining the validated avoidance tail.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Static avoidance geometry merged; publishing a closed global handoff loop until "
        "STATE_GLOBAL confirmation.");
    }
  }
  if (has_commitment_ && merge_geometry_confirmed_) {
    // GLOBAL 확인 전에는 동일한 handoff 경로를 계속 발행한다.
    publishResult(committed_result_, ego, static_obstacles_);
    return;
  }

  if (!has_commitment_) {
    // 새 회피 시작 전에는 여러 detector 메시지의 보수적 합집합을 Guard로 확장한다.
    auto conservative_obstacles = buildInitialStabilizationInput();
    planning_obstacles = buildGuardedObstacles(conservative_obstacles);
    if (obstacle_perception_degraded_) {
      // perception이 stale이면 새 관측으로 안정화가 나아질 수 없다. 다음 lap에서도 이미 승인한
      // 마지막 스냅샷과 uncertainty Guard를 즉시 재사용한다.
      resetInitialStabilization();
    } else {
      // 안정화 중에는 장애물 앞에서 감속하는 검증된 preparation prefix를 발행한다.
      auto preparation = planner_.buildPreparationStop(ego, planning_obstacles);
      if (preparation.kind == SplinePlanKind::kNoObstacle) {
        const bool published_preparation = initial_prepare_published_;
        resetInitialStabilization();
        if (published_preparation && activateGlobalHandoff(ego)) {
          publishResult(committed_result_, ego, static_obstacles_);
          return;
        }
      } else if (preparation.kind == SplinePlanKind::kNoSafePath) {
        resetInitialStabilization();
        latchSafeStop(std::move(preparation), ego, planning_obstacles);
        publishResult(safe_stop_result_, ego, planning_obstacles);
        return;
      } else {
        const bool stable = updateInitialStabilization(
          preparation.obstacle_ids, conservative_obstacles, now());
        if (!stable) {
          initial_prepare_published_ = true;
          publishResult(preparation, ego, planning_obstacles);
          return;
        }
        // 최종 Guard는 여러 메시지의 보수적 합집합과 최악 위치 분산으로 만든다. 이후 같은 ID의
        // 관측이 이 안에 머무르면 commitment 경로는 움직이지 않는다.
        conservative_obstacles = buildInitialStabilizationInput();
        planning_obstacles = buildGuardedObstacles(conservative_obstacles);
        resetInitialStabilization();
      }
    }
  } else {
    // 팽창된 앞면이 현재 merge 뒤에서 시작하는 장애물은 다음 maneuver 소속이다. controller
    // tail이 merge 뒤까지 이어진다는 이유만으로 현재 commitment를 무효화하면 안 된다.
    planning_obstacles = buildCurrentManeuverInput(ego);
  }

  // ── 동결 commitment 재검증 ────────────────────────────────────────────────
  // 새 spline을 매 주기 만들기 전에 현재 경로의 실제 merge까지 남은 부분만 최신 envelope로
  // 검사한다. 유효하면 기하를 그대로 재발행해 perception jitter가 제어기로 전달되지 않게 한다.
  std::string commitment_error;
  PathValidationFailure commitment_failure;
  if (has_commitment_) {
    const double collision_horizon = remainingDistanceToMerge(ego);
    const bool commitment_valid = planner_.validatePath(
      ego, committed_result_.path, planning_obstacles,
      &commitment_error, &commitment_failure, std::nullopt, collision_horizon);
    if (commitment_valid) {
      if (commitment_soft_violation_count_ > 0) {
        RCLCPP_INFO(
          get_logger(),
          "Soft commitment violation cleared after %d/%d confirmation cycles; "
          "keeping the frozen path.",
          commitment_soft_violation_count_, commitment_soft_violation_confirm_cycles_);
      }
      resetCommitmentViolationConfirmation();
      // 확정 기하가 여전히 안전하다. 여기서 여러 spline 후보를 다시 만들면 perception jitter가
      // 하위 노드에 보이고 동일한 기하/곡률 계산만 반복된다.
      publishResult(committed_result_, ego, static_obstacles_);
      return;
    }

    if (commitment_failure.kind == PathValidationFailureKind::kObstacleCollision) {
      // uncertainty Guard로 실패했으면 detector 원 경계 + 차량 반폭 + hard margin으로 다시
      // 검사한다. 실제 물체 충돌은 즉시 처리하고 Guard 여유만의 충돌은 연속 확인한다.
      const auto hard_collision_obstacles = buildCurrentManeuverInput(ego, false);
      const double hard_clearance =
        planner_parameters_.vehicle_half_width_m + hard_collision_margin_m_;
      PathValidationFailure hard_failure;
      const bool hard_collision_free = planner_.validatePath(
        ego, committed_result_.path, hard_collision_obstacles,
        nullptr, &hard_failure, hard_clearance, collision_horizon);
      const bool hard_collision =
        !hard_collision_free &&
        hard_failure.kind == PathValidationFailureKind::kObstacleCollision;
      if (hard_collision) {
        resetCommitmentViolationConfirmation();
        logObstacleCollision("Hard commitment collision; replanning immediately", hard_failure);
      } else {
        // soft collision은 설정 횟수만큼 연속될 때만 재계획해 일시적인 분산 증가를 흡수한다.
        ++commitment_soft_violation_count_;
        if (commitment_soft_violation_count_ == 1 ||
          commitment_soft_violation_count_ >= commitment_soft_violation_confirm_cycles_)
        {
          logObstacleCollision(
            commitment_soft_violation_count_ >=
            commitment_soft_violation_confirm_cycles_ ?
            "Soft commitment collision confirmed; replanning" :
            "Soft commitment collision pending",
            commitment_failure, commitment_soft_violation_count_);
        }
        if (commitment_soft_violation_count_ <
          commitment_soft_violation_confirm_cycles_)
        {
          publishResult(committed_result_, ego, static_obstacles_);
          return;
        }
        resetCommitmentViolationConfirmation();
      }
    } else {
      resetCommitmentViolationConfirmation();
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Committed path needs replacement: %s", commitment_error.c_str());
  }

  // ── 새 회피 spline 계획 ───────────────────────────────────────────────────
  // commitment 진입 뒤 방향이 잠겼다면 같은 쪽만 검사한다. 잠기기 전에는 기존 쪽이 실패할 때
  // 반대쪽을 평가할 수 있다.
  const std::optional<bool> preferred_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  const bool allow_side_switch =
    !preferred_side.has_value() || !commitmentSideLocked(ego);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, preferred_side, allow_side_switch);

  if (result.kind == SplinePlanKind::kAvoidance) {
    commitAvoidance(std::move(result), ego, planning_obstacles);
    publishResult(committed_result_, ego, planning_obstacles);
    return;
  }

  if (has_commitment_ && result.kind == SplinePlanKind::kNoObstacle) {
    // perception은 spline tail 도달 전에 이미 지난 장애물을 자주 제거한다. 기하 합류가 끝날
    // 때까지 순서가 고정된 기존 commitment를 계속 유지한다.
    publishResult(committed_result_, ego, static_obstacles_);
    return;
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_, ego, planning_obstacles);
    return;
  }
  if (has_commitment_) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_, ego, static_obstacles_);
    return;
  }
  if (result.kind == SplinePlanKind::kNoSafePath && result.obstacle_id >= 0) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_, ego, planning_obstacles);
    return;
  }
  clearCommitment();
  publishEmpty(result.reason);
}

// f110 waypoint 배열을 RViz와 범용 도구가 사용하는 nav_msgs/Path로 변환한다.
nav_msgs::msg::Path LocalPlannerNode::makePath(
  const std::vector<f110_msgs::msg::Wpnt> & waypoints,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(waypoints.size());
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.orientation = yawQuaternion(waypoint.psi_rad);
    path.poses.push_back(pose);
  }
  return path;
}

// ============================================================================
// RViz marker 생성
// ============================================================================
// 매 주기 DELETEALL 뒤 경로, spline 제어점, 유효한 Cartesian 장애물 AABB를 다시 그린다.
visualization_msgs::msg::MarkerArray LocalPlannerNode::makeMarkers(
  const RacelineSplineResult & result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles) const
{
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  Marker clear;
  clear.header.stamp = now();
  clear.header.frame_id = frame_id_;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);

  // 회피 경로는 초록색, 준비/안전 정지 경로는 주황색 LINE_STRIP으로 구분한다.
  Marker path;
  path.header = clear.header;
  path.ns = "raceline_offset_spline";
  path.id = 0;
  path.type = Marker::LINE_STRIP;
  path.action = Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = path_marker_width_m_;
  path.color.a = 1.0F;
  const bool stop_like =
    result.kind == SplinePlanKind::kSafeStop ||
    result.kind == SplinePlanKind::kPreparation;
  path.color.g = stop_like ? 0.55F : 1.0F;
  path.color.r = stop_like ? 1.0F : 0.05F;
  for (const auto & waypoint : result.path.wpnts) {
    geometry_msgs::msg::Point point;
    point.x = waypoint.x_m;
    point.y = waypoint.y_m;
    point.z = 0.03;
    path.points.push_back(point);
  }
  markers.markers.push_back(path);

  // Frenet 제어점을 map 좌표로 변환해 보라색 구로 표시한다.
  Marker control;
  control.header = clear.header;
  control.ns = "spline_control_points";
  control.id = 0;
  control.type = Marker::SPHERE_LIST;
  control.action = Marker::ADD;
  control.pose.orientation.w = 1.0;
  control.scale.x = 0.14;
  control.scale.y = 0.14;
  control.scale.z = 0.14;
  control.color.a = 1.0F;
  control.color.b = 1.0F;
  control.color.r = 0.8F;
  for (const auto & point_sd : result.control_points) {
    geometry_msgs::msg::Point point;
    double yaw = 0.0;
    planner_.toCartesian(ego.s + point_sd.forward_s, point_sd.d, point.x, point.y, yaw);
    point.z = 0.08;
    control.points.push_back(point);
  }
  markers.markers.push_back(control);

  // planner 기하는 Frenet 경계를 사용하지만 marker는 detector가 제공한 선택적 Cartesian AABB다.
  int marker_id = 0;
  for (const auto & obstacle : obstacles) {
    if (!validCartesianAabb(obstacle)) {
      continue;
    }
    Marker marker;
    marker.header = clear.header;
    marker.ns = "static_obstacles";
    marker.id = marker_id++;
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose.position.x = obstacle.x_center;
    marker.pose.position.y = obstacle.y_center;
    marker.scale.x = std::max(0.02, obstacle.x_max - obstacle.x_min);
    marker.scale.y = std::max(0.02, obstacle.y_max - obstacle.y_min);
    marker.scale.z = std::max(0.02, obstacle_marker_scale_m_);
    marker.pose.position.z = 0.5 * marker.scale.z;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 0.85F;
    marker.color.r = 1.0F;
    marker.color.g = 0.1F;
    markers.markers.push_back(marker);
  }
  return markers;
}

// 계획 결과를 제어용 OTWpntArray로 발행하고, 구독자가 있을 때만 디버그 변환 비용을 지불한다.
void LocalPlannerNode::publishResult(
  const RacelineSplineResult & result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.wpnts = result.path.wpnts;

  // state_machine/wpnt_publisher가 동작 단계를 구분할 수 있도록 문자열 계약을 설정한다.
  const bool stop_like =
    result.kind == SplinePlanKind::kSafeStop ||
    result.kind == SplinePlanKind::kPreparation;
  output.ot_side = stop_like ?
    "stop" : (result.go_left ? "left" : "right");
  output.ot_line =
    result.kind == SplinePlanKind::kPreparation ? "raceline_static_prepare" :
    (result.kind == SplinePlanKind::kSafeStop ? "raceline_static_safe_stop" :
    (handoff_active_ ? "raceline_global_handoff" : "raceline_local_d_offset_spline"));

  // 실제 발행 방향이 바뀐 첫 메시지에만 side_switch를 표시하고 마지막 전환 시각을 보존한다.
  const std::optional<bool> current_side = result.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(result.go_left) : std::nullopt;
  output.side_switch = current_side.has_value() &&
    (!last_published_side_.has_value() || current_side.value() != last_published_side_.value());
  if (output.side_switch) {
    last_side_switch_time_ = now();
  }
  output.last_switch_time = last_side_switch_time_;
  last_published_side_ = current_side;
  avoid_waypoints_pub_->publish(output);

  // 제어 경로 발행은 항상 수행하지만 Path/Marker는 구독자가 있을 때만 생성한다.
  const bool publish_local_path = local_path_pub_->get_subscription_count() > 0U;
  const bool publish_compatibility_path =
    compatibility_path_pub_->get_subscription_count() > 0U;
  if (publish_local_path || publish_compatibility_path) {
    const auto path = makePath(result.path.wpnts, output.header);
    if (publish_local_path) {
      local_path_pub_->publish(path);
    }
    if (publish_compatibility_path) {
      compatibility_path_pub_->publish(path);
    }
  }
  if (markers_pub_->get_subscription_count() > 0U) {
    markers_pub_->publish(makeMarkers(result, ego, obstacles));
  }
}

// 아직 입력이 준비되지 않았거나 정상적으로 계획할 것이 없을 때 빈 경로와 이유를 발행한다.
// 단, active commitment/safe-stop/global handoff 중에는 이 함수를 호출하지 않는 것이 핵심이다.
void LocalPlannerNode::publishEmpty(const std::string & reason)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.last_switch_time = last_side_switch_time_;
  output.ot_line = reason;
  avoid_waypoints_pub_->publish(output);

  const bool publish_local_path = local_path_pub_->get_subscription_count() > 0U;
  const bool publish_compatibility_path =
    compatibility_path_pub_->get_subscription_count() > 0U;
  if (publish_local_path || publish_compatibility_path) {
    nav_msgs::msg::Path empty_path;
    empty_path.header = output.header;
    if (publish_local_path) {
      local_path_pub_->publish(empty_path);
    }
    if (publish_compatibility_path) {
      compatibility_path_pub_->publish(empty_path);
    }
  }
  if (markers_pub_->get_subscription_count() > 0U) {
    RacelineSplineResult empty_result;
    markers_pub_->publish(makeMarkers(empty_result, EgoFrenetState(), {}));
  }
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
}

}  // namespace local_planning
