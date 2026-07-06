#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "state_machine/state_machine_node.hpp"

namespace state_machine
{

StateMachineNode::StateMachineNode()
: Node("state_machine_node")
{
  declare_parameter<std::string>("state_topic", "/state");
  //StateMachine 메시지를 publish할 토픽
  declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  //차량의 현재 Frenet 좌표를 받는 토픽입니다. 현재 코드에서는 pose.pose.position.x를 s, pose.pose.position.y를 d로 사용
  declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  //global waypoint가 존재하는지 확인하고, 마지막 waypoint의 s_m 값을 이용해 track length를 추정
  declare_parameter<std::string>("avoid_waypoints_topic", "/avoid_waypoints");
  //정적 장애물 회피용 local path 토픽. STATE_AVOID 전이에 사용함.
  declare_parameter<std::string>("overtake_waypoints_topic", "/overtake_waypoints");
  //동적 장애물/상대 차량 추월용 local path 토픽. STATE_OVERTAKE 전이에 사용함.
  declare_parameter<std::string>("obstacles_topic", "/perception/obstacles");
  //perception 노드가 발행하는 obstacle array를 받는 토픽
  declare_parameter<std::string>("frame_id", "map");
  //발행하는 /state 메시지의 header.frame_id에 들어가는 값
  declare_parameter<std::string>("default_state", "global");
  //global, avoid, overtake 등이 들어갈 수 있습니다.
  //다만 avoid는 fresh한 avoid waypoint, overtake는 fresh한 overtake waypoint가 있어야 유지
  declare_parameter<double>("publish_rate_hz", 10.0);
  //state를 몇 Hz로 publish할지 정함.
  declare_parameter<double>("avoid_stale_timeout_sec", 0.5);
  //avoid_waypoints가 얼마나 오래되면 stale로 볼지 정함.
  declare_parameter<double>("overtake_stale_timeout_sec", 0.5);
  //overtake_waypoints가 얼마나 오래되면 stale로 볼지 정함.
  declare_parameter<double>("global_stale_timeout_sec", 2.0);
  //global_waypoints가 얼마나 오래되면 stale로 볼지 정함.
  declare_parameter<double>("frenet_stale_timeout_sec", 0.5);
  ///car_state/frenet/odom이 얼마나 오래되면 stale로 볼지 정함.
  declare_parameter<double>("obstacles_stale_timeout_sec", 0.5);
  ///perception/obstacles가 얼마나 오래되면 stale로 볼지 정함.
  //obstacle 정보가 stale이어도 /state 발행은 멈추지 않고 warning만 남김.
  declare_parameter<double>("obstacle_lookahead_m", 3.0);
  //ego 차량 기준 전방 몇 m 안의 obstacle을 lane blocking 후보로 볼지 정함.
  declare_parameter<double>("global_blocking_d_threshold_m", 0.4);
  //obstacle의 lateral 위치가 global lane을 막고 있는지 판단하는 threshold임.  abs(obstacle.d_center) <= 0.4이면 global lane 근처 obstacle로 봄.

  // --- Local path evaluation parameters ---
  declare_parameter<bool>("path_eval_enabled", true);
  //true면 fresh 여부에 더해 local path 품질 평가를 통과해야 AVOID/OVERTAKE 전이를 허용.
  declare_parameter<double>("path_min_length_m", 1.5);
  //local path가 최소한 이 길이(s span)를 덮어야 유효한 경로로 봄.
  declare_parameter<double>("path_start_max_gap_m", 1.0);
  //local path 시작점이 ego의 현재 s에서 이 거리 안에 있어야 함(오래된/엉뚱한 경로 거부).
  declare_parameter<double>("path_min_bound_margin_m", 0.05);
  //각 waypoint가 트랙 경계까지 유지해야 하는 최소 마진.
  declare_parameter<double>("path_min_obstacle_gap_m", 0.3);
  //local path가 최신 obstacle과 유지해야 하는 최소 lateral 간격.
  declare_parameter<double>("path_max_kappa_radpm", 3.0);
  //local path의 최대 곡률 한계. 초과하면 주행 불가능한 경로로 보고 거부.

  // --- Transition stability (anti-oscillation) parameters ---
  declare_parameter<double>("min_state_dwell_sec", 1.0);
  //한 state로 전이한 뒤 다른 state로 다시 전이하기까지 요구하는 최소 체류 시간.
  //단, 지탱 path가 stale/invalid가 되어 GLOBAL로 떨어지는 안전 fallback은 이 시간을 무시하고 즉시 수행.
  declare_parameter<int>("transition_confirm_ticks", 3);
  //요청 state가 연속으로 이 tick 수(publish 주기 기준)만큼 유지되어야 실제로 전이함(debounce).

  state_topic_ = get_parameter("state_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  default_state_name_ = get_parameter("default_state").as_string();
  avoid_stale_timeout_sec_ = get_parameter("avoid_stale_timeout_sec").as_double();
  overtake_stale_timeout_sec_ = get_parameter("overtake_stale_timeout_sec").as_double();
  global_stale_timeout_sec_ = get_parameter("global_stale_timeout_sec").as_double();
  frenet_stale_timeout_sec_ = get_parameter("frenet_stale_timeout_sec").as_double();
  obstacles_stale_timeout_sec_ = get_parameter("obstacles_stale_timeout_sec").as_double();
  obstacle_lookahead_m_ = get_parameter("obstacle_lookahead_m").as_double();
  global_blocking_d_threshold_m_ = get_parameter("global_blocking_d_threshold_m").as_double();
  path_eval_enabled_ = get_parameter("path_eval_enabled").as_bool();
  path_min_length_m_ = get_parameter("path_min_length_m").as_double();
  path_start_max_gap_m_ = get_parameter("path_start_max_gap_m").as_double();
  path_min_bound_margin_m_ = get_parameter("path_min_bound_margin_m").as_double();
  path_min_obstacle_gap_m_ = get_parameter("path_min_obstacle_gap_m").as_double();
  path_max_kappa_radpm_ = get_parameter("path_max_kappa_radpm").as_double();
  min_state_dwell_sec_ = get_parameter("min_state_dwell_sec").as_double();
  transition_confirm_ticks_ = static_cast<int>(get_parameter("transition_confirm_ticks").as_int());

  const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  state_pub_ = create_publisher<f110_msgs::msg::StateMachine>(state_topic_, state_qos);

  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

  frenet_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("frenet_odom_topic").as_string(),
    volatile_qos,
    std::bind(&StateMachineNode::on_frenet_odom, this, std::placeholders::_1));

  global_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    get_parameter("global_waypoints_topic").as_string(),
    global_qos,
    std::bind(&StateMachineNode::on_global_waypoints, this, std::placeholders::_1));

  avoid_sub_ = create_subscription<f110_msgs::msg::OTWpntArray>(
    get_parameter("avoid_waypoints_topic").as_string(),
    volatile_qos,
    std::bind(&StateMachineNode::on_avoid_wpnts, this, std::placeholders::_1));

  overtake_sub_ = create_subscription<f110_msgs::msg::OTWpntArray>(
    get_parameter("overtake_waypoints_topic").as_string(),
    volatile_qos,
    std::bind(&StateMachineNode::on_overtake_wpnts, this, std::placeholders::_1));

  obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    get_parameter("obstacles_topic").as_string(),
    volatile_qos,
    std::bind(&StateMachineNode::on_obstacles, this, std::placeholders::_1));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(period, std::bind(&StateMachineNode::publish_state, this));

  RCLCPP_INFO(
    get_logger(),
    "state_machine_node started. Publishing %s with default_state='%s'.",
    state_topic_.c_str(),
    default_state_name_.c_str());
}

std::optional<uint8_t> StateMachineNode::parse_state(const std::string & state_name) const
{
  if (state_name == "global") {
    return f110_msgs::msg::StateMachine::STATE_GLOBAL;
  }
  if (state_name == "avoid") {
    return f110_msgs::msg::StateMachine::STATE_AVOID;
  }
  if (state_name == "overtake") {
    return f110_msgs::msg::StateMachine::STATE_OVERTAKE;
  }
  return std::nullopt;
}

bool StateMachineNode::is_fresh(const rclcpp::Time & stamp, double timeout_sec) const
{
  return (now() - stamp).seconds() <= timeout_sec;
}

bool StateMachineNode::has_fresh_global() const
{
  return has_global_ && is_fresh(last_global_time_, global_stale_timeout_sec_);
}

bool StateMachineNode::has_fresh_frenet() const
{
  return has_frenet_ && is_fresh(last_frenet_time_, frenet_stale_timeout_sec_);
}

bool StateMachineNode::has_fresh_avoid_wpnts() const
{
  return has_avoid_wpnts_ && is_fresh(last_avoid_time_, avoid_stale_timeout_sec_);
}

bool StateMachineNode::has_fresh_overtake_wpnts() const
{
  return has_overtake_wpnts_ && is_fresh(last_overtake_time_, overtake_stale_timeout_sec_);
}

bool StateMachineNode::has_fresh_obstacles() const
{
  return has_obstacles_ && is_fresh(last_obstacles_time_, obstacles_stale_timeout_sec_);
}

///perception/obstacles 수신
// on_obstacles()
// compute_obstacle_evidence(*msg)
//obstacle_evidence_에 저장
//resolve_requested_state()에서 obstacle_evidence_를 보고 state 결정
StateMachineNode::ObstacleEvidence StateMachineNode::compute_obstacle_evidence(
  const f110_msgs::msg::ObstacleArray & msg) const
{
  ObstacleEvidence evidence; //ObstacleEvidence는 구조체 종류, evidence는 구조체 변수
  const f110_msgs::msg::Obstacle * closest_obstacle = nullptr; //가장 가까운 장애물을 가리킬 포인터 변수
  bool closest_is_visible = false; ///closest_obstacle로 선택해둔 장애물이 visible인지 아닌지 기억하는 bool 변수

  for (const auto & obstacle : msg.obstacles) {
    if (obstacle.is_actually_a_gap) {
      continue;
    }

    evidence.has_obstacle = true;
    evidence.has_visible_obstacle = evidence.has_visible_obstacle || obstacle.is_visible;
    if (obstacle.is_static) {
      evidence.has_static_obstacle = true;
    } else {
      evidence.has_dynamic_obstacle = true;
    }

    if (!has_current_frenet_pose_) {
      continue;
    }

    double s_gap = obstacle.s_center - current_s_m_;
    if (has_track_length_ && track_length_m_ > 0.0) {
      // TODO: Replace this wrap model if the track representation changes.
      s_gap = std::fmod(s_gap + track_length_m_, track_length_m_);
      if (s_gap < 0.0) {
        s_gap += track_length_m_;
      }
    } else if (s_gap < 0.0) {
      continue;
    }

    const bool should_replace_closest =
      closest_obstacle == nullptr ||
      (obstacle.is_visible && !closest_is_visible) ||
      (obstacle.is_visible == closest_is_visible && s_gap < evidence.closest_s_gap_m);

    if (should_replace_closest) {
      closest_obstacle = &obstacle;
      closest_is_visible = obstacle.is_visible;
      evidence.closest_s_gap_m = s_gap;
    }

    if (obstacle.is_static && s_gap < evidence.closest_static_s_gap_m) {
      evidence.closest_static_id = obstacle.id;
      evidence.closest_static_s_gap_m = s_gap;
      evidence.closest_static_d_center_m = obstacle.d_center;
    }

    if (!obstacle.is_static && s_gap < evidence.closest_dynamic_s_gap_m) {
      evidence.closest_dynamic_id = obstacle.id;
      evidence.closest_dynamic_s_gap_m = s_gap;
      evidence.closest_dynamic_d_center_m = obstacle.d_center;
      evidence.closest_dynamic_vs_mps = obstacle.vs;
      evidence.closest_dynamic_vd_mps = obstacle.vd;
    }

    if (!obstacle.is_visible) {
      continue;
    }

    const bool blocks_global =
      s_gap <= obstacle_lookahead_m_ &&
      std::abs(obstacle.d_center) <= global_blocking_d_threshold_m_;
    if (blocks_global) {
      if (obstacle.is_static) {
        evidence.static_blocks_global = true;
      } else {
        evidence.dynamic_blocks_global = true;
      }
    }
  }

  if (closest_obstacle != nullptr) {
    evidence.closest_obstacle_id = closest_obstacle->id;
    evidence.closest_d_center_m = closest_obstacle->d_center;
    evidence.closest_size_m = closest_obstacle->size;
    evidence.closest_is_static = closest_obstacle->is_static;
    evidence.closest_is_visible = closest_obstacle->is_visible;
  }

  evidence.global_blocked = evidence.static_blocks_global || evidence.dynamic_blocks_global;
  evidence.simultaneous_static_dynamic_on_global =
    evidence.has_static_obstacle && evidence.has_dynamic_obstacle && evidence.global_blocked;
  evidence.avoidance_needed =
    evidence.static_blocks_global || evidence.simultaneous_static_dynamic_on_global;
  evidence.overtake_candidate =
    evidence.dynamic_blocks_global && !evidence.static_blocks_global &&
    !evidence.simultaneous_static_dynamic_on_global;
  return evidence;
}

//track wrap을 고려해 from_s에서 to_s까지의 전방(s 증가 방향) 거리를 계산.
//track length를 모르면 단순 차이를 반환.
double StateMachineNode::wrapped_s_gap(double from_s, double to_s) const
{
  double gap = to_s - from_s;
  if (has_track_length_ && track_length_m_ > 0.0) {
    gap = std::fmod(gap + track_length_m_, track_length_m_);
    if (gap < 0.0) {
      gap += track_length_m_;
    }
  }
  return gap;
}

//발행된 local path(avoid/overtake waypoint array)를 전이 조건으로 쓰기 위한 품질 평가.
//publish tick마다 최신 ego pose와 obstacle set 기준으로 재평가됨.
StateMachineNode::LocalPathAssessment StateMachineNode::evaluate_local_path(
  const f110_msgs::msg::OTWpntArray & msg) const
{
  LocalPathAssessment result;
  result.stamp = now();
  result.has_path = !msg.wpnts.empty();
  if (!result.has_path) {
    return result;
  }

  const auto & first = msg.wpnts.front();
  const auto & last = msg.wpnts.back();

  // --- Check 1: path length (s span, wrap 고려) ---
  result.path_length_m = wrapped_s_gap(first.s_m, last.s_m);
  result.long_enough = result.path_length_m >= path_min_length_m_;

  // --- Check 2: path가 ego 근처에서 시작하는지 (엉뚱한 구간의 잔여 경로 거부) ---
  if (has_current_frenet_pose_) {
    const double forward_gap = wrapped_s_gap(current_s_m_, first.s_m);
    const double backward_gap = wrapped_s_gap(first.s_m, current_s_m_);
    result.start_s_gap_m = std::min(forward_gap, backward_gap);
    result.starts_near_ego = result.start_s_gap_m <= path_start_max_gap_m_;
  } else {
    //ego pose를 모르면 보수적으로 거부
    result.starts_near_ego = false;
  }

  // --- Check 3: 트랙 경계 마진 ---
  //Wpnt.d_right/d_left는 waypoint에서 좌우 트랙 경계까지의 거리.
  //planner가 경계 거리를 채우지 않는 경우(전부 0)는 "경계 정보 없음"으로 보고 check를 통과시킴.
  bool bounds_known = false;
  for (const auto & wpnt : msg.wpnts) {
    if (wpnt.d_right != 0.0 || wpnt.d_left != 0.0) {
      bounds_known = true;
    }
    const double margin = std::min(wpnt.d_right, wpnt.d_left);
    result.min_bound_margin_m = std::min(result.min_bound_margin_m, margin);
  }
  result.within_track_bounds =
    !bounds_known || result.min_bound_margin_m >= path_min_bound_margin_m_;

  // --- Check 4: 최신 obstacle 대비 충돌 마진 (coarse frenet overlap) ---
  //각 waypoint가 obstacle의 s 구간 안에 들어올 때 obstacle의 [d_right, d_left] 폭까지의
  //lateral 간격을 계산하고, 최솟값이 path_min_obstacle_gap_m 이상이어야 통과.
  //TODO: dynamic obstacle은 vs/vd로 도달 시점의 위치를 예측해서 검사하도록 개선.
  //TODO: ego 차폭/obstacle size를 반영한 inflation 적용.
  for (const auto & obstacle : last_obstacles_.obstacles) {
    if (obstacle.is_actually_a_gap) {
      continue;
    }
    for (const auto & wpnt : msg.wpnts) {
      const double gap_from_start = wrapped_s_gap(obstacle.s_start, wpnt.s_m);
      const double obstacle_span = wrapped_s_gap(obstacle.s_start, obstacle.s_end);
      const bool overlaps_s = gap_from_start <= obstacle_span;
      if (!overlaps_s) {
        continue;
      }
      double lateral_gap = 0.0;
      if (wpnt.d_m < obstacle.d_right) {
        lateral_gap = obstacle.d_right - wpnt.d_m;
      } else if (wpnt.d_m > obstacle.d_left) {
        lateral_gap = wpnt.d_m - obstacle.d_left;
      } //그 사이면 obstacle 폭 안 -> lateral_gap 0
      result.min_obstacle_gap_m = std::min(result.min_obstacle_gap_m, lateral_gap);
    }
  }
  result.collision_free = result.min_obstacle_gap_m >= path_min_obstacle_gap_m_;

  // --- Check 5: 곡률 한계 (주행 가능성) ---
  //TODO: 곡률뿐 아니라 vx_mps 프로파일과 마찰 한계(kappa * v^2)를 함께 검사하도록 개선.
  for (const auto & wpnt : msg.wpnts) {
    result.max_abs_kappa_radpm = std::max(result.max_abs_kappa_radpm, std::abs(wpnt.kappa_radpm));
  }
  result.curvature_ok = result.max_abs_kappa_radpm <= path_max_kappa_radpm_;

  result.valid =
    result.has_path &&
    result.long_enough &&
    result.starts_near_ego &&
    result.within_track_bounds &&
    result.collision_free &&
    result.curvature_ok;

  //TODO: 개별 check 통과 여부를 하드 게이트로만 쓰지 말고,
  //마진/길이/곡률을 정규화해 score를 계산하고 score 기반 히스테리시스
  //(enter는 score_hi 이상, 유지에는 score_lo 이상)로 확장.
  result.score = result.valid ? 1.0 : 0.0;

  return result;
}

//STATE_AVOID 전이 게이트: fresh + (평가 활성화 시) 유효한 avoid path.
bool StateMachineNode::can_enter_avoid() const
{
  if (!has_fresh_avoid_wpnts()) {
    return false;
  }
  return !path_eval_enabled_ || avoid_assessment_.valid;
}

//STATE_OVERTAKE 전이 게이트: fresh + (평가 활성화 시) 유효한 overtake path.
bool StateMachineNode::can_enter_overtake() const
{
  if (!has_fresh_overtake_wpnts()) {
    return false;
  }
  return !path_eval_enabled_ || overtake_assessment_.valid;
}

//현재 확정(committed) state를 지탱하는 path가 아직 유효한지.
//GLOBAL은 별도 path가 필요 없으므로 항상 지탱됨.
bool StateMachineNode::committed_state_supported() const
{
  switch (committed_state_) {
    case f110_msgs::msg::StateMachine::STATE_AVOID:
      return can_enter_avoid();
    case f110_msgs::msg::StateMachine::STATE_OVERTAKE:
      return can_enter_overtake();
    default:
      return true;
  }
}

//state oscillation 방지 계층.
//- 안전 fallback: 확정 state의 지탱 path가 stale/invalid가 되면 즉시 GLOBAL로 복귀 (지연 없음).
//- debounce: 새 요청 state는 연속 transition_confirm_ticks tick 동안 유지되어야 전이.
//- dwell: 마지막 전이 후 min_state_dwell_sec이 지나야 다음 전이 허용.
uint8_t StateMachineNode::apply_transition_stability(uint8_t requested_state)
{
  const rclcpp::Time now_time = now();

  //1) 안전 fallback은 안정화 필터를 우회: 유효하지 않은 path로 AVOID/OVERTAKE를 유지하지 않음.
  if (!committed_state_supported()) {
    if (committed_state_ != f110_msgs::msg::StateMachine::STATE_GLOBAL) {
      RCLCPP_WARN(
        get_logger(),
        "Committed state %u lost its supporting path. Immediate fallback to STATE_GLOBAL.",
        committed_state_);
      committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
      last_transition_time_ = now_time; //fallback 직후 재진입도 dwell을 다시 채워야 함
    }
    candidate_state_ = committed_state_;
    candidate_ticks_ = 0;
    return committed_state_;
  }

  //2) 요청이 현재 확정 state와 같으면 후보 누적을 리셋하고 유지.
  if (requested_state == committed_state_) {
    candidate_state_ = committed_state_;
    candidate_ticks_ = 0;
    return committed_state_;
  }

  //3) debounce: 같은 후보 state가 연속으로 유지될 때만 tick 누적.
  if (requested_state != candidate_state_) {
    candidate_state_ = requested_state;
    candidate_ticks_ = 1;
  } else {
    ++candidate_ticks_;
  }

  //4) dwell + debounce를 모두 만족해야 전이 확정.
  const bool dwell_satisfied =
    (now_time - last_transition_time_).seconds() >= min_state_dwell_sec_;
  const bool confirm_satisfied = candidate_ticks_ >= transition_confirm_ticks_;

  if (dwell_satisfied && confirm_satisfied) {
    RCLCPP_INFO(
      get_logger(),
      "Transition %u -> %u confirmed (ticks=%d, dwell ok).",
      committed_state_,
      candidate_state_,
      candidate_ticks_);
    committed_state_ = candidate_state_;
    last_transition_time_ = now_time;
    candidate_ticks_ = 0;
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Holding state %u; candidate %u pending (ticks=%d/%d, dwell=%s).",
      committed_state_,
      candidate_state_,
      candidate_ticks_,
      transition_confirm_ticks_,
      dwell_satisfied ? "ok" : "waiting");
  }

  return committed_state_;
}

void StateMachineNode::on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  has_frenet_ = true;
  last_frenet_time_ = now();
  has_current_frenet_pose_ = true;
  current_s_m_ = msg->pose.pose.position.x;
  current_d_m_ = msg->pose.pose.position.y;
}

void StateMachineNode::on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  has_global_ = !msg->wpnts.empty();
  last_global_time_ = now();
  has_track_length_ = has_global_ && msg->wpnts.back().s_m > 0.0;
  if (has_track_length_) {
    track_length_m_ = msg->wpnts.back().s_m;
  }
  if (!has_global_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty global waypoints. Conservative state output remains available.");
  }
}

void StateMachineNode::on_avoid_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
{
  has_avoid_wpnts_ = !msg->wpnts.empty();
  last_avoid_time_ = now();
  last_avoid_wpnts_ = *msg; //매 tick 최신 pose/obstacle 기준으로 재평가할 수 있게 원본 보관
  if (!has_avoid_wpnts_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty avoid waypoints. STATE_AVOID transition will be blocked.");
  }
}

void StateMachineNode::on_overtake_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
{
  has_overtake_wpnts_ = !msg->wpnts.empty();
  last_overtake_time_ = now();
  last_overtake_wpnts_ = *msg;
  if (!has_overtake_wpnts_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty overtake waypoints. STATE_OVERTAKE transition will be blocked.");
  }
}

void StateMachineNode::on_obstacles(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  has_obstacles_ = true;
  last_obstacles_time_ = now();
  last_obstacles_ = *msg; //local path 충돌 마진 평가용 원본 보관
  obstacle_evidence_ = compute_obstacle_evidence(*msg);
  obstacle_evidence_.stamp = last_obstacles_time_;

  if (obstacle_evidence_.global_blocked) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Obstacle evidence marks global lane blocked: static=%s dynamic=%s closest_id=%d s_gap=%.2f d=%.2f.",
      obstacle_evidence_.static_blocks_global ? "true" : "false",
      obstacle_evidence_.dynamic_blocks_global ? "true" : "false",
      obstacle_evidence_.closest_obstacle_id,
      obstacle_evidence_.closest_s_gap_m,
      obstacle_evidence_.closest_d_center_m);
  }
}

uint8_t StateMachineNode::resolve_requested_state()
{
  const auto parsed_state = parse_state(default_state_name_);
  uint8_t requested_state = f110_msgs::msg::StateMachine::STATE_GLOBAL;
  if (parsed_state.has_value()) {
    requested_state = parsed_state.value();
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid default_state '%s'. Falling back to STATE_GLOBAL.",
      default_state_name_.c_str());
  }

  if (has_fresh_obstacles()) {
    // Team policy: if static and dynamic obstacles are detected together while
    // the global path is blocked, prioritize STATE_AVOID over STATE_OVERTAKE.
    if (obstacle_evidence_.simultaneous_static_dynamic_on_global) {
      if (can_enter_avoid()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Static and dynamic obstacles detected on global path. Requesting STATE_AVOID.");
        return f110_msgs::msg::StateMachine::STATE_AVOID;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Static+dynamic obstacle evidence requests STATE_AVOID, but avoid path is stale, empty, or failed evaluation. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    if (obstacle_evidence_.static_blocks_global) {
      if (can_enter_avoid()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Static obstacle blocks global path. Requesting STATE_AVOID.");
        return f110_msgs::msg::StateMachine::STATE_AVOID;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Static obstacle blocks global path, but avoid path is stale, empty, or failed evaluation. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    if (obstacle_evidence_.dynamic_blocks_global) {
      if (can_enter_overtake()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Dynamic obstacle blocks global path. Requesting STATE_OVERTAKE.");
        return f110_msgs::msg::StateMachine::STATE_OVERTAKE;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Dynamic obstacle blocks global path, but overtake path is stale, empty, or failed evaluation. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_GLOBAL) {
    return requested_state;
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_AVOID) {
    if (!can_enter_avoid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Requested STATE_AVOID has no fresh/valid avoid path. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
    return requested_state;
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_OVERTAKE) {
    if (!can_enter_overtake()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Requested STATE_OVERTAKE has no fresh/valid overtake path. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
    return requested_state;
  }

  return requested_state;
}

void StateMachineNode::publish_state()
{
  //매 tick 최신 ego pose/obstacle 기준으로 local path를 재평가한 뒤 state를 결정.
  avoid_assessment_ = evaluate_local_path(last_avoid_wpnts_);
  overtake_assessment_ = evaluate_local_path(last_overtake_wpnts_);

  if (path_eval_enabled_) {
    if (has_fresh_avoid_wpnts() && !avoid_assessment_.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Avoid path failed evaluation: len=%s start=%s bounds=%s collision=%s kappa=%s.",
        avoid_assessment_.long_enough ? "ok" : "short",
        avoid_assessment_.starts_near_ego ? "ok" : "far",
        avoid_assessment_.within_track_bounds ? "ok" : "out",
        avoid_assessment_.collision_free ? "ok" : "hit",
        avoid_assessment_.curvature_ok ? "ok" : "high");
    }
    if (has_fresh_overtake_wpnts() && !overtake_assessment_.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Overtake path failed evaluation: len=%s start=%s bounds=%s collision=%s kappa=%s.",
        overtake_assessment_.long_enough ? "ok" : "short",
        overtake_assessment_.starts_near_ego ? "ok" : "far",
        overtake_assessment_.within_track_bounds ? "ok" : "out",
        overtake_assessment_.collision_free ? "ok" : "hit",
        overtake_assessment_.curvature_ok ? "ok" : "high");
    }
  }

  const bool global_ready = has_fresh_global();
  const bool frenet_ready = has_fresh_frenet();
  const bool obstacles_ready = has_fresh_obstacles();
  const bool avoid_ready = has_fresh_avoid_wpnts();
  const bool overtake_ready = has_fresh_overtake_wpnts();
  if (!global_ready || !frenet_ready || !obstacles_ready || !avoid_ready || !overtake_ready) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "State publisher inputs: global=%s frenet=%s obstacles=%s avoid_wpnts=%s overtake_wpnts=%s. Publishing conservative state.",
      global_ready ? "true" : "false",
      frenet_ready ? "true" : "false",
      obstacles_ready ? "true" : "false",
      avoid_ready ? "true" : "false",
      overtake_ready ? "true" : "false");
  }

  //raw 요청 state에 anti-oscillation 필터(dwell + debounce, 안전 fallback은 즉시)를 적용.
  const uint8_t requested_state = resolve_requested_state();
  const uint8_t state = apply_transition_stability(requested_state);
  f110_msgs::msg::StateMachine msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.state = state;
  state_pub_->publish(msg);

  if (!last_published_state_.has_value() || last_published_state_.value() != state) {
    RCLCPP_INFO(get_logger(), "Published state changed to %u.", state);
    last_published_state_ = state;
  }
}

}  // namespace state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
