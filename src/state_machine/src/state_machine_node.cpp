#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "state_machine/state_machine_node.hpp"

namespace state_machine
{
namespace
{

//닫힌 트랙에서의 순환 s 거리(전방/후방 중 짧은 쪽). NaN 입력 시 NaN 반환 -> 비교에서 안전하게 false.
double circular_s_distance(double a, double b, double track_length)
{
  double diff = std::fmod(std::abs(a - b), track_length);
  return std::min(diff, track_length - diff);
}

//global waypoint 배열로부터 트랙 총 길이 추정(마지막 s_m + 마지막 구간 간격).
double track_length_from(const f110_msgs::msg::WpntArray & global_wpnts)
{
  const auto & wpnts = global_wpnts.wpnts;
  if (wpnts.size() < 2) {
    return 0.0;
  }
  const double last_gap = wpnts.back().s_m - wpnts[wpnts.size() - 2].s_m;
  return wpnts.back().s_m + std::max(0.0, last_gap);
}

}  // namespace

StateMachineNode::StateMachineNode()
: Node("state_machine_node")
{
  declare_parameter<std::string>("state_topic", "/state");
  //StateMachine 메시지를 publish할 토픽
  declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  //차량의 현재 Frenet odom 토픽. 현재는 freshness 모니터링에만 사용.
  declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  //global waypoint 존재/freshness 확인용 토픽.
  declare_parameter<std::string>("avoid_waypoints_topic", "/avoid_waypoints");
  //정적 장애물 회피용 local path 토픽. STATE_AVOID 전이에 사용함.
  declare_parameter<std::string>("overtake_waypoints_topic", "/overtake_waypoints");
  //동적 장애물/상대 차량 추월용 local path 토픽. STATE_OVERTAKE 전이에 사용함.
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
  declare_parameter<double>("global_stale_timeout_sec", 0.5);
  //global_waypoints가 얼마나 오래되면 stale로 볼지 정함.
  declare_parameter<double>("frenet_stale_timeout_sec", 0.5);
  ///car_state/frenet/odom이 얼마나 오래되면 stale로 볼지 정함.

  // --- Global re-entry parameters ---
  declare_parameter<double>("enter_global_sec", 0.5);
  ///////local path 에서 global path로 합류할때 global 위에 ego 차가 존재해야하는 최소시간
  declare_parameter<double>("enter_global_threshold", 0.2);
  ///////s,d domain기준으로 ego 차가 global 위에 존재하는지 판단하기 위한 기준 threshold 단위는 미터
  declare_parameter<double>("enter_global_tail_ratio", 0.1);
  //local path(세그먼트) 후방에서 tail(merge 합류 구간)로 간주하는 비율. 원안의 "뒤 10%".
  declare_parameter<double>("enter_global_s_gap_tol_m", 0.5);
  //ego가 tail 구간에 실제 도달했는지 판정하는 순환 s-gap 상한. 조기 복귀(F3) 방지 게이트.
  state_topic_ = get_parameter("state_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  default_state_name_ = get_parameter("default_state").as_string();
  avoid_stale_timeout_sec_ = get_parameter("avoid_stale_timeout_sec").as_double();
  overtake_stale_timeout_sec_ = get_parameter("overtake_stale_timeout_sec").as_double();
  global_stale_timeout_sec_ = get_parameter("global_stale_timeout_sec").as_double();
  frenet_stale_timeout_sec_ = get_parameter("frenet_stale_timeout_sec").as_double();
  enter_global_sec_ = get_parameter("enter_global_sec").as_double();
  enter_global_threshold_ = get_parameter("enter_global_threshold").as_double();
  enter_global_tail_ratio_ = get_parameter("enter_global_tail_ratio").as_double();
  enter_global_s_gap_tol_m_ = get_parameter("enter_global_s_gap_tol_m").as_double();

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

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(period, std::bind(&StateMachineNode::publish_state, this));

  //초기 FSM 상태를 default_state로 지정(없거나 유효하지 않으면 GLOBAL).
  const auto parsed_default = parse_state(default_state_name_);
  if (!parsed_default.has_value()) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid default_state '%s'. Starting in STATE_GLOBAL.",
      default_state_name_.c_str());
  }
  committed_state_ = parsed_default.value_or(f110_msgs::msg::StateMachine::STATE_GLOBAL);

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

bool StateMachineNode::is_not_null_ptr(
  const rclcpp::Time & stamp,
  double timeout_sec,
  const f110_msgs::msg::OTWpntArray::SharedPtr msg) const
{
  //msg가 비어있으면(포인터 자체가 null이거나 wpnts가 비어있으면) "채워진 상태 유지"가 성립하지 않으므로 false.
  if (msg == nullptr || msg->wpnts.empty()) {
    return false;
  }
  //msg가 채워진 채로 stamp(채워지기 시작한 시점)로부터 timeout_sec보다 오래 지속되었으면 true.
  return (now() - stamp).seconds() > timeout_sec;
}/////////일정시간동안 받아들이는 wpnt가nullptr이 아니라면 true 반환

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

//STATE_AVOID 전이 게이트: fresh한 avoid path가 있어야 함.
bool StateMachineNode::can_enter_avoid(
  const rclcpp::Time & stamp,
  double timeout_sec,
  const f110_msgs::msg::OTWpntArray::SharedPtr msg) const
{
  return is_not_null_ptr(stamp,timeout_sec,msg);
}

//STATE_OVERTAKE 전이 게이트: fresh한 overtake path가 있어야 함. 지금당장은 can_enter_avoid와 can_enter_overtake 구조가 같지만 추후 달라질걸 염두해 따로 둠.
bool StateMachineNode::can_enter_overtake(
  const rclcpp::Time & stamp,
  double timeout_sec,
  const f110_msgs::msg::OTWpntArray::SharedPtr msg) const
{
  return is_not_null_ptr(stamp,timeout_sec,msg);
}

void StateMachineNode::on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  //ego 차량의 현재 frenet pose. enter_to_global() 판단(global 복귀)에 사용한다.
  has_frenet_ = true;
  frenet_odom_msg_ = msg;
  last_frenet_time_ = now();
}

void StateMachineNode::on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  has_global_ = !msg->wpnts.empty();
  global_wpnts_msg_ = msg;
  last_global_time_ = now();
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
  const bool non_empty = (msg != nullptr) && !msg->wpnts.empty();
  //직전 msg가 fresh하게 채워져 있었을 때만 streak를 이어가고, 비어있었거나 오래 끊겼으면 streak를 새로 시작.
  const bool streak_continues = has_avoid_wpnts_ && is_fresh(last_avoid_time_, avoid_stale_timeout_sec_);
  if (non_empty && !streak_continues) {
    avoid_nonempty_since_ = now();
  }
  has_avoid_wpnts_ = non_empty;
  avoid_wpnts_msg_ = msg;
  last_avoid_time_ = now();
  if (!non_empty) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty avoid waypoints. STATE_AVOID transition is not allowed.");
  }
}

void StateMachineNode::on_overtake_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
{
  const bool non_empty = (msg != nullptr) && !msg->wpnts.empty();
  //직전 msg가 fresh하게 채워져 있었을 때만 streak를 이어가고, 비어있었거나 오래 끊겼으면 streak를 새로 시작.
  const bool streak_continues = has_overtake_wpnts_ && is_fresh(last_overtake_time_, overtake_stale_timeout_sec_);
  if (non_empty && !streak_continues) {
    overtake_nonempty_since_ = now();
  }
  has_overtake_wpnts_ = non_empty;
  overtake_wpnts_msg_ = msg;
  last_overtake_time_ = now();
  if (!non_empty) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty overtake waypoints. STATE_OVERTAKE transition is not allowed.");
  }
}

//local path(avoid/overtake)에서 global path로 합류(복귀)해도 되는지 판단한다.
//전제(발행 규약): local path는 ego에서 시작해 merge 지점에서 끝나는 세그먼트이며,
//s_m/d_m은 global raceline 기준 frenet 좌표, tail(후방)은 merge를 향해 d->0으로 수렴한다.
//판정: tail 최근접 wpnt(순환 s) 도달(s-gap 게이트) + tail d와 ego d 일치 + ego가 global 라인 위
//      세 조건을 enter_global_sec_ 동안 연속 만족하면 true.
bool StateMachineNode::enter_to_global(
  const nav_msgs::msg::Odometry::SharedPtr frenet_odom,
  const f110_msgs::msg::OTWpntArray::SharedPtr local_wpnts,
  const f110_msgs::msg::WpntArray::SharedPtr global_wpnts)
{
  //필수 입력이 없거나 트랙 길이를 정할 수 없으면 합류 판단 불가 -> 타이머 리셋 후 차단.
  if (frenet_odom == nullptr || local_wpnts == nullptr || global_wpnts == nullptr ||
    local_wpnts->wpnts.empty())
  {
    enter_global_ok_since_.reset();
    return false;
  }
  const double track_length = track_length_from(*global_wpnts);
  if (!(track_length > 0.0)) {
    enter_global_ok_since_.reset();
    return false;
  }

  //frenet odom 규약: position.x = s, position.y = d (global raceline 기준).
  //NaN(publish_nan 정책)이 들어와도 아래 비교가 전부 false -> 안전하게 차단됨.
  const double ego_s = frenet_odom->pose.pose.position.x;
  const double ego_d = frenet_odom->pose.pose.position.y;

  //1) tail = 인덱스 기준 후방 tail_ratio 구간(merge 합류 구간).
  const std::size_t total = local_wpnts->wpnts.size();
  const std::size_t tail_count = std::max<std::size_t>(
    1,
    static_cast<std::size_t>(
      std::ceil(enter_global_tail_ratio_ * static_cast<double>(total))));
  const std::size_t tail_begin = total - std::min(tail_count, total);

  //2) tail 내에서 순환 s 기준 ego 최근접 wpnt 탐색. (F1: 랩 경계에서도 정상 동작)
  double best_gap = std::numeric_limits<double>::infinity();
  double best_d = 0.0;
  for (std::size_t i = tail_begin; i < total; ++i) {
    const auto & wpnt = local_wpnts->wpnts[i];
    const double gap = circular_s_distance(ego_s, wpnt.s_m, track_length);
    if (gap < best_gap) {
      best_gap = gap;
      best_d = wpnt.d_m;
    }
  }

  //3) s-gap 게이트: ego가 실제로 tail 구간에 도달했을 때만 판정. (F2/F3: 조기 복귀 방지)
  const bool reached_tail = best_gap <= enter_global_s_gap_tol_m_;
  //4) local tail의 d와 ego d 일치 (원안 조건 1).
  const bool matches_local_tail = std::abs(best_d - ego_d) <= enter_global_threshold_;
  //5) ego가 global 라인 위 (원안 조건 2; global d_m이 0이므로 |ego_d| 비교와 동치).
  const bool on_global_line = std::abs(ego_d) <= enter_global_threshold_;

  if (!(reached_tail && matches_local_tail && on_global_line)) {
    enter_global_ok_since_.reset();
    return false;
  }

  //6) 조건을 enter_global_sec_ 동안 연속 만족해야 확정. (F4: 지속시간 타이머)
  const rclcpp::Time now_time = now();
  if (!enter_global_ok_since_.has_value()) {
    enter_global_ok_since_ = now_time;
  }
  return (now_time - enter_global_ok_since_.value()).seconds() >= enter_global_sec_;
}

//enter_to_global 판정 진입점.
//평가 대상 state(avoid/overtake)가 바뀌면 타이머를 리셋해 이전 기동의 만족 시간이
//새 판정으로 넘어가지 않게 하고, 입력 freshness를 만족할 때만 본 판정에 위임한다.
bool StateMachineNode::evaluate_enter_to_global(
  uint8_t eval_state,
  bool local_fresh,
  const f110_msgs::msg::OTWpntArray::SharedPtr & local_wpnts)
{
  if (enter_global_eval_state_ != eval_state) {
    enter_global_eval_state_ = eval_state;
    enter_global_ok_since_.reset();
  }
  if (!local_fresh || !has_fresh_frenet() || !has_fresh_global()) {
    enter_global_ok_since_.reset();
    return false;
  }
  return enter_to_global(frenet_odom_msg_, local_wpnts, global_wpnts_msg_);
}

//committed_state_ 기반 FSM 1-step. dwell/안전 fallback 없이 조건 만족 즉시 전이한다.
//- GLOBAL: 지속된 avoid path면 AVOID(우선), 아니면 지속된 overtake path면 OVERTAKE로 진입.
//- AVOID/OVERTAKE: enter_to_global 합류 조건을 만족하면 GLOBAL로 복귀.
uint8_t StateMachineNode::resolve_requested_state()
{
  switch (committed_state_) {
    case f110_msgs::msg::StateMachine::STATE_GLOBAL:
      //진입 우선순위: AVOID > OVERTAKE.
      if (can_enter_avoid(avoid_nonempty_since_, avoid_stale_timeout_sec_, avoid_wpnts_msg_)) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_AVOID;
        enter_global_ok_since_.reset();  //새 기동 진입: 이전 합류 타이머 잔재 제거.
        RCLCPP_INFO(get_logger(), "STATE_GLOBAL -> STATE_AVOID (avoid path sustained).");
      } else if (can_enter_overtake(
          overtake_nonempty_since_, overtake_stale_timeout_sec_, overtake_wpnts_msg_)) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_OVERTAKE;
        enter_global_ok_since_.reset();
        RCLCPP_INFO(get_logger(), "STATE_GLOBAL -> STATE_OVERTAKE (overtake path sustained).");
      }
      break;

    case f110_msgs::msg::StateMachine::STATE_AVOID:
      //avoid 단계: ego(frenet) + 정적 회피경로(세그먼트) + global 경로로 global 합류 여부 판단.
      if (evaluate_enter_to_global(
          f110_msgs::msg::StateMachine::STATE_AVOID,
          has_fresh_avoid_wpnts(),
          avoid_wpnts_msg_))
      {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
        RCLCPP_INFO(get_logger(), "STATE_AVOID -> STATE_GLOBAL (merged back to global line).");
      }
      break;

    case f110_msgs::msg::StateMachine::STATE_OVERTAKE:
      //overtake 단계: ego(frenet) + 동적 회피경로(세그먼트) + global 경로로 global 합류 여부 판단.
      if (evaluate_enter_to_global(
          f110_msgs::msg::StateMachine::STATE_OVERTAKE,
          has_fresh_overtake_wpnts(),
          overtake_wpnts_msg_))
      {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
        RCLCPP_INFO(get_logger(), "STATE_OVERTAKE -> STATE_GLOBAL (merged back to global line).");
      }
      break;

    default:
      committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
      break;
  }

  return committed_state_;
}

void StateMachineNode::publish_state()
{

  const bool global_ready = has_fresh_global();
  const bool frenet_ready = has_fresh_frenet();
  const bool avoid_ready = has_fresh_avoid_wpnts();
  const bool overtake_ready = has_fresh_overtake_wpnts();
  if (!global_ready || !frenet_ready || !avoid_ready || !overtake_ready) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "State publisher inputs: global=%s frenet=%s avoid_wpnts=%s overtake_wpnts=%s. Publishing conservative state.",
      global_ready ? "true" : "false",
      frenet_ready ? "true" : "false",
      avoid_ready ? "true" : "false",
      overtake_ready ? "true" : "false");
  }

  //committed_state_ 기반 FSM 1-step으로 다음 상태 결정 후 발행.
  const uint8_t state = resolve_requested_state();
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
