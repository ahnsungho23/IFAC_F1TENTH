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
      if (has_fresh_avoid_wpnts()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Static and dynamic obstacles detected on global path. Publishing STATE_AVOID.");
        return f110_msgs::msg::StateMachine::STATE_AVOID;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Static+dynamic obstacle evidence requests STATE_AVOID, but avoid waypoints are not fresh. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    if (obstacle_evidence_.static_blocks_global) {
      if (has_fresh_avoid_wpnts()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Static obstacle blocks global path. Publishing STATE_AVOID.");
        return f110_msgs::msg::StateMachine::STATE_AVOID;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Static obstacle blocks global path, but avoid waypoints are not fresh. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    if (obstacle_evidence_.dynamic_blocks_global) {
      if (has_fresh_overtake_wpnts()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Dynamic obstacle blocks global path. Publishing STATE_OVERTAKE.");
        return f110_msgs::msg::StateMachine::STATE_OVERTAKE;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Dynamic obstacle blocks global path, but overtake waypoints are not fresh. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_GLOBAL) {
    return requested_state;
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_AVOID) {
    if (!has_fresh_avoid_wpnts()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Requested STATE_AVOID has no fresh avoid waypoints. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
    return requested_state;
  }

  if (requested_state == f110_msgs::msg::StateMachine::STATE_OVERTAKE) {
    if (!has_fresh_overtake_wpnts()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Requested STATE_OVERTAKE has no fresh overtake waypoints. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
    return requested_state;
  }

  return requested_state;
}

void StateMachineNode::publish_state()
{
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
