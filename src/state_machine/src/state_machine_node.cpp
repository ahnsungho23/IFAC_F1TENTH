#include "state_machine/state_machine_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace state_machine
{
namespace
{

double circular_s_distance(double a, double b, double track_length)
{
  const double diff = std::fmod(std::abs(a - b), track_length);
  return std::min(diff, track_length - diff);
}

double forward_s_distance(double from, double to, double track_length)
{
  return std::fmod(to - from + track_length, track_length);
}

double track_length_from(const f110_msgs::msg::WpntArray & global_wpnts)
{
  const auto & wpnts = global_wpnts.wpnts;
  if (wpnts.size() < 2U) {
    return 0.0;
  }
  const double last_gap = wpnts.back().s_m - wpnts[wpnts.size() - 2U].s_m;
  return wpnts.back().s_m + std::max(0.0, last_gap);
}

}  // namespace

StateMachineNode::StateMachineNode()
: Node("state_machine_node")
{
  declare_parameter<std::string>("state_topic", "/state");
  declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
  declare_parameter<std::string>("local_path_topic", "/local_waypoints/path");
  declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  declare_parameter<std::string>("avoid_waypoints_topic", "/avoid_waypoints");
  declare_parameter<std::string>("opponent_topic", "/opp_obs");
  declare_parameter<std::string>("static_obstacles_topic", "/static_obs");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<std::string>("default_state", "global");
  declare_parameter<std::string>("invalid_local_path_policy", "global_fallback");

  declare_parameter<double>("publish_rate_hz", 100.0);
  declare_parameter<int>("waypoint_num", 50);
  declare_parameter<double>("global_publisher_warn_timeout_sec", 5.0);
  declare_parameter<double>("frenet_stale_timeout_sec", 0.5);
  declare_parameter<double>("opponent_stale_timeout_sec", 0.3);

  declare_parameter<bool>("allow_avoid_transition", true);
  declare_parameter<bool>("allow_cruise_transition", true);
  declare_parameter<int64_t>("local_path_confirmation_window_size", 5);
  declare_parameter<int64_t>("local_path_confirmation_min_hits", 3);

  declare_parameter<double>("enter_global_sec", 0.5);
  declare_parameter<double>("enter_global_threshold", 0.2);
  declare_parameter<double>("enter_global_tail_ratio", 0.1);
  declare_parameter<double>("enter_global_s_gap_tol_m", 0.5);
  declare_parameter<double>("avoid_path_stale_timeout_sec", 0.5);
  declare_parameter<double>("avoid_path_exhaustion_sec", 0.5);
  declare_parameter<double>("avoid_path_exhaustion_s_gap_tol_m", 0.75);
  declare_parameter<double>("static_obstacles_stale_timeout_sec", 0.3);
  declare_parameter<double>("stopped_path_clear_sec", 1.0);
  declare_parameter<double>("stopped_path_speed_threshold_mps", 0.01);
  declare_parameter<double>("stopped_path_obstacle_lookahead_m", 3.0);
  declare_parameter<double>("stopped_path_ego_half_width_m", 0.16);

  state_topic_ = get_parameter("state_topic").as_string();
  local_waypoints_topic_ = get_parameter("local_waypoints_topic").as_string();
  local_path_topic_ = get_parameter("local_path_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  default_state_name_ = get_parameter("default_state").as_string();
  invalid_local_path_policy_ = get_parameter("invalid_local_path_policy").as_string();
  static_obstacles_topic_ = get_parameter("static_obstacles_topic").as_string();

  allow_avoid_transition_ = get_parameter("allow_avoid_transition").as_bool();
  allow_cruise_transition_ = get_parameter("allow_cruise_transition").as_bool();
  local_path_confirmation_window_size_ = std::max<int64_t>(
    1, get_parameter("local_path_confirmation_window_size").as_int());
  local_path_confirmation_min_hits_ = std::clamp<int64_t>(
    get_parameter("local_path_confirmation_min_hits").as_int(),
    1,
    local_path_confirmation_window_size_);

  const int64_t waypoint_num = get_parameter("waypoint_num").as_int();
  const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
  global_publisher_warn_timeout_sec_ =
    get_parameter("global_publisher_warn_timeout_sec").as_double();
  frenet_stale_timeout_sec_ = get_parameter("frenet_stale_timeout_sec").as_double();
  opponent_stale_timeout_sec_ = get_parameter("opponent_stale_timeout_sec").as_double();
  enter_global_sec_ = get_parameter("enter_global_sec").as_double();
  enter_global_threshold_ = get_parameter("enter_global_threshold").as_double();
  enter_global_tail_ratio_ = get_parameter("enter_global_tail_ratio").as_double();
  enter_global_s_gap_tol_m_ = get_parameter("enter_global_s_gap_tol_m").as_double();
  avoid_path_stale_timeout_sec_ =
    get_parameter("avoid_path_stale_timeout_sec").as_double();
  avoid_path_exhaustion_sec_ = get_parameter("avoid_path_exhaustion_sec").as_double();
  avoid_path_exhaustion_s_gap_tol_m_ =
    get_parameter("avoid_path_exhaustion_s_gap_tol_m").as_double();
  static_obstacles_stale_timeout_sec_ =
    get_parameter("static_obstacles_stale_timeout_sec").as_double();
  stopped_path_clear_sec_ = get_parameter("stopped_path_clear_sec").as_double();
  stopped_path_speed_threshold_mps_ =
    get_parameter("stopped_path_speed_threshold_mps").as_double();
  stopped_path_obstacle_lookahead_m_ =
    get_parameter("stopped_path_obstacle_lookahead_m").as_double();
  stopped_path_ego_half_width_m_ =
    get_parameter("stopped_path_ego_half_width_m").as_double();

  if (waypoint_num <= 0 || waypoint_num > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("waypoint_num must be in the range [1, INT_MAX]");
  }
  waypoint_num_ = static_cast<int>(waypoint_num);
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be finite and positive");
  }
  if (!std::isfinite(global_publisher_warn_timeout_sec_) ||
    global_publisher_warn_timeout_sec_ <= 0.0)
  {
    throw std::invalid_argument(
            "global_publisher_warn_timeout_sec must be finite and positive");
  }
  if (!std::isfinite(frenet_stale_timeout_sec_) || frenet_stale_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("frenet_stale_timeout_sec must be finite and positive");
  }
  if (!std::isfinite(opponent_stale_timeout_sec_) || opponent_stale_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("opponent_stale_timeout_sec must be finite and positive");
  }
  if (!std::isfinite(enter_global_sec_) || enter_global_sec_ < 0.0 ||
    !std::isfinite(enter_global_threshold_) || enter_global_threshold_ < 0.0 ||
    !std::isfinite(enter_global_tail_ratio_) || enter_global_tail_ratio_ <= 0.0 ||
    enter_global_tail_ratio_ > 1.0 || !std::isfinite(enter_global_s_gap_tol_m_) ||
    enter_global_s_gap_tol_m_ < 0.0)
  {
    throw std::invalid_argument("invalid global re-entry parameter value");
  }
  if (!std::isfinite(avoid_path_stale_timeout_sec_) ||
    avoid_path_stale_timeout_sec_ <= 0.0 ||
    !std::isfinite(avoid_path_exhaustion_sec_) || avoid_path_exhaustion_sec_ < 0.0 ||
    !std::isfinite(avoid_path_exhaustion_s_gap_tol_m_) ||
    avoid_path_exhaustion_s_gap_tol_m_ < 0.0)
  {
    throw std::invalid_argument("invalid avoid path exhaustion parameter value");
  }
  if (!std::isfinite(static_obstacles_stale_timeout_sec_) ||
    static_obstacles_stale_timeout_sec_ <= 0.0 ||
    !std::isfinite(stopped_path_clear_sec_) || stopped_path_clear_sec_ < 0.0 ||
    !std::isfinite(stopped_path_speed_threshold_mps_) ||
    stopped_path_speed_threshold_mps_ < 0.0 ||
    !std::isfinite(stopped_path_obstacle_lookahead_m_) ||
    stopped_path_obstacle_lookahead_m_ <= 0.0 ||
    !std::isfinite(stopped_path_ego_half_width_m_) ||
    stopped_path_ego_half_width_m_ < 0.0)
  {
    throw std::invalid_argument("invalid stopped path clear parameter value");
  }
  if (invalid_local_path_policy_ != "global_fallback") {
    throw std::invalid_argument(
            "invalid_local_path_policy currently supports only 'global_fallback'");
  }

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

  state_pub_ = create_publisher<f110_msgs::msg::StateMachine>(state_topic_, state_qos);
  local_waypoints_pub_ =
    create_publisher<f110_msgs::msg::WpntArray>(local_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, volatile_qos);

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
  opponent_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    get_parameter("opponent_topic").as_string(),
    volatile_qos,
    std::bind(&StateMachineNode::on_opponent, this, std::placeholders::_1));
  static_obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    static_obstacles_topic_,
    volatile_qos,
    std::bind(&StateMachineNode::on_static_obstacles, this, std::placeholders::_1));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  timer_ = create_wall_timer(period, std::bind(&StateMachineNode::publish_state_cycle, this));

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
    "Integrated state_machine_node started: state='%s', local_waypoints='%s', "
    "waypoint_num=%d, default_state='%s'.",
    state_topic_.c_str(),
    local_waypoints_topic_.c_str(),
    waypoint_num_,
    default_state_name_.c_str());
  if (!allow_avoid_transition_ && !allow_cruise_transition_) {
    RCLCPP_WARN(
      get_logger(),
      "Both AVOID and CRUISE transitions are disabled. The FSM will stay in its default state.");
  }
}

std::optional<uint8_t> StateMachineNode::parse_state(const std::string & state_name) const
{
  if (state_name == "global") {
    return f110_msgs::msg::StateMachine::STATE_GLOBAL;
  }
  if (state_name == "avoid") {
    return f110_msgs::msg::StateMachine::STATE_AVOID;
  }
  if (state_name == "cruise") {
    return f110_msgs::msg::StateMachine::STATE_CRUISE;
  }
  return std::nullopt;
}

std::optional<int> StateMachineNode::parse_waypoint_index(const std::string & value) const
{
  if (value.empty()) {
    return std::nullopt;
  }

  std::size_t begin = 0U;
  std::size_t end = value.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1U]))) {
    --end;
  }
  if (begin == end) {
    return std::nullopt;
  }

  const std::string trimmed = value.substr(begin, end - begin);
  for (const char character : trimmed) {
    if (!std::isdigit(static_cast<unsigned char>(character))) {
      return std::nullopt;
    }
  }

  try {
    return std::stoi(trimmed);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool StateMachineNode::is_fresh(const rclcpp::Time & stamp, double timeout_sec) const
{
  return (now() - stamp).seconds() <= timeout_sec;
}

bool StateMachineNode::local_path_confirmed(
  const std::deque<bool> & history,
  const f110_msgs::msg::OTWpntArray::SharedPtr msg) const
{
  if (msg == nullptr || msg->wpnts.empty()) {
    return false;
  }
  return std::count(history.begin(), history.end(), true) >=
         local_path_confirmation_min_hits_;
}

bool StateMachineNode::has_valid_global() const
{
  return has_global_ && global_wpnts_msg_ != nullptr && global_wpnts_msg_->wpnts.size() >= 2U;
}

bool StateMachineNode::has_fresh_frenet() const
{
  return has_frenet_ && frenet_odom_msg_ != nullptr &&
         is_fresh(last_frenet_time_, frenet_stale_timeout_sec_);
}

bool StateMachineNode::has_avoid_wpnts() const
{
  return has_avoid_wpnts_ && avoid_wpnts_msg_ != nullptr && !avoid_wpnts_msg_->wpnts.empty();
}

bool StateMachineNode::has_interfering_opponent() const
{
  return allow_cruise_transition_ && opponent_seen_ && opponent_interfering_ &&
         is_fresh(last_opponent_time_, opponent_stale_timeout_sec_);
}

bool StateMachineNode::validate_global_waypoints(
  const f110_msgs::msg::WpntArray & message,
  std::string * error) const
{
  if (message.wpnts.size() < 2U) {
    *error = "global path requires at least two waypoints";
    return false;
  }

  for (std::size_t index = 0U; index < message.wpnts.size(); ++index) {
    const auto & waypoint = message.wpnts[index];
    if (!std::isfinite(waypoint.s_m) || !std::isfinite(waypoint.x_m) ||
      !std::isfinite(waypoint.y_m))
    {
      *error = "global path contains non-finite s_m/x_m/y_m values";
      return false;
    }
    if (index > 0U && waypoint.s_m <= message.wpnts[index - 1U].s_m) {
      *error = "global s_m must be strictly increasing";
      return false;
    }
  }
  return true;
}

bool StateMachineNode::can_enter_avoid() const
{
  if (!allow_avoid_transition_ || stopped_path_clear_latched_) {
    return false;
  }
  return local_path_confirmed(avoid_path_history_, avoid_wpnts_msg_);
}

void StateMachineNode::on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  has_frenet_ = true;
  frenet_odom_msg_ = msg;
  last_frenet_time_ = now();
  publish_selected_waypoints(committed_state_);
}

void StateMachineNode::on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  std::string error = "global waypoint message is null";
  if (msg == nullptr || !validate_global_waypoints(*msg, &error)) {
    has_global_ = false;
    global_wpnts_msg_.reset();
    RCLCPP_ERROR(get_logger(), "Rejected global waypoints: %s.", error.c_str());
    return;
  }

  global_wpnts_msg_ = msg;
  has_global_ = true;
  last_global_receive_time_ = now();
}

void StateMachineNode::on_avoid_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
{
  const bool non_empty = msg != nullptr && !msg->wpnts.empty();
  last_avoid_receive_time_ = now();
  avoid_path_history_.push_back(non_empty);
  while (static_cast<int64_t>(avoid_path_history_.size()) >
    local_path_confirmation_window_size_)
  {
    avoid_path_history_.pop_front();
  }

  has_avoid_wpnts_ = non_empty;
  avoid_wpnts_msg_ = msg;
  if (non_empty) {
    last_non_empty_avoid_wpnts_msg_ = msg;
    if (msg->wpnts.back().vx_mps > stopped_path_speed_threshold_mps_) {
      stopped_path_clear_latched_ = false;
    }
  }
  if (!non_empty) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received empty avoid waypoints; STATE_AVOID will use the configured fallback.");
  }
}

void StateMachineNode::on_opponent(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  opponent_seen_ = true;
  opponent_interfering_ = std::any_of(
    msg->obstacles.begin(), msg->obstacles.end(),
    [](const auto & obstacle) {return !obstacle.is_static && obstacle.is_interfering;});
  last_opponent_time_ = now();
}

void StateMachineNode::on_static_obstacles(
  const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  static_obstacles_msg_ = msg;
  last_static_obstacles_time_ = now();
}

bool StateMachineNode::has_front_static_obstacle() const
{
  if (static_obstacles_msg_ == nullptr ||
    !is_fresh(last_static_obstacles_time_, static_obstacles_stale_timeout_sec_) ||
    !has_fresh_frenet() || !has_valid_global())
  {
    // A stale detector stream is not evidence that the corridor is clear.
    return true;
  }

  const double track_length = track_length_from(*global_wpnts_msg_);
  if (!(track_length > 0.0)) {
    return true;
  }
  const double ego_s = frenet_odom_msg_->pose.pose.position.x;
  const double ego_d = frenet_odom_msg_->pose.pose.position.y;
  const double ego_right = ego_d - stopped_path_ego_half_width_m_;
  const double ego_left = ego_d + stopped_path_ego_half_width_m_;

  return std::any_of(
    static_obstacles_msg_->obstacles.begin(), static_obstacles_msg_->obstacles.end(),
    [&, track_length](const auto & obstacle) {
      if (obstacle.is_actually_a_gap || !std::isfinite(obstacle.s_center) ||
        !std::isfinite(obstacle.d_right) || !std::isfinite(obstacle.d_left))
      {
        return false;
      }
      const double forward = forward_s_distance(ego_s, obstacle.s_center, track_length);
      const double obstacle_right = std::min(obstacle.d_right, obstacle.d_left);
      const double obstacle_left = std::max(obstacle.d_right, obstacle.d_left);
      return forward <= stopped_path_obstacle_lookahead_m_ &&
             obstacle_left >= ego_right && obstacle_right <= ego_left;
    });
}

bool StateMachineNode::enter_to_global(
  const nav_msgs::msg::Odometry::SharedPtr frenet_odom,
  const f110_msgs::msg::OTWpntArray::SharedPtr local_wpnts,
  const f110_msgs::msg::WpntArray::SharedPtr global_wpnts)
{
  if (frenet_odom == nullptr || local_wpnts == nullptr || global_wpnts == nullptr ||
    local_wpnts->wpnts.empty())
  {
    enter_global_ok_since_.reset();
    return false;
  }
  // 이게 없으면 장애물 앞 홀드(꼬리=자차, |d|<threshold)가 아래 세 조건을 정지해 있다는
  // 이유만으로 전부 만족해 0.7초 주기 AVOID↔GLOBAL 요동이 생기고, GLOBAL 틱마다
  // 장애물 관통 글로벌 라인+속도 명령이 잠깐 전달되어 차가 장애물 쪽으로 기어간다
  // (2026-08-13 실차 run_0813_220641/231123: 홀드 중 /drive 0.35~2.0 펄스, 0.3 m 전진).
  if (local_wpnts->wpnts.size() < 3U || local_wpnts->wpnts.back().vx_mps <= 0.0) {
    enter_global_ok_since_.reset();
    return false;
  }
  const double track_length = track_length_from(*global_wpnts);
  if (!(track_length > 0.0)) {
    enter_global_ok_since_.reset();
    return false;
  }

  const double ego_s = frenet_odom->pose.pose.position.x;
  const double ego_d = frenet_odom->pose.pose.position.y;
  const std::size_t total = local_wpnts->wpnts.size();
  const std::size_t tail_count = std::max<std::size_t>(
    1U,
    static_cast<std::size_t>(
      std::ceil(enter_global_tail_ratio_ * static_cast<double>(total))));
  const std::size_t tail_begin = total - std::min(tail_count, total);

  double best_gap = std::numeric_limits<double>::infinity();
  double best_d = 0.0;
  for (std::size_t index = tail_begin; index < total; ++index) {
    const auto & waypoint = local_wpnts->wpnts[index];
    const double gap = circular_s_distance(ego_s, waypoint.s_m, track_length);
    if (gap < best_gap) {
      best_gap = gap;
      best_d = waypoint.d_m;
    }
  }

  const bool reached_tail = best_gap <= enter_global_s_gap_tol_m_;
  const bool matches_local_tail = std::abs(best_d - ego_d) <= enter_global_threshold_;
  const bool on_global_line = std::abs(ego_d) <= enter_global_threshold_;
  if (!(reached_tail && matches_local_tail && on_global_line)) {
    enter_global_ok_since_.reset();
    return false;
  }

  const rclcpp::Time current_time = now();
  if (!enter_global_ok_since_.has_value()) {
    enter_global_ok_since_ = current_time;
  }
  return (current_time - enter_global_ok_since_.value()).seconds() >= enter_global_sec_;
}

bool StateMachineNode::evaluate_enter_to_global(
  uint8_t eval_state,
  bool local_available,
  const f110_msgs::msg::OTWpntArray::SharedPtr & local_wpnts)
{
  if (enter_global_eval_state_ != eval_state) {
    enter_global_eval_state_ = eval_state;
    enter_global_ok_since_.reset();
  }
  if (!local_available || !has_fresh_frenet() || !has_valid_global()) {
    enter_global_ok_since_.reset();
    return false;
  }
  return enter_to_global(frenet_odom_msg_, local_wpnts, global_wpnts_msg_);
}

bool StateMachineNode::evaluate_avoid_path_exhausted()
{
  if (!has_fresh_frenet() || !has_valid_global() ||
    last_non_empty_avoid_wpnts_msg_ == nullptr ||
    last_non_empty_avoid_wpnts_msg_->wpnts.empty())
  {
    avoid_path_exhausted_since_.reset();
    return false;
  }

  const bool empty_path = !has_avoid_wpnts();
  const bool stale_path = !is_fresh(last_avoid_receive_time_, avoid_path_stale_timeout_sec_);
  if (!(empty_path || stale_path)) {
    avoid_path_exhausted_since_.reset();
    return false;
  }

  const double track_length = track_length_from(*global_wpnts_msg_);
  if (!(track_length > 0.0)) {
    avoid_path_exhausted_since_.reset();
    return false;
  }

  const double ego_s = frenet_odom_msg_->pose.pose.position.x;
  const auto & wpnts = last_non_empty_avoid_wpnts_msg_->wpnts;
  const std::size_t tail_count = std::max<std::size_t>(
    1U,
    static_cast<std::size_t>(
      std::ceil(enter_global_tail_ratio_ * static_cast<double>(wpnts.size()))));
  const std::size_t tail_begin = wpnts.size() - std::min(tail_count, wpnts.size());

  double best_tail_gap = std::numeric_limits<double>::infinity();
  for (std::size_t index = tail_begin; index < wpnts.size(); ++index) {
    best_tail_gap = std::min(
      best_tail_gap,
      circular_s_distance(ego_s, wpnts[index].s_m, track_length));
  }
  if (best_tail_gap > avoid_path_exhaustion_s_gap_tol_m_) {
    avoid_path_exhausted_since_.reset();
    return false;
  }

  const rclcpp::Time current_time = now();
  if (!avoid_path_exhausted_since_.has_value()) {
    avoid_path_exhausted_since_ = current_time;
  }
  return (current_time - avoid_path_exhausted_since_.value()).seconds() >=
         avoid_path_exhaustion_sec_;
}

bool StateMachineNode::evaluate_stopped_path_clear()
{
  if (!has_avoid_wpnts() || avoid_wpnts_msg_ == nullptr || avoid_wpnts_msg_->wpnts.empty() ||
    avoid_wpnts_msg_->wpnts.back().vx_mps > stopped_path_speed_threshold_mps_ ||
    has_front_static_obstacle())
  {
    stopped_path_clear_since_.reset();
    return false;
  }

  const rclcpp::Time current_time = now();
  if (!stopped_path_clear_since_.has_value()) {
    stopped_path_clear_since_ = current_time;
  }
  return (current_time - stopped_path_clear_since_.value()).seconds() >=
         stopped_path_clear_sec_;
}

uint8_t StateMachineNode::resolve_requested_state()
{
  if (stopped_path_clear_latched_ && has_front_static_obstacle()) {
    stopped_path_clear_latched_ = false;
  }
  const bool avoid_requested = can_enter_avoid();
  const bool cruise_requested = has_interfering_opponent();

  switch (committed_state_) {
    case f110_msgs::msg::StateMachine::STATE_GLOBAL:
      if (avoid_requested) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_AVOID;
        enter_global_ok_since_.reset();
        RCLCPP_INFO(get_logger(), "STATE_GLOBAL -> STATE_AVOID (avoid path confirmed M-of-N).");
      } else if (cruise_requested) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_CRUISE;
        enter_global_ok_since_.reset();
        RCLCPP_INFO(
          get_logger(), "STATE_GLOBAL -> STATE_CRUISE (/opp_obs interference=true).");
      }
      break;

    case f110_msgs::msg::StateMachine::STATE_AVOID:
      {
        const bool merged_to_global = evaluate_enter_to_global(
          f110_msgs::msg::StateMachine::STATE_AVOID,
          has_avoid_wpnts(),
          avoid_wpnts_msg_);
        const bool path_exhausted = evaluate_avoid_path_exhausted();
        const bool stopped_path_clear = evaluate_stopped_path_clear();
        if (!(merged_to_global || path_exhausted || stopped_path_clear)) {
          break;
        }
        committed_state_ = stopped_path_clear ?
          f110_msgs::msg::StateMachine::STATE_GLOBAL : (cruise_requested ?
          f110_msgs::msg::StateMachine::STATE_CRUISE :
          f110_msgs::msg::StateMachine::STATE_GLOBAL);
        RCLCPP_INFO(
          get_logger(), "STATE_AVOID -> %s (%s).",
          stopped_path_clear || !cruise_requested ? "STATE_GLOBAL" : "STATE_CRUISE",
          stopped_path_clear ? "stopped with forward corridor clear" :
          (path_exhausted ? "avoid path exhausted" : "merged to global line"));
        if (path_exhausted || stopped_path_clear) {
          avoid_path_history_.clear();
          has_avoid_wpnts_ = false;
          avoid_wpnts_msg_.reset();
          last_non_empty_avoid_wpnts_msg_.reset();
        }
        if (stopped_path_clear) {
          stopped_path_clear_latched_ = true;
        }
        avoid_path_exhausted_since_.reset();
        stopped_path_clear_since_.reset();
      }
      break;

    case f110_msgs::msg::StateMachine::STATE_CRUISE:
      if (avoid_requested) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_AVOID;
        enter_global_ok_since_.reset();
        RCLCPP_INFO(
          get_logger(), "STATE_CRUISE -> STATE_AVOID (avoid path confirmed M-of-N).");
      } else if (!cruise_requested) {
        committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
        RCLCPP_INFO(
          get_logger(), "STATE_CRUISE -> STATE_GLOBAL (/opp_obs interference=false/stale).");
      }
      break;

    default:
      committed_state_ = f110_msgs::msg::StateMachine::STATE_GLOBAL;
      break;
  }
  return committed_state_;
}

void StateMachineNode::publish_state_cycle()
{
  const bool global_ready = has_valid_global();
  const bool frenet_ready = has_fresh_frenet();
  const bool avoid_ready = has_avoid_wpnts();
  const bool avoid_required = allow_avoid_transition_ ||
    committed_state_ == f110_msgs::msg::StateMachine::STATE_AVOID;

  if (!global_ready || !frenet_ready || (avoid_required && !avoid_ready)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "FSM inputs: global=%s frenet=%s avoid_wpnts=%s opp_obs=%s.",
      global_ready ? "true" : "false",
      frenet_ready ? "true" : "false",
      avoid_ready ? "true" : "false",
      opponent_seen_ ? (has_interfering_opponent() ? "interfering" : "clear") : "unseen");
  }
  if (global_ready &&
    (now() - last_global_receive_time_).seconds() > global_publisher_warn_timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Global waypoint publisher is silent; continuing with the last validated static path.");
  }

  const uint8_t state = resolve_requested_state();
  f110_msgs::msg::StateMachine message;
  message.header.stamp = now();
  message.header.frame_id = frame_id_;
  message.state = state;
  state_pub_->publish(message);

  if (!last_published_state_.has_value() || last_published_state_.value() != state) {
    RCLCPP_INFO(get_logger(), "Published state changed to %u.", state);
    last_published_state_ = state;
  }
}

void StateMachineNode::publish_selected_waypoints(uint8_t state)
{
  if (!has_fresh_frenet()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Frenet odometry is stale; local waypoint publication stopped.");
    return;
  }
  if (!has_valid_global()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "No validated global waypoints; local waypoint publication stopped.");
    return;
  }

  const rclcpp::Time stamp = now();
  const auto waypoints = select_waypoints(state, stamp);
  if (!waypoints.has_value()) {
    return;
  }

  local_waypoints_pub_->publish(waypoints.value());
  local_path_pub_->publish(build_path(waypoints.value()));
}

std::optional<f110_msgs::msg::WpntArray> StateMachineNode::select_waypoints(
  uint8_t state,
  const rclcpp::Time & stamp)
{
  if (state == f110_msgs::msg::StateMachine::STATE_AVOID && has_avoid_wpnts()) {
    return convert_ot_waypoints(*avoid_wpnts_msg_, stamp);
  }

  // GLOBAL and CRUISE both use the global path. CRUISE changes only longitudinal speed in the
  // control package; it never asks this selector for a different geometric path.
  return build_global_waypoints(stamp);
}

std::optional<f110_msgs::msg::WpntArray> StateMachineNode::build_global_waypoints(
  const rclcpp::Time & stamp)
{
  if (!has_valid_global() || frenet_odom_msg_ == nullptr) {
    return std::nullopt;
  }

  const auto index = parse_waypoint_index(frenet_odom_msg_->child_frame_id);
  if (!index.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid Frenet child_frame_id: '%s'.",
      frenet_odom_msg_->child_frame_id.c_str());
    return std::nullopt;
  }

  const int total = static_cast<int>(global_wpnts_msg_->wpnts.size());
  const int closest_index = index.value();
  if (closest_index < 0 || closest_index >= total) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Global waypoint index out of range: %d (0..%d).",
      closest_index,
      total - 1);
    return std::nullopt;
  }

  const int start = (closest_index + 1) % total;
  const int count = std::min(waypoint_num_, total);
  f110_msgs::msg::WpntArray output;
  output.header.stamp = stamp;
  output.header.frame_id = global_wpnts_msg_->header.frame_id.empty() ?
    frame_id_ : global_wpnts_msg_->header.frame_id;
  output.wpnts.reserve(static_cast<std::size_t>(count));
  for (int offset = 0; offset < count; ++offset) {
    output.wpnts.push_back(global_wpnts_msg_->wpnts[(start + offset) % total]);
  }
  return output;
}

f110_msgs::msg::WpntArray StateMachineNode::convert_ot_waypoints(
  const f110_msgs::msg::OTWpntArray & source,
  const rclcpp::Time & stamp) const
{
  f110_msgs::msg::WpntArray output;
  output.header = source.header;
  output.header.stamp = stamp;
  if (output.header.frame_id.empty()) {
    output.header.frame_id = frame_id_;
  }
  output.wpnts = source.wpnts;
  return output;
}

nav_msgs::msg::Path StateMachineNode::build_path(
  const f110_msgs::msg::WpntArray & waypoints) const
{
  nav_msgs::msg::Path path;
  path.header = waypoints.header;
  path.poses.reserve(waypoints.wpnts.size());
  for (const auto & waypoint : waypoints.wpnts) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.position.z = 0.0;
    const double half_yaw = waypoint.psi_rad * 0.5;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    path.poses.push_back(std::move(pose));
  }
  return path;
}

}  // namespace state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
