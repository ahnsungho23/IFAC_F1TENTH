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

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace local_planning
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-9;

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

bool finiteOdometry(const nav_msgs::msg::Odometry & odometry)
{
  return std::isfinite(odometry.pose.pose.position.x) &&
         std::isfinite(odometry.pose.pose.position.y) &&
         std::isfinite(odometry.twist.twist.linear.x);
}

double conservativeVariance(double first, double second)
{
  const double finite_first = std::isfinite(first) && first >= 0.0 ? first : 0.0;
  const double finite_second = std::isfinite(second) && second >= 0.0 ? second : 0.0;
  return std::max(finite_first, finite_second);
}

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

double wrapS(double s, double track_length)
{
  if (!(track_length > kGeometryEpsilon) || !std::isfinite(s)) {
    return s;
  }
  s = std::fmod(s, track_length);
  return s < 0.0 ? s + track_length : s;
}

double forwardDistance(double from_s, double to_s, double track_length)
{
  return wrapS(to_s - from_s, track_length);
}

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

double signedTrackDelta(double from_s, double to_s, double track_length)
{
  double delta = forwardDistance(from_s, to_s, track_length);
  if (delta > 0.5 * track_length) {
    delta -= track_length;
  }
  return delta;
}

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

void LocalPlannerNode::initializeParameters()
{
  planner_parameters_.detection_lookahead_m =
    declare_parameter<double>("detection_lookahead_m", 12.0);
  planner_parameters_.obstacle_cluster_gap_m =
    declare_parameter<double>("obstacle_cluster_gap_m", 0.8);
  planner_parameters_.obstacle_longitudinal_padding_m =
    declare_parameter<double>("obstacle_longitudinal_padding_m", 0.35);
  planner_parameters_.vehicle_half_width_m =
    declare_parameter<double>("vehicle_half_width_m", 0.1435);
  planner_parameters_.safety_margin_m =
    declare_parameter<double>("safety_margin_m", 0.03);
  planner_parameters_.tracking_error_reserve_m =
    declare_parameter<double>("tracking_error_reserve_m", 0.14);
  planner_parameters_.fallback_track_half_width_m =
    declare_parameter<double>("fallback_track_half_width_m", 1.50);
  planner_parameters_.pre_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "pre_apex_distances_m", std::vector<double>{6.0, 4.0, 2.0});
  planner_parameters_.post_apex_distances_m =
    declare_parameter<std::vector<double>>(
    "post_apex_distances_m", std::vector<double>{1.0, 2.0, 3.0});
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
  planner_parameters_.side_tie_epsilon_m =
    declare_parameter<double>("side_tie_epsilon_m", 0.02);
  planner_parameters_.maximum_lateral_slope =
    declare_parameter<double>("maximum_lateral_slope", 0.65);
  planner_parameters_.maximum_curvature_radpm =
    declare_parameter<double>("maximum_curvature_radpm", 3.20);
  planner_parameters_.maximum_curvature_rate_radpm2 =
    declare_parameter<double>("maximum_curvature_rate_radpm2", 20.0);
  planner_parameters_.safe_stop_buffer_m =
    declare_parameter<double>("safe_stop_buffer_m", 0.40);
  planner_parameters_.safe_stop_deceleration_mps2 =
    declare_parameter<double>("safe_stop_deceleration_mps2", 2.5);
  planner_parameters_.minimum_path_points =
    declare_parameter<int>("minimum_path_points", 8);

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
  chain_release_distance_m_ =
    declare_parameter<double>("chain_release_distance_m", 0.20);
  guard_parameters_.uncertainty_sigma_scale =
    declare_parameter<double>("uncertainty_sigma_scale", 3.0);
  guard_parameters_.minimum_longitudinal_inflation_m =
    declare_parameter<double>("uncertainty_min_longitudinal_inflation_m", 0.05);
  guard_parameters_.minimum_lateral_inflation_m =
    declare_parameter<double>("uncertainty_min_lateral_inflation_m", 0.03);
  guard_parameters_.maximum_lateral_inflation_m =
    declare_parameter<double>("uncertainty_max_lateral_inflation_m", 0.15);
  commitment_lock_lateral_threshold_m_ =
    declare_parameter<double>("commitment_lock_lateral_threshold_m", 0.10);
  commitment_lock_longitudinal_m_ =
    declare_parameter<double>("commitment_lock_longitudinal_m", 0.50);

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
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

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
  if (planning_period_ms_ <= 0 || merge_confirm_cycles_ <= 0 ||
    safe_stop_release_cycles_ <= 0 ||
    !std::isfinite(planner_parameters_.vehicle_half_width_m) ||
    !(planner_parameters_.vehicle_half_width_m > 0.0) ||
    !std::isfinite(planner_parameters_.safety_margin_m) ||
    planner_parameters_.safety_margin_m < 0.0 ||
    !std::isfinite(planner_parameters_.tracking_error_reserve_m) ||
    planner_parameters_.tracking_error_reserve_m < 0.0 ||
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
    !std::isfinite(chain_release_distance_m_) ||
    chain_release_distance_m_ < 0.0 ||
    !std::isfinite(guard_parameters_.uncertainty_sigma_scale) ||
    guard_parameters_.uncertainty_sigma_scale < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_longitudinal_inflation_m) ||
    guard_parameters_.minimum_longitudinal_inflation_m < 0.0 ||
    !std::isfinite(guard_parameters_.minimum_lateral_inflation_m) ||
    guard_parameters_.minimum_lateral_inflation_m < 0.0 ||
    guard_parameters_.maximum_lateral_inflation_m <
    guard_parameters_.minimum_lateral_inflation_m ||
    !std::isfinite(planner_parameters_.side_tie_epsilon_m) ||
    planner_parameters_.side_tie_epsilon_m < 0.0 ||
    commitment_lock_lateral_threshold_m_ < 0.0 ||
    commitment_lock_longitudinal_m_ < 0.0 ||
    planner_parameters_.minimum_path_points < 2)
  {
    throw std::invalid_argument(
            "planning periods, confirmation counts, unified lateral safety clearance, handoff "
            "settings, observation/uncertainty guard settings, commitment chain release, "
            "commitment locks, and point counts must be valid");
  }
}

void LocalPlannerNode::initializeInterfaces()
{
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions odometry_options;
  odometry_options.callback_group = odometry_callback_group_;
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

  avoid_waypoints_pub_ =
    create_publisher<f110_msgs::msg::OTWpntArray>(ot_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, volatile_qos);

  planning_timer_ = create_wall_timer(
    std::chrono::milliseconds(planning_period_ms_),
    std::bind(&LocalPlannerNode::onPlanningTimer, this), planning_callback_group_);
}

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
  ++obstacles_message_sequence_;
  if (obstacle_perception_degraded_) {
    obstacle_perception_degraded_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Static-obstacle perception recovered; replaced retained memory with a fresh snapshot.");
  }
}

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
  pre_engagement_side_switched_ = false;
  committed_obstacle_guards_.clear();
}

void LocalPlannerNode::resetInitialStabilization()
{
  initial_stabilization_active_ = false;
  initial_prepare_published_ = false;
  initial_has_counted_sequence_ = false;
  initial_cluster_union_.clear();
  initial_observation_counts_.clear();
}

void LocalPlannerNode::resetNextManeuverStabilization()
{
  next_stabilization_active_ = false;
  next_has_counted_sequence_ = false;
  next_cluster_union_.clear();
  next_observation_counts_.clear();
}

void LocalPlannerNode::resetCommitmentViolationConfirmation()
{
  commitment_soft_violation_count_ = 0;
}

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
  std::set<int> excluded = completed_obstacle_ids_;
  if (has_commitment_) {
    excluded.insert(
      committed_result_.obstacle_ids.begin(), committed_result_.obstacle_ids.end());
    if (committed_result_.obstacle_id >= 0) {
      excluded.insert(committed_result_.obstacle_id);
    }
  }

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

bool LocalPlannerNode::updateNextManeuverStabilization(
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
  const rclcpp::Time & update_time)
{
  if (!has_commitment_ || committed_result_.kind != SplinePlanKind::kAvoidance) {
    resetNextManeuverStabilization();
    return false;
  }

  // Stabilize the next cluster from the current ego state. A long, smooth return to d=0 must not
  // hide an already-visible obstacle merely because it lies before the old maneuver's merge_s.
  const auto cluster_ids = planner_.blockingClusterIds(ego, next_obstacles);
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
  return driven_distance + 1.0e-6 >= active_rear_distance + chain_release_distance_m_;
}

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
      if (apply_uncertainty_guard &&
        committed_guard != committed_obstacle_guards_.end() &&
        obstacleEnvelopeContained(
          validation_obstacle, committed_guard->second, planner_.trackLength()))
      {
        validation_obstacle = committed_guard->second;
      }
    }

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
  pre_engagement_side_switched_ = false;
  committed_obstacle_guards_.clear();
}

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

void LocalPlannerNode::commitAvoidance(
  RacelineSplineResult result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles)
{
  const bool replacing = has_commitment_;
  const bool avoid_was_observed = avoid_state_observed_;
  if (replacing && committed_result_.kind == SplinePlanKind::kAvoidance &&
    result.go_left != committed_result_.go_left && !commitmentSideLocked(ego))
  {
    pre_engagement_side_switched_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Pre-engagement side switch to %s; further side switches are disabled until lateral "
      "engagement or the next maneuver.",
      result.go_left ? "left" : "right");
  }
  if (!replacing) {
    pre_engagement_side_switched_ = false;
  }
  std::set<int> committed_ids(result.obstacle_ids.begin(), result.obstacle_ids.end());
  if (result.obstacle_id >= 0) {
    committed_ids.insert(result.obstacle_id);
  }
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

void LocalPlannerNode::handleSafeStopLatch(const EgoFrenetState & ego)
{
  const auto planning_obstacles = has_commitment_ ?
    buildCurrentManeuverInput(ego) :
    buildGuardedObstacles(buildInitialStabilizationInput());
  const std::optional<bool> locked_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  const bool allow_side_switch =
    !locked_side.has_value() ||
    (!commitmentSideLocked(ego) && !pre_engagement_side_switched_);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, locked_side, allow_side_switch);

  if (result.kind == SplinePlanKind::kAvoidance) {
    ++safe_stop_release_count_;
    if (safe_stop_release_count_ >= safe_stop_release_cycles_) {
      RCLCPP_INFO(
        get_logger(),
        "Safe-stop released after %d consecutive feasible avoidance plans.",
        safe_stop_release_count_);
      commitAvoidance(std::move(result), ego, planning_obstacles);
      publishResult(committed_result_);
      return;
    }
  } else if (result.kind == SplinePlanKind::kNoObstacle) {
    ++safe_stop_release_count_;
    if (safe_stop_release_count_ >= safe_stop_release_cycles_) {
      if (activateGlobalHandoff(ego)) {
        RCLCPP_INFO(
          get_logger(),
          "Safe-stop released after %d clear cycles; publishing global handoff.",
          safe_stop_release_count_);
        publishResult(committed_result_);
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

  publishResult(safe_stop_result_);
}

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

  EgoFrenetState ego;
  ego.s = odometry.pose.pose.position.x;
  ego.d = odometry.pose.pose.position.y;
  ego.speed = std::abs(odometry.twist.twist.linear.x);

  if ((now() - odometry_time).seconds() > odometry_stale_timeout_sec_) {
    RacelineSplineResult emergency_hold;
    emergency_hold.kind = SplinePlanKind::kSafeStop;
    emergency_hold.path = planner_.buildEmergencyStopPath(ego);
    emergency_hold.reason =
      "Frenet odometry is stale; publishing a zero-speed hold at the last known pose";
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s", emergency_hold.reason.c_str());
    publishResult(emergency_hold);
    return;
  }
  if (require_obstacles_message_ && !has_obstacles_message_) {
    publishEmpty("waiting for static-obstacle perception");
    return;
  }
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
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "global handoff");
    if (!chained_maneuver_started) {
      if (avoid_state_observed_ && has_state_ &&
        current_state_ == f110_msgs::msg::StateMachine::STATE_GLOBAL)
      {
        RCLCPP_INFO(
          get_logger(),
          "State machine confirmed GLOBAL after all static obstacles cleared; "
          "releasing committed tail.");
        clearCommitment();
        publishEmpty("state machine confirmed global handoff");
        return;
      }
      publishResult(committed_result_);
      return;
    }
  }
  if (safe_stop_latched_) {
    handleSafeStopLatch(ego);
    return;
  }
  if (has_commitment_ && !merge_geometry_confirmed_) {
    std::vector<f110_msgs::msg::Obstacle> early_next_obstacles;
    if (tryEarlyChainedManeuver(ego, early_next_obstacles)) {
      publishResult(committed_result_);
      return;
    }
  }
  if (has_commitment_ && !merge_geometry_confirmed_ && commitmentComplete(ego)) {
    chained_maneuver_started = beginChainedManeuverIfNeeded(
      ego, planning_obstacles, "merge completion");
    if (chained_maneuver_started) {
      // Fall through to the ordinary initial-stabilization path. This deliberately starts the
      // next obstacle with no preferred side, while the non-empty preparation path keeps AVOID.
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
    publishResult(committed_result_);
    return;
  }

  if (!has_commitment_) {
    auto conservative_obstacles = buildInitialStabilizationInput();
    planning_obstacles = buildGuardedObstacles(conservative_obstacles);
    if (obstacle_perception_degraded_) {
      // No new observation can improve stabilization while perception is stale. Reuse the
      // already accepted snapshot and its uncertainty guard immediately on a later lap.
      resetInitialStabilization();
    } else {
      auto preparation = planner_.buildPreparationStop(ego, planning_obstacles);
      if (preparation.kind == SplinePlanKind::kNoObstacle) {
        const bool published_preparation = initial_prepare_published_;
        resetInitialStabilization();
        if (published_preparation && activateGlobalHandoff(ego)) {
          publishResult(committed_result_);
          return;
        }
      } else if (preparation.kind == SplinePlanKind::kNoSafePath) {
        resetInitialStabilization();
        latchSafeStop(std::move(preparation), ego, planning_obstacles);
        publishResult(safe_stop_result_);
        return;
      } else {
        const bool stable = updateInitialStabilization(
          preparation.obstacle_ids, conservative_obstacles, now());
        if (!stable) {
          initial_prepare_published_ = true;
          publishResult(preparation);
          return;
        }
        // The final guard is frozen from the conservative multi-message union and its worst
        // positional variance. Subsequent same-ID observations inside it cannot move the path.
        conservative_obstacles = buildInitialStabilizationInput();
        planning_obstacles = buildGuardedObstacles(conservative_obstacles);
        resetInitialStabilization();
      }
    }
  } else {
    // Obstacles whose expanded front face starts after this maneuver's merge belong to the next
    // maneuver. They must not invalidate the current path merely because its controller tail
    // extends beyond the merge point.
    planning_obstacles = buildCurrentManeuverInput(ego);
  }

  std::string commitment_error;
  PathValidationFailure commitment_failure;
  if (has_commitment_) {
    const double collision_horizon = remainingDistanceToMerge(ego);
    const bool commitment_valid = planner_.validatePath(
      ego, committed_result_.path, planning_obstacles,
      &commitment_error, &commitment_failure, collision_horizon);
    if (commitment_valid) {
      if (commitment_soft_violation_count_ > 0) {
        RCLCPP_INFO(
          get_logger(),
          "Soft commitment violation cleared after %d/%d confirmation cycles; "
          "keeping the frozen path.",
          commitment_soft_violation_count_, commitment_soft_violation_confirm_cycles_);
      }
      resetCommitmentViolationConfirmation();
      // The committed geometry is still safe. Rebuilding six spline candidates here only makes
      // perception jitter visible downstream and repeats all geometry/curvature work.
      publishResult(committed_result_);
      return;
    }

    if (commitment_failure.kind == PathValidationFailureKind::kObstacleCollision) {
      const auto hard_collision_obstacles = buildCurrentManeuverInput(ego, false);
      PathValidationFailure hard_failure;
      const bool hard_collision_free = planner_.validatePath(
        ego, committed_result_.path, hard_collision_obstacles,
        nullptr, &hard_failure, collision_horizon);
      const bool hard_collision =
        !hard_collision_free &&
        hard_failure.kind == PathValidationFailureKind::kObstacleCollision;
      if (hard_collision) {
        resetCommitmentViolationConfirmation();
        logObstacleCollision("Hard commitment collision; replanning immediately", hard_failure);
      } else {
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
          publishResult(committed_result_);
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

  const std::optional<bool> preferred_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  const bool allow_side_switch =
    !preferred_side.has_value() ||
    (!commitmentSideLocked(ego) && !pre_engagement_side_switched_);
  RacelineSplineResult result = planner_.plan(
    ego, planning_obstacles, preferred_side, allow_side_switch);

  if (result.kind == SplinePlanKind::kAvoidance) {
    commitAvoidance(std::move(result), ego, planning_obstacles);
    publishResult(committed_result_);
    return;
  }

  if (has_commitment_ && result.kind == SplinePlanKind::kNoObstacle) {
    // Perception commonly drops the passed obstacle before the spline tail is reached. Keep the
    // already race-line-locked commitment until its geometric merge is complete.
    publishResult(committed_result_);
    return;
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  if (has_commitment_) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  if (result.kind == SplinePlanKind::kNoSafePath && result.obstacle_id >= 0) {
    latchSafeStop(std::move(result), ego, planning_obstacles);
    publishResult(safe_stop_result_);
    return;
  }
  clearCommitment();
  publishEmpty(result.reason);
}

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

void LocalPlannerNode::publishResult(const RacelineSplineResult & result)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.wpnts = result.path.wpnts;
  const bool stop_like =
    result.kind == SplinePlanKind::kSafeStop ||
    result.kind == SplinePlanKind::kPreparation;
  output.ot_side = stop_like ?
    "stop" : (result.go_left ? "left" : "right");
  output.ot_line =
    result.kind == SplinePlanKind::kPreparation ? "raceline_static_prepare" :
    (result.kind == SplinePlanKind::kSafeStop ? "raceline_static_safe_stop" :
    (handoff_active_ ? "raceline_global_handoff" : "raceline_local_d_offset_spline"));
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

  if (local_path_pub_->get_subscription_count() > 0U) {
    local_path_pub_->publish(makePath(result.path.wpnts, output.header));
  }
}

void LocalPlannerNode::publishEmpty(const std::string & reason)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.last_switch_time = last_side_switch_time_;
  output.ot_line = reason;
  avoid_waypoints_pub_->publish(output);

  if (local_path_pub_->get_subscription_count() > 0U) {
    nav_msgs::msg::Path empty_path;
    empty_path.header = output.header;
    local_path_pub_->publish(empty_path);
  }
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
}

}  // namespace local_planning
