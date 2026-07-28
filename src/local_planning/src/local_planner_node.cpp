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
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace local_planning
{
namespace
{

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
  planner_parameters_.obstacle_clearance_m =
    declare_parameter<double>("obstacle_clearance_m", 0.30);
  planner_parameters_.blocking_margin_m =
    declare_parameter<double>("blocking_margin_m", 0.10);
  planner_parameters_.vehicle_half_width_m =
    declare_parameter<double>("vehicle_half_width_m", 0.121);
  planner_parameters_.boundary_margin_m =
    declare_parameter<double>("boundary_margin_m", 0.10);
  planner_parameters_.fallback_track_half_width_m =
    declare_parameter<double>("fallback_track_half_width_m", 1.50);
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
  planner_parameters_.minimum_target_offset_m =
    declare_parameter<double>("minimum_target_offset_m", 0.20);
  planner_parameters_.maximum_target_offset_m =
    declare_parameter<double>("maximum_target_offset_m", 1.50);
  planner_parameters_.maximum_lateral_slope =
    declare_parameter<double>("maximum_lateral_slope", 0.65);
  planner_parameters_.maximum_curvature_radpm =
    declare_parameter<double>("maximum_curvature_radpm", 3.20);
  planner_parameters_.maximum_curvature_rate_radpm2 =
    declare_parameter<double>("maximum_curvature_rate_radpm2", 20.0);
  planner_parameters_.avoidance_speed_scale =
    declare_parameter<double>("avoidance_speed_scale", 0.80);
  planner_parameters_.maximum_lateral_accel_mps2 =
    declare_parameter<double>("maximum_lateral_accel_mps2", 5.5);
  planner_parameters_.maximum_longitudinal_accel_mps2 =
    declare_parameter<double>("maximum_longitudinal_accel_mps2", 3.0);
  planner_parameters_.maximum_longitudinal_decel_mps2 =
    declare_parameter<double>("maximum_longitudinal_decel_mps2", 5.0);
  planner_parameters_.safe_stop_buffer_m =
    declare_parameter<double>("safe_stop_buffer_m", 0.80);
  planner_parameters_.safe_stop_deceleration_mps2 =
    declare_parameter<double>("safe_stop_deceleration_mps2", 2.5);
  planner_parameters_.minimum_path_points =
    declare_parameter<int>("minimum_path_points", 8);

  require_obstacles_message_ = declare_parameter<bool>("require_obstacles_message", true);
  static_obstacles_only_ = declare_parameter<bool>("static_obstacles_only", true);
  static_speed_threshold_mps_ = declare_parameter<double>("static_speed_threshold_mps", 0.25);
  obstacle_stale_timeout_sec_ = declare_parameter<double>("obstacle_stale_timeout_sec", 0.75);
  odometry_stale_timeout_sec_ = declare_parameter<double>("odometry_stale_timeout_sec", 0.50);
  merge_lateral_tolerance_m_ = declare_parameter<double>("merge_lateral_tolerance_m", 0.15);
  merge_confirm_cycles_ = declare_parameter<int>("merge_confirm_cycles", 15);
  planning_period_ms_ = declare_parameter<int>("planning_period_ms", 50);
  publish_standalone_local_ = declare_parameter<bool>("publish_standalone_local", false);
  obstacle_marker_scale_m_ = declare_parameter<double>("obstacle_marker_scale_m", 0.35);
  path_marker_width_m_ = declare_parameter<double>("path_marker_width_m", 0.06);

  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  obstacles_topic_ =
    declare_parameter<std::string>(
    "obstacles_topic", "/static_obs");
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  ot_waypoints_topic_ =
    declare_parameter<std::string>("ot_waypoints_topic", "/avoid_waypoints");
  local_waypoints_topic_ =
    declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
  local_path_topic_ =
    declare_parameter<std::string>("local_path_topic", "/local_planning/path");
  compatibility_path_topic_ =
    declare_parameter<std::string>("compatibility_path_topic", "/local_path");
  markers_topic_ =
    declare_parameter<std::string>("markers_topic", "/local_planning/markers");
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
    [](double scale) {return std::isfinite(scale) && scale > 0.0;});
  if (!control_points_valid || !scales_valid) {
    throw std::invalid_argument(
            "pre/post apex arrays must contain three ordered positive values and transition "
            "scales must be positive");
  }
  if (planning_period_ms_ <= 0 || merge_confirm_cycles_ <= 0 ||
    planner_parameters_.minimum_path_points < 2)
  {
    throw std::invalid_argument(
            "planning periods, confirmation counts, and point counts must be positive");
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

  avoid_waypoints_pub_ =
    create_publisher<f110_msgs::msg::OTWpntArray>(ot_waypoints_topic_, volatile_qos);
  standalone_waypoints_pub_ =
    create_publisher<f110_msgs::msg::WpntArray>(local_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, volatile_qos);
  compatibility_path_pub_ =
    create_publisher<nav_msgs::msg::Path>(compatibility_path_topic_, volatile_qos);
  markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, volatile_qos);

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
  std::vector<global_planning::ReferenceWaypoint> reference;
  reference.reserve(message->wpnts.size());
  for (const auto & waypoint : message->wpnts) {
    reference.push_back({waypoint.x_m, waypoint.y_m, waypoint.s_m});
  }
  try {
    global_planning::ClcsFrenetConfig config;
    config.closed_loop = true;
    clcs_converter_ = global_planning::ClcsFrenetConverter::create(
      reference, config, ++clcs_version_);
  } catch (const std::exception & exception) {
    has_global_waypoints_ = false;
    clcs_converter_.reset();
    clearCommitment();
    RCLCPP_ERROR(
      get_logger(), "Failed to build CLCS converter for Cartesian obstacles: %s",
      exception.what());
    return;
  }
  global_waypoints_ = *message;
  has_global_waypoints_ = true;
  clearCommitment();
  RCLCPP_INFO(
    get_logger(), "Loaded %zu ordered global race-line waypoints (track %.2f m).",
    global_waypoints_.wpnts.size(), planner_.trackLength());
}

bool LocalPlannerNode::isStaticObstacle(const f110_msgs::msg::Obstacle & obstacle) const
{
  if (!static_obstacles_only_) {
    return true;
  }
  return obstacle.is_static ||
         (std::isfinite(obstacle.vs) && std::isfinite(obstacle.vd) &&
         std::hypot(obstacle.vs, obstacle.vd) <= static_speed_threshold_mps_);
}

std::optional<f110_msgs::msg::Obstacle> LocalPlannerNode::projectCartesianObstacle(
  const f110_msgs::msg::Obstacle & obstacle) const
{
  if (!clcs_converter_ || !obstacle.has_cartesian ||
    !std::isfinite(obstacle.x_center) || !std::isfinite(obstacle.y_center) ||
    !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0)
  {
    return std::nullopt;
  }

  global_planning::ClcsConversionInput input;
  input.x = obstacle.x_center;
  input.y = obstacle.y_center;
  const auto projection = clcs_converter_->convert(input);
  if (!projection.valid) {
    return std::nullopt;
  }

  // Every static obstacle is conservatively represented by a circle. Its radius is invariant
  // under Cartesian-to-Frenet rotation, so both local longitudinal and lateral extents are r.
  const double longitudinal_half_extent = obstacle.radius;
  const double lateral_half_extent = obstacle.radius;

  const double track_length = planner_.trackLength();
  const auto wrap_s = [track_length](double s) {
      if (track_length <= 0.0) {
        return s;
      }
      s = std::fmod(s, track_length);
      return s < 0.0 ? s + track_length : s;
    };

  auto projected = obstacle;
  projected.s_center = projection.s;
  projected.d_center = projection.d;
  projected.s_start = wrap_s(projection.s - longitudinal_half_extent);
  projected.s_end = wrap_s(projection.s + longitudinal_half_extent);
  projected.d_right = projection.d - lateral_half_extent;
  projected.d_left = projection.d + lateral_half_extent;
  projected.size = 2.0 * obstacle.radius;
  return projected;
}

void LocalPlannerNode::onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message)
{
  static_obstacles_.clear();
  static_obstacles_.reserve(message->obstacles.size());
  if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring Cartesian obstacle array in frame '%s'; expected '%s'.",
      message->header.frame_id.c_str(), frame_id_.c_str());
    has_obstacles_message_ = false;
    return;
  }
  std::size_t rejected = 0;
  for (const auto & obstacle : message->obstacles) {
    if (isStaticObstacle(obstacle)) {
      const auto projected = projectCartesianObstacle(obstacle);
      if (projected.has_value()) {
        static_obstacles_.push_back(projected.value());
      } else {
        ++rejected;
      }
    }
  }
  if (rejected > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected %zu static obstacles with invalid x/y/radius or CLCS projection.",
      rejected);
  }
  has_obstacles_message_ = true;
  last_obstacles_time_ = now();
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

void LocalPlannerNode::clearCommitment()
{
  has_commitment_ = false;
  committed_result_ = RacelineSplineResult();
  merge_complete_count_ = 0;
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
  if ((now() - odometry_time).seconds() > odometry_stale_timeout_sec_) {
    clearCommitment();
    publishEmpty("Frenet odometry is stale");
    return;
  }
  if (require_obstacles_message_ && !has_obstacles_message_) {
    publishEmpty("waiting for static-obstacle perception");
    return;
  }
  if (has_obstacles_message_ &&
    (now() - last_obstacles_time_).seconds() > obstacle_stale_timeout_sec_)
  {
    clearCommitment();
    publishEmpty("static-obstacle perception is stale");
    return;
  }

  EgoFrenetState ego;
  ego.s = odometry.pose.pose.position.x;
  ego.d = odometry.pose.pose.position.y;
  ego.speed = std::abs(odometry.twist.twist.linear.x);
  if (commitmentComplete(ego)) {
    RCLCPP_INFO(get_logger(), "Static avoidance completed and merged onto the global race line.");
    clearCommitment();
    publishEmpty("avoidance merge complete");
    return;
  }

  const std::optional<bool> preferred_side =
    has_commitment_ && committed_result_.kind == SplinePlanKind::kAvoidance ?
    std::optional<bool>(committed_result_.go_left) : std::nullopt;
  RacelineSplineResult result = planner_.plan(ego, static_obstacles_, preferred_side);

  if (result.kind == SplinePlanKind::kAvoidance) {
    const bool new_commitment = !has_commitment_;
    const bool side_changed = has_commitment_ &&
      committed_result_.kind == SplinePlanKind::kAvoidance &&
      committed_result_.go_left != result.go_left;
    committed_result_ = result;
    has_commitment_ = true;
    if (new_commitment || side_changed) {
      commitment_start_s_ = ego.s;
      merge_complete_count_ = 0;
      RCLCPP_INFO(
        get_logger(), "Committed %s d-offset spline around static obstacle %d (target d=%.2f).",
        result.go_left ? "left" : "right", result.obstacle_id, result.target_d);
    }
    publishResult(committed_result_, ego, static_obstacles_);
    return;
  }

  if (has_commitment_ && result.kind == SplinePlanKind::kNoObstacle) {
    // Perception commonly drops the passed obstacle before the spline tail is reached. Keep the
    // already race-line-locked commitment until its geometric merge is complete.
    publishResult(committed_result_, ego, static_obstacles_);
    return;
  }
  if (result.kind == SplinePlanKind::kSafeStop) {
    clearCommitment();
    publishResult(result, ego, static_obstacles_);
    return;
  }
  clearCommitment();
  publishEmpty(result.reason);
}

nav_msgs::msg::Path LocalPlannerNode::makePath(
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
    pose.pose.orientation = yawQuaternion(waypoint.psi_rad);
    path.poses.push_back(pose);
  }
  return path;
}

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

  Marker path;
  path.header = clear.header;
  path.ns = "raceline_offset_spline";
  path.id = 0;
  path.type = Marker::LINE_STRIP;
  path.action = Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = path_marker_width_m_;
  path.color.a = 1.0F;
  path.color.g = result.kind == SplinePlanKind::kSafeStop ? 0.55F : 1.0F;
  path.color.r = result.kind == SplinePlanKind::kSafeStop ? 1.0F : 0.05F;
  for (const auto & waypoint : result.path.wpnts) {
    geometry_msgs::msg::Point point;
    point.x = waypoint.x_m;
    point.y = waypoint.y_m;
    point.z = 0.03;
    path.points.push_back(point);
  }
  markers.markers.push_back(path);

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

  int marker_id = 0;
  for (const auto & obstacle : obstacles) {
    Marker marker;
    marker.header = clear.header;
    marker.ns = "static_obstacles";
    marker.id = marker_id++;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;
    double yaw = 0.0;
    planner_.toCartesian(
      obstacle.s_center, obstacle.d_center,
      marker.pose.position.x, marker.pose.position.y, yaw);
    marker.pose.position.z = 0.12;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obstacle_marker_scale_m_;
    marker.scale.y = obstacle_marker_scale_m_;
    marker.scale.z = obstacle_marker_scale_m_;
    marker.color.a = 0.85F;
    marker.color.r = 1.0F;
    marker.color.g = 0.1F;
    markers.markers.push_back(marker);
  }
  return markers;
}

void LocalPlannerNode::publishResult(
  const RacelineSplineResult & result,
  const EgoFrenetState & ego,
  const std::vector<f110_msgs::msg::Obstacle> & obstacles)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.wpnts = result.path.wpnts;
  output.ot_side = result.kind == SplinePlanKind::kSafeStop ?
    "stop" : (result.go_left ? "left" : "right");
  output.ot_line = result.kind == SplinePlanKind::kSafeStop ?
    "raceline_static_safe_stop" : "raceline_local_d_offset_spline";
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

  f110_msgs::msg::WpntArray waypoints;
  waypoints.header = output.header;
  waypoints.wpnts = output.wpnts;
  if (publish_standalone_local_) {
    standalone_waypoints_pub_->publish(waypoints);
  }
  const auto path = makePath(waypoints);
  local_path_pub_->publish(path);
  compatibility_path_pub_->publish(path);
  markers_pub_->publish(makeMarkers(result, ego, obstacles));
}

void LocalPlannerNode::publishEmpty(const std::string & reason)
{
  f110_msgs::msg::OTWpntArray output;
  output.header.stamp = now();
  output.header.frame_id = frame_id_;
  output.last_switch_time = last_side_switch_time_;
  output.ot_line = reason;
  avoid_waypoints_pub_->publish(output);

  f110_msgs::msg::WpntArray empty_waypoints;
  empty_waypoints.header = output.header;
  const auto empty_path = makePath(empty_waypoints);
  local_path_pub_->publish(empty_path);
  compatibility_path_pub_->publish(empty_path);
  RacelineSplineResult empty_result;
  markers_pub_->publish(makeMarkers(empty_result, EgoFrenetState(), {}));
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
}

}  // namespace local_planning
