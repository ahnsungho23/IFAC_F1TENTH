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
<<<<<<< HEAD
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
    "obstacles_topic", "/perception/static_obstacles/cartesian");
=======
  lookahead_wpnt_num_ = declare_parameter<int>("lookahead_wpnt_num", 160);
  detection_lookahead_wpnt_num_ =
    declare_parameter<int>("detection_lookahead_wpnt_num", 120);
  spline_window_margin_wpnts_ =
    declare_parameter<int>("spline_window_margin_wpnts", 40);
  corner_spline_window_margin_wpnts_ =
    declare_parameter<int>("corner_spline_window_margin_wpnts", 18);
  safety_margin_ = declare_parameter<double>("safety_margin", 0.35);
  wall_margin_ = declare_parameter<double>("wall_margin", 0.30);
  avoid_offset_ = declare_parameter<double>("avoid_offset", 0.45);
  corner_avoid_offset_ = declare_parameter<double>("corner_avoid_offset", 0.35);
  vehicle_radius_ = declare_parameter<double>("vehicle_radius", 0.20);
  path_clearance_margin_ =
    declare_parameter<double>("path_clearance_margin", 0.05);
  vehicle_front_extent_m_ =
    declare_parameter<double>("vehicle_front_extent_m", 0.3302);
  vehicle_rear_extent_m_ =
    declare_parameter<double>("vehicle_rear_extent_m", 0.05);
  vehicle_width_m_ = declare_parameter<double>("vehicle_width_m", 0.2413);
  localization_margin_m_ =
    declare_parameter<double>("localization_margin_m", 0.05);
  corridor_safety_margin_m_ =
    declare_parameter<double>("corridor_safety_margin_m", 0.05);
  preserve_circular_collision_check_ =
    declare_parameter<bool>("preserve_circular_collision_check", true);
  legacy_clamp_candidate_d_ =
    declare_parameter<bool>("legacy_clamp_candidate_d", false);
  corner_tracking_margin_ =
    declare_parameter<double>("corner_tracking_margin", 0.10);
  corner_curvature_threshold_ =
    declare_parameter<double>("corner_curvature_threshold", 1.20);
  longitudinal_search_half_width_m_ =
    declare_parameter<double>("longitudinal_search_half_width_m", 0.15);
  obstacle_component_min_area_m2_ =
    declare_parameter<double>("obstacle_component_min_area_m2", 0.002);
  obstacle_component_max_area_m2_ =
    declare_parameter<double>("obstacle_component_max_area_m2", 2.0);
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
  use_perception_obstacles_ =
    declare_parameter<bool>("use_perception_obstacles", true);
  require_perception_obstacles_ =
    declare_parameter<bool>("require_perception_obstacles", false);
  perception_static_only_ =
    declare_parameter<bool>("perception_static_only", true);
  perception_static_speed_threshold_mps_ =
    declare_parameter<double>("perception_static_speed_threshold_mps", 0.25);
  perception_obstacle_padding_m_ =
    declare_parameter<double>("perception_obstacle_padding_m", 0.05);
  perception_grid_update_period_sec_ =
    declare_parameter<double>("perception_grid_update_period_sec", 1.0);
  perception_grid_rebuild_motion_m_ =
    declare_parameter<double>("perception_grid_rebuild_motion_m", 0.12);
  perception_static_grid_match_gate_m_ =
    declare_parameter<double>("perception_static_grid_match_gate_m", 0.75);
  perception_static_history_size_ =
    declare_parameter<int>("perception_static_history_size", 6);
  perception_static_expand_confirm_cycles_ =
    declare_parameter<int>("perception_static_expand_confirm_cycles", 2);
  perception_static_shrink_confirm_cycles_ =
    declare_parameter<int>("perception_static_shrink_confirm_cycles", 3);
  perception_filter_alpha_ =
    declare_parameter<double>("perception_filter_alpha", 0.25);
  perception_track_hold_sec_ =
    declare_parameter<double>("perception_track_hold_sec", 1.0);
  perception_association_gate_m_ =
    declare_parameter<double>("perception_association_gate_m", 0.35);
  perception_max_s_step_m_ =
    declare_parameter<double>("perception_max_s_step_m", 0.20);
  perception_max_d_step_m_ =
    declare_parameter<double>("perception_max_d_step_m", 0.12);
  freeze_committed_static_obstacles_ =
    declare_parameter<bool>("freeze_committed_static_obstacles", true);
  committed_obstacle_match_distance_m_ =
    declare_parameter<double>("committed_obstacle_match_distance_m", 0.35);
  unknown_cell_policy_name_ =
    declare_parameter<std::string>("unknown_cell_policy", "reject_candidate");
  unknown_cell_policy_ =
    SafeCorridorBuilder::parseUnknownCellPolicy(unknown_cell_policy_name_);
  speed_reduction_ratio_ = declare_parameter<double>("speed_reduction_ratio", 0.76);
  corner_speed_reduction_ratio_ =
    declare_parameter<double>("corner_speed_reduction_ratio", 0.55);
  post_obstacle_speed_recovery_ratio_ =
    declare_parameter<double>("post_obstacle_speed_recovery_ratio", 1.0);
  lattice_max_longitudinal_accel_mps2_ =
    declare_parameter<double>("lattice_max_longitudinal_accel_mps2", 3.0);
  lattice_max_longitudinal_decel_mps2_ =
    declare_parameter<double>("lattice_max_longitudinal_decel_mps2", 6.0);
  lattice_lateral_samples_ = declare_parameter<int>("lattice_lateral_samples", 4);
  lattice_lateral_step_m_ = declare_parameter<double>("lattice_lateral_step_m", 0.06);
  lattice_corridor_target_inset_m_ =
    declare_parameter<double>("lattice_corridor_target_inset_m", 0.020);
  lattice_corridor_validation_tolerance_m_ =
    declare_parameter<double>("lattice_corridor_validation_tolerance_m", 0.010);
  lattice_narrow_corridor_width_threshold_m_ =
    declare_parameter<double>("lattice_narrow_corridor_width_threshold_m", 0.20);
  lattice_narrow_corridor_transition_scales_ = declare_parameter<std::vector<double>>(
    "lattice_narrow_corridor_transition_scales", {1.40, 1.80, 2.20});
  lattice_obstacle_cluster_gap_wpnts_ =
    declare_parameter<int>("lattice_obstacle_cluster_gap_wpnts", 16);
  lattice_corridor_knot_stride_wpnts_ =
    declare_parameter<int>("lattice_corridor_knot_stride_wpnts", 3);
  lattice_corridor_beam_width_ =
    declare_parameter<int>("lattice_corridor_beam_width", 16);
  lattice_transition_scales_ = declare_parameter<std::vector<double>>(
    "lattice_transition_scales", {1.80, 1.40, 1.20, 1.0, 0.75});
  lattice_min_transition_wpnts_ =
    declare_parameter<int>("lattice_min_transition_wpnts", 10);
  lattice_recovery_lateral_samples_ =
    declare_parameter<int>("lattice_recovery_lateral_samples", 7);
  lattice_recovery_lateral_step_m_ =
    declare_parameter<double>("lattice_recovery_lateral_step_m", 0.04);
  lattice_recovery_transition_scales_ = declare_parameter<std::vector<double>>(
    "lattice_recovery_transition_scales", {2.20, 1.80, 1.40, 1.15, 0.90, 0.70});
  lattice_recovery_min_transition_wpnts_ =
    declare_parameter<int>("lattice_recovery_min_transition_wpnts", 10);
  lattice_recovery_profile_limit_ =
    declare_parameter<int>("lattice_recovery_profile_limit", 6);
  lattice_primary_search_budget_ms_ =
    declare_parameter<int>("lattice_primary_search_budget_ms", 120);
  lattice_recovery_search_budget_ms_ =
    declare_parameter<int>("lattice_recovery_search_budget_ms", 180);
  lattice_max_result_pose_drift_m_ =
    declare_parameter<double>("lattice_max_result_pose_drift_m", 1.00);
  lattice_recovery_max_curvature_scale_ =
    declare_parameter<double>("lattice_recovery_max_curvature_scale", 1.25);
  lattice_recovery_max_curvature_rate_scale_ =
    declare_parameter<double>("lattice_recovery_max_curvature_rate_scale", 1.25);
  lattice_max_curvature_radpm_ =
    declare_parameter<double>("lattice_max_curvature_radpm", 2.80);
  lattice_max_curvature_rate_radpm2_ =
    declare_parameter<double>("lattice_max_curvature_rate_radpm2", 18.0);
  lattice_max_lateral_accel_mps2_ =
    declare_parameter<double>("lattice_max_lateral_accel_mps2", 5.80);
  lattice_speed_safety_factor_ =
    declare_parameter<double>("lattice_speed_safety_factor", 0.95);
  lattice_collision_sample_step_m_ =
    declare_parameter<double>("lattice_collision_sample_step_m", 0.04);
  lattice_preferred_clearance_m_ =
    declare_parameter<double>("lattice_preferred_clearance_m", 0.40);
  lattice_weight_clearance_ =
    declare_parameter<double>("lattice_weight_clearance", 2.0);
  lattice_weight_min_clearance_ =
    declare_parameter<double>("lattice_weight_min_clearance", 12.0);
  lattice_weight_deviation_ =
    declare_parameter<double>("lattice_weight_deviation", 1.0);
  lattice_weight_curvature_ =
    declare_parameter<double>("lattice_weight_curvature", 0.30);
  lattice_weight_curvature_change_ =
    declare_parameter<double>("lattice_weight_curvature_change", 0.12);
  lattice_weight_spatial_lateral_jerk_ =
    declare_parameter<double>("lattice_weight_spatial_lateral_jerk", 0.02);
  lattice_weight_maneuver_length_ =
    declare_parameter<double>("lattice_weight_maneuver_length", 0.08);
  lattice_weight_path_length_ =
    declare_parameter<double>("lattice_weight_path_length", 0.30);
  lattice_weight_speed_loss_ =
    declare_parameter<double>("lattice_weight_speed_loss", 0.20);
  lattice_weight_opposite_side_ =
    declare_parameter<double>("lattice_weight_opposite_side", 2.0);
  lattice_commit_path_until_clear_ =
    declare_parameter<bool>("lattice_commit_path_until_clear", true);
  lattice_preplan_before_merge_m_ =
    declare_parameter<double>("lattice_preplan_before_merge_m", 10.0);
  lattice_preplan_max_start_gap_m_ =
    declare_parameter<double>("lattice_preplan_max_start_gap_m", 0.35);
  lattice_post_merge_lookahead_wpnts_ =
    declare_parameter<int>("lattice_post_merge_lookahead_wpnts", 40);
  lattice_merge_lateral_tolerance_m_ =
    declare_parameter<double>("lattice_merge_lateral_tolerance_m", 0.15);
  lattice_merge_settle_max_wpnts_ =
    declare_parameter<int>("lattice_merge_settle_max_wpnts", 30);
  lattice_safe_stop_buffer_wpnts_ =
    declare_parameter<int>("lattice_safe_stop_buffer_wpnts", 8);
  lattice_safe_stop_deceleration_mps2_ =
    declare_parameter<double>("lattice_safe_stop_deceleration_mps2", 2.50);
  lattice_replan_brake_timeout_sec_ =
    declare_parameter<double>("lattice_replan_brake_timeout_sec", 1.50);
  stationary_hold_point_spacing_m_ =
    declare_parameter<double>("stationary_hold_point_spacing_m", 0.03);
  publish_standalone_local_ =
    declare_parameter<bool>("publish_standalone_local", false);
  timer_period_ms_ = declare_parameter<int>("timer_period_ms", 50);
  debug_publish_period_ms_ = declare_parameter<int>("debug_publish_period_ms", 200);
  detection_confirm_cycles_ = declare_parameter<int>("detection_confirm_cycles", 1);
  detection_clear_cycles_ = declare_parameter<int>("detection_clear_cycles", 3);
  frenet_odom_confirm_cycles_ =
    declare_parameter<int>("frenet_odom_confirm_cycles", 3);
  frenet_odom_invalid_grace_cycles_ =
    declare_parameter<int>("frenet_odom_invalid_grace_cycles", 5);
  frenet_odom_stale_timeout_sec_ =
    declare_parameter<double>("frenet_odom_stale_timeout_sec", 0.50);
  frenet_odom_path_hold_timeout_sec_ =
    declare_parameter<double>("frenet_odom_path_hold_timeout_sec", 3.00);
  frenet_odom_max_s_jump_m_ =
    declare_parameter<double>("frenet_odom_max_s_jump_m", 1.00);
  frenet_odom_speed_jump_scale_ =
    declare_parameter<double>("frenet_odom_speed_jump_scale", 1.50);
  frenet_odom_jump_slack_m_ =
    declare_parameter<double>("frenet_odom_jump_slack_m", 0.25);
  frenet_odom_track_margin_m_ =
    declare_parameter<double>("frenet_odom_track_margin_m", 0.10);
  obstacle_marker_scale_ = declare_parameter<double>("obstacle_marker_scale", 0.35);
  path_marker_width_ = declare_parameter<double>("path_marker_width", 0.08);
  corridor_debug_stride_ = declare_parameter<int>("corridor_debug_stride", 2);

  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  obstacles_topic_ = declare_parameter<std::string>(
    "obstacles_topic", "/perception/obstacles");
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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
<<<<<<< HEAD
=======
  lattice_recovery_min_transition_wpnts_ = std::max(
    2, std::min(lattice_recovery_min_transition_wpnts_, lattice_min_transition_wpnts_));
  lattice_recovery_profile_limit_ = std::clamp(
    lattice_recovery_profile_limit_, 1, lattice_corridor_beam_width_);
  lattice_primary_search_budget_ms_ = std::clamp(
    lattice_primary_search_budget_ms_, 20, 1000);
  lattice_recovery_search_budget_ms_ = std::clamp(
    lattice_recovery_search_budget_ms_, 20, 1000);
  lattice_max_result_pose_drift_m_ = std::clamp(
    lattice_max_result_pose_drift_m_, 0.10, 2.00);
  lattice_recovery_max_curvature_scale_ = std::max(
    1.0, lattice_recovery_max_curvature_scale_);
  lattice_recovery_max_curvature_rate_scale_ = std::max(
    1.0, lattice_recovery_max_curvature_rate_scale_);
  lattice_max_curvature_radpm_ = std::max(0.1, lattice_max_curvature_radpm_);
  lattice_max_curvature_rate_radpm2_ = std::max(
    0.1, lattice_max_curvature_rate_radpm2_);
  lattice_max_lateral_accel_mps2_ = std::max(0.1, lattice_max_lateral_accel_mps2_);
  lattice_speed_safety_factor_ = std::clamp(lattice_speed_safety_factor_, 0.1, 1.0);
  lattice_collision_sample_step_m_ =
    std::max(0.01, lattice_collision_sample_step_m_);
  lattice_preferred_clearance_m_ = std::max(
    vehicle_radius_ + path_clearance_margin_, lattice_preferred_clearance_m_);
  lattice_weight_clearance_ = std::max(0.0, lattice_weight_clearance_);
  lattice_weight_min_clearance_ = std::max(0.0, lattice_weight_min_clearance_);
  lattice_weight_deviation_ = std::max(0.0, lattice_weight_deviation_);
  lattice_weight_curvature_ = std::max(0.0, lattice_weight_curvature_);
  lattice_weight_curvature_change_ =
    std::max(0.0, lattice_weight_curvature_change_);
  lattice_weight_spatial_lateral_jerk_ =
    std::max(0.0, lattice_weight_spatial_lateral_jerk_);
  lattice_weight_maneuver_length_ =
    std::max(0.0, lattice_weight_maneuver_length_);
  lattice_weight_path_length_ = std::max(0.0, lattice_weight_path_length_);
  lattice_weight_speed_loss_ = std::max(0.0, lattice_weight_speed_loss_);
  lattice_weight_opposite_side_ = std::max(0.0, lattice_weight_opposite_side_);
  lattice_preplan_before_merge_m_ = std::max(0.0, lattice_preplan_before_merge_m_);
  lattice_preplan_max_start_gap_m_ = std::max(0.05, lattice_preplan_max_start_gap_m_);
  lattice_post_merge_lookahead_wpnts_ =
    std::max(0, lattice_post_merge_lookahead_wpnts_);
  lattice_merge_lateral_tolerance_m_ = std::max(
    0.01, lattice_merge_lateral_tolerance_m_);
  lattice_merge_settle_max_wpnts_ = std::max(0, lattice_merge_settle_max_wpnts_);
  lattice_safe_stop_buffer_wpnts_ = std::max(0, lattice_safe_stop_buffer_wpnts_);
  lattice_safe_stop_deceleration_mps2_ = std::max(
    0.10, lattice_safe_stop_deceleration_mps2_);
  lattice_replan_brake_timeout_sec_ = std::max(
    0.10, lattice_replan_brake_timeout_sec_);
  stationary_hold_point_spacing_m_ = std::clamp(
    stationary_hold_point_spacing_m_, 0.01, 0.10);
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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
<<<<<<< HEAD
  if (!static_obstacles_only_) {
=======
  const std::size_t expected_cell_count = msg ?
    static_cast<std::size_t>(msg->info.width) * msg->info.height : 0U;
  if (!msg || msg->info.width == 0 || msg->info.height == 0 ||
    msg->info.resolution <= 0.0 || msg->data.size() != expected_cell_count)
  {
    RCLCPP_WARN(get_logger(), "Received an invalid occupancy grid.");
    return;
  }

  // Map servers may periodically republish an unchanged latched map. Avoid
  // repeating the connected-component pass when only the message stamp changed.
  uint64_t signature = 1469598103934665603ULL;
  const auto mix = [&signature](const uint64_t value) {
      signature ^= value;
      signature *= 1099511628211ULL;
    };
  mix(msg->info.width);
  mix(msg->info.height);
  mix(static_cast<uint64_t>(std::llround(msg->info.resolution * 1e9)));
  mix(static_cast<uint64_t>(std::llround(msg->info.origin.position.x * 1e6)));
  mix(static_cast<uint64_t>(std::llround(msg->info.origin.position.y * 1e6)));
  mix(static_cast<uint64_t>(std::llround(yawFromQuaternion(msg->info.origin.orientation) * 1e9)));
  for (const int8_t value : msg->data) {
    mix(static_cast<uint8_t>(value));
  }
  if (has_map_signature_ && signature == map_signature_) {
    return;
  }
  map_signature_ = signature;
  has_map_signature_ = true;

  base_grid_map_ = *msg;
  grid_map_ = base_grid_map_;
  grid_map_.data.clear();
  map_origin_yaw_ = yawFromQuaternion(grid_map_.info.origin.orientation);
  map_origin_cos_ = std::cos(map_origin_yaw_);
  map_origin_sin_ = std::sin(map_origin_yaw_);
  has_map_ = true;
  grid_obstacle_snapshot_.clear();
  grid_obstacle_history_.clear();
  grid_obstacle_expand_counts_.clear();
  grid_obstacle_shrink_counts_.clear();
  invalidateSafeCorridorCache();
  clearAvoidanceCommitment();
  has_held_avoidance_segment_ = false;
  held_avoidance_segment_.wpnts.clear();
  has_last_validated_full_path_ = false;
  last_validated_full_path_.wpnts.clear();
  rebuildPlanningGrid();
}

void LocalPlannerNode::onObstacles(
  const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto update_time = now();
  const double track_length = has_global_ && !global_wpnts_.wpnts.empty() ?
    global_wpnts_.wpnts.back().s_m : 0.0;
  const auto signed_s_delta = [track_length](const double target, const double reference) {
      double delta = target - reference;
      if (track_length > 1e-6) {
        delta = std::remainder(delta, track_length);
      }
      return delta;
    };

  // Associate nearby static detections with one persistent track. Multiple
  // detections of the same physical object in one message therefore collapse
  // into one filtered obstacle instead of producing competing map boxes.
  for (const auto & measurement : msg->obstacles) {
    if (measurement.is_actually_a_gap ||
      (perception_static_only_ && !isPlanningStaticObstacle(measurement)))
    {
      continue;
    }
    int best_index = -1;
    double best_distance = perception_association_gate_m_;
    for (std::size_t i = 0; i < filtered_obstacle_tracks_.size(); ++i) {
      const auto & tracked = filtered_obstacle_tracks_[i].obstacle;
      const double ds = signed_s_delta(measurement.s_center, tracked.s_center);
      const double dd = measurement.d_center - tracked.d_center;
      const double distance = std::hypot(ds, dd);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = static_cast<int>(i);
      }
    }

    if (best_index < 0) {
      filtered_obstacle_tracks_.push_back({measurement, update_time});
      continue;
    }

    auto & track = filtered_obstacle_tracks_[static_cast<std::size_t>(best_index)];
    const auto previous = track.obstacle;
    const double raw_ds = signed_s_delta(measurement.s_center, previous.s_center);
    const double raw_dd = measurement.d_center - previous.d_center;
    const double filtered_ds = perception_filter_alpha_ * std::clamp(
      raw_ds, -perception_max_s_step_m_, perception_max_s_step_m_);
    const double filtered_dd = perception_filter_alpha_ * std::clamp(
      raw_dd, -perception_max_d_step_m_, perception_max_d_step_m_);
    const double half_length = std::max(
      0.5 * std::abs(measurement.s_end - measurement.s_start),
      0.5 * measurement.size);
    const double right_extent = std::max(
      0.0, measurement.d_center - measurement.d_right);
    const double left_extent = std::max(
      0.0, measurement.d_left - measurement.d_center);

    track.obstacle = measurement;
    track.obstacle.s_center = previous.s_center + filtered_ds;
    if (track_length > 1e-6) {
      track.obstacle.s_center = std::fmod(track.obstacle.s_center, track_length);
      if (track.obstacle.s_center < 0.0) {
        track.obstacle.s_center += track_length;
      }
    }
    track.obstacle.d_center = previous.d_center + filtered_dd;
    track.obstacle.s_start = track.obstacle.s_center - half_length;
    track.obstacle.s_end = track.obstacle.s_center + half_length;
    track.obstacle.d_right = track.obstacle.d_center - right_extent;
    track.obstacle.d_left = track.obstacle.d_center + left_extent;
    track.obstacle.is_visible = true;
    track.last_seen = update_time;
  }

  filtered_obstacle_tracks_.erase(
    std::remove_if(
      filtered_obstacle_tracks_.begin(), filtered_obstacle_tracks_.end(),
      [&](const FilteredObstacleTrack & track) {
        return (update_time - track.last_seen).seconds() > perception_track_hold_sec_;
      }),
    filtered_obstacle_tracks_.end());
  perceived_obstacles_.header = msg->header;
  perceived_obstacles_.obstacles.clear();
  perceived_obstacles_.obstacles.reserve(filtered_obstacle_tracks_.size());
  for (const auto & track : filtered_obstacle_tracks_) {
    perceived_obstacles_.obstacles.push_back(track.obstacle);
  }
  const bool first_obstacle_message = !has_obstacles_;
  has_obstacles_ = true;
  last_perception_receive_time_ = update_time;
  if (!perceived_obstacles_.obstacles.empty()) {
    const double ego_s = has_odom_ ? current_odom_.pose.pose.position.x :
      std::numeric_limits<double>::quiet_NaN();
    const auto forward_s_delta = [&](const double target) {
        double delta = target - ego_s;
        if (track_length > 1e-6) {
          delta = std::fmod(delta, track_length);
          if (delta < 0.0) {
            delta += track_length;
          }
        }
        return delta;
      };
    const auto nearest = std::min_element(
      perceived_obstacles_.obstacles.begin(), perceived_obstacles_.obstacles.end(),
      [&](const auto & lhs, const auto & rhs) {
        return forward_s_delta(lhs.s_start) < forward_s_delta(rhs.s_start);
      });
    const double forward_distance = forward_s_delta(nearest->s_start);
    const bool first_static_detection = !diagnostic_static_obstacle_seen_;
    diagnostic_static_obstacle_seen_ = true;
    static rclcpp::Clock perception_diagnostic_clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(
      get_logger(), perception_diagnostic_clock, 1000,
      "DIAG perception: first_static=%s ego_s=%.3f obstacle_id=%d "
      "s=[%.3f, %.3f] center=%.3f d=[%.3f, %.3f] forward=%.3f m "
      "static=%s visible=%s tracks=%zu.",
      first_static_detection ? "true" : "false", ego_s, nearest->id,
      nearest->s_start, nearest->s_end, nearest->s_center,
      nearest->d_right, nearest->d_left, forward_distance,
      nearest->is_static ? "true" : "false",
      nearest->is_visible ? "true" : "false",
      perceived_obstacles_.obstacles.size());
  }
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Received %zu tracked obstacles from %s.",
    perceived_obstacles_.obstacles.size(), obstacles_topic_.c_str());
  const bool update_due = first_obstacle_message ||
    last_perception_grid_update_time_.nanoseconds() == 0 ||
    (update_time - last_perception_grid_update_time_).seconds() >=
    perception_grid_update_period_sec_;
  if (has_map_ && has_global_ && update_due) {
    last_perception_grid_update_time_ = update_time;
    if (perception_static_only_) {
      bool canonical_geometry_changed = false;
      for (const auto & current : perceived_obstacles_.obstacles) {
        const auto interval_gap = [](double a_min, double a_max, double b_min, double b_max) {
            if (a_min > a_max) {
              std::swap(a_min, a_max);
            }
            if (b_min > b_max) {
              std::swap(b_min, b_max);
            }
            return std::max({0.0, a_min - b_max, b_min - a_max});
          };
        const auto canonical = std::find_if(
          grid_obstacle_snapshot_.begin(), grid_obstacle_snapshot_.end(),
          [&](const auto & frozen) {
            const double center_distance = std::hypot(
              signed_s_delta(current.s_center, frozen.s_center),
              current.d_center - frozen.d_center);
            const double interval_match_gate = std::max(
              0.30, 2.5 * perception_grid_rebuild_motion_m_);
            return center_distance <= perception_static_grid_match_gate_m_ &&
            interval_gap(
              current.s_start, current.s_end,
              frozen.s_start, frozen.s_end) <= interval_match_gate &&
            interval_gap(
              current.d_right, current.d_left,
              frozen.d_right, frozen.d_left) <= interval_match_gate;
          });
        if (canonical == grid_obstacle_snapshot_.end()) {
          grid_obstacle_snapshot_.push_back(current);
          grid_obstacle_history_.emplace_back(1, current);
          grid_obstacle_expand_counts_.push_back(0);
          grid_obstacle_shrink_counts_.push_back(0);
          canonical_geometry_changed = true;
        } else {
          const auto canonical_index = static_cast<std::size_t>(
            std::distance(grid_obstacle_snapshot_.begin(), canonical));
          if (grid_obstacle_history_.size() < grid_obstacle_snapshot_.size()) {
            grid_obstacle_history_.resize(grid_obstacle_snapshot_.size());
          }
          if (grid_obstacle_shrink_counts_.size() < grid_obstacle_snapshot_.size()) {
            grid_obstacle_shrink_counts_.resize(grid_obstacle_snapshot_.size(), 0);
          }
          if (grid_obstacle_expand_counts_.size() < grid_obstacle_snapshot_.size()) {
            grid_obstacle_expand_counts_.resize(grid_obstacle_snapshot_.size(), 0);
          }
          auto & history = grid_obstacle_history_[canonical_index];
          history.push_back(current);
          while (history.size() > static_cast<std::size_t>(perception_static_history_size_)) {
            history.pop_front();
          }

          // Use the median recent boundary instead of the moving min/max.
          // A single edge outlier can therefore neither grow nor shrink the
          // rasterized obstacle and force a full-map distance-field rebuild.
          std::vector<double> s_starts;
          std::vector<double> s_ends;
          std::vector<double> d_rights;
          std::vector<double> d_lefts;
          s_starts.reserve(history.size());
          s_ends.reserve(history.size());
          d_rights.reserve(history.size());
          d_lefts.reserve(history.size());
          for (const auto & observation : history) {
            s_starts.push_back(observation.s_start);
            s_ends.push_back(observation.s_end);
            d_rights.push_back(observation.d_right);
            d_lefts.push_back(observation.d_left);
          }
          const auto median = [](std::vector<double> values) {
              const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
              std::nth_element(values.begin(), middle, values.end());
              return *middle;
            };
          const double recent_s_start = median(std::move(s_starts));
          const double recent_s_end = median(std::move(s_ends));
          const double recent_d_right = median(std::move(d_rights));
          const double recent_d_left = median(std::move(d_lefts));
          const double threshold = perception_grid_rebuild_motion_m_;
          const bool expands =
            canonical->s_start - recent_s_start > threshold ||
            recent_s_end - canonical->s_end > threshold ||
            canonical->d_right - recent_d_right > threshold ||
            recent_d_left - canonical->d_left > threshold;
          const bool shrinks =
            recent_s_start - canonical->s_start > threshold ||
            canonical->s_end - recent_s_end > threshold ||
            recent_d_right - canonical->d_right > threshold ||
            canonical->d_left - recent_d_left > threshold;
          auto & expand_count = grid_obstacle_expand_counts_[canonical_index];
          auto & shrink_count = grid_obstacle_shrink_counts_[canonical_index];
          if (expands) {
            shrink_count = 0;
            ++expand_count;
            if (expand_count >= perception_static_expand_confirm_cycles_) {
              canonical->s_start = std::min(canonical->s_start, recent_s_start);
              canonical->s_end = std::max(canonical->s_end, recent_s_end);
              canonical->d_right = std::min(canonical->d_right, recent_d_right);
              canonical->d_left = std::max(canonical->d_left, recent_d_left);
              expand_count = 0;
              canonical_geometry_changed = true;
            }
          } else if (shrinks) {
            expand_count = 0;
            ++shrink_count;
            if (shrink_count >= perception_static_shrink_confirm_cycles_) {
              canonical->s_start = recent_s_start;
              canonical->s_end = recent_s_end;
              canonical->d_right = recent_d_right;
              canonical->d_left = recent_d_left;
              shrink_count = 0;
              canonical_geometry_changed = true;
            }
          } else {
            expand_count = 0;
            shrink_count = 0;
          }
          if (canonical_geometry_changed) {
            canonical->s_center = 0.5 * (canonical->s_start + canonical->s_end);
            canonical->d_center = 0.5 * (canonical->d_right + canonical->d_left);
            canonical->size = std::max(
              canonical->s_end - canonical->s_start,
              canonical->d_left - canonical->d_right);
          }
        }
      }
      if (canonical_geometry_changed) {
        rebuildPlanningGrid();
      } else {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Reusing canonical static planning grid with %zu physical obstacles.",
          grid_obstacle_snapshot_.size());
      }
      return;
    }
    const auto obstacle_extent_changed = [&](const auto & current, const auto & previous) {
        return std::abs(signed_s_delta(current.s_center, previous.s_center)) >
               perception_grid_rebuild_motion_m_ ||
               std::abs(current.d_center - previous.d_center) >
               perception_grid_rebuild_motion_m_ ||
               std::abs(current.s_start - previous.s_start) >
               perception_grid_rebuild_motion_m_ ||
               std::abs(current.s_end - previous.s_end) >
               perception_grid_rebuild_motion_m_ ||
               std::abs(current.d_right - previous.d_right) >
               perception_grid_rebuild_motion_m_ ||
               std::abs(current.d_left - previous.d_left) >
               perception_grid_rebuild_motion_m_;
      };
    bool geometry_changed =
      perceived_obstacles_.obstacles.size() != grid_obstacle_snapshot_.size();
    if (!geometry_changed) {
      for (const auto & current : perceived_obstacles_.obstacles) {
        const auto previous = std::find_if(
          grid_obstacle_snapshot_.begin(), grid_obstacle_snapshot_.end(),
          [&](const auto & candidate) {
            return std::hypot(
              signed_s_delta(current.s_center, candidate.s_center),
              current.d_center - candidate.d_center) <=
            perception_grid_rebuild_motion_m_;
          });
        if (previous == grid_obstacle_snapshot_.end() ||
          obstacle_extent_changed(current, *previous))
        {
          geometry_changed = true;
          break;
        }
      }
    }
    if (geometry_changed || grid_obstacle_snapshot_.empty()) {
      grid_obstacle_snapshot_ = perceived_obstacles_.obstacles;
      rebuildPlanningGrid();
    } else {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Reusing static planning grid: %zu tracks changed less than %.3f m.",
        grid_obstacle_snapshot_.size(), perception_grid_rebuild_motion_m_);
    }
  }
}

bool LocalPlannerNode::isPlanningStaticObstacle(
  const f110_msgs::msg::Obstacle & obstacle) const
{
  if (obstacle.is_static) {
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
    return true;
  }
  return obstacle.is_static ||
         (std::isfinite(obstacle.vs) && std::isfinite(obstacle.vd) &&
         std::hypot(obstacle.vs, obstacle.vd) <= static_speed_threshold_mps_);
}

std::optional<f110_msgs::msg::Obstacle> LocalPlannerNode::projectCartesianObstacle(
  const f110_msgs::msg::Obstacle & obstacle) const
{
<<<<<<< HEAD
  if (!clcs_converter_ || !obstacle.has_cartesian ||
    !std::isfinite(obstacle.x_center) || !std::isfinite(obstacle.y_center) ||
    !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0)
=======
  const auto & waypoints = global_wpnts_.wpnts;
  if (waypoints.size() < 2 || !std::isfinite(s) || !std::isfinite(d)) {
    return false;
  }
  const double max_s = waypoints.back().s_m;
  double wrapped_s = s;
  if (max_s > 0.0) {
    wrapped_s = std::fmod(s, max_s);
    if (wrapped_s < 0.0) {
      wrapped_s += max_s;
    }
  }
  const auto upper = std::lower_bound(
    waypoints.begin(), waypoints.end(), wrapped_s,
    [](const f110_msgs::msg::Wpnt & waypoint, const double value) {
      return waypoint.s_m < value;
    });
  const std::size_t next = upper == waypoints.end() ? 0U :
    static_cast<std::size_t>(std::distance(waypoints.begin(), upper));
  const std::size_t previous = next == 0U ? waypoints.size() - 1U : next - 1U;
  const auto & a = waypoints[previous];
  const auto & b = waypoints[next];
  double segment_s = b.s_m - a.s_m;
  double query_s = wrapped_s - a.s_m;
  if (next == 0U) {
    segment_s = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    query_s = wrapped_s + std::max(max_s, a.s_m) - a.s_m;
  }
  const double ratio = segment_s > 1e-6 ? std::clamp(query_s / segment_s, 0.0, 1.0) : 0.0;
  const double center_x = a.x_m + ratio * (b.x_m - a.x_m);
  const double center_y = a.y_m + ratio * (b.y_m - a.y_m);
  yaw = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
  x = center_x - d * std::sin(yaw);
  y = center_y + d * std::cos(yaw);
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

void LocalPlannerNode::rebuildPlanningGrid()
{
  if (!has_map_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid updated = base_grid_map_;
  std::size_t rasterized_obstacle_count = 0U;
  if (use_perception_obstacles_ && has_global_ && has_obstacles_) {
    const int width = static_cast<int>(updated.info.width);
    const int height = static_cast<int>(updated.info.height);
    const double resolution = updated.info.resolution;
    std::vector<f110_msgs::msg::Obstacle> planning_obstacles;
    // A committed path and the obstacle geometry used to validate it form one
    // immutable safety decision. Prefer that snapshot over the continuously
    // updated canonical tracker geometry until the commitment is cleared.
    const bool use_committed_snapshot = freeze_committed_static_obstacles_ &&
      has_avoidance_commitment_ && has_committed_obstacle_snapshot_;
    const bool use_canonical_static_snapshot = perception_static_only_ &&
      !grid_obstacle_snapshot_.empty();
    if (use_committed_snapshot) {
      planning_obstacles = committed_obstacle_snapshot_.obstacles;
      const double track_length = global_wpnts_.wpnts.empty() ? 0.0 :
        global_wpnts_.wpnts.back().s_m;
      const auto interval_gap = [](double a_min, double a_max, double b_min, double b_max) {
          if (a_min > a_max) {
            std::swap(a_min, a_max);
          }
          if (b_min > b_max) {
            std::swap(b_min, b_max);
          }
          return std::max({0.0, a_min - b_max, b_min - a_max});
        };
      for (const auto & current : perceived_obstacles_.obstacles) {
        const bool matches_snapshot = std::any_of(
          committed_obstacle_snapshot_.obstacles.begin(),
          committed_obstacle_snapshot_.obstacles.end(),
          [&](const f110_msgs::msg::Obstacle & frozen) {
            double ds = current.s_center - frozen.s_center;
            if (track_length > 1e-6) {
              ds = std::remainder(ds, track_length);
            }
            const double center_distance = std::hypot(ds, current.d_center - frozen.d_center);
            const double interval_match_gate = std::max(
              committed_obstacle_match_distance_m_,
              2.5 * perception_grid_rebuild_motion_m_);
            return center_distance <= perception_static_grid_match_gate_m_ &&
                   interval_gap(
                     current.s_start, current.s_end,
                     frozen.s_start, frozen.s_end) <= interval_match_gate &&
                   interval_gap(
                     current.d_right, current.d_left,
                     frozen.d_right, frozen.d_left) <= interval_match_gate;
          });
        if (!matches_snapshot) {
          planning_obstacles.push_back(current);
        }
      }
    } else if (use_canonical_static_snapshot) {
      planning_obstacles = grid_obstacle_snapshot_;
    } else {
      planning_obstacles = perceived_obstacles_.obstacles;
    }

    for (const auto & obstacle : planning_obstacles) {
      // The tracker deliberately keeps a missed static track for ttl_static
      // frames. Keep that last observation in the grid instead of making a
      // stationary obstacle blink whenever one scan is occluded.
      if ((!obstacle.is_visible && !obstacle.is_static) || obstacle.is_actually_a_gap ||
        (perception_static_only_ && !isPlanningStaticObstacle(obstacle)))
      {
        continue;
      }
      double center_x = 0.0;
      double center_y = 0.0;
      double yaw = 0.0;
      if (!obstacleCenterPose(obstacle.s_center, obstacle.d_center, center_x, center_y, yaw)) {
        continue;
      }
      ++rasterized_obstacle_count;
      const double longitudinal_span = std::abs(obstacle.s_end - obstacle.s_start);
      const double lateral_span = std::abs(obstacle.d_left - obstacle.d_right);
      const bool valid_longitudinal_span =
        std::isfinite(longitudinal_span) && longitudinal_span > 1e-3;
      const bool valid_lateral_span =
        std::isfinite(lateral_span) && lateral_span > 1e-3;
      const double fallback_half_size = std::isfinite(obstacle.size) ?
        0.5 * std::max(0.0, obstacle.size) : 0.0;
      const double half_length =
        (valid_longitudinal_span ? 0.5 * longitudinal_span : fallback_half_size) +
        perception_obstacle_padding_m_;
      const double half_width =
        (valid_lateral_span ? 0.5 * lateral_span : fallback_half_size) +
        perception_obstacle_padding_m_;
      const double radius = std::hypot(half_length, half_width);
      int center_column = 0;
      int center_row = 0;
      if (!worldToMap(center_x, center_y, center_column, center_row)) {
        continue;
      }
      const int cell_radius = static_cast<int>(std::ceil(radius / resolution)) + 1;
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      for (int row = std::max(0, center_row - cell_radius);
        row <= std::min(height - 1, center_row + cell_radius); ++row)
      {
        for (int column = std::max(0, center_column - cell_radius);
          column <= std::min(width - 1, center_column + cell_radius); ++column)
        {
          const auto point = mapCellCenter(column, row);
          const double dx = point.x - center_x;
          const double dy = point.y - center_y;
          const double longitudinal = cosine * dx + sine * dy;
          const double lateral = -sine * dx + cosine * dy;
          if (std::abs(longitudinal) <= half_length && std::abs(lateral) <= half_width) {
            updated.data[static_cast<std::size_t>(row) * width + column] = 100;
          }
        }
      }
    }
  }
  if (updated.data == grid_map_.data && updated.info.width == grid_map_.info.width &&
    updated.info.height == grid_map_.info.height)
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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

<<<<<<< HEAD
  EgoFrenetState ego;
  ego.s = odometry.pose.pose.position.x;
  ego.d = odometry.pose.pose.position.y;
  ego.speed = std::abs(odometry.twist.twist.linear.x);
  if (commitmentComplete(ego)) {
    RCLCPP_INFO(get_logger(), "Static avoidance completed and merged onto the global race line.");
    clearCommitment();
    publishEmpty("avoidance merge complete");
=======
  for (int k = window_begin; k <= window_end; ++k) {
    const int index = (start_idx + k) % total;
    const int previous = (index - 1 + total) % total;
    const int next = (index + 1) % total;
    candidate.wpnts[index].psi_rad = std::atan2(
      candidate.wpnts[next].y_m - candidate.wpnts[previous].y_m,
      candidate.wpnts[next].x_m - candidate.wpnts[previous].x_m);
  }

  for (int k = window_begin; k <= window_end; ++k) {
    const int index = (start_idx + k) % total;
    const int previous = (index - 1 + total) % total;
    const int next = (index + 1) % total;
    const auto & p0 = candidate.wpnts[previous];
    const auto & p1 = candidate.wpnts[index];
    const auto & p2 = candidate.wpnts[next];
    const double ax = p1.x_m - p0.x_m;
    const double ay = p1.y_m - p0.y_m;
    const double bx = p2.x_m - p1.x_m;
    const double by = p2.y_m - p1.y_m;
    const double a = std::hypot(ax, ay);
    const double b = std::hypot(bx, by);
    const double c = std::hypot(p2.x_m - p0.x_m, p2.y_m - p0.y_m);
    const double denominator = a * b * c;
    candidate.wpnts[index].kappa_radpm = denominator > 1e-9 ?
      2.0 * (ax * by - ay * bx) / denominator : 0.0;
  }
}

void LocalPlannerNode::updateCandidateVelocity(
  f110_msgs::msg::WpntArray & candidate,
  const int start_idx, const int window_begin,
  const int collision_end, const int window_end) const
{
  const int total = static_cast<int>(candidate.wpnts.size());
  double avoidance_offset = 0.05;
  for (int k = window_begin; k <= collision_end; ++k) {
    const int index = (start_idx + k) % total;
    avoidance_offset = std::max(
      avoidance_offset, std::abs(candidate.wpnts[index].d_m));
  }

  // Generate the velocity profile from the current planning point, not only
  // inside the modified geometry. The backward pass can then start braking
  // before the lateral transition and avoids a speed discontinuity at its
  // first waypoint.
  for (int k = 0; k <= window_end; ++k) {
    const int index = (start_idx + k) % total;
    const auto & reference = global_wpnts_.wpnts[index];
    auto & waypoint = candidate.wpnts[index];
    double target_speed = reference.vx_mps;
    if (k >= window_begin && std::abs(waypoint.d_m) > 0.05) {
      const double avoidance_speed_ratio = std::abs(reference.kappa_radpm) >=
        corner_curvature_threshold_ ?
        std::min(speed_reduction_ratio_, corner_speed_reduction_ratio_) :
        speed_reduction_ratio_;
      double speed_ratio = avoidance_speed_ratio;
      if (k > collision_end) {
        const double recovery_progress = 1.0 - std::clamp(
          std::abs(waypoint.d_m) / avoidance_offset, 0.0, 1.0);
        const double recovery_blend = smoothstepQuintic(recovery_progress);
        speed_ratio = avoidance_speed_ratio +
          (post_obstacle_speed_recovery_ratio_ - avoidance_speed_ratio) *
          recovery_blend;
      }
      target_speed *= speed_ratio;
    }
    const double curvature = std::abs(waypoint.kappa_radpm);
    if (curvature > 1e-6) {
      const double lateral_acceleration_cap = lattice_speed_safety_factor_ *
        std::sqrt(lattice_max_lateral_accel_mps2_ / curvature);
      target_speed = std::min(target_speed, lateral_acceleration_cap);
    }
    waypoint.vx_mps = std::max(0.0, target_speed);
  }

  // Forward acceleration pass: recover speed quickly enough for lap time, but
  // never command a step larger than the configured longitudinal capability.
  for (int k = 1; k <= window_end; ++k) {
    const int previous = (start_idx + k - 1) % total;
    const int index = (start_idx + k) % total;
    const double ds = std::max(
      1e-3, std::hypot(
        candidate.wpnts[index].x_m - candidate.wpnts[previous].x_m,
        candidate.wpnts[index].y_m - candidate.wpnts[previous].y_m));
    const double reachable_speed = std::sqrt(
      candidate.wpnts[previous].vx_mps * candidate.wpnts[previous].vx_mps +
      2.0 * lattice_max_longitudinal_accel_mps2_ * ds);
    candidate.wpnts[index].vx_mps = std::min(
      candidate.wpnts[index].vx_mps, reachable_speed);
  }

  // Backward braking pass: make every future low-speed point reachable before
  // the obstacle or a high-curvature section.
  for (int k = window_end - 1; k >= 0; --k) {
    const int index = (start_idx + k) % total;
    const int next = (start_idx + k + 1) % total;
    const double ds = std::max(
      1e-3, std::hypot(
        candidate.wpnts[next].x_m - candidate.wpnts[index].x_m,
        candidate.wpnts[next].y_m - candidate.wpnts[index].y_m));
    const double reachable_speed = std::sqrt(
      candidate.wpnts[next].vx_mps * candidate.wpnts[next].vx_mps +
      2.0 * lattice_max_longitudinal_decel_mps2_ * ds);
    candidate.wpnts[index].vx_mps = std::min(
      candidate.wpnts[index].vx_mps, reachable_speed);
  }

  for (int k = 0; k < window_end; ++k) {
    const int index = (start_idx + k) % total;
    const int next = (index + 1) % total;
    const double ds = std::max(
      1e-3, std::hypot(
        candidate.wpnts[next].x_m - candidate.wpnts[index].x_m,
        candidate.wpnts[next].y_m - candidate.wpnts[index].y_m));
    candidate.wpnts[index].ax_mps2 =
      (candidate.wpnts[next].vx_mps * candidate.wpnts[next].vx_mps -
      candidate.wpnts[index].vx_mps * candidate.wpnts[index].vx_mps) / (2.0 * ds);
  }
  const int end_index = (start_idx + window_end) % total;
  const int previous_index = (end_index - 1 + total) % total;
  candidate.wpnts[end_index].ax_mps2 = candidate.wpnts[previous_index].ax_mps2;
}

bool LocalPlannerNode::evaluateLatticeCandidate(
  LatticeCandidate & candidate,
  const int start_idx, const int window_begin, const int window_end,
  const bool preferred_left, const bool recovery_mode) const
{
  const int total = static_cast<int>(candidate.path.wpnts.size());
  const int sample_count = window_end - window_begin + 1;
  if (sample_count < 2 || total < 3) {
    ++rejection_counts_.invalid_geometry;
    return false;
  }

  const double collision_clearance = vehicle_radius_ + path_clearance_margin_;
  double clearance_cost = 0.0;
  double deviation_cost = 0.0;
  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double speed_loss_cost = 0.0;
  double candidate_length = 0.0;
  double reference_length = 0.0;
  double previous_curvature = 0.0;
  double previous_reference_curvature = 0.0;
  bool has_previous_curvature = false;
  int clearance_sample_count = 0;
  candidate.minimum_clearance = std::numeric_limits<double>::infinity();
  candidate.maximum_curvature = 0.0;
  candidate.maximum_curvature_rate = 0.0;

  for (int k = window_begin; k <= window_end; ++k) {
    const int index = (start_idx + k) % total;
    const auto & waypoint = candidate.path.wpnts[index];
    const auto & reference = global_wpnts_.wpnts[index];
    const double heading_error = std::remainder(
      waypoint.psi_rad - reference.psi_rad, 2.0 * kPi);
    const double footprint_margin = localization_margin_m_ + corridor_safety_margin_m_ +
      path_clearance_margin_;
    const double footprint_lateral_extent =
      (std::max(vehicle_front_extent_m_, vehicle_rear_extent_m_) + footprint_margin) *
      std::abs(std::sin(heading_error)) +
      (0.5 * vehicle_width_m_ + footprint_margin) *
      std::abs(std::cos(heading_error));
    const double track_lateral_support = std::max(
      collision_clearance, footprint_lateral_extent);
    if (waypoint.d_m + track_lateral_support > reference.d_left ||
      waypoint.d_m - track_lateral_support < -reference.d_right)
    {
      if (rejection_counts_.first_track_offset < 0) {
        rejection_counts_.first_track_offset = k;
        rejection_counts_.first_track_d = waypoint.d_m;
        rejection_counts_.first_track_heading_error = heading_error;
        rejection_counts_.first_track_support = track_lateral_support;
        rejection_counts_.first_track_min_d = -reference.d_right;
        rejection_counts_.first_track_max_d = reference.d_left;
      }
      ++rejection_counts_.track_boundary;
      return false;
    }
    if (!isCandidateInsideSafeCorridor(k, waypoint.d_m)) {
      ++rejection_counts_.corridor;
      return false;
    }
    const double clearance = clearanceAtWorld(waypoint.x_m, waypoint.y_m);
    candidate.minimum_clearance = std::min(candidate.minimum_clearance, clearance);
    const double free_margin = std::max(0.02, clearance - collision_clearance);
    clearance_cost += 1.0 / free_margin;
    ++clearance_sample_count;
    deviation_cost += waypoint.d_m * waypoint.d_m;

    const double curvature = std::abs(waypoint.kappa_radpm);
    candidate.maximum_curvature = std::max(candidate.maximum_curvature, curvature);
    // Do not reject a path merely for following an already-sharp reference
    // corner. The configured limit remains the ordinary hard limit, while a
    // hairpin may use up to 15% more curvature than the reference at that
    // exact station. This still rejects a sharp avoidance kink and leaves the
    // lateral-acceleration and collision checks unchanged.
    const double recovery_scale = recovery_mode ?
      lattice_recovery_max_curvature_scale_ : 1.0;
    const double reference_curvature = std::abs(reference.kappa_radpm);
    const double maximum_curvature = std::max(
      lattice_max_curvature_radpm_ * recovery_scale,
      reference_curvature * 1.15);
    if (!std::isfinite(curvature) || curvature > maximum_curvature) {
      if (rejection_counts_.first_curvature_offset < 0) {
        rejection_counts_.first_curvature_offset = k;
        rejection_counts_.first_curvature = curvature;
        rejection_counts_.first_curvature_limit = maximum_curvature;
        rejection_counts_.first_curvature_d = waypoint.d_m;
      }
      ++rejection_counts_.curvature;
      return false;
    }
    const double lateral_acceleration = waypoint.vx_mps * waypoint.vx_mps * curvature;
    if (!std::isfinite(lateral_acceleration) ||
      lateral_acceleration > lattice_max_lateral_accel_mps2_ * 1.001)
    {
      ++rejection_counts_.lateral_acceleration;
      return false;
    }
    curvature_cost += curvature * curvature;
    if (has_previous_curvature) {
      const double curvature_change = waypoint.kappa_radpm - previous_curvature;
      const int previous_index = (index - 1 + total) % total;
      const auto & previous_waypoint = candidate.path.wpnts[previous_index];
      const double ds = std::max(
        1e-3, std::hypot(
          waypoint.x_m - previous_waypoint.x_m,
          waypoint.y_m - previous_waypoint.y_m));
      const double curvature_rate = std::abs(curvature_change) / ds;
      const double reference_curvature_change =
        reference.kappa_radpm - previous_reference_curvature;
      const double reference_curvature_rate =
        std::abs(reference_curvature_change) / ds;
      const double maximum_curvature_rate = std::max(
        lattice_max_curvature_rate_radpm2_ *
        (recovery_mode ? lattice_recovery_max_curvature_rate_scale_ : 1.0),
        reference_curvature_rate + 4.0);
      candidate.maximum_curvature_rate = std::max(
        candidate.maximum_curvature_rate, curvature_rate);
      if (!std::isfinite(curvature_rate) || curvature_rate > maximum_curvature_rate) {
        if (rejection_counts_.first_curvature_rate_offset < 0) {
          rejection_counts_.first_curvature_rate_offset = k;
          rejection_counts_.first_curvature_rate = curvature_rate;
          rejection_counts_.first_curvature_rate_limit = maximum_curvature_rate;
        }
        ++rejection_counts_.curvature_rate;
        return false;
      }
      curvature_change_cost += curvature_rate * curvature_rate;
    }
    previous_curvature = waypoint.kappa_radpm;
    previous_reference_curvature = reference.kappa_radpm;
    has_previous_curvature = true;

    if (reference.vx_mps > 0.05) {
      const double relative_loss = std::max(
        0.0, (reference.vx_mps - waypoint.vx_mps) / reference.vx_mps);
      speed_loss_cost += relative_loss * relative_loss;
    }

    CollisionRejectionReason collision_reason = CollisionRejectionReason::kNone;
    if (!isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad,
        collision_clearance, &collision_reason))
    {
      if (rejection_counts_.first_collision_offset < 0) {
        rejection_counts_.first_collision_offset = k;
        rejection_counts_.first_collision_d = waypoint.d_m;
        rejection_counts_.first_collision_x = waypoint.x_m;
        rejection_counts_.first_collision_y = waypoint.y_m;
        rejection_counts_.first_collision_yaw = waypoint.psi_rad;
      }
      countCollisionRejection(collision_reason);
      return false;
    }
    if (k < window_end) {
      const int next = (index + 1) % total;
      const auto & next_waypoint = candidate.path.wpnts[next];
      double segment_clearance_sum = 0.0;
      int segment_clearance_samples = 0;
      double segment_minimum_clearance = std::numeric_limits<double>::infinity();
      if (!isPathSegmentCollisionFree(
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad,
          next_waypoint.x_m, next_waypoint.y_m, next_waypoint.psi_rad,
          collision_clearance, &collision_reason,
          &segment_clearance_sum, &segment_clearance_samples,
          &segment_minimum_clearance))
      {
        if (rejection_counts_.first_collision_offset < 0) {
          rejection_counts_.first_collision_offset = k;
          rejection_counts_.first_collision_d = waypoint.d_m;
          rejection_counts_.first_collision_x = waypoint.x_m;
          rejection_counts_.first_collision_y = waypoint.y_m;
          rejection_counts_.first_collision_yaw = waypoint.psi_rad;
        }
        countCollisionRejection(collision_reason);
        return false;
      }
      clearance_cost += segment_clearance_sum;
      clearance_sample_count += segment_clearance_samples;
      candidate.minimum_clearance = std::min(
        candidate.minimum_clearance, segment_minimum_clearance);
      candidate_length += std::hypot(
        next_waypoint.x_m - waypoint.x_m, next_waypoint.y_m - waypoint.y_m);
      const auto & next_reference = global_wpnts_.wpnts[next];
      reference_length += std::hypot(
        next_reference.x_m - reference.x_m, next_reference.y_m - reference.y_m);
    }
  }

  const double normalizer = 1.0 / static_cast<double>(sample_count);
  const double clearance_normalizer = 1.0 /
    static_cast<double>(std::max(1, clearance_sample_count));
  const double path_length_increase = std::max(0.0, candidate_length - reference_length);
  const double preferred_free_margin = std::max(
    0.02, lattice_preferred_clearance_m_ - collision_clearance);
  const double minimum_clearance_deficit = std::max(
    0.0, lattice_preferred_clearance_m_ - candidate.minimum_clearance);
  const double normalized_minimum_clearance_deficit =
    minimum_clearance_deficit / preferred_free_margin;
  candidate.cost =
    lattice_weight_clearance_ * clearance_cost * clearance_normalizer +
    lattice_weight_min_clearance_ * normalized_minimum_clearance_deficit *
    normalized_minimum_clearance_deficit +
    lattice_weight_deviation_ * deviation_cost * normalizer +
    lattice_weight_curvature_ * curvature_cost * normalizer +
    lattice_weight_curvature_change_ * curvature_change_cost * normalizer +
    lattice_weight_spatial_lateral_jerk_ * candidate.spatial_lateral_jerk_cost +
    lattice_weight_maneuver_length_ * candidate.maneuver_length_m +
    lattice_weight_path_length_ * path_length_increase +
    lattice_weight_speed_loss_ * speed_loss_cost * normalizer +
    (candidate.avoid_left == preferred_left ? 0.0 : lattice_weight_opposite_side_);
  return std::isfinite(candidate.cost);
}

bool LocalPlannerNode::buildSafeStopSegment(
  const int start_idx, const int count,
  const std::vector<int> & collision_indices,
  f110_msgs::msg::WpntArray & segment) const
{
  if (collision_indices.empty() || global_wpnts_.wpnts.size() < 2) {
    return false;
  }

  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  const int collision_begin = std::clamp(collision_indices.front(), 0, count - 1);
  const double clearance = vehicle_radius_ + path_clearance_margin_;
  int collision_free_count = 0;
  for (int k = 0; k < collision_begin; ++k) {
    const int index = (start_idx + k) % total;
    const auto & waypoint = global_wpnts_.wpnts[index];
    if (!isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, clearance))
    {
      break;
    }
    if (k > 0) {
      const int previous = (index - 1 + total) % total;
      const auto & previous_waypoint = global_wpnts_.wpnts[previous];
      if (!isPathSegmentCollisionFree(
          previous_waypoint.x_m, previous_waypoint.y_m, previous_waypoint.psi_rad,
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad, clearance))
      {
        break;
      }
    }
    collision_free_count = k + 1;
  }

  const int desired_count = collision_begin - lattice_safe_stop_buffer_wpnts_;
  const int stop_count = std::min(collision_free_count, std::max(2, desired_count));
  if (stop_count < 2) {
    return false;
  }

  segment = makeForwardSegment(global_wpnts_, start_idx, stop_count);
  applyBrakingProfile(segment);
  return true;
}

bool LocalPlannerNode::buildReplanBrakingSegment(
  const f110_msgs::msg::WpntArray & source,
  const int start_idx, const int count,
  f110_msgs::msg::WpntArray & segment) const
{
  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (total < 3 || static_cast<int>(source.wpnts.size()) != total ||
    start_idx < 0 || start_idx >= total || count < 2)
  {
    return false;
  }

  const int maximum_count = std::min(count, total);
  const double collision_clearance = vehicle_radius_ + path_clearance_margin_;
  int collision_free_count = 0;
  for (int k = 0; k < maximum_count; ++k) {
    const int index = (start_idx + k) % total;
    const auto & waypoint = source.wpnts[index];
    const auto & reference = global_wpnts_.wpnts[index];
    const double heading_error = std::remainder(
      waypoint.psi_rad - reference.psi_rad, 2.0 * kPi);
    const double footprint_margin = localization_margin_m_ + corridor_safety_margin_m_ +
      path_clearance_margin_;
    const double footprint_lateral_extent =
      (std::max(vehicle_front_extent_m_, vehicle_rear_extent_m_) + footprint_margin) *
      std::abs(std::sin(heading_error)) +
      (0.5 * vehicle_width_m_ + footprint_margin) *
      std::abs(std::cos(heading_error));
    const double track_lateral_support = std::max(
      collision_clearance, footprint_lateral_extent);
    if (waypoint.d_m + track_lateral_support > reference.d_left ||
      waypoint.d_m - track_lateral_support < -reference.d_right ||
      !isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, collision_clearance))
    {
      break;
    }
    if (k > 0) {
      const int previous = (index - 1 + total) % total;
      const auto & previous_waypoint = source.wpnts[previous];
      if (!isPathSegmentCollisionFree(
          previous_waypoint.x_m, previous_waypoint.y_m, previous_waypoint.psi_rad,
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad, collision_clearance))
      {
        break;
      }
    }
    collision_free_count = k + 1;
  }

  if (collision_free_count < 2) {
    return false;
  }
  const int desired_count = collision_free_count - lattice_safe_stop_buffer_wpnts_;
  const int stop_count = std::min(collision_free_count, std::max(2, desired_count));
  segment = makeForwardSegment(source, start_idx, stop_count);
  applyBrakingProfile(segment);
  return true;
}

bool LocalPlannerNode::buildSegmentBrakingPrefix(
  const f110_msgs::msg::WpntArray & source,
  f110_msgs::msg::WpntArray & segment) const
{
  if (source.wpnts.size() < 2 || !has_odom_) {
    return false;
  }
  const double current_s = current_odom_.pose.pose.position.x;
  const double track_length = global_wpnts_.wpnts.empty() ? 0.0 :
    global_wpnts_.wpnts.back().s_m;
  std::size_t start = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < source.wpnts.size(); ++i) {
    double ds = source.wpnts[i].s_m - current_s;
    if (track_length > 1e-6) {
      ds = std::remainder(ds, track_length);
    }
    if (std::abs(ds) < best_distance) {
      best_distance = std::abs(ds);
      start = i;
    }
  }

  const double clearance = vehicle_radius_ + path_clearance_margin_;
  std::size_t safe_count = 0U;
  for (std::size_t i = start; i < source.wpnts.size(); ++i) {
    const auto & waypoint = source.wpnts[i];
    if (!isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, clearance))
    {
      break;
    }
    if (i > start) {
      const auto & previous = source.wpnts[i - 1U];
      if (!isPathSegmentCollisionFree(
          previous.x_m, previous.y_m, previous.psi_rad,
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad, clearance))
      {
        break;
      }
    }
    ++safe_count;
  }
  if (safe_count < 2U) {
    return false;
  }
  const std::size_t buffered = safe_count >
    static_cast<std::size_t>(lattice_safe_stop_buffer_wpnts_) + 1U ?
    safe_count - static_cast<std::size_t>(lattice_safe_stop_buffer_wpnts_) : safe_count;
  const std::size_t stop_count = std::max<std::size_t>(2U, buffered);
  segment.header = source.header;
  segment.wpnts.assign(
    source.wpnts.begin() + static_cast<std::ptrdiff_t>(start),
    source.wpnts.begin() + static_cast<std::ptrdiff_t>(start + stop_count));
  applyBrakingProfile(segment);
  return true;
}

bool LocalPlannerNode::buildStationaryHoldSegment(
  f110_msgs::msg::WpntArray & segment) const
{
  if (!has_odom_ || global_wpnts_.wpnts.size() < 2) {
    return false;
  }

  const double current_s = current_odom_.pose.pose.position.x;
  const double current_d = current_odom_.pose.pose.position.y;
  const int reference_index = findClosestWaypointIndexByS(current_s);
  if (reference_index < 0) {
    return false;
  }

  double current_x = 0.0;
  double current_y = 0.0;
  double current_yaw = 0.0;
  double next_x = 0.0;
  double next_y = 0.0;
  double next_yaw = 0.0;
  if (!obstacleCenterPose(
      current_s, current_d, current_x, current_y, current_yaw) ||
    !obstacleCenterPose(
      current_s + stationary_hold_point_spacing_m_, current_d,
      next_x, next_y, next_yaw))
  {
    return false;
  }

  const double clearance = vehicle_radius_ + path_clearance_margin_;
  if (!isPathPointCollisionFree(
      current_x, current_y, current_yaw, clearance) ||
    !isPathPointCollisionFree(next_x, next_y, next_yaw, clearance) ||
    !isPathSegmentCollisionFree(
      current_x, current_y, current_yaw,
      next_x, next_y, next_yaw, clearance))
  {
    return false;
  }

  const auto & reference = global_wpnts_.wpnts[static_cast<std::size_t>(reference_index)];
  auto current = reference;
  current.id = 0;
  current.s_m = current_s;
  current.d_m = current_d;
  current.x_m = current_x;
  current.y_m = current_y;
  current.psi_rad = current_yaw;
  current.kappa_radpm = 0.0;
  current.vx_mps = 0.0;
  current.ax_mps2 = 0.0;

  auto next = current;
  next.id = 1;
  next.s_m = current_s + stationary_hold_point_spacing_m_;
  next.x_m = next_x;
  next.y_m = next_y;
  next.psi_rad = next_yaw;

  segment.header.stamp = now();
  segment.header.frame_id = frame_id_;
  segment.wpnts = {current, next};
  return true;
}

bool LocalPlannerNode::isPlanningResultStale(
  const uint64_t planning_localization_generation,
  double & pose_drift,
  bool & localization_changed) const
{
  nav_msgs::msg::Odometry newest_odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    newest_odom = latest_odom_;
  }
  const double closing_length = std::hypot(
    global_wpnts_.wpnts.front().x_m - global_wpnts_.wpnts.back().x_m,
    global_wpnts_.wpnts.front().y_m - global_wpnts_.wpnts.back().y_m);
  const double track_length = global_wpnts_.wpnts.back().s_m + closing_length;
  double s_drift = std::abs(
    newest_odom.pose.pose.position.x - current_odom_.pose.pose.position.x);
  if (track_length > 0.0) {
    s_drift = std::fmod(s_drift, track_length);
    s_drift = std::min(s_drift, track_length - s_drift);
  }
  pose_drift = std::hypot(
    s_drift, newest_odom.pose.pose.position.y - current_odom_.pose.pose.position.y);
  localization_changed = localization_generation_.load(std::memory_order_acquire) !=
    planning_localization_generation;
  return localization_changed || pose_drift > lattice_max_result_pose_drift_m_;
}

void LocalPlannerNode::applyBrakingProfile(
  f110_msgs::msg::WpntArray & segment) const
{
  if (segment.wpnts.empty()) {
    return;
  }
  double remaining_distance = 0.0;
  for (int i = static_cast<int>(segment.wpnts.size()) - 1; i >= 0; --i) {
    if (i + 1 < static_cast<int>(segment.wpnts.size())) {
      remaining_distance += std::hypot(
        segment.wpnts[i + 1].x_m - segment.wpnts[i].x_m,
        segment.wpnts[i + 1].y_m - segment.wpnts[i].y_m);
    }
    const double braking_speed = std::sqrt(
      2.0 * lattice_safe_stop_deceleration_mps2_ * remaining_distance);
    segment.wpnts[i].vx_mps = std::min(segment.wpnts[i].vx_mps, braking_speed);
  }

  // An emergency handoff must never accelerate merely because the previous
  // avoidance profile was recovering speed after an obstacle. Preserve the
  // current/front speed as an upper bound for every following point.
  for (std::size_t i = 1U; i < segment.wpnts.size(); ++i) {
    segment.wpnts[i].vx_mps = std::min(
      segment.wpnts[i].vx_mps, segment.wpnts[i - 1U].vx_mps);
  }
  for (std::size_t i = 0U; i + 1U < segment.wpnts.size(); ++i) {
    const double ds = std::max(
      1e-3, std::hypot(
        segment.wpnts[i + 1U].x_m - segment.wpnts[i].x_m,
        segment.wpnts[i + 1U].y_m - segment.wpnts[i].y_m));
    segment.wpnts[i].ax_mps2 =
      (segment.wpnts[i + 1U].vx_mps * segment.wpnts[i + 1U].vx_mps -
      segment.wpnts[i].vx_mps * segment.wpnts[i].vx_mps) / (2.0 * ds);
  }
  segment.wpnts.back().ax_mps2 = 0.0;
}

bool LocalPlannerNode::reuseCommittedAvoidancePath(
  const int start_idx, const int max_count,
  f110_msgs::msg::WpntArray & candidate,
  int & segment_count)
{
  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (!lattice_commit_path_until_clear_ || !has_avoidance_commitment_ || total < 2 ||
    static_cast<int>(committed_avoidance_path_.wpnts.size()) != total)
  {
    return false;
  }

  int forward_count =
    (committed_merge_index_ - start_idx + total) % total + 1;
  if (forward_count < 2 || forward_count > max_count) {
    const int passed_merge_wpnts =
      (start_idx - committed_merge_index_ + total) % total;
    const bool recently_passed_merge =
      passed_merge_wpnts <= lattice_merge_settle_max_wpnts_;
    const bool ego_still_offset =
      std::abs(current_odom_.pose.pose.position.y) > lattice_merge_lateral_tolerance_m_;
    if (!recently_passed_merge || !ego_still_offset || max_count < 2) {
      return false;
    }
    // The geometric merge point has passed, but the actual vehicle has not
    // settled on d=0 yet. Continue publishing the committed path's global
    // continuation instead of switching path sources mid-correction.
    forward_count = std::min(max_count, total);
  }

  const double collision_clearance = vehicle_radius_ + path_clearance_margin_;
  for (int k = 0; k < forward_count; ++k) {
    const int index = (start_idx + k) % total;
    const auto & waypoint = committed_avoidance_path_.wpnts[index];
    if (committed_point_validated_[static_cast<std::size_t>(index)] == 0U) {
      if (!isPathPointCollisionFree(
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad, collision_clearance))
      {
        return false;
      }
      committed_point_validated_[static_cast<std::size_t>(index)] = 1U;
    }
    if (k + 1 < forward_count) {
      const int next = (index + 1) % total;
      const auto & next_waypoint = committed_avoidance_path_.wpnts[next];
      if (committed_segment_validated_[static_cast<std::size_t>(index)] == 0U) {
        if (!isPathSegmentCollisionFree(
            waypoint.x_m, waypoint.y_m, waypoint.psi_rad,
            next_waypoint.x_m, next_waypoint.y_m, next_waypoint.psi_rad,
            collision_clearance))
        {
          return false;
        }
        committed_segment_validated_[static_cast<std::size_t>(index)] = 1U;
      }
    }
  }

  candidate = committed_avoidance_path_;
  candidate.header.stamp = now();
  segment_count = forward_count;
  return true;
}

int LocalPlannerNode::extendSegmentPastMerge(
  const f110_msgs::msg::WpntArray & candidate,
  const int start_idx, const int merge_segment_count) const
{
  const int total = static_cast<int>(candidate.wpnts.size());
  if (total < 2 || merge_segment_count < 1 ||
    lattice_post_merge_lookahead_wpnts_ <= 0)
  {
    return std::clamp(merge_segment_count, 0, total);
  }

  const int desired_count = std::min(
    total, merge_segment_count + lattice_post_merge_lookahead_wpnts_);
  const double collision_clearance = vehicle_radius_ + path_clearance_margin_;
  int safe_count = std::min(merge_segment_count, total);
  for (int k = safe_count; k < desired_count; ++k) {
    const int previous = (start_idx + k - 1) % total;
    const int index = (start_idx + k) % total;
    const auto & previous_waypoint = candidate.wpnts[previous];
    const auto & waypoint = candidate.wpnts[index];
    if (!isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, collision_clearance) ||
      !isPathSegmentCollisionFree(
        previous_waypoint.x_m, previous_waypoint.y_m, previous_waypoint.psi_rad,
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, collision_clearance))
    {
      RCLCPP_DEBUG(
        get_logger(),
        "Post-merge global continuation stopped after %d/%d extra waypoints due to occupancy.",
        safe_count - merge_segment_count, lattice_post_merge_lookahead_wpnts_);
      break;
    }
    ++safe_count;
  }
  return safe_count;
}

void LocalPlannerNode::commitAvoidancePath(
  const f110_msgs::msg::WpntArray & candidate,
  const int start_idx, const int segment_count,
  const bool avoid_left, const std::string & planner_name,
  const double lattice_cost)
{
  const int total = static_cast<int>(candidate.wpnts.size());
  if (!lattice_commit_path_until_clear_ || total < 2 || segment_count < 2) {
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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

<<<<<<< HEAD
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
=======
  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header = marker_header;
  obstacle_marker.ns = "confirmed_obstacles";
  obstacle_marker.id = 0;
  obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  obstacle_marker.pose.orientation.w = 1.0;
  obstacle_marker.scale.x = obstacle_marker_scale_;
  obstacle_marker.scale.y = obstacle_marker_scale_;
  obstacle_marker.scale.z = obstacle_marker_scale_;
  obstacle_marker.color.r = 1.0F;
  obstacle_marker.color.g = 0.0F;
  obstacle_marker.color.b = 0.0F;
  obstacle_marker.color.a = 0.9F;
  obstacle_marker.points = obstacle_points;
  markers.markers.push_back(std::move(obstacle_marker));

  const auto corridor_point = [](const SafeCorridorSample & sample, const double d) {
      geometry_msgs::msg::Point point;
      point.x = sample.reference.x - d * std::sin(sample.reference.yaw);
      point.y = sample.reference.y + d * std::cos(sample.reference.yaw);
      point.z = 0.07;
      return point;
    };
  const int debug_stride = std::max(1, corridor_debug_stride_);

  visualization_msgs::msg::Marker corridor_boundaries;
  corridor_boundaries.header = marker_header;
  corridor_boundaries.ns = "safe_corridor_boundaries";
  corridor_boundaries.id = 2;
  corridor_boundaries.type = visualization_msgs::msg::Marker::LINE_LIST;
  corridor_boundaries.action = visualization_msgs::msg::Marker::ADD;
  corridor_boundaries.pose.orientation.w = 1.0;
  corridor_boundaries.scale.x = 0.035;
  corridor_boundaries.color.r = 0.1F;
  corridor_boundaries.color.g = 1.0F;
  corridor_boundaries.color.b = 0.2F;
  corridor_boundaries.color.a = 0.85F;
  for (std::size_t index = 0; index + static_cast<std::size_t>(debug_stride) <
    safe_corridor_.samples.size(); index += static_cast<std::size_t>(debug_stride))
  {
    const auto & current = safe_corridor_.samples[index];
    const auto & next = safe_corridor_.samples[index + static_cast<std::size_t>(debug_stride)];
    corridor_boundaries.points.push_back(corridor_point(current, current.track_min_d));
    corridor_boundaries.points.push_back(corridor_point(next, next.track_min_d));
    corridor_boundaries.points.push_back(corridor_point(current, current.track_max_d));
    corridor_boundaries.points.push_back(corridor_point(next, next.track_max_d));
  }
  markers.markers.push_back(std::move(corridor_boundaries));

  visualization_msgs::msg::Marker blocked_intervals;
  blocked_intervals.header = marker_header;
  blocked_intervals.ns = "safe_corridor_blocked";
  blocked_intervals.id = 3;
  blocked_intervals.type = visualization_msgs::msg::Marker::LINE_LIST;
  blocked_intervals.action = visualization_msgs::msg::Marker::ADD;
  blocked_intervals.pose.orientation.w = 1.0;
  blocked_intervals.scale.x = 0.07;
  blocked_intervals.color.r = 1.0F;
  blocked_intervals.color.g = 0.05F;
  blocked_intervals.color.b = 0.05F;
  blocked_intervals.color.a = 0.85F;

  visualization_msgs::msg::Marker left_space = blocked_intervals;
  left_space.ns = "safe_corridor_left_feasible";
  left_space.id = 4;
  left_space.scale.x = 0.045;
  left_space.color.r = 0.0F;
  left_space.color.g = 0.85F;
  left_space.color.b = 1.0F;
  visualization_msgs::msg::Marker right_space = left_space;
  right_space.ns = "safe_corridor_right_feasible";
  right_space.id = 5;
  right_space.color.r = 0.15F;
  right_space.color.g = 0.35F;
  right_space.color.b = 1.0F;
  for (std::size_t index = 0; index < safe_corridor_.samples.size();
    index += static_cast<std::size_t>(debug_stride))
  {
    const auto & sample = safe_corridor_.samples[index];
    for (const auto & interval : sample.blocked_intervals) {
      blocked_intervals.points.push_back(corridor_point(sample, interval.min_d));
      blocked_intervals.points.push_back(corridor_point(sample, interval.max_d));
    }
    for (const auto & interval : sample.feasible_intervals) {
      if (interval.max_d > 0.0) {
        left_space.points.push_back(corridor_point(sample, std::max(0.0, interval.min_d)));
        left_space.points.push_back(corridor_point(sample, interval.max_d));
      }
      if (interval.min_d < 0.0) {
        right_space.points.push_back(corridor_point(sample, interval.min_d));
        right_space.points.push_back(corridor_point(sample, std::min(0.0, interval.max_d)));
      }
    }
  }
  markers.markers.push_back(std::move(blocked_intervals));
  markers.markers.push_back(std::move(left_space));
  markers.markers.push_back(std::move(right_space));

  visualization_msgs::msg::Marker inflated_obstacles;
  inflated_obstacles.header = marker_header;
  inflated_obstacles.ns = "inflated_obstacles";
  inflated_obstacles.id = 6;
  inflated_obstacles.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  inflated_obstacles.action = visualization_msgs::msg::Marker::ADD;
  inflated_obstacles.pose.orientation.w = 1.0;
  const double inflation_diameter = 2.0 * std::max(
    vehicle_radius_ + path_clearance_margin_,
    0.5 * vehicle_width_m_ + localization_margin_m_ + corridor_safety_margin_m_);
  inflated_obstacles.scale.x = inflation_diameter;
  inflated_obstacles.scale.y = inflation_diameter;
  inflated_obstacles.scale.z = 0.03;
  inflated_obstacles.color.r = 1.0F;
  inflated_obstacles.color.g = 0.45F;
  inflated_obstacles.color.b = 0.0F;
  inflated_obstacles.color.a = 0.18F;
  const int width = static_cast<int>(grid_map_.info.width);
  for (const int map_index : safe_corridor_.inflated_cell_indices) {
    if (map_index >= 0 && width > 0 &&
      map_index < static_cast<int>(grid_map_.data.size()))
    {
      inflated_obstacles.points.push_back(
        mapCellCenter(map_index % width, map_index / width));
    }
  }
  markers.markers.push_back(std::move(inflated_obstacles));

  visualization_msgs::msg::Marker rejection_text;
  rejection_text.header = marker_header;
  rejection_text.ns = "candidate_rejection_counts";
  rejection_text.id = 7;
  rejection_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  rejection_text.action = visualization_msgs::msg::Marker::ADD;
  rejection_text.pose.orientation.w = 1.0;
  rejection_text.scale.z = 0.24;
  rejection_text.color.r = 1.0F;
  rejection_text.color.g = 1.0F;
  rejection_text.color.b = 1.0F;
  rejection_text.color.a = 0.95F;
  if (!safe_corridor_.samples.empty()) {
    rejection_text.pose.position = corridor_point(
      safe_corridor_.samples.front(), safe_corridor_.samples.front().track_max_d);
    rejection_text.pose.position.z = 0.30;
  }
  std::ostringstream rejection_stream;
  rejection_stream << "reject corridor=" << rejection_counts_.corridor <<
    " track=" << rejection_counts_.track_boundary <<
    " occ=" << rejection_counts_.occupied <<
    " unknown=" << rejection_counts_.unknown <<
    " outside=" << rejection_counts_.outside_map <<
    " kappa=" << rejection_counts_.curvature <<
    " dk=" << rejection_counts_.curvature_rate <<
    " alat=" << rejection_counts_.lateral_acceleration <<
    " invalid=" << rejection_counts_.invalid_geometry <<
    " latency=" << last_planning_latency_ms_ << "ms";
  rejection_text.text = rejection_stream.str();
  markers.markers.push_back(std::move(rejection_text));

  visualization_msgs::msg::Marker path_marker;
  path_marker.header = marker_header;
  path_marker.ns = "local_planning_segment";
  path_marker.id = 1;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = path_marker_width_;
  path_marker.color.r = safe_path_available ? 0.0F : 1.0F;
  path_marker.color.g = safe_path_available ? 1.0F : 0.65F;
  path_marker.color.b = 0.2F;
  path_marker.color.a = 0.9F;
  for (const auto & waypoint : local_segment.wpnts) {
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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
<<<<<<< HEAD
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
=======
  const uint64_t planning_localization_generation =
    localization_generation_.load(std::memory_order_acquire);
  bool odom_available = false;
  bool odom_ready = false;
  int valid_odom_count = 0;
  rclcpp::Time odom_receive_time{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_available = has_odom_;
    odom_ready = odom_ready_;
    valid_odom_count = consecutive_valid_odom_count_;
    odom_receive_time = last_odom_receive_time_;
    if (odom_available) {
      current_odom_ = latest_odom_;
    }
  }
  if (localization_reset_pending_.exchange(false, std::memory_order_acq_rel)) {
    clearAvoidanceCommitment();
    has_held_avoidance_segment_ = false;
    held_avoidance_segment_.wpnts.clear();
    has_last_validated_full_path_ = false;
    last_validated_full_path_.wpnts.clear();
    avoidance_active_ = false;
    detection_count_ = 0;
    clear_count_ = 0;
    RCLCPP_WARN(
      get_logger(),
      "Frenet localization jump accepted as a pose reset; cleared every path "
      "and waiting for stable odometry confirmation.");
  }
  if (!has_global_ || !has_map_ || !odom_available) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for global waypoints, map, and Frenet odometry.");
    return;
  }

  if (use_perception_obstacles_ && require_perception_obstacles_ && !has_obstacles_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Local planning inhibited: waiting for required obstacle stream %s. "
      "Start opponent_detector and verify that this topic has a publisher.",
      obstacles_topic_.c_str());
    return;
  }

  const auto planning_begin = std::chrono::steady_clock::now();
  rejection_counts_ = CandidateRejectionCounts{};

  const bool odom_stale =
    (now() - odom_receive_time).seconds() > frenet_odom_stale_timeout_sec_;
  if (!odom_ready || odom_stale) {
    if (odom_stale) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_ready_ = false;
      has_previous_frenet_s_ = false;
      consecutive_valid_odom_count_ = 0;
    }
    const double held_path_age = has_held_avoidance_segment_ ?
      (now() - held_segment_time_).seconds() :
      std::numeric_limits<double>::infinity();
    if (has_held_avoidance_segment_ &&
      held_path_age <= frenet_odom_path_hold_timeout_sec_)
    {
      held_avoidance_segment_.header.stamp = now();
      publishAvoidWaypoints(
        &held_avoidance_segment_, held_avoid_left_, held_planner_name_ + "_held");
      if (publish_standalone_local_) {
        local_wpnts_pub_->publish(held_avoidance_segment_);
      }
      local_path_pub_->publish(makePath(held_avoidance_segment_));
      exact_local_path_pub_->publish(makePath(held_avoidance_segment_));
      publishDebugVisualization(held_avoidance_segment_, {}, true);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Frenet odometry is %s; holding the last collision-checked avoidance path "
        "for %.2f/%.2f s.",
        odom_stale ? "stale" : "unstable", held_path_age,
        frenet_odom_path_hold_timeout_sec_);
      return;
    }
    clearAvoidanceCommitment();
    has_held_avoidance_segment_ = false;
    held_avoidance_segment_.wpnts.clear();
    has_last_validated_full_path_ = false;
    last_validated_full_path_.wpnts.clear();
    avoidance_active_ = false;
    detection_count_ = 0;
    clear_count_ = 0;
    f110_msgs::msg::WpntArray empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = frame_id_;
    publishAvoidWaypoints(nullptr, true, "invalid_frenet_odom");
    if (publish_standalone_local_) {
      local_wpnts_pub_->publish(empty_path);
    }
    local_path_pub_->publish(makePath(empty_path));
    exact_local_path_pub_->publish(makePath(empty_path));
    publishDebugVisualization(empty_path, {}, false);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Local planning inhibited: Frenet odometry is %s (%d/%d stable samples).",
      odom_stale ? "stale" : "not stable",
      valid_odom_count, frenet_odom_confirm_cycles_);
    return;
  }

  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  const int planning_count = std::min(total, lookahead_wpnt_num_);
  const int detection_count = std::min(planning_count, detection_lookahead_wpnt_num_);
  const int start_idx = findClosestWaypointIndexByS(current_odom_.pose.pose.position.x);
  safe_corridor_start_idx_ = start_idx;
  safe_corridor_ = buildSafeCorridor(start_idx, planning_count);

  bool raw_obstacle_detected = false;
  bool avoid_left = true;
  std::vector<int> collision_indices;
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  detectObstaclesAndDecideDirection(
    start_idx, detection_count, raw_obstacle_detected, avoid_left,
    collision_indices, obstacle_points);

  if (raw_obstacle_detected) {
    detection_count_ = std::min(detection_count_ + 1, detection_confirm_cycles_);
    clear_count_ = 0;
    if (detection_count_ >= detection_confirm_cycles_) {
      avoidance_active_ = true;
    }
  } else {
    detection_count_ = 0;
    clear_count_ = std::min(clear_count_ + 1, detection_clear_cycles_);
    // Once a collision-checked path is committed, a disappearing obstacle is
    // expected after the ego passes it. Keep that path until its merge point;
    // clearing it here makes the controller alternate local/global sources in
    // the middle of the lateral return maneuver.
    if (clear_count_ >= detection_clear_cycles_ && !has_avoidance_commitment_) {
      avoidance_active_ = false;
      clearAvoidanceCommitment();
    }
  }

  auto full_local_path = global_wpnts_;
  full_local_path.header.stamp = now();
  auto visualization_segment = makeForwardSegment(
    global_wpnts_, start_idx, planning_count);
  auto marker_segment = visualization_segment;
  bool safe_path_available = true;
  bool avoidance_published = false;
  bool avoidance_merge_reached = false;
  bool preplan_next_cluster = false;
  int detected_collision_start_index = -1;
  int detected_collision_end_index = -1;
  if (!collision_indices.empty()) {
    const int collision_begin = collision_indices.front();
    int collision_end = collision_begin;
    for (std::size_t i = 1; i < collision_indices.size(); ++i) {
      if (collision_indices[i] - collision_end > lattice_obstacle_cluster_gap_wpnts_) {
        break;
      }
      collision_end = collision_indices[i];
    }
    detected_collision_start_index = (start_idx + collision_begin) % total;
    detected_collision_end_index = (start_idx + collision_end) % total;
  }

  if (has_avoidance_commitment_) {
    const int remaining_to_merge =
      (committed_merge_index_ - start_idx + total) % total;
    double remaining_merge_distance_m = 0.0;
    for (int k = 0; k < remaining_to_merge; ++k) {
      const int index = (start_idx + k) % total;
      const int next = (index + 1) % total;
      remaining_merge_distance_m += std::hypot(
        global_wpnts_.wpnts[next].x_m - global_wpnts_.wpnts[index].x_m,
        global_wpnts_.wpnts[next].y_m - global_wpnts_.wpnts[index].y_m);
    }
    const auto circular_index_distance = [total](const int lhs, const int rhs) {
        const int direct = std::abs(lhs - rhs);
        return std::min(direct, total - direct);
      };
    // As the ego enters a cluster, its already-passed prefix disappears from
    // the moving horizon and the detected start index advances. The cluster
    // end remains stable, so compare endpoints with the same gap used to join
    // collision slices instead of requiring an exact moving-window key.
    const bool already_planning_detected_cluster =
      detected_collision_start_index >= 0 &&
      committed_collision_end_index_ >= 0 &&
      circular_index_distance(
      detected_collision_end_index, committed_collision_end_index_) <=
      lattice_obstacle_cluster_gap_wpnts_;
    preplan_next_cluster = raw_obstacle_detected && !already_planning_detected_cluster &&
      remaining_to_merge > 0 &&
      remaining_to_merge <= planning_count &&
      remaining_merge_distance_m <= lattice_preplan_before_merge_m_;
    const int passed_merge_wpnts =
      (start_idx - committed_merge_index_ + total) % total;
    const bool merge_reached = remaining_to_merge == 0 ||
      remaining_to_merge > planning_count;
    if (merge_reached) {
      const bool recently_passed_merge =
        passed_merge_wpnts <= lattice_merge_settle_max_wpnts_;
      const bool ego_on_global_line =
        std::abs(current_odom_.pose.pose.position.y) <=
        lattice_merge_lateral_tolerance_m_;
      if (raw_obstacle_detected) {
        // The next overlapping obstacle is already in the detection window.
        // Keep the current commitment alive until a replacement has actually
        // passed every check. Clearing it before planning left no usable path
        // whenever the first replacement cycle failed, which made the car
        // stop and start again after its geometry changed.
        preplan_next_cluster = true;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Avoidance merge reached with another obstacle ahead; retaining the "
          "committed continuation until its replacement succeeds.");
      } else if (recently_passed_merge && !ego_on_global_line) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Geometric merge reached, but ego d=%.3f m is outside %.3f m; "
          "holding the global continuation for stable settling.",
          current_odom_.pose.pose.position.y, lattice_merge_lateral_tolerance_m_);
      } else {
        clearAvoidanceCommitment();
        has_held_avoidance_segment_ = false;
        held_avoidance_segment_.wpnts.clear();
        has_last_validated_full_path_ = false;
        last_validated_full_path_.wpnts.clear();
        avoidance_merge_reached = true;
        avoidance_active_ = false;
        detection_count_ = 0;
        clear_count_ = detection_clear_cycles_;
        RCLCPP_INFO(
          get_logger(),
          "Avoidance merge and lateral settling completed; transitioning to GLOBAL.");
      }
    }
  }

  if (!avoidance_merge_reached && avoidance_active_ &&
    (raw_obstacle_detected || has_avoidance_commitment_))
  {
    f110_msgs::msg::WpntArray candidate;
    int segment_count = planning_count;
    const bool preferred_side = has_avoidance_commitment_ ?
      committed_avoid_left_ : avoid_left;
    double lattice_cost = std::numeric_limits<double>::infinity();
    std::string planner_name = "frenet_lattice_segment";
    bool reused_commitment = false;
    bool candidate_valid = false;
    bool proactive_replan = false;

    if (preplan_next_cluster) {
      reused_commitment = false;
      candidate_valid = buildFrenetLatticeCandidate(
        start_idx, planning_count, collision_indices,
        preferred_side, candidate, segment_count, avoid_left, lattice_cost);
      if (!candidate_valid) {
        candidate_valid = buildFrenetLatticeCandidate(
          start_idx, planning_count, collision_indices,
          preferred_side, candidate, segment_count, avoid_left, lattice_cost, true);
        if (candidate_valid) {
          planner_name = "frenet_lattice_recovery_preplanned";
        }
      } else {
        planner_name = "frenet_lattice_segment_preplanned";
      }
      if (candidate_valid) {
        const auto & start_waypoint = candidate.wpnts[static_cast<std::size_t>(start_idx)];
        // current_odom_ is Frenet odometry: pose.x=s and pose.y=d. Compare lateral
        // continuity in the same coordinate domain instead of mixing (s,d) with map (x,y).
        const double start_gap = std::abs(
          start_waypoint.d_m - current_odom_.pose.pose.position.y);
        if (start_gap > lattice_preplan_max_start_gap_m_) {
          candidate_valid = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Discarding proactive next-cluster path with %.3f m start gap (limit %.3f m).",
            start_gap, lattice_preplan_max_start_gap_m_);
        } else {
          proactive_replan = true;
          RCLCPP_INFO(
            get_logger(),
            "Preplanned the next obstacle cluster %.2f m before the current merge.",
            lattice_preplan_before_merge_m_);
        }
      }
    }

    if (!candidate_valid) {
      reused_commitment = reuseCommittedAvoidancePath(
        start_idx, planning_count, candidate, segment_count);
      candidate_valid = reused_commitment;
    }
    if (reused_commitment) {
      avoid_left = committed_avoid_left_;
      planner_name = committed_planner_name_;
      lattice_cost = committed_lattice_cost_;
    } else if (!candidate_valid && raw_obstacle_detected) {
      candidate_valid = buildFrenetLatticeCandidate(
        start_idx, planning_count, collision_indices,
        preferred_side, candidate, segment_count, avoid_left, lattice_cost);
      if (!candidate_valid) {
        candidate_valid = buildFrenetLatticeCandidate(
          start_idx, planning_count, collision_indices,
          preferred_side, candidate, segment_count, avoid_left, lattice_cost, true);
        if (candidate_valid) {
          planner_name = "frenet_lattice_recovery";
        }
      }
    }

    double planning_pose_drift = 0.0;
    bool localization_changed_during_planning = false;
    if (isPlanningResultStale(
        planning_localization_generation, planning_pose_drift,
        localization_changed_during_planning))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding stale local-planning result: localization_reset=%s, "
        "ego_drift=%.3f m (limit %.3f m).",
        localization_changed_during_planning ? "true" : "false",
        planning_pose_drift, lattice_max_result_pose_drift_m_);
      return;
    }

    if (candidate_valid) {
      const int merge_segment_count = segment_count;
      if (!reused_commitment) {
        commitAvoidancePath(
          candidate, start_idx, merge_segment_count, avoid_left, planner_name, lattice_cost);
        committed_collision_start_index_ = detected_collision_start_index;
        committed_collision_end_index_ = detected_collision_end_index;
      }
      segment_count = extendSegmentPastMerge(
        candidate, start_idx, merge_segment_count);
      full_local_path = candidate;
      last_validated_full_path_ = candidate;
      last_validated_avoid_left_ = avoid_left;
      last_validated_path_time_ = now();
      has_last_validated_full_path_ = true;
      visualization_segment = makeForwardSegment(
        candidate, start_idx, segment_count);
      marker_segment = visualization_segment;
      held_avoidance_segment_ = visualization_segment;
      held_avoid_left_ = avoid_left;
      held_planner_name_ = planner_name;
      held_segment_time_ = now();
      has_held_avoidance_segment_ = true;
      publishAvoidWaypoints(&visualization_segment, avoid_left, planner_name);
      avoidance_published = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Publishing Frenet lattice avoidance: side=%s, cost=%.3f, points=%d "
        "(merge=%d), committed=%s.",
        avoid_left ? "left" : "right", lattice_cost, segment_count,
        merge_segment_count,
        reused_commitment ? "true" : (proactive_replan ? "preplanned" : "false"));
    } else {
      const double last_validated_path_age = has_last_validated_full_path_ ?
        (now() - last_validated_path_time_).seconds() :
        std::numeric_limits<double>::infinity();
      const bool held_path_is_braking =
        held_planner_name_.find("frenet_lattice_safe_stop") == 0U ||
        held_planner_name_.find("frenet_lattice_replan_brake") == 0U;
      const bool can_hold_previous_braking = has_held_avoidance_segment_ &&
        held_path_is_braking &&
        (now() - held_segment_time_).seconds() <= frenet_odom_path_hold_timeout_sec_;
      f110_msgs::msg::WpntArray replan_braking_segment;
      f110_msgs::msg::WpntArray held_braking_segment;
      f110_msgs::msg::WpntArray safe_stop_segment;
      f110_msgs::msg::WpntArray stationary_hold_segment;
      const bool replan_brake_available =
        last_validated_path_age <= lattice_replan_brake_timeout_sec_ &&
        buildReplanBrakingSegment(
        last_validated_full_path_, start_idx, planning_count,
        replan_braking_segment);
      const bool safe_stop_available = buildSafeStopSegment(
        start_idx, planning_count, collision_indices, safe_stop_segment);
      const bool held_brake_available = has_held_avoidance_segment_ &&
        buildSegmentBrakingPrefix(held_avoidance_segment_, held_braking_segment);
      const bool stationary_hold_available = buildStationaryHoldSegment(
        stationary_hold_segment);
      if (replan_brake_available) {
        visualization_segment = replan_braking_segment;
        full_local_path = replan_braking_segment;
        marker_segment = replan_braking_segment;
        avoid_left = last_validated_avoid_left_;
        held_avoidance_segment_ = replan_braking_segment;
        held_avoid_left_ = last_validated_avoid_left_;
        held_planner_name_ = "frenet_lattice_replan_brake";
        held_segment_time_ = now();
        has_held_avoidance_segment_ = true;
        publishAvoidWaypoints(
          &replan_braking_segment, last_validated_avoid_left_,
          "frenet_lattice_replan_brake");
        avoidance_published = true;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Replacement lattice is temporarily unavailable; publishing %zu "
          "newly collision-checked braking points from the previous path.",
          replan_braking_segment.wpnts.size());
      } else if (safe_stop_available) {
        visualization_segment = safe_stop_segment;
        full_local_path = safe_stop_segment;
        marker_segment = safe_stop_segment;
        held_avoidance_segment_ = safe_stop_segment;
        held_avoid_left_ = preferred_side;
        held_planner_name_ = "frenet_lattice_safe_stop";
        held_segment_time_ = now();
        has_held_avoidance_segment_ = true;
        publishAvoidWaypoints(
          &safe_stop_segment, preferred_side, "frenet_lattice_safe_stop");
        avoidance_published = true;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Primary and recovery Frenet lattices failed; publishing a gradual "
          "collision-free braking segment before the obstacle.");
      } else if (held_brake_available && !held_path_is_braking) {
        visualization_segment = held_braking_segment;
        full_local_path = held_braking_segment;
        marker_segment = held_braking_segment;
        held_avoidance_segment_ = held_braking_segment;
        held_planner_name_ = "frenet_lattice_replan_brake";
        held_segment_time_ = now();
        publishAvoidWaypoints(
          &held_braking_segment, held_avoid_left_, held_planner_name_);
        avoidance_published = true;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Replacement lattice failed; braking on the collision-free prefix "
          "of the held avoidance path.");
      } else if (can_hold_previous_braking) {
        held_avoidance_segment_.header.stamp = now();
        visualization_segment = held_avoidance_segment_;
        full_local_path = held_avoidance_segment_;
        marker_segment = held_avoidance_segment_;
        avoid_left = held_avoid_left_;
        publishAvoidWaypoints(
          &held_avoidance_segment_, held_avoid_left_,
          held_planner_name_ + "_held");
        avoidance_published = true;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No new braking segment can be formed; holding the previous "
          "collision-checked braking path instead of switching to GLOBAL.");
      } else if (stationary_hold_available) {
        visualization_segment = stationary_hold_segment;
        full_local_path = stationary_hold_segment;
        marker_segment = stationary_hold_segment;
        held_avoidance_segment_ = stationary_hold_segment;
        held_avoid_left_ = preferred_side;
        held_planner_name_ = "frenet_lattice_stationary_hold";
        held_segment_time_ = now();
        has_held_avoidance_segment_ = true;
        publishAvoidWaypoints(
          &stationary_hold_segment, preferred_side,
          "frenet_lattice_stationary_hold");
        avoidance_published = true;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No Frenet lattice or safe-stop segment passed validation; "
          "publishing a collision-checked zero-speed hold while replanning.");
      } else {
        has_held_avoidance_segment_ = false;
        held_avoidance_segment_.wpnts.clear();
        safe_path_available = false;
        // Never label the original global segment as a local avoidance path
        // after an obstacle has been confirmed.
        visualization_segment.wpnts.clear();
        full_local_path.wpnts.clear();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No moving, braking, or collision-free stationary path passed "
          "validation; publishing no local path.");
      }
    }
  }

  // A safe-stop segment has no merge commitment. Keep the last validated
  // segment through the configured obstacle-clear hysteresis as well, so a
  // one-cycle component miss cannot replace it with an empty path.
  const double held_path_age = has_held_avoidance_segment_ ?
    (now() - held_segment_time_).seconds() :
    std::numeric_limits<double>::infinity();
  if (!avoidance_published && avoidance_active_ &&
    has_held_avoidance_segment_ &&
    held_path_age <= frenet_odom_path_hold_timeout_sec_)
  {
    held_avoidance_segment_.header.stamp = now();
    visualization_segment = held_avoidance_segment_;
    full_local_path = held_avoidance_segment_;
    marker_segment = held_avoidance_segment_;
    avoid_left = held_avoid_left_;
    publishAvoidWaypoints(
      &held_avoidance_segment_, held_avoid_left_, held_planner_name_ + "_held");
    avoidance_published = true;
  }

  if (!avoidance_published) {
    publishAvoidWaypoints(nullptr, avoid_left, "no_safe_path");
    if (!avoidance_active_) {
      obstacle_points.clear();
      has_held_avoidance_segment_ = false;
      held_avoidance_segment_.wpnts.clear();
      has_last_validated_full_path_ = false;
      last_validated_full_path_.wpnts.clear();
    }
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
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
