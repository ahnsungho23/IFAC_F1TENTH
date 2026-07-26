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
#include <deque>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <utility>

using std::placeholders::_1;

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double smoothstepQuintic(const double value)
{
  const double t = std::clamp(value, 0.0, 1.0);
  return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

struct QuinticHermiteSegment
{
  double c0{0.0};
  double c1{0.0};
  double c2{0.0};
  double c3{0.0};
  double c4{0.0};
  double c5{0.0};
  double length_m{1.0};
};

QuinticHermiteSegment makeQuinticHermiteSegment(
  const double start_offset, const double end_offset,
  const double start_slope, const double end_slope,
  const double length_m)
{
  QuinticHermiteSegment segment;
  segment.length_m = length_m;
  segment.c0 = start_offset;
  segment.c1 = start_slope * length_m;
  segment.c2 = 0.0;
  const double position_residual = end_offset - segment.c0 - segment.c1;
  const double slope_residual = end_slope * length_m - segment.c1;
  segment.c3 = 10.0 * position_residual - 4.0 * slope_residual;
  segment.c4 = -15.0 * position_residual + 7.0 * slope_residual;
  segment.c5 = 6.0 * position_residual - 3.0 * slope_residual;
  return segment;
}

double evaluateQuinticHermite(
  const QuinticHermiteSegment & segment, const double ratio)
{
  const double t = std::clamp(ratio, 0.0, 1.0);
  return segment.c0 + t *
         (segment.c1 + t *
         (segment.c2 + t *
         (segment.c3 + t * (segment.c4 + t * segment.c5))));
}

double quinticSpatialJerkCost(const QuinticHermiteSegment & segment)
{
  if (!std::isfinite(segment.length_m) || segment.length_m <= 1e-3) {
    return std::numeric_limits<double>::infinity();
  }
  const double a = 6.0 * segment.c3;
  const double b = 24.0 * segment.c4;
  const double c = 60.0 * segment.c5;
  const double normalized_integral =
    a * a + a * b + (b * b + 2.0 * a * c) / 3.0 + b * c / 2.0 + c * c / 5.0;
  return normalized_integral / std::pow(segment.length_m, 5.0);
}

}  // namespace

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options)
{
  initParameters();
  initInterfaces();
  RCLCPP_INFO(
    get_logger(),
    "LocalPlannerNode started (planning=%d wp, detection=%d wp, "
    "lattice=%zu primary/%zu recovery candidates)",
    lookahead_wpnt_num_, detection_lookahead_wpnt_num_,
    static_cast<std::size_t>(2 * lattice_lateral_samples_) *
    lattice_transition_scales_.size(),
    static_cast<std::size_t>(2 * lattice_recovery_lateral_samples_) *
    lattice_recovery_transition_scales_.size());
}

void LocalPlannerNode::initParameters()
{
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
  lattice_braking_buffer_wpnts_ =
    declare_parameter<int>("lattice_braking_buffer_wpnts", 8);
  lattice_braking_deceleration_mps2_ =
    declare_parameter<double>("lattice_braking_deceleration_mps2", 2.50);
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
    "obstacles_topic", "/perception/static_obstacles/cartesian");
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  ot_waypoints_topic_ =
    declare_parameter<std::string>("ot_waypoints_topic", "/avoid_waypoints");
  local_waypoints_topic_ =
    declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
  local_path_topic_ =
    declare_parameter<std::string>("local_path_topic", "/local_planning/path");
  exact_local_path_topic_ =
    declare_parameter<std::string>("exact_local_path_topic", "/local_path");
  marker_topic_ =
    declare_parameter<std::string>("marker_topic", "/local_planning/markers");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  lookahead_wpnt_num_ = std::max(4, lookahead_wpnt_num_);
  detection_lookahead_wpnt_num_ = std::clamp(
    detection_lookahead_wpnt_num_, 1, lookahead_wpnt_num_);
  spline_window_margin_wpnts_ = std::max(1, spline_window_margin_wpnts_);
  corner_spline_window_margin_wpnts_ = std::clamp(
    corner_spline_window_margin_wpnts_, 1, spline_window_margin_wpnts_);
  timer_period_ms_ = std::max(10, timer_period_ms_);
  debug_publish_period_ms_ = std::max(timer_period_ms_, debug_publish_period_ms_);
  detection_confirm_cycles_ = std::max(1, detection_confirm_cycles_);
  detection_clear_cycles_ = std::max(1, detection_clear_cycles_);
  frenet_odom_confirm_cycles_ = std::max(1, frenet_odom_confirm_cycles_);
  frenet_odom_invalid_grace_cycles_ = std::max(0, frenet_odom_invalid_grace_cycles_);
  frenet_odom_stale_timeout_sec_ = std::max(0.05, frenet_odom_stale_timeout_sec_);
  frenet_odom_path_hold_timeout_sec_ = std::max(
    frenet_odom_stale_timeout_sec_, frenet_odom_path_hold_timeout_sec_);
  frenet_odom_max_s_jump_m_ = std::max(0.05, frenet_odom_max_s_jump_m_);
  frenet_odom_speed_jump_scale_ = std::max(1.0, frenet_odom_speed_jump_scale_);
  frenet_odom_jump_slack_m_ = std::max(0.0, frenet_odom_jump_slack_m_);
  frenet_odom_track_margin_m_ = std::max(0.0, frenet_odom_track_margin_m_);
  occupied_threshold_ = std::clamp(occupied_threshold_, 0, 100);
  perception_obstacle_padding_m_ = std::max(0.0, perception_obstacle_padding_m_);
  perception_grid_update_period_sec_ = std::max(0.05, perception_grid_update_period_sec_);
  perception_grid_rebuild_motion_m_ = std::max(0.01, perception_grid_rebuild_motion_m_);
  perception_static_grid_match_gate_m_ = std::max(
    perception_grid_rebuild_motion_m_, perception_static_grid_match_gate_m_);
  perception_static_history_size_ = std::clamp(perception_static_history_size_, 2, 20);
  perception_static_expand_confirm_cycles_ = std::clamp(
    perception_static_expand_confirm_cycles_, 1, 20);
  perception_static_shrink_confirm_cycles_ = std::clamp(
    perception_static_shrink_confirm_cycles_, 1, 20);
  perception_filter_alpha_ = std::clamp(perception_filter_alpha_, 0.01, 1.0);
  perception_track_hold_sec_ = std::max(0.0, perception_track_hold_sec_);
  perception_association_gate_m_ = std::max(0.01, perception_association_gate_m_);
  perception_max_s_step_m_ = std::max(0.01, perception_max_s_step_m_);
  perception_max_d_step_m_ = std::max(0.01, perception_max_d_step_m_);
  perception_static_speed_threshold_mps_ = std::max(
    0.0, perception_static_speed_threshold_mps_);
  committed_obstacle_match_distance_m_ = std::max(
    0.01, committed_obstacle_match_distance_m_);
  corner_tracking_margin_ = std::max(0.0, corner_tracking_margin_);
  corner_avoid_offset_ = std::max(0.0, corner_avoid_offset_);
  corner_curvature_threshold_ = std::max(0.01, corner_curvature_threshold_);
  speed_reduction_ratio_ = std::clamp(speed_reduction_ratio_, 0.05, 1.0);
  corner_speed_reduction_ratio_ =
    std::clamp(corner_speed_reduction_ratio_, 0.05, 1.0);
  post_obstacle_speed_recovery_ratio_ = std::clamp(
    post_obstacle_speed_recovery_ratio_, speed_reduction_ratio_, 1.0);
  lattice_max_longitudinal_accel_mps2_ = std::max(
    0.1, lattice_max_longitudinal_accel_mps2_);
  lattice_max_longitudinal_decel_mps2_ = std::max(
    0.1, lattice_max_longitudinal_decel_mps2_);
  lattice_lateral_samples_ = std::clamp(lattice_lateral_samples_, 1, 12);
  lattice_lateral_step_m_ = std::max(0.0, lattice_lateral_step_m_);
  lattice_corridor_target_inset_m_ = std::max(0.0, lattice_corridor_target_inset_m_);
  lattice_narrow_corridor_width_threshold_m_ =
    std::max(0.0, lattice_narrow_corridor_width_threshold_m_);
  lattice_obstacle_cluster_gap_wpnts_ = std::max(0, lattice_obstacle_cluster_gap_wpnts_);
  lattice_corridor_knot_stride_wpnts_ = std::max(1, lattice_corridor_knot_stride_wpnts_);
  lattice_corridor_beam_width_ = std::max(1, lattice_corridor_beam_width_);
  lattice_corridor_validation_tolerance_m_ = std::clamp(
    lattice_corridor_validation_tolerance_m_, 0.0, 0.01);
  lattice_narrow_corridor_transition_scales_.erase(
    std::remove_if(
      lattice_narrow_corridor_transition_scales_.begin(),
      lattice_narrow_corridor_transition_scales_.end(),
      [](const double scale) {return !std::isfinite(scale) || scale <= 0.0;}),
    lattice_narrow_corridor_transition_scales_.end());
  lattice_transition_scales_.erase(
    std::remove_if(
      lattice_transition_scales_.begin(), lattice_transition_scales_.end(),
      [](const double scale) {return !std::isfinite(scale) || scale <= 0.0;}),
    lattice_transition_scales_.end());
  if (lattice_transition_scales_.empty()) {
    lattice_transition_scales_.push_back(1.0);
  }
  lattice_min_transition_wpnts_ = std::max(2, lattice_min_transition_wpnts_);
  lattice_recovery_lateral_samples_ = std::clamp(
    lattice_recovery_lateral_samples_, lattice_lateral_samples_, 16);
  lattice_recovery_lateral_step_m_ = std::max(0.0, lattice_recovery_lateral_step_m_);
  lattice_recovery_transition_scales_.erase(
    std::remove_if(
      lattice_recovery_transition_scales_.begin(),
      lattice_recovery_transition_scales_.end(),
      [](const double scale) {return !std::isfinite(scale) || scale <= 0.0;}),
    lattice_recovery_transition_scales_.end());
  if (lattice_recovery_transition_scales_.empty()) {
    lattice_recovery_transition_scales_ = lattice_transition_scales_;
  }
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
  lattice_braking_buffer_wpnts_ = std::max(0, lattice_braking_buffer_wpnts_);
  lattice_braking_deceleration_mps2_ = std::max(
    0.10, lattice_braking_deceleration_mps2_);
  lattice_replan_brake_timeout_sec_ = std::max(
    0.10, lattice_replan_brake_timeout_sec_);
  stationary_hold_point_spacing_m_ = std::clamp(
    stationary_hold_point_spacing_m_, 0.01, 0.10);
}

void LocalPlannerNode::initInterfaces()
{
  const auto transient_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  planning_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions planning_options;
  planning_options.callback_group = planning_callback_group_;
  rclcpp::SubscriptionOptions odom_options;
  odom_options.callback_group = odom_callback_group_;

  global_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    global_waypoints_topic_, transient_qos,
    std::bind(&LocalPlannerNode::onGlobalWaypoints, this, _1), planning_options);
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, transient_qos, std::bind(&LocalPlannerNode::onMap, this, _1),
    planning_options);
  obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    obstacles_topic_, rclcpp::QoS(10),
    std::bind(&LocalPlannerNode::onObstacles, this, _1), planning_options);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, volatile_qos, std::bind(&LocalPlannerNode::onOdom, this, _1),
    odom_options);

  ot_pub_ = create_publisher<f110_msgs::msg::OTWpntArray>(
    ot_waypoints_topic_, volatile_qos);
  local_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>(
    local_waypoints_topic_, volatile_qos);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    local_path_topic_, volatile_qos);
  exact_local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    exact_local_path_topic_, volatile_qos);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, volatile_qos);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(timer_period_ms_),
    std::bind(&LocalPlannerNode::onTimer, this), planning_callback_group_);
}

void LocalPlannerNode::onGlobalWaypoints(
  const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  if (!msg || msg->wpnts.size() < 3) {
    RCLCPP_WARN(get_logger(), "Received fewer than three global waypoints.");
    return;
  }

  for (std::size_t i = 0; i < msg->wpnts.size(); ++i) {
    const auto & waypoint = msg->wpnts[i];
    const bool finite = std::isfinite(waypoint.s_m) && std::isfinite(waypoint.d_m) &&
      std::isfinite(waypoint.x_m) && std::isfinite(waypoint.y_m) &&
      std::isfinite(waypoint.d_left) && std::isfinite(waypoint.d_right) &&
      std::isfinite(waypoint.psi_rad) && std::isfinite(waypoint.kappa_radpm) &&
      std::isfinite(waypoint.vx_mps) && std::isfinite(waypoint.ax_mps2);
    const bool ordered = i == 0 || waypoint.s_m > msg->wpnts[i - 1].s_m;
    if (!finite || !ordered) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting malformed global waypoints at index %zu (finite=%s, ordered_s=%s).",
        i, finite ? "true" : "false", ordered ? "true" : "false");
      return;
    }
  }

  const auto same_waypoint = [](const auto & lhs, const auto & rhs) {
      return lhs.id == rhs.id && lhs.s_m == rhs.s_m && lhs.d_m == rhs.d_m &&
             lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m &&
             lhs.d_right == rhs.d_right && lhs.d_left == rhs.d_left &&
             lhs.psi_rad == rhs.psi_rad && lhs.kappa_radpm == rhs.kappa_radpm &&
             lhs.vx_mps == rhs.vx_mps && lhs.ax_mps2 == rhs.ax_mps2;
    };
  const bool same_path = has_global_ &&
    msg->wpnts.size() == global_wpnts_.wpnts.size() &&
    std::equal(
    msg->wpnts.begin(), msg->wpnts.end(), global_wpnts_.wpnts.begin(), same_waypoint);
  if (same_path) {
    global_wpnts_.header = msg->header;
    return;
  }

  global_wpnts_ = *msg;
  has_global_ = true;
  invalidateSafeCorridorCache();
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_ready_ = false;
    has_previous_frenet_s_ = false;
    consecutive_valid_odom_count_ = 0;
  }
  avoidance_active_ = false;
  detection_count_ = 0;
  clear_count_ = 0;
  clearAvoidanceCommitment();
  has_held_avoidance_segment_ = false;
  held_avoidance_segment_.wpnts.clear();
  has_last_validated_full_path_ = false;
  last_validated_full_path_.wpnts.clear();
  if (has_map_ && has_obstacles_) {
    rebuildPlanningGrid();
  }
}

void LocalPlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
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
  if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring static Cartesian obstacles in frame '%s'; expected '%s'.",
      msg->header.frame_id.c_str(), frame_id_.c_str());
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
  for (const auto & raw_measurement : msg->obstacles) {
    if (!raw_measurement.has_cartesian ||
      !std::isfinite(raw_measurement.x_center) ||
      !std::isfinite(raw_measurement.y_center) ||
      !std::isfinite(raw_measurement.radius) ||
      raw_measurement.radius <= 0.0 ||
      !std::isfinite(raw_measurement.s_center) ||
      !std::isfinite(raw_measurement.d_center))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring a static obstacle without finite x/y/s/d and positive radius.");
      continue;
    }
    auto measurement = raw_measurement;
    // Preserve main's Frenet lattice and lateral filtering. The Cartesian enclosing-circle radius
    // is invariant under the local CLCS rotation, so use it as both longitudinal and lateral
    // half-extent before passing the obstacle into the existing filtered planning grid.
    measurement.size = 2.0 * measurement.radius;
    measurement.s_start = measurement.s_center - measurement.radius;
    measurement.s_end = measurement.s_center + measurement.radius;
    measurement.d_right = measurement.d_center - measurement.radius;
    measurement.d_left = measurement.d_center + measurement.radius;
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
    return true;
  }
  // opponent_detector needs several observations before promoting a new track
  // to is_static. At simulation speed, waiting for that bit can consume the
  // entire braking/avoidance distance. Admit a not-yet-promoted track only
  // when both Frenet velocity components already look stationary; moving
  // opponents remain outside the local static-obstacle planner.
  return std::isfinite(obstacle.vs) && std::isfinite(obstacle.vd) &&
         std::hypot(obstacle.vs, obstacle.vd) <=
         perception_static_speed_threshold_mps_;
}

bool LocalPlannerNode::obstacleCenterPose(
  const double s, const double d, double & x, double & y, double & yaw) const
{
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
  {
    return;
  }
  grid_map_ = std::move(updated);
  invalidateSafeCorridorCache();
  // A perception update must cause the committed route to be collision-checked
  // again, but should not erase it before that validation happens. Otherwise
  // small tracker jitter makes the planner alternate between global/local paths.
  if (has_avoidance_commitment_) {
    std::fill(committed_point_validated_.begin(), committed_point_validated_.end(), 0U);
    std::fill(committed_segment_validated_.begin(), committed_segment_validated_.end(), 0U);
  }

  rebuildObstacleMask();
  const double perception_to_grid_ms =
    last_perception_receive_time_.nanoseconds() == 0 ? -1.0 :
    (now() - last_perception_receive_time_).seconds() * 1000.0;
  RCLCPP_INFO(
    get_logger(),
    "DIAG planning_grid: rasterized_static=%zu perception_to_grid=%.3f ms "
    "update_period=%.3f s.",
    rasterized_obstacle_count, perception_to_grid_ms,
    perception_grid_update_period_sec_);

  // If a newly observed obstacle cuts into the held path, keep only its
  // collision-free prefix and turn it into a braking path. Do not erase the
  // handoff outright: an empty /avoid_waypoints makes the downstream selector
  // switch back to the colliding global path.
  if (has_held_avoidance_segment_) {
    const double clearance = vehicle_radius_ + path_clearance_margin_;
    bool held_path_safe = !held_avoidance_segment_.wpnts.empty();
    for (std::size_t i = 0; held_path_safe && i < held_avoidance_segment_.wpnts.size(); ++i) {
      const auto & waypoint = held_avoidance_segment_.wpnts[i];
      held_path_safe = isPathPointCollisionFree(
        waypoint.x_m, waypoint.y_m, waypoint.psi_rad, clearance);
      if (held_path_safe && i + 1U < held_avoidance_segment_.wpnts.size()) {
        const auto & next = held_avoidance_segment_.wpnts[i + 1U];
        held_path_safe = isPathSegmentCollisionFree(
          waypoint.x_m, waypoint.y_m, waypoint.psi_rad,
          next.x_m, next.y_m, next.psi_rad, clearance);
      }
    }
    if (!held_path_safe) {
      f110_msgs::msg::WpntArray braking_prefix;
      if (buildSegmentBrakingPrefix(held_avoidance_segment_, braking_prefix)) {
        held_avoidance_segment_ = braking_prefix;
        held_planner_name_ = "frenet_lattice_replan_brake";
        held_segment_time_ = now();
      } else {
        has_held_avoidance_segment_ = false;
        held_avoidance_segment_.wpnts.clear();
      }
    }
  }
  // Retain the previous full path as braking source. It is never replayed
  // directly: buildReplanBrakingSegment validates every point and segment
  // against this newly rebuilt grid before publishing a prefix.
}

void LocalPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!msg || !std::isfinite(msg->pose.pose.position.x) ||
    !std::isfinite(msg->pose.pose.position.y))
  {
    has_odom_ = true;
    ++consecutive_invalid_odom_count_;
    if (consecutive_invalid_odom_count_ > frenet_odom_invalid_grace_cycles_) {
      odom_ready_ = false;
      consecutive_valid_odom_count_ = 0;
    }
    return;
  }

  has_odom_ = true;

  if (!has_global_ || global_wpnts_.wpnts.size() < 3) {
    odom_ready_ = false;
    return;
  }

  const double current_s = msg->pose.pose.position.x;
  const double current_d = msg->pose.pose.position.y;
  const int closest_index = findClosestWaypointIndexByS(current_s);
  const auto & closest_waypoint = global_wpnts_.wpnts[closest_index];
  const bool has_track_bounds =
    closest_waypoint.d_left > 0.0 && closest_waypoint.d_right > 0.0;
  const double left_limit = std::max(
    0.0, closest_waypoint.d_left - vehicle_radius_) + frenet_odom_track_margin_m_;
  const double right_limit = std::max(
    0.0, closest_waypoint.d_right - vehicle_radius_) + frenet_odom_track_margin_m_;
  const bool within_track = !has_track_bounds ||
    (current_d <= left_limit && current_d >= -right_limit);

  bool continuous_s = true;
  if (has_previous_frenet_s_) {
    double s_step = std::abs(current_s - previous_frenet_s_);
    const double last_s = global_wpnts_.wpnts.back().s_m;
    const double closing_length = std::hypot(
      global_wpnts_.wpnts.front().x_m - global_wpnts_.wpnts.back().x_m,
      global_wpnts_.wpnts.front().y_m - global_wpnts_.wpnts.back().y_m);
    const double track_length = last_s + closing_length;
    if (track_length > 0.0) {
      s_step = std::fmod(s_step, track_length);
      s_step = std::min(s_step, track_length - s_step);
    }
    const double receive_dt = std::max(
      0.0, (now() - last_odom_receive_time_).seconds());
    const double measured_speed = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    const double dynamic_jump_limit =
      measured_speed * receive_dt * frenet_odom_speed_jump_scale_ +
      frenet_odom_jump_slack_m_;
    continuous_s = s_step <= std::max(
      frenet_odom_max_s_jump_m_, dynamic_jump_limit);
  }

  if (!within_track || !continuous_s) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting unstable Frenet odometry (s=%.3f, d=%.3f, track=%s, s_step=%s).",
      current_s, current_d, within_track ? "valid" : "invalid",
      continuous_s ? "valid" : "jump");
    // A deliberate localization reset is also a large Frenet jump. Keeping
    // the old previous_frenet_s_ would reject every following sample against
    // an obsolete pose. Seed a new confirmation sequence and let the planning
    // thread clear paths that were generated for the previous pose.
    if (within_track && !continuous_s) {
      latest_odom_ = *msg;
      last_odom_receive_time_ = now();
      previous_frenet_s_ = current_s;
      has_previous_frenet_s_ = true;
      odom_ready_ = false;
      consecutive_valid_odom_count_ = 1;
      consecutive_invalid_odom_count_ = 0;
      localization_generation_.fetch_add(1U, std::memory_order_acq_rel);
      localization_reset_pending_.store(true, std::memory_order_release);
      return;
    }
    ++consecutive_invalid_odom_count_;
    if (consecutive_invalid_odom_count_ > frenet_odom_invalid_grace_cycles_) {
      odom_ready_ = false;
      consecutive_valid_odom_count_ = 0;
    }
    return;
  }

  latest_odom_ = *msg;
  last_odom_receive_time_ = now();
  static rclcpp::Clock accepted_odom_log_clock(RCL_STEADY_TIME);
  RCLCPP_INFO_THROTTLE(
    get_logger(), accepted_odom_log_clock, 1000,
    "DIAG odom accepted: s=%.3f d=%.3f valid_cycles=%d/%d.",
    current_s, current_d, consecutive_valid_odom_count_ + 1,
    frenet_odom_confirm_cycles_);
  consecutive_invalid_odom_count_ = 0;
  consecutive_valid_odom_count_ = std::min(
    consecutive_valid_odom_count_ + 1, frenet_odom_confirm_cycles_);
  odom_ready_ = consecutive_valid_odom_count_ >= frenet_odom_confirm_cycles_;
  previous_frenet_s_ = current_s;
  has_previous_frenet_s_ = true;
}

void LocalPlannerNode::rebuildObstacleMask()
{
  const int width = static_cast<int>(grid_map_.info.width);
  const int height = static_cast<int>(grid_map_.info.height);
  const int cell_count = width * height;
  obstacle_mask_.assign(static_cast<std::size_t>(cell_count), 0U);
  obstacle_component_ids_.assign(static_cast<std::size_t>(cell_count), -1);
  obstacle_centroids_.clear();

  if (static_cast<int>(grid_map_.data.size()) != cell_count) {
    grid_clearance_m_.clear();
    RCLCPP_ERROR(get_logger(), "Occupancy grid data size does not match map dimensions.");
    return;
  }

  // Cache the complete occupancy mask without component-area filtering. The
  // safe-corridor builder reads the same full grid directly, so wall-attached
  // and large obstacles cannot disappear due to visualization grouping.
  for (int index = 0; index < cell_count; ++index) {
    const bool unknown_blocks = grid_map_.data[index] < 0 &&
      unknown_cell_policy_ != UnknownCellPolicy::kTreatAsFree;
    if (grid_map_.data[index] > occupied_threshold_ || unknown_blocks) {
      obstacle_mask_[index] = 1U;
    }
  }

  std::vector<uint8_t> visited(static_cast<std::size_t>(cell_count), 0U);
  int rejected_large = 0;
  int rejected_small = 0;
  constexpr int kNeighbors[8][2] = {
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
    {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

  for (int seed = 0; seed < cell_count; ++seed) {
    if (visited[seed] != 0U || grid_map_.data[seed] <= occupied_threshold_) {
      continue;
    }

    std::deque<int> queue;
    std::vector<int> cells;
    queue.push_back(seed);
    visited[seed] = 1U;
    double column_sum = 0.0;
    double row_sum = 0.0;

    while (!queue.empty()) {
      const int index = queue.front();
      queue.pop_front();
      cells.push_back(index);
      const int column = index % width;
      const int row = index / width;
      column_sum += static_cast<double>(column);
      row_sum += static_cast<double>(row);

      for (const auto & neighbor : kNeighbors) {
        const int next_column = column + neighbor[0];
        const int next_row = row + neighbor[1];
        if (next_column < 0 || next_column >= width || next_row < 0 || next_row >= height) {
          continue;
        }
        const int next_index = next_row * width + next_column;
        if (visited[next_index] != 0U ||
          grid_map_.data[next_index] <= occupied_threshold_)
        {
          continue;
        }
        visited[next_index] = 1U;
        queue.push_back(next_index);
      }
    }

    const double area = static_cast<double>(cells.size()) *
      grid_map_.info.resolution * grid_map_.info.resolution;
    if (area < obstacle_component_min_area_m2_) {
      ++rejected_small;
      continue;
    }
    if (area > obstacle_component_max_area_m2_) {
      ++rejected_large;
      continue;
    }

    const int component_id = static_cast<int>(obstacle_centroids_.size());
    for (const int index : cells) {
      obstacle_component_ids_[index] = component_id;
    }

    const double mean_column = column_sum / static_cast<double>(cells.size());
    const double mean_row = row_sum / static_cast<double>(cells.size());
    const double local_x = (mean_column + 0.5) * grid_map_.info.resolution;
    const double local_y = (mean_row + 0.5) * grid_map_.info.resolution;
    geometry_msgs::msg::Point centroid;
    centroid.x = grid_map_.info.origin.position.x +
      map_origin_cos_ * local_x - map_origin_sin_ * local_y;
    centroid.y = grid_map_.info.origin.position.y +
      map_origin_sin_ * local_x + map_origin_cos_ * local_y;
    centroid.z = 0.12;
    obstacle_centroids_.push_back(centroid);
  }

  rebuildClearanceField();

  RCLCPP_INFO(
    get_logger(),
    "Map components classified: obstacles=%zu, walls=%d, noise=%d",
    obstacle_centroids_.size(), rejected_large, rejected_small);
}

void LocalPlannerNode::rebuildClearanceField()
{
  const int width = static_cast<int>(grid_map_.info.width);
  const int height = static_cast<int>(grid_map_.info.height);
  const int cell_count = width * height;
  const double infinity = std::numeric_limits<double>::infinity();
  grid_clearance_m_.assign(static_cast<std::size_t>(cell_count), infinity);

  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  for (int index = 0; index < cell_count; ++index) {
    const bool unknown_blocks = grid_map_.data[index] < 0 &&
      unknown_cell_policy_ != UnknownCellPolicy::kTreatAsFree;
    if (grid_map_.data[index] > occupied_threshold_ || unknown_blocks) {
      grid_clearance_m_[index] = 0.0;
      queue.emplace(0.0, index);
    }
  }

  constexpr int kNeighbors[8][2] = {
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
    {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
  const double resolution = grid_map_.info.resolution;

  while (!queue.empty()) {
    const auto [distance, index] = queue.top();
    queue.pop();
    if (distance > grid_clearance_m_[index]) {
      continue;
    }
    const int column = index % width;
    const int row = index / width;
    for (const auto & neighbor : kNeighbors) {
      const int next_column = column + neighbor[0];
      const int next_row = row + neighbor[1];
      if (next_column < 0 || next_column >= width || next_row < 0 || next_row >= height) {
        continue;
      }
      const int next_index = next_row * width + next_column;
      const bool diagonal = neighbor[0] != 0 && neighbor[1] != 0;
      const double step = resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      const double next_distance = distance + step;
      if (next_distance < grid_clearance_m_[next_index]) {
        grid_clearance_m_[next_index] = next_distance;
        queue.emplace(next_distance, next_index);
      }
    }
  }
}

bool LocalPlannerNode::worldToMap(
  const double x, const double y, int & column, int & row) const
{
  if (!has_map_ || grid_map_.info.resolution <= 0.0) {
    return false;
  }
  const double dx = x - grid_map_.info.origin.position.x;
  const double dy = y - grid_map_.info.origin.position.y;
  const double local_x = map_origin_cos_ * dx + map_origin_sin_ * dy;
  const double local_y = -map_origin_sin_ * dx + map_origin_cos_ * dy;
  column = static_cast<int>(std::floor(local_x / grid_map_.info.resolution));
  row = static_cast<int>(std::floor(local_y / grid_map_.info.resolution));
  return column >= 0 && column < static_cast<int>(grid_map_.info.width) &&
         row >= 0 && row < static_cast<int>(grid_map_.info.height);
}

geometry_msgs::msg::Point LocalPlannerNode::mapCellCenter(
  const int column, const int row) const
{
  const double local_x = (static_cast<double>(column) + 0.5) * grid_map_.info.resolution;
  const double local_y = (static_cast<double>(row) + 0.5) * grid_map_.info.resolution;
  geometry_msgs::msg::Point point;
  point.x = grid_map_.info.origin.position.x +
    map_origin_cos_ * local_x - map_origin_sin_ * local_y;
  point.y = grid_map_.info.origin.position.y +
    map_origin_sin_ * local_x + map_origin_cos_ * local_y;
  point.z = 0.12;
  return point;
}

double LocalPlannerNode::clearanceAtWorld(const double x, const double y) const
{
  int column = 0;
  int row = 0;
  if (!worldToMap(x, y, column, row) || grid_clearance_m_.empty()) {
    return 0.0;
  }
  const int width = static_cast<int>(grid_map_.info.width);
  const int height = static_cast<int>(grid_map_.info.height);
  const double boundary_cells = static_cast<double>(std::min(
      std::min(column, width - 1 - column),
      std::min(row, height - 1 - row))) + 0.5;
  const double boundary_clearance = boundary_cells * grid_map_.info.resolution;
  return std::min(
    grid_clearance_m_[row * width + column], boundary_clearance);
}

int LocalPlannerNode::findClosestWaypointIndexByS(const double raw_s) const
{
  const auto & waypoints = global_wpnts_.wpnts;
  if (waypoints.empty()) {
    return 0;
  }
  if (waypoints.size() == 1) {
    return 0;
  }

  const double last_gap = std::max(
    1e-3, waypoints.back().s_m - waypoints[waypoints.size() - 2].s_m);
  const double track_length = std::max(last_gap, waypoints.back().s_m + last_gap);
  double s = std::fmod(raw_s, track_length);
  if (s < 0.0) {
    s += track_length;
  }

  const auto iterator = std::lower_bound(
    waypoints.begin(), waypoints.end(), s,
    [](const f110_msgs::msg::Wpnt & waypoint, const double value) {
      return waypoint.s_m < value;
    });

  std::vector<int> candidates{0, static_cast<int>(waypoints.size() - 1)};
  if (iterator != waypoints.end()) {
    candidates.push_back(static_cast<int>(std::distance(waypoints.begin(), iterator)));
  }
  if (iterator != waypoints.begin()) {
    candidates.push_back(static_cast<int>(std::distance(waypoints.begin(), iterator - 1)));
  }

  int best_index = candidates.front();
  double best_distance = std::numeric_limits<double>::max();
  for (const int index : candidates) {
    double distance = std::abs(waypoints[index].s_m - s);
    distance = std::min(distance, track_length - distance);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

void LocalPlannerNode::invalidateSafeCorridorCache()
{
  safe_corridor_sample_cache_.clear();
  safe_corridor_sample_cache_valid_.clear();
  safe_corridor_ = SafeCorridorResult{};
}

SafeCorridorResult LocalPlannerNode::buildSafeCorridor(
  const int start_idx, const int count)
{
  SafeCorridorResult result;
  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (total < 2 || count <= 0) {
    return result;
  }
  if (safe_corridor_sample_cache_.size() != static_cast<std::size_t>(total)) {
    safe_corridor_sample_cache_.resize(static_cast<std::size_t>(total));
    safe_corridor_sample_cache_valid_.assign(static_cast<std::size_t>(total), 0U);
  }

  SafeCorridorConfig config;
  config.occupied_threshold = occupied_threshold_;
  config.unknown_policy = unknown_cell_policy_;
  config.minimum_longitudinal_half_width_m = longitudinal_search_half_width_m_;
  config.vehicle_radius_m = vehicle_radius_;
  config.path_clearance_margin_m = path_clearance_margin_;
  config.vehicle_front_extent_m = vehicle_front_extent_m_;
  config.vehicle_rear_extent_m = vehicle_rear_extent_m_;
  config.vehicle_width_m = vehicle_width_m_;
  config.localization_margin_m = localization_margin_m_;
  config.safety_margin_m = corridor_safety_margin_m_;
  config.preserve_circular_collision_check = preserve_circular_collision_check_;
  const SafeCorridorBuilder builder(config);

  result.samples.reserve(static_cast<std::size_t>(count));
  std::unordered_set<int> inflated_cells;
  double unwrapped_s = 0.0;
  for (int k = 0; k < count; ++k) {
    const int index = (start_idx + k) % total;
    const int previous = (index - 1 + total) % total;
    const int next = (index + 1) % total;
    const auto & waypoint = global_wpnts_.wpnts[index];
    const auto & previous_waypoint = global_wpnts_.wpnts[previous];
    const auto & next_waypoint = global_wpnts_.wpnts[next];
    const double previous_spacing = std::hypot(
      waypoint.x_m - previous_waypoint.x_m,
      waypoint.y_m - previous_waypoint.y_m);
    const double next_spacing = std::hypot(
      next_waypoint.x_m - waypoint.x_m,
      next_waypoint.y_m - waypoint.y_m);
    if (k > 0) {
      unwrapped_s += previous_spacing;
    }
    const double minimum_spacing = std::max(
      1e-3, std::min(previous_spacing, next_spacing));
    const double coverage_spacing = std::min(
      std::max(previous_spacing, next_spacing), 3.0 * minimum_spacing);
    if (safe_corridor_sample_cache_valid_[static_cast<std::size_t>(index)] == 0U) {
      const std::vector<CorridorReferenceSample> reference{
        CorridorReferenceSample{
          0, index, 0.0, waypoint.x_m, waypoint.y_m, waypoint.psi_rad,
          waypoint.d_left, waypoint.d_right, 0.5 * coverage_spacing}};
      auto built = builder.build(grid_map_, reference);
      if (built.samples.size() != 1U) {
        return SafeCorridorResult{};
      }
      safe_corridor_sample_cache_[static_cast<std::size_t>(index)] =
        std::move(built.samples.front());
      safe_corridor_sample_cache_valid_[static_cast<std::size_t>(index)] = 1U;
    }

    auto sample = safe_corridor_sample_cache_[static_cast<std::size_t>(index)];
    sample.reference.waypoint_offset = k;
    sample.reference.unwrapped_s = unwrapped_s;
    inflated_cells.insert(
      sample.inflated_cell_indices.begin(), sample.inflated_cell_indices.end());
    result.samples.push_back(std::move(sample));
  }
  result.inflated_cell_indices.assign(inflated_cells.begin(), inflated_cells.end());
  std::sort(result.inflated_cell_indices.begin(), result.inflated_cell_indices.end());
  return result;
}

bool LocalPlannerNode::isCandidateInsideSafeCorridor(
  const int waypoint_offset, const double d) const
{
  if (waypoint_offset < 0 ||
    waypoint_offset >= static_cast<int>(safe_corridor_.samples.size()))
  {
    return false;
  }
  const auto & sample = safe_corridor_.samples[static_cast<std::size_t>(waypoint_offset)];
  const bool inside = std::any_of(
    sample.feasible_intervals.begin(), sample.feasible_intervals.end(),
    [&](const LateralInterval & interval) {
      return d >= interval.min_d - lattice_corridor_validation_tolerance_m_ &&
      d <= interval.max_d + lattice_corridor_validation_tolerance_m_;
    });
  if (!inside && rejection_counts_.first_corridor_offset < 0) {
    rejection_counts_.first_corridor_offset = waypoint_offset;
    rejection_counts_.first_corridor_d = d;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto & interval : sample.feasible_intervals) {
      const double distance = d < interval.min_d ? interval.min_d - d :
        (d > interval.max_d ? d - interval.max_d : 0.0);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        rejection_counts_.first_corridor_min_d = interval.min_d;
        rejection_counts_.first_corridor_max_d = interval.max_d;
      }
    }
  }
  return inside;
}

void LocalPlannerNode::detectObstaclesAndDecideDirection(
  const int start_idx, const int count,
  bool & obs_detected, bool & avoid_left,
  std::vector<int> & collision_indices,
  std::vector<geometry_msgs::msg::Point> & obstacle_points) const
{
  obs_detected = false;
  avoid_left = true;
  collision_indices.clear();
  obstacle_points.clear();
  if (!has_map_ || safe_corridor_.samples.size() < static_cast<std::size_t>(count)) {
    return;
  }

  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  const bool inspect_committed_path = has_avoidance_commitment_ &&
    total > 0 && static_cast<int>(committed_avoidance_path_.wpnts.size()) == total;
  double total_room_left = 0.0;
  double total_room_right = 0.0;
  int center_collision_count = 0;
  int committed_path_collision_count = 0;
  int first_committed_path_collision_offset = -1;
  double first_committed_path_collision_d = 0.0;
  std::unordered_set<int32_t> detected_components;

  for (int k = 0; k < count; ++k) {
    const auto & corridor = safe_corridor_.samples[static_cast<std::size_t>(k)];
    bool committed_path_blocked = false;
    double active_path_d = 0.0;
    if (inspect_committed_path) {
      const int index = (start_idx + k) % total;
      active_path_d = committed_avoidance_path_.wpnts[static_cast<std::size_t>(index)].d_m;
      const bool active_path_inside_corridor = std::any_of(
        corridor.feasible_intervals.begin(), corridor.feasible_intervals.end(),
        [this, active_path_d](const LateralInterval & interval) {
          return active_path_d >= interval.min_d - lattice_corridor_validation_tolerance_m_ &&
          active_path_d <= interval.max_d + lattice_corridor_validation_tolerance_m_;
        });
      committed_path_blocked = !active_path_inside_corridor;
    }
    if (!corridor.center_blocked && !committed_path_blocked) {
      continue;
    }
    if (corridor.center_blocked) {
      ++center_collision_count;
    }
    if (committed_path_blocked) {
      ++committed_path_collision_count;
      if (first_committed_path_collision_offset < 0) {
        first_committed_path_collision_offset = k;
        first_committed_path_collision_d = active_path_d;
      }
    }
    obs_detected = true;
    collision_indices.push_back(k);
    for (const auto & interval : corridor.feasible_intervals) {
      total_room_left += std::max(0.0, interval.max_d - std::max(0.0, interval.min_d));
      total_room_right += std::max(0.0, std::min(0.0, interval.max_d) - interval.min_d);
    }
  }

  const double ego_d = has_odom_ && std::isfinite(current_odom_.pose.pose.position.y) ?
    current_odom_.pose.pose.position.y : 0.0;
  // Prefer the side with the larger collision-span corridor. Following the
  // ego's current sign of d is unsafe on a tight bend: an inside offset near
  // the reference radius creates a Frenet parallel-curve cusp even when the
  // vehicle already happens to be on that side.
  avoid_left = total_room_left >= total_room_right;
  static rclcpp::Clock direction_log_clock(RCL_STEADY_TIME);
  RCLCPP_INFO_THROTTLE(
    get_logger(), direction_log_clock, 1000,
    "Avoidance direction input: ego_d=%.3f, room_left=%.3f, room_right=%.3f, "
    "preferred=%s, collision_samples=%zu.",
    ego_d, total_room_left, total_room_right, avoid_left ? "left" : "right",
    collision_indices.size());
  for (const int map_index : safe_corridor_.inflated_cell_indices) {
    if (map_index < 0 || map_index >= static_cast<int>(obstacle_component_ids_.size())) {
      continue;
    }
    const int32_t component_id = obstacle_component_ids_[map_index];
    if (component_id >= 0) {
      detected_components.insert(component_id);
    }
  }
  obstacle_points.reserve(detected_components.size());
  for (const int32_t component_id : detected_components) {
    if (component_id >= 0 &&
      component_id < static_cast<int32_t>(obstacle_centroids_.size()))
    {
      obstacle_points.push_back(obstacle_centroids_[component_id]);
    }
  }
  if (committed_path_collision_count > 0) {
    static rclcpp::Clock active_path_log_clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(
      get_logger(), active_path_log_clock, 1000,
      "Active-path obstacle trigger: committed_path=%d samples, global_center=%d "
      "samples, first_offset=%d, committed_d=%.3f m. Replanning the swept "
      "local route instead of checking only d=0.",
      committed_path_collision_count, center_collision_count,
      first_committed_path_collision_offset, first_committed_path_collision_d);
  }
}

bool LocalPlannerNode::buildFrenetLatticeCandidate(
  const int start_idx, const int count,
  const std::vector<int> & collision_indices,
  const bool preferred_left,
  f110_msgs::msg::WpntArray & candidate,
  int & segment_count,
  bool & selected_left,
  double & selected_cost,
  const bool recovery_mode) const
{
  // Report each primary/recovery search independently. Previously recovery
  // logs included primary rejection counts, which made the totals exceed the
  // number of generated recovery candidates.
  rejection_counts_ = CandidateRejectionCounts{};
  if (collision_indices.empty()) {
    return false;
  }

  const int search_budget_ms = recovery_mode ?
    lattice_recovery_search_budget_ms_ : lattice_primary_search_budget_ms_;
  bool budget_exhausted = false;

  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  const int collision_begin = collision_indices.front();
  int collision_end = collision_begin;
  for (std::size_t index = 1; index < collision_indices.size(); ++index) {
    const int current = collision_indices[index];
    if (current - collision_end > lattice_obstacle_cluster_gap_wpnts_) {
      break;
    }
    collision_end = current;
  }
  RCLCPP_DEBUG(
    get_logger(),
    "Nearest corridor collision cluster [%d, %d] within %d waypoint horizon "
    "(%zu total blocking samples).",
    collision_begin, collision_end, count, collision_indices.size());
  bool corner_collision = false;
  int corner_trigger_offset = -1;
  int corner_trigger_index = -1;
  double corner_trigger_curvature = 0.0;
  for (const int k : collision_indices) {
    if (k > collision_end) {
      break;
    }
    const int index = (start_idx + k) % total;
    if (std::abs(global_wpnts_.wpnts[index].kappa_radpm) >=
      corner_curvature_threshold_)
    {
      corner_collision = true;
      corner_trigger_offset = k;
      corner_trigger_index = index;
      corner_trigger_curvature = global_wpnts_.wpnts[index].kappa_radpm;
      break;
    }
  }

  const double minimum_offset = corner_collision ? corner_avoid_offset_ : avoid_offset_;
  // The safe corridor has already expanded every occupied cell by the vehicle
  // footprint, localization allowance, path clearance and corridor safety
  // margin. Adding the legacy safety_margin to the raw obstacle edge here
  // applies the clearance a second time, forcing the target toward a wall and
  // making an otherwise feasible corner require excessive curvature. Treat
  // avoid_offset only as a preference; sampleTargetOffsets clamps it to the
  // already collision-safe corridor interior.
  const double preferred_offset = std::max(minimum_offset, safety_margin_);
  const double required_left_offset = preferred_offset;
  const double required_right_offset = -preferred_offset;

  const int nominal_margin = corner_collision ?
    corner_spline_window_margin_wpnts_ : spline_window_margin_wpnts_;
  static rclcpp::Clock lattice_geometry_log_clock(RCL_STEADY_TIME);
  RCLCPP_INFO_THROTTLE(
    get_logger(), lattice_geometry_log_clock, 1000,
    "Frenet lattice geometry: corner_collision=%s, nominal_margin=%d, "
    "collision_begin=%d, collision_end=%d, ego_s=%.3f, ego_d=%.3f, preferred=%s, "
    "trigger_offset=%d, trigger_index=%d, trigger_kappa=%.3f rad/m.",
    corner_collision ? "true" : "false", nominal_margin,
    collision_begin, collision_end,
    has_odom_ ? current_odom_.pose.pose.position.x :
    std::numeric_limits<double>::quiet_NaN(),
    has_odom_ ? current_odom_.pose.pose.position.y :
    std::numeric_limits<double>::quiet_NaN(),
    preferred_left ? "left" : "right",
    corner_trigger_offset, corner_trigger_index, corner_trigger_curvature);
  const int lateral_samples = recovery_mode ?
    lattice_recovery_lateral_samples_ : lattice_lateral_samples_;
  const auto & transition_scales = recovery_mode ?
    lattice_recovery_transition_scales_ : lattice_transition_scales_;
  const int minimum_transition = recovery_mode ?
    lattice_recovery_min_transition_wpnts_ : lattice_min_transition_wpnts_;
  LatticeCandidate best_preferred;
  LatticeCandidate best_opposite;
  int generated_count = 0;
  int feasible_count = 0;

  for (const bool avoid_left : {preferred_left, !preferred_left}) {
    // Give both sides a complete, independent budget. A shared deadline let
    // the preferred side consume the whole search and silently skipped the
    // opposite side, even when that was the only drivable passage.
    const auto side_search_begin = std::chrono::steady_clock::now();
    const auto search_budget_exhausted = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - side_search_begin).count() >=
               search_budget_ms;
      };
    bool side_budget_exhausted = false;
    bool recovery_candidate_found = false;
    const double required_offset = avoid_left ? required_left_offset : required_right_offset;
    const auto common_intervals = SafeCorridorBuilder::intersectFeasibleIntervals(
      safe_corridor_.samples, collision_begin, collision_end, avoid_left);
    std::vector<std::vector<LateralTargetKnot>> target_profiles;
    const auto common_targets = SafeCorridorBuilder::sampleTargetOffsets(
      common_intervals, required_offset, lateral_samples, lattice_corridor_target_inset_m_);
    if (!common_intervals.empty()) {
      static rclcpp::Clock corridor_target_log_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(
        get_logger(), corridor_target_log_clock, 1000,
        "DIAG corridor target: mode=%s side=%s common=[%.3f, %.3f] "
        "required=%.3f first_target=%.3f targets=%zu.",
        recovery_mode ? "recovery" : "primary", avoid_left ? "left" : "right",
        common_intervals.front().min_d, common_intervals.back().max_d,
        required_offset,
        common_targets.empty() ? std::numeric_limits<double>::quiet_NaN() :
        common_targets.front(), common_targets.size());
    }
    for (const double target : common_targets) {
      std::vector<LateralTargetKnot> profile{{collision_begin, target}};
      if (collision_end != collision_begin) {
        profile.push_back({collision_end, target});
      }
      target_profiles.push_back(std::move(profile));
    }

    // A hairpin or diagonal passage may have a common constant-d interval yet
    // still reject every single-target quintic on curvature or corridor
    // continuity. In recovery mode, evaluate corridor-guided piecewise
    // profiles alongside the constant targets instead of waiting for the
    // common intersection to disappear completely.
    if (target_profiles.empty() || recovery_mode) {
      auto guided_profiles = SafeCorridorBuilder::buildCorridorGuidedProfiles(
        safe_corridor_.samples, collision_begin, collision_end, avoid_left,
        required_offset, lateral_samples, lattice_corridor_target_inset_m_,
        lattice_corridor_knot_stride_wpnts_, lattice_corridor_beam_width_);
      std::size_t guided_insert_position = 0U;
      for (auto & guided : guided_profiles) {
        const bool duplicate = std::any_of(
          target_profiles.begin(), target_profiles.end(),
          [&guided](const auto & existing) {
            if (existing.size() != guided.size()) {
              return false;
            }
            for (std::size_t i = 0; i < existing.size(); ++i) {
              if (existing[i].waypoint_offset != guided[i].waypoint_offset ||
              std::abs(existing[i].d - guided[i].d) > 1e-6)
              {
                return false;
              }
            }
            return true;
          });
        if (!duplicate) {
          // Recovery exists specifically for a corridor whose shape cannot be represented by
          // one constant lateral offset. Put guided profiles before the legacy constant targets;
          // the bounded recovery prefix below otherwise truncates them without evaluating even
          // one. That repeatedly retries the same failed quintics until the vehicle reaches the
          // obstacle.
          if (recovery_mode) {
            target_profiles.insert(
              target_profiles.begin() + static_cast<std::ptrdiff_t>(guided_insert_position),
              std::move(guided));
            ++guided_insert_position;
          } else {
            target_profiles.push_back(std::move(guided));
          }
        }
      }
    }
    if (target_profiles.empty()) {
      RCLCPP_DEBUG(
        get_logger(), "No %s safe-corridor passage at the nearest obstacle cluster.",
        avoid_left ? "left" : "right");
      continue;
    }

    double maximum_common_width = 0.0;
    if (!common_intervals.empty()) {
      for (const auto & interval : common_intervals) {
        maximum_common_width = std::max(
          maximum_common_width, interval.max_d - interval.min_d);
      }
    } else {
      maximum_common_width = std::numeric_limits<double>::infinity();
      for (const int sample_index : {collision_begin, collision_end}) {
        const auto endpoint_intervals = SafeCorridorBuilder::intersectFeasibleIntervals(
          safe_corridor_.samples, sample_index, sample_index, avoid_left);
        double endpoint_width = 0.0;
        for (const auto & interval : endpoint_intervals) {
          endpoint_width = std::max(endpoint_width, interval.max_d - interval.min_d);
        }
        maximum_common_width = std::min(maximum_common_width, endpoint_width);
      }
    }
    std::vector<double> active_transition_scales = transition_scales;
    if (maximum_common_width < lattice_narrow_corridor_width_threshold_m_) {
      for (const double scale : lattice_narrow_corridor_transition_scales_) {
        const bool duplicate = std::any_of(
          active_transition_scales.begin(), active_transition_scales.end(),
          [scale](const double existing) {return std::abs(existing - scale) <= 1e-6;});
        if (!duplicate) {
          active_transition_scales.push_back(scale);
        }
      }
    }

    if (!recovery_mode) {
      for (const auto & target_profile : target_profiles) {
        for (const double transition_scale : active_transition_scales) {
          if (search_budget_exhausted()) {
            side_budget_exhausted = true;
            budget_exhausted = true;
            break;
          }
          const int transition_margin = std::max(
            minimum_transition,
            static_cast<int>(std::lround(
              static_cast<double>(nominal_margin) * transition_scale)));
          LatticeCandidate current;
          ++generated_count;
          if (!buildQuinticLatticePath(
              start_idx, count, collision_begin, collision_end,
              target_profile,
              transition_margin, avoid_left, current))
          {
            continue;
          }
          const int window_begin = std::max(0, collision_begin - transition_margin);
          const int window_end = std::min(count - 1, collision_end + transition_margin);
          if (!evaluateLatticeCandidate(
              current, start_idx, window_begin, window_end, preferred_left, recovery_mode))
          {
            continue;
          }
          ++feasible_count;
          auto & side_best = current.avoid_left == preferred_left ?
            best_preferred : best_opposite;
          if (current.cost < side_best.cost) {
            side_best = std::move(current);
          }
        }
        if (side_budget_exhausted) {
          break;
        }
      }
    }

    // Recovery must guide the entry and merge through the full safe corridor
    // even when one constant-d interval happens to span the obstacle itself.
    // A common obstacle interval says nothing about whether the transition
    // from the ego pose to that interval remains inside the corridor. The old
    // common_targets.empty() guard therefore retried obstacle-only quintics
    // that either overshot the entry corridor or required excessive curvature.
    bool full_horizon_feasible = false;
    if (recovery_mode) {
      const double initial_offset = has_odom_ &&
        std::isfinite(current_odom_.pose.pose.position.y) ?
        current_odom_.pose.pose.position.y : 0.0;
      for (const double transition_scale : active_transition_scales) {
        if (search_budget_exhausted()) {
          side_budget_exhausted = true;
          budget_exhausted = true;
          break;
        }
        const int transition_margin = std::max(
          minimum_transition,
          static_cast<int>(std::lround(
            static_cast<double>(nominal_margin) * transition_scale)));
        const int window_begin = std::max(0, collision_begin - transition_margin);
        const int window_end = std::min(count - 1, collision_end + transition_margin);
        const double start_offset = window_begin == 0 ? initial_offset : 0.0;
        const auto full_profiles = SafeCorridorBuilder::buildFullHorizonGuidedProfiles(
          safe_corridor_.samples, window_begin, collision_begin, collision_end,
          window_end, avoid_left, start_offset, 0.0, required_offset,
          lateral_samples, lattice_corridor_target_inset_m_,
          lattice_corridor_knot_stride_wpnts_, lattice_corridor_beam_width_);
        if (full_profiles.empty()) {
          continue;
        }
        // The beam is already ordered by corridor quality. Checking only the
        // first profile made recovery brittle: a single curvature rejection
        // discarded the remaining geometrically different passages. Evaluate
        // a small prefix to retain diversity without restoring the old
        // hundreds-of-candidates latency.
        const std::size_t profile_count = std::min(
          full_profiles.size(),
          static_cast<std::size_t>(lattice_recovery_profile_limit_));
        for (std::size_t profile_index = 0; profile_index < profile_count; ++profile_index) {
          if (search_budget_exhausted()) {
            side_budget_exhausted = true;
            budget_exhausted = true;
            break;
          }
          LatticeCandidate current;
          ++generated_count;
          if (!buildQuinticLatticePath(
              start_idx, count, collision_begin, collision_end,
              full_profiles[profile_index], transition_margin, avoid_left, current) ||
            !evaluateLatticeCandidate(
              current, start_idx, window_begin, window_end,
              preferred_left, recovery_mode))
          {
            continue;
          }
          ++feasible_count;
          full_horizon_feasible = true;
          auto & side_best = current.avoid_left == preferred_left ?
            best_preferred : best_opposite;
          if (current.cost < side_best.cost) {
            side_best = std::move(current);
          }
          recovery_candidate_found = true;
          break;
        }
        if (recovery_candidate_found) {
          break;
        }
      }
    }

    // Retain the simpler obstacle-only quintics as a compatibility fallback,
    // but do not spend the remaining budget on them after a full-horizon path
    // has already passed every geometry and collision check on this side.
    if (recovery_mode && !common_targets.empty() &&
      !full_horizon_feasible && !side_budget_exhausted && !recovery_candidate_found)
    {
      const std::size_t profile_count = std::min(
        target_profiles.size(), static_cast<std::size_t>(lateral_samples));
      for (std::size_t profile_index = 0; profile_index < profile_count; ++profile_index) {
        for (const double transition_scale : active_transition_scales) {
          if (search_budget_exhausted()) {
            side_budget_exhausted = true;
            budget_exhausted = true;
            break;
          }
          const int transition_margin = std::max(
            minimum_transition,
            static_cast<int>(std::lround(
              static_cast<double>(nominal_margin) * transition_scale)));
          LatticeCandidate current;
          ++generated_count;
          if (!buildQuinticLatticePath(
              start_idx, count, collision_begin, collision_end,
              target_profiles[profile_index], transition_margin, avoid_left, current))
          {
            continue;
          }
          const int window_begin = std::max(0, collision_begin - transition_margin);
          const int window_end = std::min(count - 1, collision_end + transition_margin);
          if (!evaluateLatticeCandidate(
              current, start_idx, window_begin, window_end,
              preferred_left, recovery_mode))
          {
            continue;
          }
          ++feasible_count;
          auto & side_best = current.avoid_left == preferred_left ?
            best_preferred : best_opposite;
          if (current.cost < side_best.cost) {
            side_best = std::move(current);
          }
        }
        if (side_budget_exhausted) {
          break;
        }
      }
    }
  }
  if (!std::isfinite(best_preferred.cost) && !std::isfinite(best_opposite.cost)) {
    static rclcpp::Clock primary_rejection_log_clock(RCL_STEADY_TIME);
    static rclcpp::Clock recovery_rejection_log_clock(RCL_STEADY_TIME);
    auto & rejection_log_clock = recovery_mode ?
      recovery_rejection_log_clock : primary_rejection_log_clock;
    RCLCPP_INFO_THROTTLE(
      get_logger(), rejection_log_clock, 1000,
      "%s Frenet lattice rejected all %d generated candidates: corridor=%d, "
      "track=%d, occupied=%d, unknown=%d, outside=%d, curvature=%d, "
      "curvature_rate=%d, lateral_accel=%d, invalid=%d, budget_exhausted=%s; "
      "corridor_first=(k=%d d=%.3f interval=[%.3f, %.3f]); "
      "track_first=(k=%d d=%.3f heading_error=%.3f support=%.3f bounds=[%.3f, %.3f]); "
      "collision_first=(k=%d d=%.3f x=%.3f y=%.3f yaw=%.3f); "
      "curvature_first=(k=%d value=%.3f limit=%.3f d=%.3f); "
      "curvature_rate_first=(k=%d value=%.3f limit=%.3f).",
      recovery_mode ? "Recovery" : "Primary", generated_count,
      rejection_counts_.corridor, rejection_counts_.track_boundary,
      rejection_counts_.occupied, rejection_counts_.unknown,
      rejection_counts_.outside_map, rejection_counts_.curvature,
      rejection_counts_.curvature_rate, rejection_counts_.lateral_acceleration,
      rejection_counts_.invalid_geometry,
      budget_exhausted ? "true" : "false",
      rejection_counts_.first_corridor_offset,
      rejection_counts_.first_corridor_d,
      rejection_counts_.first_corridor_min_d,
      rejection_counts_.first_corridor_max_d,
      rejection_counts_.first_track_offset,
      rejection_counts_.first_track_d,
      rejection_counts_.first_track_heading_error,
      rejection_counts_.first_track_support,
      rejection_counts_.first_track_min_d,
      rejection_counts_.first_track_max_d,
      rejection_counts_.first_collision_offset,
      rejection_counts_.first_collision_d,
      rejection_counts_.first_collision_x,
      rejection_counts_.first_collision_y,
      rejection_counts_.first_collision_yaw,
      rejection_counts_.first_curvature_offset,
      rejection_counts_.first_curvature,
      rejection_counts_.first_curvature_limit,
      rejection_counts_.first_curvature_d,
      rejection_counts_.first_curvature_rate_offset,
      rejection_counts_.first_curvature_rate,
      rejection_counts_.first_curvature_rate_limit);
    return false;
  }

  // Keep a committed side whenever it has any validated candidate. Before commitment, retain
  // the ordinary minimum-cost comparison so the initial side can still be selected freely.
  LatticeCandidate best;
  if (has_avoidance_commitment_ && std::isfinite(best_preferred.cost)) {
    best = std::move(best_preferred);
  } else if (has_avoidance_commitment_ && std::isfinite(best_opposite.cost)) {
    best = std::move(best_opposite);
  } else if (best_preferred.cost <= best_opposite.cost) {
    best = std::move(best_preferred);
  } else {
    best = std::move(best_opposite);
  }

  candidate = std::move(best.path);
  segment_count = best.segment_count;
  selected_left = best.avoid_left;
  selected_cost = best.cost;
  RCLCPP_DEBUG(
    get_logger(),
    "%s Frenet lattice selected %s candidate: cost=%.3f, target_d=%.3f, "
    "transition=%d wp, clearance=%.3f m, max_kappa=%.3f, "
    "max_dkappa_ds=%.3f, J_s=%.3f, S=%.3f m (%d/%d feasible).",
    recovery_mode ? "Recovery" : "Primary",
    selected_left ? "left" : "right", best.cost, best.target_offset,
    best.transition_margin_wpnts, best.minimum_clearance,
    best.maximum_curvature, best.maximum_curvature_rate,
    best.spatial_lateral_jerk_cost, best.maneuver_length_m,
    feasible_count, generated_count);
  return true;
}

bool LocalPlannerNode::buildQuinticLatticePath(
  const int start_idx, const int count,
  const int collision_begin, const int collision_end,
  const std::vector<LateralTargetKnot> & obstacle_knots,
  const int transition_margin_wpnts,
  const bool avoid_left,
  LatticeCandidate & candidate) const
{
  const int total = static_cast<int>(global_wpnts_.wpnts.size());
  const int window_begin = std::max(0, collision_begin - transition_margin_wpnts);
  const int window_end = std::min(count - 1, collision_end + transition_margin_wpnts);
  if (window_end <= window_begin || total < 3 || obstacle_knots.empty()) {
    ++rejection_counts_.invalid_geometry;
    return false;
  }
  for (std::size_t index = 1; index < obstacle_knots.size(); ++index) {
    if (obstacle_knots[index].waypoint_offset <=
      obstacle_knots[index - 1U].waypoint_offset)
    {
      ++rejection_counts_.invalid_geometry;
      return false;
    }
  }
  // When the obstacle has already entered the first horizon sample,
  // collision_begin and window_begin are both zero.  The ego pose is the
  // boundary condition at that sample; inserting a second obstacle knot at
  // the same offset used to make every candidate invalid.  Drop only that
  // duplicate boundary knot and let the following corridor knot guide a
  // continuous transition from the measured ego d.
  std::vector<LateralTargetKnot> effective_obstacle_knots = obstacle_knots;
  if (collision_begin == window_begin) {
    effective_obstacle_knots.erase(
      effective_obstacle_knots.begin(),
      std::find_if(
        effective_obstacle_knots.begin(), effective_obstacle_knots.end(),
        [window_begin](const LateralTargetKnot & knot) {
          return knot.waypoint_offset > window_begin;
        }));
  }
  if (effective_obstacle_knots.empty()) {
    ++rejection_counts_.invalid_geometry;
    return false;
  }

  // A guided profile only needs to span the occupied interval. Requiring a
  // knot exactly at both occupied edges forces a zero/near-zero lateral
  // tangent there and can create a Frenet offset cusp on a curved reference.
  const bool starts_at_ego_boundary = collision_begin == window_begin;
  const bool spans_collision =
    (starts_at_ego_boundary ||
    effective_obstacle_knots.front().waypoint_offset <= collision_begin) &&
    effective_obstacle_knots.back().waypoint_offset >= collision_end;
  if (!spans_collision ||
    effective_obstacle_knots.front().waypoint_offset <= window_begin ||
    effective_obstacle_knots.back().waypoint_offset >= window_end)
  {
    ++rejection_counts_.invalid_geometry;
    return false;
  }

  candidate.path = global_wpnts_;
  candidate.path.header.stamp = now();
  candidate.path.header.frame_id = global_wpnts_.header.frame_id.empty() ?
    frame_id_ : global_wpnts_.header.frame_id;
  candidate.segment_count = window_end + 1;
  candidate.avoid_left = avoid_left;
  candidate.target_offset = 0.0;
  for (const auto & knot : effective_obstacle_knots) {
    if (std::abs(knot.d) > std::abs(candidate.target_offset)) {
      candidate.target_offset = knot.d;
    }
  }
  candidate.transition_margin_wpnts = transition_margin_wpnts;

  const double initial_offset = window_begin == 0 && has_odom_ &&
    std::isfinite(current_odom_.pose.pose.position.y) ?
    current_odom_.pose.pose.position.y : 0.0;

  // Werling et al.'s low-speed formulation is a quintic polynomial d(s),
  // where s is the reference-line arc length. Using waypoint indices as the
  // independent variable only has the same meaning for perfectly uniform
  // waypoint spacing and otherwise introduces artificial steering changes.
  std::vector<double> cumulative_s(static_cast<std::size_t>(count), 0.0);
  for (int k = 1; k < count; ++k) {
    const int previous_index = (start_idx + k - 1) % total;
    const int index = (start_idx + k) % total;
    const auto & previous = global_wpnts_.wpnts[previous_index];
    const auto & current = global_wpnts_.wpnts[index];
    const double ds = std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m);
    if (!std::isfinite(ds) || ds <= 1e-4) {
      ++rejection_counts_.invalid_geometry;
      return false;
    }
    cumulative_s[static_cast<std::size_t>(k)] =
      cumulative_s[static_cast<std::size_t>(k - 1)] + ds;
  }

  std::vector<LateralTargetKnot> all_knots;
  all_knots.reserve(effective_obstacle_knots.size() + 2U);
  all_knots.push_back({window_begin, initial_offset});
  all_knots.insert(
    all_knots.end(), effective_obstacle_knots.begin(), effective_obstacle_knots.end());
  all_knots.push_back({window_end, 0.0});
  if (all_knots[1].waypoint_offset <= window_begin ||
    all_knots[all_knots.size() - 2U].waypoint_offset >= window_end)
  {
    ++rejection_counts_.invalid_geometry;
    return false;
  }

  // Piecewise corridor profiles are a recovery representation for places
  // where a Frenet parallel curve becomes singular.  Build their anchor
  // geometry in Cartesian space so a large d on a tight reference bend does
  // not collapse the path through the factor (1 - kappa*d).  Constant-target
  // primary candidates keep the ordinary d(s) construction below.
  const bool use_cartesian_guided_curve = effective_obstacle_knots.size() > 2U;
  std::vector<double> anchor_x(all_knots.size(), 0.0);
  std::vector<double> anchor_y(all_knots.size(), 0.0);
  std::vector<double> anchor_dx_ds(all_knots.size(), 0.0);
  std::vector<double> anchor_dy_ds(all_knots.size(), 0.0);
  if (use_cartesian_guided_curve) {
    for (std::size_t knot_index = 0U; knot_index < all_knots.size(); ++knot_index) {
      const int waypoint_index =
        (start_idx + all_knots[knot_index].waypoint_offset) % total;
      const auto & reference = global_wpnts_.wpnts[waypoint_index];
      anchor_x[knot_index] = reference.x_m -
        all_knots[knot_index].d * std::sin(reference.psi_rad);
      anchor_y[knot_index] = reference.y_m +
        all_knots[knot_index].d * std::cos(reference.psi_rad);
    }
    for (std::size_t knot_index = 0U; knot_index < all_knots.size(); ++knot_index) {
      const std::size_t previous = knot_index == 0U ? 0U : knot_index - 1U;
      const std::size_t next = std::min(knot_index + 1U, all_knots.size() - 1U);
      const double span = std::max(
        1e-3,
        cumulative_s[static_cast<std::size_t>(all_knots[next].waypoint_offset)] -
        cumulative_s[static_cast<std::size_t>(all_knots[previous].waypoint_offset)]);
      anchor_dx_ds[knot_index] = (anchor_x[next] - anchor_x[previous]) / span;
      anchor_dy_ds[knot_index] = (anchor_y[next] - anchor_y[previous]) / span;
    }
  }

  // Use a shape-preserving slope at each internal corridor knot. Giving every
  // piece zero slope (the legacy smoothstep construction) forces the vehicle
  // to straighten at every knot and creates large curvature/curvature-rate
  // spikes in a shifting hairpin corridor. The weighted harmonic mean is the
  // PCHIP slope rule: it keeps monotone knot sequences monotone and sets the
  // slope to zero at a real lateral extremum. Entry and merge slopes remain
  // zero so the published path still joins the current/global path smoothly.
  std::vector<double> knot_slopes(all_knots.size(), 0.0);
  for (std::size_t index = 1U; index + 1U < all_knots.size(); ++index) {
    const double previous_s = cumulative_s[static_cast<std::size_t>(
          all_knots[index - 1U].waypoint_offset)];
    const double current_s = cumulative_s[static_cast<std::size_t>(
          all_knots[index].waypoint_offset)];
    const double next_s = cumulative_s[static_cast<std::size_t>(
          all_knots[index + 1U].waypoint_offset)];
    const double previous_length = current_s - previous_s;
    const double next_length = next_s - current_s;
    const double previous_secant =
      (all_knots[index].d - all_knots[index - 1U].d) / previous_length;
    const double next_secant =
      (all_knots[index + 1U].d - all_knots[index].d) / next_length;
    if (previous_secant * next_secant > 0.0) {
      const double previous_weight = 2.0 * next_length + previous_length;
      const double next_weight = next_length + 2.0 * previous_length;
      knot_slopes[index] = (previous_weight + next_weight) /
        (previous_weight / previous_secant + next_weight / next_secant);
    }
  }

  std::vector<QuinticHermiteSegment> segments;
  segments.reserve(all_knots.size() - 1U);
  candidate.spatial_lateral_jerk_cost = 0.0;
  for (std::size_t index = 1; index < all_knots.size(); ++index) {
    const auto & previous = all_knots[index - 1U];
    const auto & current = all_knots[index];
    const double length = cumulative_s[static_cast<std::size_t>(current.waypoint_offset)] -
      cumulative_s[static_cast<std::size_t>(previous.waypoint_offset)];
    if (length <= 1e-3) {
      ++rejection_counts_.invalid_geometry;
      return false;
    }
    const auto segment = makeQuinticHermiteSegment(
      previous.d, current.d, knot_slopes[index - 1U], knot_slopes[index], length);
    candidate.spatial_lateral_jerk_cost += quinticSpatialJerkCost(segment);
    segments.push_back(segment);
  }
  candidate.maneuver_length_m =
    cumulative_s[static_cast<std::size_t>(window_end)] -
    cumulative_s[static_cast<std::size_t>(window_begin)];
  if (!std::isfinite(candidate.spatial_lateral_jerk_cost) ||
    !std::isfinite(candidate.maneuver_length_m))
  {
    ++rejection_counts_.invalid_geometry;
    return false;
  }

  std::size_t active_segment = 1U;
  for (int k = window_begin; k <= window_end; ++k) {
    const int index = (start_idx + k) % total;
    const auto & reference = global_wpnts_.wpnts[index];
    auto & waypoint = candidate.path.wpnts[index];

    while (active_segment + 1U < all_knots.size() &&
      k > all_knots[active_segment].waypoint_offset)
    {
      ++active_segment;
    }
    const auto & left_knot = all_knots[active_segment - 1U];
    const auto & right_knot = all_knots[active_segment];
    const double segment_start_s =
      cumulative_s[static_cast<std::size_t>(left_knot.waypoint_offset)];
    const double segment_length =
      cumulative_s[static_cast<std::size_t>(right_knot.waypoint_offset)] - segment_start_s;
    const double ratio =
      (cumulative_s[static_cast<std::size_t>(k)] - segment_start_s) / segment_length;
    double final_offset = evaluateQuinticHermite(
      segments[active_segment - 1U], ratio);

    if (legacy_clamp_candidate_d_) {
      const double effective_wall_margin = std::max(
        wall_margin_, vehicle_radius_ + path_clearance_margin_);
      const double max_left = std::max(0.05, reference.d_left - effective_wall_margin);
      const double max_right = std::max(0.05, reference.d_right - effective_wall_margin);
      final_offset = std::clamp(final_offset, -max_right, max_left);
    } else {
      if (k >= static_cast<int>(safe_corridor_.samples.size())) {
        ++rejection_counts_.invalid_geometry;
        return false;
      }
      const auto & corridor = safe_corridor_.samples[static_cast<std::size_t>(k)];
      if (final_offset < corridor.track_min_d || final_offset > corridor.track_max_d) {
        ++rejection_counts_.track_boundary;
        return false;
      }
      if (!isCandidateInsideSafeCorridor(k, final_offset)) {
        ++rejection_counts_.corridor;
        return false;
      }
    }
    if (use_cartesian_guided_curve) {
      const double t = std::clamp(ratio, 0.0, 1.0);
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
      const double h10 = t3 - 2.0 * t2 + t;
      const double h01 = -2.0 * t3 + 3.0 * t2;
      const double h11 = t3 - t2;
      const std::size_t left_anchor = active_segment - 1U;
      const std::size_t right_anchor = active_segment;
      waypoint.x_m = h00 * anchor_x[left_anchor] +
        h10 * segment_length * anchor_dx_ds[left_anchor] +
        h01 * anchor_x[right_anchor] +
        h11 * segment_length * anchor_dx_ds[right_anchor];
      waypoint.y_m = h00 * anchor_y[left_anchor] +
        h10 * segment_length * anchor_dy_ds[left_anchor] +
        h01 * anchor_y[right_anchor] +
        h11 * segment_length * anchor_dy_ds[right_anchor];
      const double dx = waypoint.x_m - reference.x_m;
      const double dy = waypoint.y_m - reference.y_m;
      waypoint.d_m = -dx * std::sin(reference.psi_rad) +
        dy * std::cos(reference.psi_rad);
    } else {
      waypoint.x_m = reference.x_m - final_offset * std::sin(reference.psi_rad);
      waypoint.y_m = reference.y_m + final_offset * std::cos(reference.psi_rad);
      waypoint.d_m = final_offset;
    }
  }

  updateCandidateGeometry(candidate.path, start_idx, window_begin, window_end);
  updateCandidateVelocity(
    candidate.path, start_idx, window_begin, collision_end, window_end);
  return true;
}

void LocalPlannerNode::updateCandidateGeometry(
  f110_msgs::msg::WpntArray & candidate,
  const int start_idx, const int window_begin, const int window_end) const
{
  const int total = static_cast<int>(candidate.wpnts.size());
  if (total < 3) {
    return;
  }

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
  const int desired_count = collision_free_count - lattice_braking_buffer_wpnts_;
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
    static_cast<std::size_t>(lattice_braking_buffer_wpnts_) + 1U ?
    safe_count - static_cast<std::size_t>(lattice_braking_buffer_wpnts_) : safe_count;
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
      2.0 * lattice_braking_deceleration_mps2_ * remaining_distance);
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
    return;
  }

  committed_avoidance_path_ = candidate;
  committed_point_validated_.assign(static_cast<std::size_t>(total), 0U);
  committed_segment_validated_.assign(static_cast<std::size_t>(total), 0U);
  committed_merge_index_ = (start_idx + segment_count - 1) % total;
  committed_avoid_left_ = avoid_left;
  committed_planner_name_ = planner_name;
  committed_lattice_cost_ = lattice_cost;
  committed_obstacle_snapshot_.header = perceived_obstacles_.header;
  committed_obstacle_snapshot_.obstacles.clear();
  const auto & snapshot_source =
    perception_static_only_ && !grid_obstacle_snapshot_.empty() ?
    grid_obstacle_snapshot_ : perceived_obstacles_.obstacles;
  for (const auto & obstacle : snapshot_source) {
    if (obstacle.is_actually_a_gap ||
      (perception_static_only_ && !isPlanningStaticObstacle(obstacle)))
    {
      continue;
    }
    committed_obstacle_snapshot_.obstacles.push_back(obstacle);
  }
  has_committed_obstacle_snapshot_ =
    !committed_obstacle_snapshot_.obstacles.empty();
  has_avoidance_commitment_ = true;
}

void LocalPlannerNode::clearAvoidanceCommitment()
{
  has_avoidance_commitment_ = false;
  committed_collision_start_index_ = -1;
  committed_collision_end_index_ = -1;
  committed_avoidance_path_.wpnts.clear();
  committed_point_validated_.clear();
  committed_segment_validated_.clear();
  committed_obstacle_snapshot_.obstacles.clear();
  has_committed_obstacle_snapshot_ = false;
  committed_lattice_cost_ = std::numeric_limits<double>::infinity();
  committed_planner_name_ = "frenet_lattice_segment";
}

CollisionRejectionReason LocalPlannerNode::checkPathPoseCollision(
  const double x, const double y, const double yaw, const double clearance) const
{
  int center_column = 0;
  int center_row = 0;
  if (!worldToMap(x, y, center_column, center_row)) {
    return CollisionRejectionReason::kOutsideMap;
  }
  const int width = static_cast<int>(grid_map_.info.width);
  const int height = static_cast<int>(grid_map_.info.height);
  const double cell_radius = grid_map_.info.resolution * std::sqrt(0.5);
  const double footprint_margin = localization_margin_m_ + corridor_safety_margin_m_ +
    path_clearance_margin_;
  const double front = vehicle_front_extent_m_ + footprint_margin;
  const double rear = vehicle_rear_extent_m_ + footprint_margin;
  const double half_width = 0.5 * vehicle_width_m_ + footprint_margin;
  const double rectangle_radius = std::hypot(std::max(front, rear), half_width);
  const double search_radius = std::max(
    rectangle_radius, preserve_circular_collision_check_ ? clearance : 0.0);
  const int radius_cells = static_cast<int>(std::ceil(
      (search_radius + cell_radius) / grid_map_.info.resolution));

  // Reject a footprint that leaves the known grid before scanning cells.
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  for (const double longitudinal : {-rear, front}) {
    for (const double lateral : {-half_width, half_width}) {
      const double corner_x = x + longitudinal * cos_yaw - lateral * sin_yaw;
      const double corner_y = y + longitudinal * sin_yaw + lateral * cos_yaw;
      int column = 0;
      int row = 0;
      if (!worldToMap(corner_x, corner_y, column, row)) {
        return CollisionRejectionReason::kOutsideMap;
      }
    }
  }
  if (preserve_circular_collision_check_) {
    for (int direction = 0; direction < 8; ++direction) {
      const double angle = static_cast<double>(direction) * kPi / 4.0;
      int column = 0;
      int row = 0;
      if (!worldToMap(
          x + clearance * std::cos(angle), y + clearance * std::sin(angle),
          column, row))
      {
        return CollisionRejectionReason::kOutsideMap;
      }
    }
  }

  for (int row_offset = -radius_cells; row_offset <= radius_cells; ++row_offset) {
    for (int column_offset = -radius_cells; column_offset <= radius_cells; ++column_offset) {
      const int column = center_column + column_offset;
      const int row = center_row + row_offset;
      if (column < 0 || column >= width || row < 0 || row >= height) {
        continue;
      }
      const auto cell = mapCellCenter(column, row);
      const double dx = cell.x - x;
      const double dy = cell.y - y;
      const double longitudinal = dx * cos_yaw + dy * sin_yaw;
      const double lateral = -dx * sin_yaw + dy * cos_yaw;
      const bool inside_rectangle =
        longitudinal >= -rear - cell_radius && longitudinal <= front + cell_radius &&
        std::abs(lateral) <= half_width + cell_radius;
      const bool inside_circle = preserve_circular_collision_check_ &&
        dx * dx + dy * dy <= (clearance + cell_radius) * (clearance + cell_radius);
      if (!inside_rectangle && !inside_circle) {
        continue;
      }
      const int8_t occupancy = grid_map_.data[row * width + column];
      if (occupancy < 0) {
        if (unknown_cell_policy_ == UnknownCellPolicy::kTreatAsFree) {
          continue;
        }
        return unknown_cell_policy_ == UnknownCellPolicy::kTreatAsOccupied ?
               CollisionRejectionReason::kOccupied : CollisionRejectionReason::kUnknown;
      }
      if (occupancy > occupied_threshold_) {
        return CollisionRejectionReason::kOccupied;
      }
    }
  }
  return CollisionRejectionReason::kNone;
}

bool LocalPlannerNode::isPathPointCollisionFree(
  const double x, const double y, const double yaw, const double clearance,
  CollisionRejectionReason * reason) const
{
  const auto collision = checkPathPoseCollision(x, y, yaw, clearance);
  if (reason != nullptr) {
    *reason = collision;
  }
  return collision == CollisionRejectionReason::kNone;
}

bool LocalPlannerNode::isPathSegmentCollisionFree(
  const double x0, const double y0, const double yaw0,
  const double x1, const double y1, const double yaw1,
  const double clearance, CollisionRejectionReason * reason,
  double * clearance_sum, int * clearance_samples,
  double * minimum_clearance) const
{
  const double length = std::hypot(x1 - x0, y1 - y0);
  const double map_step = has_map_ ? grid_map_.info.resolution * 0.5 :
    lattice_collision_sample_step_m_;
  const double sample_step = std::max(
    0.01, std::min(lattice_collision_sample_step_m_, map_step));
  const int intervals = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
  const double yaw_delta = std::remainder(yaw1 - yaw0, 2.0 * kPi);
  for (int i = 0; i <= intervals; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(intervals);
    const double x = x0 + ratio * (x1 - x0);
    const double y = y0 + ratio * (y1 - y0);
    const double yaw = yaw0 + ratio * yaw_delta;
    CollisionRejectionReason collision = CollisionRejectionReason::kNone;
    if (!isPathPointCollisionFree(x, y, yaw, clearance, &collision)) {
      if (reason != nullptr) {
        *reason = collision;
      }
      return false;
    }
    if (i > 0 && i < intervals) {
      const double sample_clearance = clearanceAtWorld(x, y);
      if (clearance_sum != nullptr) {
        const double free_margin = std::max(0.02, sample_clearance - clearance);
        *clearance_sum += 1.0 / free_margin;
      }
      if (clearance_samples != nullptr) {
        ++(*clearance_samples);
      }
      if (minimum_clearance != nullptr) {
        *minimum_clearance = std::min(*minimum_clearance, sample_clearance);
      }
    }
  }
  if (reason != nullptr) {
    *reason = CollisionRejectionReason::kNone;
  }
  return true;
}

void LocalPlannerNode::countCollisionRejection(
  const CollisionRejectionReason reason) const
{
  switch (reason) {
    case CollisionRejectionReason::kOccupied:
      ++rejection_counts_.occupied;
      break;
    case CollisionRejectionReason::kUnknown:
      ++rejection_counts_.unknown;
      break;
    case CollisionRejectionReason::kOutsideMap:
      ++rejection_counts_.outside_map;
      break;
    case CollisionRejectionReason::kNone:
      break;
  }
}

f110_msgs::msg::WpntArray LocalPlannerNode::makeForwardSegment(
  const f110_msgs::msg::WpntArray & source,
  const int start_idx, const int requested_count) const
{
  f110_msgs::msg::WpntArray segment;
  segment.header = source.header;
  segment.header.stamp = now();
  if (segment.header.frame_id.empty()) {
    segment.header.frame_id = frame_id_;
  }
  const int total = static_cast<int>(source.wpnts.size());
  const int count = std::clamp(requested_count, 0, total);
  segment.wpnts.reserve(static_cast<std::size_t>(count));
  for (int k = 0; k < count; ++k) {
    segment.wpnts.push_back(source.wpnts[(start_idx + k) % total]);
  }
  return segment;
}

nav_msgs::msg::Path LocalPlannerNode::makePath(
  const f110_msgs::msg::WpntArray & source) const
{
  nav_msgs::msg::Path path;
  path.header = source.header;
  path.poses.reserve(source.wpnts.size());
  for (const auto & waypoint : source.wpnts) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = source.header;
    pose.pose.position.x = waypoint.x_m;
    pose.pose.position.y = waypoint.y_m;
    pose.pose.position.z = 0.05;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(pose));
  }
  return path;
}

void LocalPlannerNode::publishAvoidWaypoints(
  const f110_msgs::msg::WpntArray * segment, const bool avoid_left,
  const std::string & planner_name)
{
  f110_msgs::msg::OTWpntArray message;
  message.header.stamp = now();
  message.header.frame_id = frame_id_;
  message.last_switch_time = now();
  message.side_switch = segment != nullptr && !segment->wpnts.empty();
  message.ot_side = avoid_left ? "left" : "right";
  message.ot_line = planner_name;
  if (segment != nullptr) {
    message.header = segment->header;
    message.header.stamp = now();
    message.wpnts = segment->wpnts;
  }
  ot_pub_->publish(message);
}

void LocalPlannerNode::publishDebugVisualization(
  const f110_msgs::msg::WpntArray & local_segment,
  const std::vector<geometry_msgs::msg::Point> & obstacle_points,
  const bool safe_path_available)
{
  const auto publish_time = now();
  if (last_debug_publish_time_.nanoseconds() != 0) {
    const double elapsed_ms = (publish_time - last_debug_publish_time_).seconds() * 1000.0;
    if (elapsed_ms >= 0.0 && elapsed_ms < static_cast<double>(debug_publish_period_ms_)) {
      return;
    }
  }
  last_debug_publish_time_ = publish_time;
  visualization_msgs::msg::MarkerArray markers;
  auto marker_header = local_segment.header;
  marker_header.stamp = publish_time;
  if (marker_header.frame_id.empty()) {
    marker_header.frame_id = frame_id_;
  }

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
    geometry_msgs::msg::Point point;
    point.x = waypoint.x_m;
    point.y = waypoint.y_m;
    point.z = 0.10;
    path_marker.points.push_back(point);
  }
  markers.markers.push_back(std::move(path_marker));
  marker_pub_->publish(markers);
}

void LocalPlannerNode::onTimer()
{
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
        held_planner_name_.find("frenet_lattice_replan_brake") == 0U;
      const bool can_hold_previous_braking = has_held_avoidance_segment_ &&
        held_path_is_braking &&
        (now() - held_segment_time_).seconds() <= frenet_odom_path_hold_timeout_sec_;
      f110_msgs::msg::WpntArray replan_braking_segment;
      f110_msgs::msg::WpntArray held_braking_segment;
      f110_msgs::msg::WpntArray stationary_hold_segment;
      const bool replan_brake_available =
        last_validated_path_age <= lattice_replan_brake_timeout_sec_ &&
        buildReplanBrakingSegment(
        last_validated_full_path_, start_idx, planning_count,
        replan_braking_segment);
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
          "No Frenet lattice or braking segment passed validation; "
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

  // Keep the last validated segment through the configured obstacle-clear
  // hysteresis so a one-cycle component miss cannot replace it with an empty path.
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
  }

  if (publish_standalone_local_) {
    local_wpnts_pub_->publish(visualization_segment);
  }
  local_path_pub_->publish(makePath(visualization_segment));
  exact_local_path_pub_->publish(makePath(full_local_path));
  publishDebugVisualization(
    marker_segment, obstacle_points, safe_path_available);
  last_planning_latency_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - planning_begin).count();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Local planner latency %.3f ms (corridor samples=%zu, inflated cells=%zu).",
    last_planning_latency_ms_, safe_corridor_.samples.size(),
    safe_corridor_.inflated_cell_indices.size());
}

}  // namespace local_planning
