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

#include <atomic>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "local_planning/safe_corridor.hpp"

namespace local_planning
{

struct LatticeCandidate
{
  f110_msgs::msg::WpntArray path;
  int segment_count{0};
  bool avoid_left{true};
  double target_offset{0.0};
  int transition_margin_wpnts{0};
  double cost{std::numeric_limits<double>::infinity()};
  double minimum_clearance{0.0};
  double maximum_curvature{0.0};
  double maximum_curvature_rate{0.0};
  double spatial_lateral_jerk_cost{0.0};
  double maneuver_length_m{0.0};
};

enum class CollisionRejectionReason
{
  kNone,
  kOccupied,
  kUnknown,
  kOutsideMap
};

struct CandidateRejectionCounts
{
  int corridor{0};
  int track_boundary{0};
  int occupied{0};
  int unknown{0};
  int outside_map{0};
  int curvature{0};
  int curvature_rate{0};
  int lateral_acceleration{0};
  int invalid_geometry{0};
  int first_corridor_offset{-1};
  double first_corridor_d{std::numeric_limits<double>::quiet_NaN()};
  double first_corridor_min_d{std::numeric_limits<double>::quiet_NaN()};
  double first_corridor_max_d{std::numeric_limits<double>::quiet_NaN()};
  int first_track_offset{-1};
  double first_track_d{std::numeric_limits<double>::quiet_NaN()};
  double first_track_heading_error{std::numeric_limits<double>::quiet_NaN()};
  double first_track_support{std::numeric_limits<double>::quiet_NaN()};
  double first_track_min_d{std::numeric_limits<double>::quiet_NaN()};
  double first_track_max_d{std::numeric_limits<double>::quiet_NaN()};
  int first_collision_offset{-1};
  double first_collision_d{std::numeric_limits<double>::quiet_NaN()};
  double first_collision_x{std::numeric_limits<double>::quiet_NaN()};
  double first_collision_y{std::numeric_limits<double>::quiet_NaN()};
  double first_collision_yaw{std::numeric_limits<double>::quiet_NaN()};
  int first_curvature_offset{-1};
  double first_curvature{std::numeric_limits<double>::quiet_NaN()};
  double first_curvature_limit{std::numeric_limits<double>::quiet_NaN()};
  double first_curvature_d{std::numeric_limits<double>::quiet_NaN()};
  int first_curvature_rate_offset{-1};
  double first_curvature_rate{std::numeric_limits<double>::quiet_NaN()};
  double first_curvature_rate_limit{std::numeric_limits<double>::quiet_NaN()};
};

struct FilteredObstacleTrack
{
  f110_msgs::msg::Obstacle obstacle;
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalPlannerNode() override = default;

private:
  void initParameters();
  void initInterfaces();

  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  bool isPlanningStaticObstacle(const f110_msgs::msg::Obstacle & obstacle) const;
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  void rebuildObstacleMask();
  void rebuildPlanningGrid();
  bool obstacleCenterPose(double s, double d, double & x, double & y, double & yaw) const;
  void rebuildClearanceField();
  bool worldToMap(double x, double y, int & column, int & row) const;
  geometry_msgs::msg::Point mapCellCenter(int column, int row) const;
  double clearanceAtWorld(double x, double y) const;
  int findClosestWaypointIndexByS(double s) const;
  SafeCorridorResult buildSafeCorridor(int start_idx, int count);
  void invalidateSafeCorridorCache();
  bool isCandidateInsideSafeCorridor(int waypoint_offset, double d) const;

  void detectObstaclesAndDecideDirection(
    int start_idx, int count, bool & obs_detected, bool & avoid_left,
    std::vector<int> & collision_indices,
    std::vector<geometry_msgs::msg::Point> & obstacle_points) const;

  bool buildFrenetLatticeCandidate(
    int start_idx, int count,
    const std::vector<int> & collision_indices,
    bool preferred_left,
    f110_msgs::msg::WpntArray & candidate,
    int & segment_count,
    bool & selected_left,
    double & selected_cost,
    bool recovery_mode = false) const;

  bool buildQuinticLatticePath(
    int start_idx, int count,
    int collision_begin, int collision_end,
    const std::vector<LateralTargetKnot> & obstacle_knots,
    int transition_margin_wpnts,
    bool avoid_left,
    LatticeCandidate & candidate) const;

  void updateCandidateGeometry(
    f110_msgs::msg::WpntArray & candidate,
    int start_idx, int window_begin, int window_end) const;

  void updateCandidateVelocity(
    f110_msgs::msg::WpntArray & candidate,
    int start_idx, int window_begin, int collision_end, int window_end) const;

  bool evaluateLatticeCandidate(
    LatticeCandidate & candidate,
    int start_idx, int window_begin, int window_end,
    bool preferred_left, bool recovery_mode) const;
  bool buildSafeStopSegment(
    int start_idx, int count,
    const std::vector<int> & collision_indices,
    f110_msgs::msg::WpntArray & segment) const;
  bool buildReplanBrakingSegment(
    const f110_msgs::msg::WpntArray & source,
    int start_idx, int count,
    f110_msgs::msg::WpntArray & segment) const;
  bool buildSegmentBrakingPrefix(
    const f110_msgs::msg::WpntArray & source,
    f110_msgs::msg::WpntArray & segment) const;
  bool buildStationaryHoldSegment(
    f110_msgs::msg::WpntArray & segment) const;
  bool isPlanningResultStale(
    uint64_t planning_localization_generation,
    double & pose_drift,
    bool & localization_changed) const;
  void applyBrakingProfile(f110_msgs::msg::WpntArray & segment) const;

  bool reuseCommittedAvoidancePath(
    int start_idx, int max_count,
    f110_msgs::msg::WpntArray & candidate,
    int & segment_count);
  int extendSegmentPastMerge(
    const f110_msgs::msg::WpntArray & candidate,
    int start_idx, int merge_segment_count) const;
  void commitAvoidancePath(
    const f110_msgs::msg::WpntArray & candidate,
    int start_idx, int segment_count,
    bool avoid_left, const std::string & planner_name,
    double lattice_cost);
  void clearAvoidanceCommitment();

  CollisionRejectionReason checkPathPoseCollision(
    double x, double y, double yaw, double clearance) const;
  bool isPathPointCollisionFree(
    double x, double y, double yaw, double clearance,
    CollisionRejectionReason * reason = nullptr) const;
  bool isPathSegmentCollisionFree(
    double x0, double y0, double yaw0,
    double x1, double y1, double yaw1,
    double clearance, CollisionRejectionReason * reason = nullptr,
    double * clearance_sum = nullptr, int * clearance_samples = nullptr,
    double * minimum_clearance = nullptr) const;
  void countCollisionRejection(CollisionRejectionReason reason) const;

  f110_msgs::msg::WpntArray makeForwardSegment(
    const f110_msgs::msg::WpntArray & source,
    int start_idx, int count) const;
  nav_msgs::msg::Path makePath(const f110_msgs::msg::WpntArray & source) const;
  void publishAvoidWaypoints(
    const f110_msgs::msg::WpntArray * segment,
    bool avoid_left,
    const std::string & planner_name);
  void publishDebugVisualization(
    const f110_msgs::msg::WpntArray & local_segment,
    const std::vector<geometry_msgs::msg::Point> & obstacle_points,
    bool safe_path_available);

  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr ot_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr local_wpnts_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exact_local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  f110_msgs::msg::WpntArray global_wpnts_;
  nav_msgs::msg::OccupancyGrid grid_map_;
  nav_msgs::msg::OccupancyGrid base_grid_map_;
  f110_msgs::msg::ObstacleArray perceived_obstacles_;
  std::vector<FilteredObstacleTrack> filtered_obstacle_tracks_;
  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Odometry latest_odom_;
  mutable std::mutex odom_mutex_;
  std::vector<uint8_t> obstacle_mask_;
  std::vector<int32_t> obstacle_component_ids_;
  std::vector<geometry_msgs::msg::Point> obstacle_centroids_;
  std::vector<double> grid_clearance_m_;
  SafeCorridorResult safe_corridor_;
  std::vector<SafeCorridorSample> safe_corridor_sample_cache_;
  std::vector<uint8_t> safe_corridor_sample_cache_valid_;
  int safe_corridor_start_idx_{0};
  mutable CandidateRejectionCounts rejection_counts_;
  double last_planning_latency_ms_{0.0};

  bool has_global_{false};
  bool has_map_{false};
  bool has_obstacles_{false};
  bool has_odom_{false};
  bool odom_ready_{false};
  bool has_previous_frenet_s_{false};
  std::atomic_bool localization_reset_pending_{false};
  std::atomic_uint64_t localization_generation_{0U};
  bool has_map_signature_{false};
  uint64_t map_signature_{0U};
  double map_origin_yaw_{0.0};
  double map_origin_cos_{1.0};
  double map_origin_sin_{0.0};
  double previous_frenet_s_{0.0};
  int consecutive_valid_odom_count_{0};
  int consecutive_invalid_odom_count_{0};
  rclcpp::Time last_odom_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_perception_grid_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_perception_receive_time_{0, 0, RCL_ROS_TIME};
  bool diagnostic_static_obstacle_seen_{false};
  std::vector<f110_msgs::msg::Obstacle> grid_obstacle_snapshot_;
  std::vector<std::deque<f110_msgs::msg::Obstacle>> grid_obstacle_history_;
  std::vector<int> grid_obstacle_expand_counts_;
  std::vector<int> grid_obstacle_shrink_counts_;

  int detection_count_{0};
  int clear_count_{0};
  bool avoidance_active_{false};
  bool has_avoidance_commitment_{false};
  bool committed_avoid_left_{true};
  int committed_merge_index_{0};
  int committed_collision_start_index_{-1};
  int committed_collision_end_index_{-1};
  double committed_lattice_cost_{std::numeric_limits<double>::infinity()};
  std::string committed_planner_name_{"frenet_lattice_segment"};
  f110_msgs::msg::WpntArray committed_avoidance_path_;
  f110_msgs::msg::ObstacleArray committed_obstacle_snapshot_;
  bool has_committed_obstacle_snapshot_{false};
  std::vector<uint8_t> committed_point_validated_;
  std::vector<uint8_t> committed_segment_validated_;
  bool has_held_avoidance_segment_{false};
  bool held_avoid_left_{true};
  std::string held_planner_name_{"frenet_lattice_segment"};
  f110_msgs::msg::WpntArray held_avoidance_segment_;
  rclcpp::Time held_segment_time_{0, 0, RCL_ROS_TIME};
  bool has_last_validated_full_path_{false};
  bool last_validated_avoid_left_{true};
  f110_msgs::msg::WpntArray last_validated_full_path_;
  rclcpp::Time last_validated_path_time_{0, 0, RCL_ROS_TIME};

  int lookahead_wpnt_num_{160};
  int detection_lookahead_wpnt_num_{120};
  int spline_window_margin_wpnts_{40};
  int corner_spline_window_margin_wpnts_{18};
  double safety_margin_{0.35};
  double wall_margin_{0.30};
  double avoid_offset_{0.45};
  double corner_avoid_offset_{0.35};
  double vehicle_radius_{0.20};
  double path_clearance_margin_{0.05};
  double vehicle_front_extent_m_{0.3302};
  double vehicle_rear_extent_m_{0.05};
  double vehicle_width_m_{0.2413};
  double localization_margin_m_{0.05};
  double corridor_safety_margin_m_{0.05};
  bool preserve_circular_collision_check_{true};
  bool legacy_clamp_candidate_d_{false};
  double corner_tracking_margin_{0.10};
  double corner_curvature_threshold_{1.20};
  double longitudinal_search_half_width_m_{0.15};
  double obstacle_component_min_area_m2_{0.002};
  double obstacle_component_max_area_m2_{2.0};
  int occupied_threshold_{50};
  bool use_perception_obstacles_{true};
  bool require_perception_obstacles_{false};
  bool perception_static_only_{true};
  double perception_static_speed_threshold_mps_{0.25};
  double perception_obstacle_padding_m_{0.05};
  double perception_grid_update_period_sec_{1.0};
  double perception_grid_rebuild_motion_m_{0.12};
  double perception_static_grid_match_gate_m_{0.75};
  int perception_static_history_size_{6};
  int perception_static_expand_confirm_cycles_{2};
  int perception_static_shrink_confirm_cycles_{3};
  double perception_filter_alpha_{0.25};
  double perception_track_hold_sec_{1.0};
  double perception_association_gate_m_{0.35};
  double perception_max_s_step_m_{0.20};
  double perception_max_d_step_m_{0.12};
  bool freeze_committed_static_obstacles_{true};
  double committed_obstacle_match_distance_m_{0.35};
  double speed_reduction_ratio_{0.76};
  double corner_speed_reduction_ratio_{0.55};
  double post_obstacle_speed_recovery_ratio_{1.0};
  double lattice_max_longitudinal_accel_mps2_{3.0};
  double lattice_max_longitudinal_decel_mps2_{6.0};
  int lattice_lateral_samples_{4};
  double lattice_lateral_step_m_{0.06};
  double lattice_corridor_target_inset_m_{0.020};
  double lattice_corridor_validation_tolerance_m_{0.010};
  double lattice_narrow_corridor_width_threshold_m_{0.20};
  std::vector<double> lattice_narrow_corridor_transition_scales_{1.40, 1.80, 2.20};
  int lattice_obstacle_cluster_gap_wpnts_{16};
  int lattice_corridor_knot_stride_wpnts_{3};
  int lattice_corridor_beam_width_{16};
  std::vector<double> lattice_transition_scales_{1.80, 1.40, 1.20, 1.0, 0.75};
  int lattice_min_transition_wpnts_{10};
  int lattice_recovery_lateral_samples_{7};
  double lattice_recovery_lateral_step_m_{0.04};
  std::vector<double> lattice_recovery_transition_scales_{2.20, 1.80, 1.40, 1.15, 0.90, 0.70};
  int lattice_recovery_min_transition_wpnts_{10};
  int lattice_recovery_profile_limit_{6};
  int lattice_primary_search_budget_ms_{120};
  int lattice_recovery_search_budget_ms_{180};
  double lattice_max_result_pose_drift_m_{1.00};
  double lattice_recovery_max_curvature_scale_{1.25};
  double lattice_recovery_max_curvature_rate_scale_{1.25};
  double lattice_max_curvature_radpm_{2.80};
  double lattice_max_curvature_rate_radpm2_{18.0};
  double lattice_max_lateral_accel_mps2_{5.80};
  double lattice_speed_safety_factor_{0.95};
  double lattice_collision_sample_step_m_{0.04};
  double lattice_preferred_clearance_m_{0.40};
  double lattice_weight_clearance_{2.0};
  double lattice_weight_min_clearance_{12.0};
  double lattice_weight_deviation_{1.0};
  double lattice_weight_curvature_{0.30};
  double lattice_weight_curvature_change_{0.12};
  double lattice_weight_spatial_lateral_jerk_{0.02};
  double lattice_weight_maneuver_length_{0.08};
  double lattice_weight_path_length_{0.30};
  double lattice_weight_speed_loss_{0.20};
  double lattice_weight_opposite_side_{2.0};
  bool lattice_commit_path_until_clear_{true};
  double lattice_preplan_before_merge_m_{10.0};
  double lattice_preplan_max_start_gap_m_{0.35};
  int lattice_post_merge_lookahead_wpnts_{40};
  double lattice_merge_lateral_tolerance_m_{0.15};
  int lattice_merge_settle_max_wpnts_{30};
  int lattice_safe_stop_buffer_wpnts_{8};
  double lattice_safe_stop_deceleration_mps2_{2.50};
  double lattice_replan_brake_timeout_sec_{1.50};
  double stationary_hold_point_spacing_m_{0.03};
  bool publish_standalone_local_{false};
  int timer_period_ms_{50};
  int debug_publish_period_ms_{200};
  int detection_confirm_cycles_{1};
  int detection_clear_cycles_{3};
  int frenet_odom_confirm_cycles_{3};
  int frenet_odom_invalid_grace_cycles_{5};
  double frenet_odom_stale_timeout_sec_{0.50};
  double frenet_odom_path_hold_timeout_sec_{3.00};
  double frenet_odom_max_s_jump_m_{1.00};
  double frenet_odom_speed_jump_scale_{1.50};
  double frenet_odom_jump_slack_m_{0.25};
  double frenet_odom_track_margin_m_{0.10};
  double obstacle_marker_scale_{0.35};
  double path_marker_width_{0.08};
  int corridor_debug_stride_{2};
  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};

  std::string global_waypoints_topic_{"/global_waypoints"};
  std::string map_topic_{"/map"};
  std::string obstacles_topic_{"/perception/obstacles"};
  std::string frenet_odom_topic_{"/car_state/frenet/odom"};
  std::string ot_waypoints_topic_{"/avoid_waypoints"};
  std::string local_waypoints_topic_{"/local_waypoints"};
  std::string local_path_topic_{"/local_planning/path"};
  std::string exact_local_path_topic_{"/local_path"};
  std::string marker_topic_{"/local_planning/markers"};
  std::string frame_id_{"map"};
  std::string unknown_cell_policy_name_{"reject_candidate"};
  UnknownCellPolicy unknown_cell_policy_{UnknownCellPolicy::kRejectCandidate};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
