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

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "global_planning/clcs_frenet_converter.hpp"
#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initializeParameters();
  void initializeInterfaces();
  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr message);
  void onObstacles(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void onFrenetOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void onPlanningTimer();

  bool isStaticObstacle(const f110_msgs::msg::Obstacle & obstacle) const;
  std::optional<f110_msgs::msg::Obstacle> projectCartesianObstacle(
    const f110_msgs::msg::Obstacle & obstacle) const;
  bool sameReference(const f110_msgs::msg::WpntArray & message) const;
  void clearCommitment();
  bool commitmentComplete(const EgoFrenetState & ego);
  void publishResult(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles);
  void publishEmpty(const std::string & reason);
  nav_msgs::msg::Path makePath(const f110_msgs::msg::WpntArray & waypoints) const;
  visualization_msgs::msg::MarkerArray makeMarkers(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

<<<<<<< HEAD
=======
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
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odometry_sub_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_waypoints_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr standalone_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr compatibility_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  RacelineSplineParameters planner_parameters_;
  RacelineSplinePlanner planner_;
  global_planning::ClcsFrenetConverter::Ptr clcs_converter_;
  std::uint64_t clcs_version_{0};
  f110_msgs::msg::WpntArray global_waypoints_;
  std::vector<f110_msgs::msg::Obstacle> static_obstacles_;
  nav_msgs::msg::Odometry latest_odometry_;
  mutable std::mutex odometry_mutex_;
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacles_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_switch_time_{0, 0, RCL_ROS_TIME};

  bool has_global_waypoints_{false};
  bool has_obstacles_message_{false};
  bool has_odometry_{false};
  bool has_commitment_{false};
  RacelineSplineResult committed_result_;
  double commitment_start_s_{0.0};
  int merge_complete_count_{0};
  std::optional<bool> last_published_side_;

<<<<<<< HEAD
  bool require_obstacles_message_{true};
  bool static_obstacles_only_{true};
  double static_speed_threshold_mps_{0.25};
  double obstacle_stale_timeout_sec_{0.75};
  double odometry_stale_timeout_sec_{0.50};
  double merge_lateral_tolerance_m_{0.15};
  int merge_confirm_cycles_{15};
  int planning_period_ms_{50};
=======
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
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
  bool publish_standalone_local_{false};
  double obstacle_marker_scale_m_{0.35};
  double path_marker_width_m_{0.06};

  std::string global_waypoints_topic_{"/global_waypoints"};
<<<<<<< HEAD
  std::string obstacles_topic_{"/perception/static_obstacles/cartesian"};
=======
  std::string map_topic_{"/map"};
  std::string obstacles_topic_{"/perception/obstacles"};
>>>>>>> f9713510246c01603e88db1d5002ca178b07a970
  std::string frenet_odom_topic_{"/car_state/frenet/odom"};
  std::string ot_waypoints_topic_{"/avoid_waypoints"};
  std::string local_waypoints_topic_{"/local_waypoints"};
  std::string local_path_topic_{"/local_planning/path"};
  std::string compatibility_path_topic_{"/local_path"};
  std::string markers_topic_{"/local_planning/markers"};
  std::string frame_id_{"map"};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
