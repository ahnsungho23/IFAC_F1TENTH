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

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/state_machine.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "local_planning/obstacle_guard.hpp"
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
  void onState(const f110_msgs::msg::StateMachine::SharedPtr message);
  void onPlanningTimer();

  bool sameReference(const f110_msgs::msg::WpntArray & message) const;
  void clearCommitment();
  void commitAvoidance(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  void resetInitialStabilization();
  std::vector<f110_msgs::msg::Obstacle> buildInitialStabilizationInput() const;
  std::vector<f110_msgs::msg::Obstacle> buildGuardedObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool updateInitialStabilization(
    const std::vector<int> & cluster_ids,
    const std::vector<f110_msgs::msg::Obstacle> & conservative_obstacles,
    const rclcpp::Time & update_time);
  std::vector<f110_msgs::msg::Obstacle> buildNextManeuverInput() const;
  std::vector<f110_msgs::msg::Obstacle> buildCurrentManeuverInput(
    const EgoFrenetState & ego,
    bool apply_uncertainty_guard = true) const;
  void resetNextManeuverStabilization();
  bool updateNextManeuverStabilization(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
    const rclcpp::Time & update_time);
  void promoteNextManeuverStabilization();
  bool activeManeuverObstacleCleared(const EgoFrenetState & ego) const;
  bool tryEarlyChainedManeuver(
    const EgoFrenetState & ego,
    std::vector<f110_msgs::msg::Obstacle> & next_obstacles);
  double remainingDistanceToMerge(const EgoFrenetState & ego) const;
  bool beginChainedManeuverIfNeeded(
    const EgoFrenetState & ego,
    std::vector<f110_msgs::msg::Obstacle> & next_obstacles,
    const std::string & phase);
  void resetForChainedManeuver();
  bool commitmentSideLocked(const EgoFrenetState & ego) const;
  bool activateGlobalHandoff(const EgoFrenetState & ego);
  void latchSafeStop(
    RacelineSplineResult result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & planning_obstacles);
  void handleSafeStopLatch(const EgoFrenetState & ego);
  bool commitmentComplete(const EgoFrenetState & ego);
  void resetCommitmentViolationConfirmation();
  void logObstacleCollision(
    const std::string & severity,
    const PathValidationFailure & failure,
    int confirmation_count = 0) const;
  void publishResult(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles);
  void publishEmpty(const std::string & reason);
  nav_msgs::msg::Path makePath(
    const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    const std_msgs::msg::Header & header) const;
  visualization_msgs::msg::MarkerArray makeMarkers(
    const RacelineSplineResult & result,
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odometry_sub_;
  rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr state_sub_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr compatibility_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  RacelineSplineParameters planner_parameters_;
  ObstacleGuardParameters guard_parameters_;
  RacelineSplinePlanner planner_;
  f110_msgs::msg::WpntArray global_waypoints_;
  std::vector<f110_msgs::msg::Obstacle> static_obstacles_;
  nav_msgs::msg::Odometry latest_odometry_;
  mutable std::mutex odometry_mutex_;
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacles_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_stabilization_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_stabilization_start_{0, 0, RCL_ROS_TIME};
  std::uint64_t obstacles_message_sequence_{0};
  std::uint64_t initial_last_counted_sequence_{0};
  std::uint64_t next_last_counted_sequence_{0};

  bool has_global_waypoints_{false};
  bool has_obstacles_message_{false};
  bool obstacle_perception_degraded_{false};
  bool has_odometry_{false};
  bool has_commitment_{false};
  RacelineSplineResult committed_result_;
  bool safe_stop_latched_{false};
  RacelineSplineResult safe_stop_result_;
  int safe_stop_release_count_{0};
  int commitment_soft_violation_count_{0};
  double commitment_start_s_{0.0};
  int merge_complete_count_{0};
  bool merge_geometry_confirmed_{false};
  bool handoff_active_{false};
  bool avoid_state_observed_{false};
  bool has_state_{false};
  bool initial_stabilization_active_{false};
  bool initial_prepare_published_{false};
  bool initial_has_counted_sequence_{false};
  bool next_stabilization_active_{false};
  bool next_has_counted_sequence_{false};
  uint8_t current_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  std::optional<bool> last_published_side_;
  std::map<int, f110_msgs::msg::Obstacle> initial_cluster_union_;
  std::map<int, int> initial_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> next_cluster_union_;
  std::map<int, int> next_observation_counts_;
  std::map<int, f110_msgs::msg::Obstacle> committed_obstacle_guards_;
  std::set<int> completed_obstacle_ids_;

  bool require_obstacles_message_{true};
  double obstacle_stale_timeout_sec_{0.75};
  double odometry_stale_timeout_sec_{0.50};
  double merge_lateral_tolerance_m_{0.15};
  int merge_confirm_cycles_{15};
  int safe_stop_release_cycles_{8};
  int planning_period_ms_{50};
  double state_handoff_tail_ratio_{0.10};
  double state_handoff_speed_cap_mps_{6.0};
  int initial_observation_count_{3};
  double initial_observation_min_duration_sec_{0.15};
  double initial_observation_max_wait_sec_{0.35};
  int commitment_soft_violation_confirm_cycles_{3};
  double hard_collision_margin_m_{0.03};
  double chain_release_margin_m_{0.20};
  double commitment_lock_lateral_threshold_m_{0.10};
  double commitment_lock_longitudinal_m_{0.50};
  double obstacle_marker_scale_m_{0.35};
  double path_marker_width_m_{0.06};

  std::string global_waypoints_topic_{"/global_waypoints"};
  std::string obstacles_topic_{"/static_obs"};
  std::string frenet_odom_topic_{"/car_state/frenet/odom"};
  std::string state_topic_{"/state"};
  std::string ot_waypoints_topic_{"/avoid_waypoints"};
  std::string local_path_topic_{"/local_planning/path"};
  std::string compatibility_path_topic_{"/local_path"};
  std::string markers_topic_{"/local_planning/markers"};
  std::string frame_id_{"map"};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
