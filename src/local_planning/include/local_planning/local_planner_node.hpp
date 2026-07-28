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

  bool require_obstacles_message_{true};
  bool static_obstacles_only_{true};
  double static_speed_threshold_mps_{0.25};
  double obstacle_stale_timeout_sec_{0.75};
  double odometry_stale_timeout_sec_{0.50};
  double merge_lateral_tolerance_m_{0.15};
  int merge_confirm_cycles_{15};
  int planning_period_ms_{50};
  bool publish_standalone_local_{false};
  double obstacle_marker_scale_m_{0.35};
  double path_marker_width_m_{0.06};

  std::string global_waypoints_topic_{"/global_waypoints"};
  std::string obstacles_topic_{"/static_obs"};
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
