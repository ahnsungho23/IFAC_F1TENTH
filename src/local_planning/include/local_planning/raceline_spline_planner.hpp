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

#ifndef LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
#define LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

namespace local_planning
{

struct RacelineSplineParameters
{
  double detection_lookahead_m{12.0};
  double obstacle_cluster_gap_m{0.8};
  double obstacle_longitudinal_padding_m{0.35};
  double vehicle_half_width_m{0.1435};
  // Obstacle bounds are raw detector geometry, so vehicle size, physical margin, and measured
  // closed-loop tracking error are applied exactly once. Global d_left/d_right already encode
  // vehicle half-width and the generator's wall margin; only the independent wall reserve is
  // subtracted from those centre-of-vehicle limits.
  double safety_margin_m{0.03};
  // Fallback used when the LUT arrays below are all empty. A configured LUT is row-major with
  // speed as the outer axis and absolute curvature as the inner axis.
  double tracking_error_reserve_m{0.14};
  std::vector<double> tracking_error_lut_speed_bins_mps;
  std::vector<double> tracking_error_lut_curvature_bins_radpm;
  std::vector<double> tracking_error_lut_values_m;
  // Speed-dependent lateral-acceleration table copied from jazzy_main velocity_limits.csv.
  // Avoidance waypoint speeds are capped so v^2 * |kappa| stays within the interpolated limit.
  std::vector<double> avoidance_velocity_limit_speed_bins_mps;
  std::vector<double> avoidance_velocity_limit_lateral_accel_mps2;
  double wall_safety_margin_m{0.0};
  double fallback_track_half_width_m{1.50};

  std::vector<double> pre_apex_distances_m{6.0, 4.0, 2.0};
  std::vector<double> post_apex_distances_m{1.0, 2.0, 3.0};
  std::vector<double> transition_distance_scales{1.0, 1.25, 1.50};
  double outside_line_transition_scale{1.35};
  double post_merge_lookahead_m{2.0};
  double post_merge_min_time_sec{1.0};
  double minimum_target_offset_m{0.20};
  double maximum_target_offset_m{1.50};
  // When left/right candidate scores differ by less than this, pick the side with more track
  // headroom instead; reference widths are jitter-free, so centred-obstacle ties stay stable.
  double side_tie_epsilon_m{0.02};
  double maximum_lateral_slope{0.65};
  double maximum_curvature_radpm{3.20};
  double maximum_curvature_rate_radpm2{20.0};

  double safe_stop_buffer_m{0.40};
  double safe_stop_deceleration_mps2{2.5};
  int minimum_path_points{8};

  bool hasTrackingErrorLut() const;
  bool trackingErrorLutValid() const;
  bool avoidanceVelocityLimitValid() const;
  double limitedAvoidanceSpeed(double requested_speed_mps, double curvature_radpm) const;
  double trackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double avoidanceTrackingErrorReserve(double speed_mps, double curvature_radpm) const;
  double obstacleBaseClearance() const;
  double obstacleSafetyClearance(double speed_mps, double curvature_radpm) const;
  double trackBoundaryReserve(double speed_mps, double curvature_radpm) const;
};

struct EgoFrenetState
{
  double s{0.0};
  double d{0.0};
  double speed{0.0};
};

enum class SplinePlanKind
{
  kNoObstacle,
  kPreparation,
  kAvoidance,
  kSafeStop,
  kNoSafePath
};

struct SplineControlPoint
{
  double forward_s{0.0};
  double d{0.0};
};

struct RacelineSplineResult
{
  SplinePlanKind kind{SplinePlanKind::kNoObstacle};
  f110_msgs::msg::WpntArray path;
  bool go_left{false};
  double target_d{0.0};
  double merge_s{0.0};
  int obstacle_id{-1};
  std::vector<int> obstacle_ids;
  std::vector<SplineControlPoint> control_points;
  std::string reason;
};

enum class PathValidationFailureKind
{
  kNone,
  kInput,
  kNoForwardPath,
  kTrackBoundary,
  kObstacleCollision,
  kGeometry
};

struct PathValidationFailure
{
  PathValidationFailureKind kind{PathValidationFailureKind::kNone};
  std::string reason;
  int obstacle_id{-1};
  std::size_t waypoint_index{std::numeric_limits<std::size_t>::max()};
  double waypoint_s{std::numeric_limits<double>::quiet_NaN()};
  double waypoint_d{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_start{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_s_end{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_source_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_right{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_test_d_left{std::numeric_limits<double>::quiet_NaN()};
  double obstacle_clearance{std::numeric_limits<double>::quiet_NaN()};
};

// Static-obstacle planner whose only geometric reference is the ordered global race line.
// A candidate never searches the map for a shortcut: it keeps every selected global waypoint's
// s/order and changes only its local Frenet d offset before converting it back to map coordinates.
class RacelineSplinePlanner
{
public:
  explicit RacelineSplinePlanner(
    RacelineSplineParameters parameters = RacelineSplineParameters());

  void setParameters(const RacelineSplineParameters & parameters);
  bool setReference(const f110_msgs::msg::WpntArray & reference, std::string * error = nullptr);
  bool ready() const;
  double trackLength() const;
  double forwardDistance(double from_s, double to_s) const;
  std::vector<int> blockingClusterIds(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  f110_msgs::msg::WpntArray buildGlobalHandoffPath(
    double ego_s, double state_tail_ratio, double speed_cap_mps) const;
  f110_msgs::msg::WpntArray buildEmergencyStopPath(const EgoFrenetState & ego) const;
  RacelineSplineResult buildCommittedPathStop(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & committed_path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  RacelineSplineResult buildPreparationStop(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  RacelineSplineResult plan(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::optional<bool> & preferred_left = std::nullopt,
    bool allow_side_switch = true) const;

  bool validatePath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::string * error = nullptr,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt) const;

  void toCartesian(double s, double d, double & x, double & y, double & yaw) const;

private:
  struct ExpandedObstacle;
  struct Candidate;

  double wrapS(double s) const;
  std::size_t nextReferenceIndex(double s) const;
  std::size_t nearestReferenceIndex(double s) const;
  double maximumReferenceTrackingErrorReserve(
    const EgoFrenetState & ego, double start, double end) const;
  std::vector<ExpandedObstacle> expandVisibleObstacles(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  bool isBlockingRaceline(const ExpandedObstacle & obstacle) const;
  std::vector<ExpandedObstacle> nearestCluster(
    const std::vector<ExpandedObstacle> & obstacles) const;
  bool outsideIsLeft(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;
  bool computeSideTarget(
    const std::vector<ExpandedObstacle> & cluster,
    bool go_left,
    double & cluster_start,
    double & cluster_end,
    double & target_d,
    std::string & reason) const;
  bool targetFitsTrackBounds(
    const EgoFrenetState & ego,
    double cluster_start,
    double cluster_end,
    bool go_left,
    double target_d,
    std::string & reason,
    double * min_headroom = nullptr) const;
  Candidate buildCandidate(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    bool go_left,
    double entry_transition_scale,
    double exit_transition_scale,
    bool outside_is_left,
    double cluster_start,
    double cluster_end,
    double target_d) const;
  RacelineSplineResult buildSafeStop(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    const ExpandedObstacle & blocking) const;
  void applyAvoidanceVelocityLimit(f110_msgs::msg::WpntArray & path) const;
  void updateGeometryAndAcceleration(f110_msgs::msg::WpntArray & path) const;
  bool validateCandidate(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<ExpandedObstacle> & visible,
    std::string & reason,
    std::size_t start_index = 0U,
    std::size_t minimum_points = 0U,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt) const;

  RacelineSplineParameters parameters_;
  f110_msgs::msg::WpntArray reference_;
  double track_length_{0.0};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
