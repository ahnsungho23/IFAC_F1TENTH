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

// 계획에 사용되는 모든 튜닝값. 노드가 YAML 값을 선언·검증한 뒤 이 구조체로 전달한다.
struct RacelineSplineParameters
{
  // 장애물 탐색, 군집화 및 충돌 envelope
  double detection_lookahead_m{12.0};
  double obstacle_cluster_gap_m{0.8};
  double obstacle_longitudinal_padding_m{0.35};
  double obstacle_clearance_m{0.35};
  double blocking_margin_m{0.10};
  double vehicle_half_width_m{0.121};
  double boundary_margin_m{0.13};
  double fallback_track_half_width_m{1.50};

  // 장애물 전후 spline 제어점 거리와 전환 길이 후보
  std::vector<double> pre_apex_distances_m{4.0, 3.0, 1.5};
  std::vector<double> post_apex_distances_m{1.5, 3.0, 4.0};
  std::vector<double> transition_distance_scales{1.0, 1.25, 1.50};
  double outside_line_transition_scale{1.35};
  double post_merge_lookahead_m{2.0};
  double post_merge_min_time_sec{1.0};
  double minimum_target_offset_m{0.20};
  double maximum_target_offset_m{1.50};
  double commitment_clearance_reserve_m{0.05};
  double maximum_lateral_slope{0.65};
  double maximum_curvature_radpm{3.20};
  double maximum_curvature_rate_radpm2{20.0};

  // 회피 경로를 만들 수 없을 때 사용하는 감속 경로 조건
  double safe_stop_buffer_m{0.80};
  double safe_stop_deceleration_mps2{2.5};
  int minimum_path_points{8};
};

// Frenet 좌표계에서 본 차량 상태. speed는 경로 뒤쪽 tail 길이를 속도에 맞추는 데도 쓰인다.
struct EgoFrenetState
{
  double s{0.0};
  double d{0.0};
  double speed{0.0};
};

// 계획 결과가 현재 파이프라인에서 의미하는 동작 단계
enum class SplinePlanKind
{
  kNoObstacle,   // 레이스 라인을 막는 장애물이 없음
  kPreparation,  // 관측 안정화 중 장애물 앞까지 감속
  kAvoidance,    // 좌/우 d-offset spline 주행
  kSafeStop,     // 검증된 정지 prefix 또는 제자리 정지
  kNoSafePath    // 입력 오류 또는 안전한 경로 생성 실패
};

// 차량 현재 s를 0으로 둔 unwrapped 전방 거리와 그 지점의 횡방향 offset
struct SplineControlPoint
{
  double forward_s{0.0};
  double d{0.0};
};

// 계획 경로와 state/시각화 계층이 함께 사용하는 부가 정보
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

// 경로 검증 실패를 안전도와 원인별로 분류한다.
enum class PathValidationFailureKind
{
  kNone,               // 실패 없음
  kInput,              // 참조 경로·ego·인자 오류
  kNoForwardPath,      // ego 앞에 남은 경로가 없음
  kTrackBoundary,      // 차량 중심 경로가 트랙 폭을 벗어남
  kObstacleCollision,  // 팽창된 장애물 envelope와 교차
  kGeometry            // 순서, 기울기, 곡률 또는 곡률 변화율 위반
};

// 실패 로그에서 재현에 필요한 waypoint와 장애물 경계를 보존한다.
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

// ============================================================================
// RacelineSplinePlanner — 글로벌 레이스 라인 순서를 보존하는 정적 장애물 회피기
// ============================================================================
// 지도에서 별도의 지름길이나 최근접 Cartesian 경로를 탐색하지 않는다. 선택한 글로벌
// waypoint의 s와 순서를 그대로 유지하고, 국소 Frenet d만 cubic spline으로 바꾼 뒤 각
// waypoint의 법선 방향으로 map 좌표를 다시 계산한다. 이 불변식이 뱀 모양 트랙의 가까운
// 반대 branch로 경로가 점프하는 것을 막는다.
class RacelineSplinePlanner
{
public:
  explicit RacelineSplinePlanner(
    RacelineSplineParameters parameters = RacelineSplineParameters());

  void setParameters(const RacelineSplineParameters & parameters);

  // 폐곡선 기준 경로를 등록한다. s_m은 유한하고 엄격히 증가해야 한다.
  bool setReference(const f110_msgs::msg::WpntArray & reference, std::string * error = nullptr);
  bool ready() const;
  double trackLength() const;

  // 폐곡선 wrap을 고려한 from_s → to_s의 전방 거리
  double forwardDistance(double from_s, double to_s) const;

  // ego 앞에서 레이스 라인을 막는 가장 가까운 장애물 군집의 ID
  std::vector<int> blockingClusterIds(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;
  f110_msgs::msg::WpntArray buildGlobalHandoffPath(
    double ego_s, double state_tail_ratio, double speed_cap_mps) const;

  // Frenet odometry까지 신뢰할 수 없을 때 현재 d에서 만드는 zero-speed hold
  f110_msgs::msg::WpntArray buildEmergencyStopPath(const EgoFrenetState & ego) const;

  // 기존 확정 경로의 남은 기하를 따라 충돌 전에 정지하는 prefix
  RacelineSplineResult buildCommittedPathStop(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & committed_path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  // 최초 장애물 군집을 안정화하는 동안 발행할 감속 prefix
  RacelineSplineResult buildPreparationStop(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles) const;

  RacelineSplineResult plan(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::optional<bool> & preferred_left = std::nullopt,
    bool allow_side_switch = true) const;

  // 이미 발행한 경로의 남은 구간을 최신 장애물 against 트랙·기하 조건으로 재검증한다.
  bool validatePath(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    std::string * error = nullptr,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & obstacle_clearance = std::nullopt,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt) const;

  // 기준 경로를 선형 보간해 Frenet (s, d)를 map (x, y, yaw)로 변환한다.
  void toCartesian(double s, double d, double & x, double & y, double & yaw) const;

private:
  // 구현 파일에서만 필요한 팽창 장애물과 spline 후보 표현
  struct ExpandedObstacle;
  struct Candidate;

  // 폐곡선 기준 경로 index/거리 보조 함수
  double wrapS(double s) const;
  std::size_t nextReferenceIndex(double s) const;
  std::size_t nearestReferenceIndex(double s) const;
  std::vector<ExpandedObstacle> expandVisibleObstacles(
    const EgoFrenetState & ego,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    const std::optional<double> & obstacle_clearance = std::nullopt) const;

  // 레이스 라인을 막는 첫 군집과 코너 바깥쪽 방향을 결정한다.
  bool isBlockingRaceline(const ExpandedObstacle & obstacle) const;
  std::vector<ExpandedObstacle> nearestCluster(
    const std::vector<ExpandedObstacle> & obstacles) const;
  bool outsideIsLeft(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & cluster) const;

  // 한쪽 회피 목표 d를 계산하고 장애물 구간의 트랙 폭에 들어가는지 빠르게 검사한다.
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
    std::string & reason) const;

  // 제어점을 만들고 spline을 표본화한 뒤 전체 안전 검증을 통과한 후보만 반환한다.
  Candidate buildCandidate(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    bool go_left,
    double transition_scale,
    bool outside_is_left,
    double cluster_start,
    double cluster_end,
    double target_d) const;
  RacelineSplineResult buildSafeStop(
    const EgoFrenetState & ego,
    const std::vector<ExpandedObstacle> & visible,
    const ExpandedObstacle & blocking) const;

  // d 이동으로 바뀐 map 기하의 heading/curvature/acceleration을 다시 계산한다.
  void updateGeometryAndAcceleration(f110_msgs::msg::WpntArray & path) const;

  // 트랙 경계, 장애물, s 순서, 횡기울기, 곡률 및 곡률 변화율을 한 번에 검사한다.
  bool validateCandidate(
    const EgoFrenetState & ego,
    const f110_msgs::msg::WpntArray & path,
    const std::vector<ExpandedObstacle> & visible,
    std::string & reason,
    std::size_t start_index = 0U,
    std::size_t minimum_points = 0U,
    PathValidationFailure * failure = nullptr,
    const std::optional<double> & maximum_collision_forward_m = std::nullopt) const;

  // 설정과 순서가 보존된 글로벌 기준 경로
  RacelineSplineParameters parameters_;
  f110_msgs::msg::WpntArray reference_;
  double track_length_{0.0};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__RACELINE_SPLINE_PLANNER_HPP_
