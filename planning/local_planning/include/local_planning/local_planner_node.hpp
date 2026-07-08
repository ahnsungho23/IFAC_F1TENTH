#ifndef LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <memory>

namespace local_planning
{

/**
 * @brief 웨이포인트별 장애물 바운드 정보 구조체
 */
struct ObstacleBound {
  double min_d{1e9};
  double max_d{-1e9};
  bool collision{false};
};

/**
 * @brief 정적 장애물 감지 및 최소자승법(Least Squares) 스플라인 기반 로컬 플래너 노드
 */
class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalPlannerNode() override = default;

private:
  // ROS 2 파라미터 로드 및 초기화
  void initParameters();
  void initInterfaces();

  // 콜백 함수들
  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  // 핵심 알고리즘 함수들
  std::optional<int> parseIndex(const std::string & s);
  int findClosestWaypointIndex(double x, double y);
  
  /**
   * @brief 장애물 간섭 검사 및 회피 방향 설정 (벽/장애물 분리 검사)
   * @param start_idx 검사 시작 웨이포인트 인덱스
   * @param count 검사할 웨이포인트 개수
   * @param obs_detected 장애물 감지 여부 반환
   * @param avoid_left 왼쪽 회피 여부 반환 (true: 좌측 회피, false: 우측 회피)
   * @param collision_indices 충돌이 감지된 웨이포인트 인덱스 목록
   * @param obs_bounds 윈도우 내 웨이포인트별 장애물 바운드 정보
   */
  void detectObstaclesAndDecideDirection(
    int start_idx, int count,
    bool & obs_detected, bool & avoid_left,
    std::vector<int> & collision_indices,
    std::vector<ObstacleBound> & obs_bounds);

  /**
   * @brief 가중 최소자승법(Weighted Least Squares) 기반 다항식 스플라인 계수 도출
   * @param s_vals 거리 s 데이터 벡터
   * @param d_vals 목표 오프셋 d 데이터 벡터
   * @param w_vals 가중치 w 데이터 벡터
   * @param degree 다항식 차수
   * @return 다항식 계수 벡터 [c0, c1, c2, ...]
   */
  Eigen::VectorXd computeLeastSquaresSpline(
    const std::vector<double> & s_vals,
    const std::vector<double> & d_vals,
    const std::vector<double> & w_vals,
    int degree);

  // 시각화 마커 및 경로 발행
  void publishDebugVisualization(
    const f110_msgs::msg::WpntArray & local_wpnts,
    const std::vector<geometry_msgs::msg::Point> & obs_points);

  // ROS 구독자 및 발행자
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr ot_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr local_wpnts_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exact_local_path_pub_; // /local_path (하나의 닫힌 곡선)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 내부 상태 변수
  f110_msgs::msg::WpntArray global_wpnts_;
  nav_msgs::msg::OccupancyGrid grid_map_;
  nav_msgs::msg::Odometry current_odom_;
  bool has_global_{false};
  bool has_map_{false};
  bool has_odom_{false};

  // 파라미터 변수
  int lookahead_wpnt_num_{40};         // 로컬 경로로 생성할 웨이포인트 수
  double safety_margin_{0.65};         // 충돌 판단 마진 (미터)
  double wall_margin_{0.35};           // 벽 안전 마진 (미터)
  double avoid_offset_{1.0};           // 회피 목표 오프셋 (미터)
  int poly_degree_{3};                 // 최소자승법 다항식 차수 (기본 3차 다항식)
  double speed_reduction_ratio_{0.6};  // 장애물 회피 시 감속 비율
  bool publish_standalone_local_{true};// otwpnts 외에 /local_waypoints 직접 발행 여부
  int timer_period_ms_{500};           // 연산 주기 (밀리초) - 0.5초마다 갱신
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__LOCAL_PLANNER_NODE_HPP_
