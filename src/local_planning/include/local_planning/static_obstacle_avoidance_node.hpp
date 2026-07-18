#ifndef LOCAL_PLANNING__STATIC_OBSTACLE_AVOIDANCE_NODE_HPP_
#define LOCAL_PLANNING__STATIC_OBSTACLE_AVOIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <f110_msgs/msg/wpnt_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <chrono>

namespace local_planning
{

/**
 * @brief 5차 다항식(Quintic Polynomial) 횡방향 보간기
 * d(s) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
 */
class QuinticPolynomial
{
public:
  QuinticPolynomial(double xs, double vxs, double axs,
                    double xe, double vxe, double axe, double T)
  : T_(T)
  {
    a0_ = xs;
    a1_ = vxs;
    a2_ = axs / 2.0;

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix3d A;
    A << T3,    T4,    T5,
         3*T2,  4*T3,  5*T4,
         6*T,   12*T2, 20*T3;

    Eigen::Vector3d b;
    b << xe - a0_ - a1_*T - a2_*T2,
         vxe - a1_ - 2*a2_*T,
         axe - 2*a2_;

    if (T > 1e-4) {
      Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
      a3_ = x(0);
      a4_ = x(1);
      a5_ = x(2);
    } else {
      a3_ = a4_ = a5_ = 0.0;
    }
  }

  double calc_d(double t) const
  {
    if (t < 0.0) return a0_;
    if (t > T_) t = T_;
    return a0_ + a1_*t + a2_*t*t + a3_*t*t*t + a4_*t*t*t*t + a5_*t*t*t*t*t;
  }

  double calc_d_dot(double t) const
  {
    if (t < 0.0 || t > T_) return 0.0;
    return a1_ + 2*a2_*t + 3*a3_*t*t + 4*a4_*t*t*t + 5*a5_*t*t*t*t;
  }

  double calc_d_ddot(double t) const
  {
    if (t < 0.0 || t > T_) return 0.0;
    return 2*a2_ + 6*a3_*t + 12*a4_*t*t + 20*a5_*t*t*t;
  }

private:
  double a0_{0.0}, a1_{0.0}, a2_{0.0}, a3_{0.0}, a4_{0.0}, a5_{0.0};
  double T_{1.0};
};

/**
 * @brief 평가를 위한 로컬 후보 경로 구조체
 */
struct CandidatePath
{
  std::vector<f110_msgs::msg::Wpnt> waypoints;
  double cost_obs{0.0};
  double cost_lat{0.0};
  double cost_smooth{0.0};
  double total_cost{std::numeric_limits<double>::max()};
  double target_offset{0.0};
  double return_length{0.0};
  std::string side_str{"none"};
  bool is_valid{false};
};

/**
 * @brief Comception 정적 장애물 감지 기반 로컬 플래너 노드
 */
class StaticObstacleAvoidanceNode : public rclcpp::Node
{
public:
  explicit StaticObstacleAvoidanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StaticObstacleAvoidanceNode() override = default;

private:
  void declareAndLoadParameters();
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  void globalWaypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void frenetOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void planningTimerCallback();

  bool isObstacleBlockingCorridor(const f110_msgs::msg::Obstacle & obs, double car_s) const;
  void evaluateAndPublishLocalPath();

  std::vector<CandidatePath> generateCandidatePaths(
    const f110_msgs::msg::Obstacle & target_obs,
    double car_s, double car_d, double car_d_dot, double car_d_ddot);

  CandidatePath generateSegmentPath(
    double car_s, double car_d, double car_d_dot, double car_d_ddot,
    double target_offset, double approach_len, double return_len,
    const std::string & side,
    const f110_msgs::msg::Obstacle & target_obs);

  f110_msgs::msg::Wpnt interpolateGlobalWpnt(double s_target) const;
  void convertFrenetToCartesian(f110_msgs::msg::Wpnt & wpnt) const;

  void evaluateCandidateCosts(
    std::vector<CandidatePath> & candidates,
    const f110_msgs::msg::Obstacle & target_obs);

  void adjustThrottleAlongPath(CandidatePath & path) const;

  void publishVisualization(
    const CandidatePath & best_path,
    const std::vector<CandidatePath> & all_candidates);

  void triggerEventDrivenPlanning();

  // ROS 2 Subscribers & Publishers
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;

  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr ot_wpnts_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cand_markers_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 내부 상태 변수
  f110_msgs::msg::WpntArray global_wpnts_;
  nav_msgs::msg::Odometry frenet_odom_;
  f110_msgs::msg::ObstacleArray obstacles_;
  bool has_global_{false};
  bool has_odom_{false};
  bool has_obs_{false};
  bool local_path_active_{false};
  double track_length_{0.0};
  rclcpp::Time last_avoidance_time_;
  rclcpp::Time last_eval_time_;
  double min_eval_interval_sec_{0.015}; // 15ms 쓰로틀링 (Event-Driven 부하 제어)

  // 파라미터
  double planning_frequency_{50.0};
  double lookahead_distance_{15.0};
  double corridor_width_{0.65};
  double safety_margin_{0.28};
  double passing_buffer_s_{1.2}; // 장애물 전후 완충 직선 통과 구간 (m)
  double waypoint_step_{0.1};
  double max_lat_accel_{6.0};
  double min_speed_{1.5};
  double static_vel_threshold_{0.35};

  double weight_obs_{10.0};
  double weight_lat_{1.0};
  double weight_smooth_{2.5};
  double weight_speed_{1.5};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__STATIC_OBSTACLE_AVOIDANCE_NODE_HPP_
