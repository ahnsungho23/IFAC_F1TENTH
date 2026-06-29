#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace state_machine
{

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode();

private:
  struct ObstacleEvidence
  {
    bool has_obstacle{false};
    bool has_visible_obstacle{false};

    bool has_static_obstacle{false};
    bool has_dynamic_obstacle{false};

    bool static_blocks_global{false};
    bool dynamic_blocks_global{false};
    bool global_blocked{false};

    bool simultaneous_static_dynamic_on_global{false};

    bool avoidance_needed{false};
    bool overtake_candidate{false};

    int32_t closest_obstacle_id{-1};
    int32_t closest_static_id{-1};
    int32_t closest_dynamic_id{-1};

    double closest_s_gap_m{std::numeric_limits<double>::infinity()};
    double closest_static_s_gap_m{std::numeric_limits<double>::infinity()};
    double closest_dynamic_s_gap_m{std::numeric_limits<double>::infinity()};

    double closest_d_center_m{0.0};
    double closest_size_m{0.0};

    double closest_static_d_center_m{0.0};
    double closest_dynamic_d_center_m{0.0};

    double closest_dynamic_vs_mps{0.0};
    double closest_dynamic_vd_mps{0.0};

    bool closest_is_static{true};
    bool closest_is_visible{false};

    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  std::optional<uint8_t> parse_state(const std::string & state_name) const;
  bool is_fresh(const rclcpp::Time & stamp, double timeout_sec) const;
  bool has_fresh_global() const;
  bool has_fresh_frenet() const;
  bool has_fresh_avoid_wpnts() const;
  bool has_fresh_overtake_wpnts() const;
  bool has_fresh_obstacles() const;
  ObstacleEvidence compute_obstacle_evidence(
    const f110_msgs::msg::ObstacleArray & msg) const;

  void on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg); //토픽 callback 함수 /car_state/frenet/odom에서 받은 nav_msgs/msg/Odometry
  void on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg); //global_waypoints에서 받은 f110_msgs/msg/WpntArray lobal waypoint freshness 판단에 쓰임
  void on_avoid_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg); //STATE_AVOID로 갈 수 있는지 판단할 때 씀
  void on_overtake_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg); //STATE_OVERTAKE로 갈 수 있는지 판단할 때 씀
  void on_obstacles(const f110_msgs::msg::ObstacleArray::SharedPtr msg);

  uint8_t resolve_requested_state(); //state 출력: 최종 상태값 uint8_t
  void publish_state(); //state 토픽으로 메시지를 publish resolve_requested_state()로 최종 state를 결정

  std::string state_topic_;
  std::string frame_id_;
  std::string default_state_name_;
  double avoid_stale_timeout_sec_{0.5};
  double overtake_stale_timeout_sec_{0.5};
  double global_stale_timeout_sec_{2.0};
  double frenet_stale_timeout_sec_{0.5};
  double obstacles_stale_timeout_sec_{0.5};
  double obstacle_lookahead_m_{3.0};
  double global_blocking_d_threshold_m_{0.4};

  bool has_frenet_{false};
  bool has_global_{false};
  bool has_avoid_wpnts_{false};
  bool has_overtake_wpnts_{false};
  bool has_obstacles_{false};
  bool has_current_frenet_pose_{false};
  bool has_track_length_{false};
  rclcpp::Time last_frenet_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoid_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_overtake_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_obstacles_time_{0, 0, RCL_ROS_TIME};
  double current_s_m_{0.0};
  double current_d_m_{0.0};
  double track_length_m_{0.0};
  ObstacleEvidence obstacle_evidence_;
  std::optional<uint8_t> last_published_state_;

  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr overtake_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
