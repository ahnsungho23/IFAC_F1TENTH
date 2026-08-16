#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace state_machine
{

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode();

private:
  std::optional<uint8_t> parse_state(const std::string & state_name) const;
  std::optional<int> parse_waypoint_index(const std::string & value) const;
  bool is_fresh(const rclcpp::Time & stamp, double timeout_sec) const;
  bool local_path_confirmed(
    const std::deque<bool> & history,
    const f110_msgs::msg::OTWpntArray::SharedPtr msg) const;
  bool has_valid_global() const;
  bool has_fresh_frenet() const;
  bool has_avoid_wpnts() const;
  bool has_interfering_opponent() const;
  bool has_front_static_obstacle() const;
  bool validate_global_waypoints(
    const f110_msgs::msg::WpntArray & message,
    std::string * error) const;

  // AVOID is the highest-priority transition from every driving state.
  bool can_enter_avoid() const;

  // Evaluate whether an ego-to-merge local segment has converged to the global raceline.
  bool enter_to_global(
    const nav_msgs::msg::Odometry::SharedPtr frenet_odom,
    const f110_msgs::msg::OTWpntArray::SharedPtr local_wpnts,
    const f110_msgs::msg::WpntArray::SharedPtr global_wpnts);
  bool evaluate_enter_to_global(
    uint8_t eval_state,
    bool local_available,
    const f110_msgs::msg::OTWpntArray::SharedPtr & local_wpnts);
  bool evaluate_avoid_path_exhausted();
  bool evaluate_stopped_path_clear();

  void on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void on_avoid_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg);
  void on_opponent(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void on_static_obstacles(const f110_msgs::msg::ObstacleArray::SharedPtr msg);

  uint8_t resolve_requested_state();
  void publish_state_cycle();
  void publish_selected_waypoints(uint8_t state);
  std::optional<f110_msgs::msg::WpntArray> select_waypoints(
    uint8_t state,
    const rclcpp::Time & stamp);
  std::optional<f110_msgs::msg::WpntArray> build_global_waypoints(
    const rclcpp::Time & stamp);
  f110_msgs::msg::WpntArray convert_ot_waypoints(
    const f110_msgs::msg::OTWpntArray & source,
    const rclcpp::Time & stamp) const;
  nav_msgs::msg::Path build_path(const f110_msgs::msg::WpntArray & waypoints) const;

  std::string state_topic_;
  std::string local_waypoints_topic_;
  std::string local_path_topic_;
  std::string frame_id_;
  std::string default_state_name_;
  std::string invalid_local_path_policy_;
  std::string static_obstacles_topic_;

  bool allow_avoid_transition_{true};
  bool allow_cruise_transition_{true};
  int64_t local_path_confirmation_window_size_{5};
  int64_t local_path_confirmation_min_hits_{3};
  int waypoint_num_{50};
  double global_publisher_warn_timeout_sec_{5.0};
  double frenet_stale_timeout_sec_{0.5};
  double opponent_stale_timeout_sec_{0.3};

  double enter_global_sec_{0.5};
  double enter_global_threshold_{0.2};
  double enter_global_tail_ratio_{0.1};
  double enter_global_s_gap_tol_m_{0.5};
  double avoid_path_stale_timeout_sec_{0.5};
  double avoid_path_exhaustion_sec_{0.5};
  double avoid_path_exhaustion_s_gap_tol_m_{0.75};
  double static_obstacles_stale_timeout_sec_{0.3};
  double stopped_path_clear_sec_{1.0};
  double stopped_path_speed_threshold_mps_{0.01};
  double stopped_path_obstacle_lookahead_m_{3.0};
  double stopped_path_ego_half_width_m_{0.16};

  std::optional<rclcpp::Time> enter_global_ok_since_;
  std::optional<rclcpp::Time> avoid_path_exhausted_since_;
  std::optional<rclcpp::Time> stopped_path_clear_since_;
  uint8_t enter_global_eval_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};

  bool has_frenet_{false};
  bool has_global_{false};
  bool has_avoid_wpnts_{false};
  bool opponent_seen_{false};
  bool opponent_interfering_{false};
  bool stopped_path_clear_latched_{false};
  rclcpp::Time last_frenet_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_opponent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoid_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_static_obstacles_time_{0, 0, RCL_ROS_TIME};
  std::optional<uint8_t> last_published_state_;

  nav_msgs::msg::Odometry::SharedPtr frenet_odom_msg_;
  f110_msgs::msg::WpntArray::SharedPtr global_wpnts_msg_;
  f110_msgs::msg::OTWpntArray::SharedPtr avoid_wpnts_msg_;
  f110_msgs::msg::OTWpntArray::SharedPtr last_non_empty_avoid_wpnts_msg_;
  f110_msgs::msg::ObstacleArray::SharedPtr static_obstacles_msg_;
  std::deque<bool> avoid_path_history_;

  uint8_t committed_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};

  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr local_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr opponent_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr static_obstacles_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
