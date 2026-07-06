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

  // Result of evaluating one published local path (/avoid_waypoints or /overtake_waypoints).
  // Used as an additional transition gate on top of freshness checks.
  struct LocalPathAssessment
  {
    bool has_path{false};            // non-empty waypoint array received
    bool long_enough{false};         // covers at least path_min_length_m
    bool starts_near_ego{false};     // first waypoint close to current ego s
    bool within_track_bounds{false}; // every waypoint keeps path_min_bound_margin_m to bounds
    bool collision_free{false};      // keeps path_min_obstacle_gap_m to latest obstacles
    bool curvature_ok{false};        // max |kappa| below path_max_kappa_radpm

    bool valid{false};               // AND of all enabled checks -> transition gate

    double path_length_m{0.0};
    double start_s_gap_m{std::numeric_limits<double>::infinity()};
    double min_bound_margin_m{std::numeric_limits<double>::infinity()};
    double min_obstacle_gap_m{std::numeric_limits<double>::infinity()};
    double max_abs_kappa_radpm{0.0};

    // TODO: normalized quality score in [0, 1] for AVOID/OVERTAKE tie-breaking
    // and for smooth hysteresis (enter above score_hi, leave below score_lo).
    double score{0.0};

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

  // --- Local path evaluation (transition gate) ---
  double wrapped_s_gap(double from_s, double to_s) const; //track wrap을 고려한 전방 s 거리
  LocalPathAssessment evaluate_local_path(const f110_msgs::msg::OTWpntArray & msg) const;
  bool can_enter_avoid() const;    //fresh + valid avoid path일 때만 STATE_AVOID 허용
  bool can_enter_overtake() const; //fresh + valid overtake path일 때만 STATE_OVERTAKE 허용

  // --- Transition stability (anti-oscillation) ---
  bool committed_state_supported() const; //현재 확정 상태를 지탱하는 path가 아직 유효한지
  uint8_t apply_transition_stability(uint8_t requested_state);
  //resolve_requested_state()의 raw 요청에 dwell time + N-tick 확인(debounce)을 적용.
  //안전 fallback(지탱 path가 사라져 GLOBAL로 복귀)은 지연 없이 즉시 통과시킴.

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

  // Local path evaluation parameters
  bool path_eval_enabled_{true};
  double path_min_length_m_{1.5};
  double path_start_max_gap_m_{1.0};
  double path_min_bound_margin_m_{0.05};
  double path_min_obstacle_gap_m_{0.3};
  double path_max_kappa_radpm_{3.0};

  // Transition stability parameters
  double min_state_dwell_sec_{1.0};
  int transition_confirm_ticks_{3};

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

  // Latest raw messages kept so local paths can be re-evaluated every tick
  // against the freshest ego pose and obstacle set.
  f110_msgs::msg::OTWpntArray last_avoid_wpnts_;
  f110_msgs::msg::OTWpntArray last_overtake_wpnts_;
  f110_msgs::msg::ObstacleArray last_obstacles_;
  LocalPathAssessment avoid_assessment_;
  LocalPathAssessment overtake_assessment_;

  // Transition stability state
  uint8_t committed_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  uint8_t candidate_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  int candidate_ticks_{0};
  rclcpp::Time last_transition_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr overtake_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
