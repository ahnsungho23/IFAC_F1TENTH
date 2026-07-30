#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

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
  std::optional<uint8_t> parse_state(const std::string & state_name) const;
  bool is_fresh(const rclcpp::Time & stamp, double timeout_sec) const;
  bool local_path_confirmed(
    const std::deque<bool> & history,
    const f110_msgs::msg::OTWpntArray::SharedPtr msg) const;
  bool has_fresh_global() const;
  bool has_fresh_frenet() const;
  bool has_avoid_wpnts() const;
  bool has_fresh_overtake_wpnts() const;

  // --- Transition gates ---
  //최근 N회 중 M회 이상 non-empty 경로가 수신되면 상태 진입 허용.
  //단, allow_*_transition_이 false면 M-of-N 평가 전에 즉시 차단한다.
  bool can_enter_avoid() const;
  bool can_enter_overtake() const;

  //local path(avoid/overtake)에서 global path로 합류(복귀)해도 되는지 판단.
  //전제: local path는 ego에서 시작해 merge 지점에서 끝나는 세그먼트(tail은 d->0 수렴).
  //frenet_odom: 현재 ego 차량, local_wpnts: 회피 경로(avoid/overtake), global_wpnts: global 경로.
  bool enter_to_global(
    const nav_msgs::msg::Odometry::SharedPtr frenet_odom,
    const f110_msgs::msg::OTWpntArray::SharedPtr local_wpnts,
    const f110_msgs::msg::WpntArray::SharedPtr global_wpnts);
  //enter_to_global 판정 진입점: 평가 대상 state가 바뀌거나 입력이 stale이면
  //지속 타이머를 리셋해 이전 기동의 만족 시간이 새 판정으로 넘어가지 않게 한다.
  bool evaluate_enter_to_global(
    uint8_t eval_state,
    bool local_fresh,
    const f110_msgs::msg::OTWpntArray::SharedPtr & local_wpnts);

  //토픽 callback: freshness 타임스탬프(및 존재 여부)만 갱신한다.
  void on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void on_avoid_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg);       //STATE_AVOID 게이트용
  void on_overtake_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg);    //STATE_OVERTAKE 게이트용

  uint8_t resolve_requested_state(); //committed_state_ 기반 FSM 1-step: 진입(can_enter_*)/복귀(enter_to_global) 판정 후 committed_state_ 갱신·반환
  void publish_state();              //anti-oscillation 필터 적용 후 /state 발행

  std::string state_topic_;
  std::string frame_id_;
  std::string default_state_name_;
  double overtake_stale_timeout_sec_{0.5};
  //상태 진입 기능 토글. false면 해당 상태로 절대 전이하지 않는다(복귀는 항상 허용).
  bool allow_avoid_transition_{true};
  bool allow_overtake_transition_{true};
  int64_t local_path_confirmation_window_size_{5};
  int64_t local_path_confirmation_min_hits_{3};
  double global_stale_timeout_sec_{2.0};
  double frenet_stale_timeout_sec_{0.5};

  // Global re-entry parameters (local path -> global path 합류 조건)
  double enter_global_sec_{0.5};        //global 위에 ego가 존재해야 하는 최소 유지 시간
  double enter_global_threshold_{0.2};  //s,d domain 기준 ego가 global 위에 있는지 판단하는 threshold(단위 m)
  double enter_global_tail_ratio_{0.1};    //local path 후방에서 tail(merge 합류 구간)로 간주하는 비율
  double enter_global_s_gap_tol_m_{0.5};   //ego가 tail 구간에 실제 도달했는지 판정하는 순환 s-gap 상한(단위 m)

  // Global re-entry state (지속시간 타이머)
  std::optional<rclcpp::Time> enter_global_ok_since_;  //합류 조건을 연속 만족하기 시작한 시각
  uint8_t enter_global_eval_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};  //타이머가 평가 중인 state

  bool has_frenet_{false};
  bool has_global_{false};
  bool has_avoid_wpnts_{false};
  bool has_overtake_wpnts_{false};
  rclcpp::Time last_frenet_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_overtake_time_{0, 0, RCL_ROS_TIME};
  std::optional<uint8_t> last_published_state_;

  //게이트/합류 판단에 넘길 최신 메시지들.
  nav_msgs::msg::Odometry::SharedPtr frenet_odom_msg_;         //현재 ego 차량(frenet)
  f110_msgs::msg::WpntArray::SharedPtr global_wpnts_msg_;      //global 경로
  f110_msgs::msg::OTWpntArray::SharedPtr avoid_wpnts_msg_;      //정적 장애물 회피 wpnt
  f110_msgs::msg::OTWpntArray::SharedPtr overtake_wpnts_msg_;   //동적 장애물 회피 wpnt
  std::deque<bool> avoid_path_history_;                         //최근 avoid 경로 non-empty 여부
  std::deque<bool> overtake_path_history_;                      //최근 overtake 경로 non-empty 여부

  // FSM 현재 상태 (committed)
  uint8_t committed_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};

  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr overtake_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
