#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <f110_msgs/msg/wpnt.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/state_machine.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <cctype>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

class WpntPublisher : public rclcpp::Node
{
public:
  WpntPublisher() : Node("wpnt_publisher"), last_ot_update_time_(this->now()),
    last_avoid_update_time_(this->now()),
    ot_hold_duration_(0, 2000000000)
  {
    waypoint_num_ = this->declare_parameter<int>("waypoint_num", 50);
    avoid_path_ttl_sec_ = this->declare_parameter<double>("avoid_path_ttl_sec", 0.75);
    global_waypoints_topic_ =
      this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    avoid_waypoints_topic_ =
      this->declare_parameter<std::string>("avoid_waypoints_topic", "/avoid_waypoints");
    frenet_odometry_topic_ =
      this->declare_parameter<std::string>("frenet_odometry_topic", "/car_state/frenet/odom");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/state");
    local_waypoints_topic_ =
      this->declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
    local_path_topic_ =
      this->declare_parameter<std::string>("local_path_topic", "/local_waypoints/path");
    if (waypoint_num_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "waypoint_num (%d) must be > 0. Forcing to 1.", waypoint_num_);
      waypoint_num_ = 1;
    }
    if (!(avoid_path_ttl_sec_ > 0.0))
    {
      throw std::invalid_argument("avoid_path_ttl_sec must be positive");
    }

    // 최근 값만 중요: KeepLast(1), reliable
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    

    local_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(local_waypoints_topic_, qos);
    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_path_topic_, qos);

    // 글로벌 웨이포인트는 latched 특성 필요 시 transient_local 사용
    auto qos_gl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    global_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
      global_waypoints_topic_, qos_gl, std::bind(&WpntPublisher::onGlobalWaypoints, this, _1));

    // OT 스플라인 웨이포인트 (일반적으로 volatile)
    // 동적 장애물 경로와 정적 회피(local_planning)를 서로 다른 콜백으로 분리 수신
    ot_sub_ = this->create_subscription<f110_msgs::msg::OTWpntArray>(
      "/overtake_waypoints", qos, std::bind(&WpntPublisher::onOTWpnts, this, _1));
    avoid_sub_ = this->create_subscription<f110_msgs::msg::OTWpntArray>(
      avoid_waypoints_topic_, qos, std::bind(&WpntPublisher::onAvoidWpnts, this, _1));

    // 프레네 오돔: 기존 로직 유지 (child_frame_id에 가까운 인덱스가 정수로 들어오는 가정)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      frenet_odometry_topic_, qos, std::bind(&WpntPublisher::onOdom, this, _1));

    // 주행 상태: state_machine이 결정한 STATE(GLOBAL/AVOID/OVERTAKE)에 따라 OT 소스를 선택 (latched)
    auto qos_state = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = this->create_subscription<f110_msgs::msg::StateMachine>(
      state_topic_, qos_state, std::bind(&WpntPublisher::onState, this, _1));

    // 파라미터 동적 갱신
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WpntPublisher::onParamSet, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "wpnt_publisher started (waypoint_num=%d).", waypoint_num_);
  }

private:
  // 파라미터 콜백
  rcl_interfaces::msg::SetParametersResult
  onParamSet(const std::vector<rclcpp::Parameter>& params)
  {
    for (const auto& p : params) {
      if (p.get_name() == "waypoint_num") {
        int v = p.as_int();
        if (v <= 0) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = false;
          res.reason = "waypoint_num must be > 0";
          return res;
        }
        waypoint_num_ = v;
        RCLCPP_INFO(this->get_logger(), "Updated waypoint_num=%d", waypoint_num_);
      }
    }
    rcl_interfaces::msg::SetParametersResult ok;
    ok.successful = true;
    return ok;
  }

  void onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    global_wpnts_ = *msg;              // header와 wpnts 모두 보존
    total_ = static_cast<int>(global_wpnts_.wpnts.size());
    has_global_ = (total_ > 0);
    if (!has_global_) {
      RCLCPP_WARN(this->get_logger(), "Received empty /global_waypoints.");
    }
  }

 // 클래스 멤버 변수에 추가

void onOTWpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
{
    auto current_time = this->now();
    
    // 새로운 경로가 들어왔을 때
    if (!msg->wpnts.empty()) {
        // 이전 경로가 없거나, 유지 시간이 지났을 때만 업데이트
        if (!has_ot_ || (current_time - last_ot_update_time_) >= ot_hold_duration_) {
            last_ot_ = *msg;
            has_ot_ = true;
            last_ot_update_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "OT 경로 업데이트됨");
        } else {
            RCLCPP_DEBUG(this->get_logger(), 
                "OT 경로 유지 중 (남은 시간: %.2f초)", 
                (ot_hold_duration_ - (current_time - last_ot_update_time_)).seconds());
        }
    } else {
        // 빈 메시지가 와도 유지 시간이 지나야 무효화
        if (has_ot_ && (current_time - last_ot_update_time_) >= ot_hold_duration_) {
            has_ot_ = false;
            RCLCPP_INFO(this->get_logger(), "OT 경로 만료됨");
        }
    }
}

  // 정적 회피 OT(local_planning): 추월 OT와 분리된 독립 상태로 관리 (별도 콜백)
  void onAvoidWpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
  {
    auto current_time = this->now();
    if (!msg->wpnts.empty()) {
      last_avoid_ = *msg;
      has_avoid_ = true;
      last_avoid_update_time_ = current_time;
    }
  }

  bool hasFreshAvoidPath() const
  {
    return has_avoid_ &&
      (this->now() - last_avoid_update_time_).seconds() <= avoid_path_ttl_sec_;
  }

  // state_machine이 발행한 주행 상태 저장 (onOdom의 소스 선택에 사용)
  void onState(const f110_msgs::msg::StateMachine::SharedPtr msg)
  {
    current_state_ = msg->state;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 소스 선택: state_machine이 발행한 STATE에 따라 결정
    //   STATE_OVERTAKE -> 추월 OT, STATE_AVOID -> 회피 OT, 그 외(STATE_GLOBAL) -> 글로벌
    if (current_state_ == f110_msgs::msg::StateMachine::STATE_OVERTAKE && has_ot_) {
      publishFromOT(last_ot_);
      return;
    }
    if (current_state_ == f110_msgs::msg::StateMachine::STATE_AVOID && hasFreshAvoidPath()) {
      publishFromOT(last_avoid_);
      return;
    }

    // 위 상태가 아니거나 해당 OT가 아직 없으면 글로벌에서 추출
    if (!has_global_ || total_ == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No global waypoints yet; skip.");
      return;
    }

    // child_frame_id -> closest index (정수 문자열 가정)
    auto idx_opt = parseIndex(msg->child_frame_id);
    if (!idx_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Invalid child_frame_id for index: '%s'",
                           msg->child_frame_id.c_str());
      return;
    }
    int closest_idx = idx_opt.value();

    if (closest_idx < 0 || closest_idx >= total_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Index out of range: %d (0..%d)", closest_idx, total_-1);
      return;
    }


    // “바로 다음 점”부터 waypoint_num개 추출 (원형 인덱싱)
    const int start = (closest_idx + 1) % total_;
    const int count = std::min(waypoint_num_, total_); // 전체보다 많이 요청하면 전체만
    f110_msgs::msg::WpntArray local;

    local.wpnts.reserve(static_cast<size_t>(count));
    for (int k = 0; k < count; ++k) {
      int gi = (start + k) % total_;
      local.wpnts.push_back(global_wpnts_.wpnts[gi]);
    }

    // header: 타임스탬프 갱신, frame은 글로벌과 동일(없으면 map)
    local.header.stamp = this->now();
    local.header.frame_id = global_wpnts_.header.frame_id.empty()
                            ? "map"
                            : global_wpnts_.header.frame_id;

    local_pub_->publish(local);
    nav_msgs::msg::Path local_path = buildPathFromWpnts(local);
    local_path_pub_->publish(local_path);
  }

  void publishFromOT(const f110_msgs::msg::OTWpntArray& ot)
  {
    // OT는 이미 (map 프레임, s/d/x/y/v 포함)로 들어온다고 가정
    // 그대로 WpntArray로 변환해 퍼블리시
    f110_msgs::msg::WpntArray local;
    local.header = ot.header;
    local.header.stamp = this->now(); // 퍼블리시 시각으로 갱신(선택)
    local.wpnts = ot.wpnts;
    local_pub_->publish(local);
    nav_msgs::msg::Path local_path = buildPathFromWpnts(local);
    local_path_pub_->publish(local_path);
  }

  nav_msgs::msg::Path buildPathFromWpnts(const f110_msgs::msg::WpntArray& src)
  {
    nav_msgs::msg::Path path;
    path.header = src.header;
    path.poses.reserve(src.wpnts.size());

    for (const auto& w : src.wpnts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = src.header;
      ps.pose.position.x = w.x_m;
      ps.pose.position.y = w.y_m;
      ps.pose.position.z = 0.0;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      path.poses.emplace_back(std::move(ps));
    }
    return path;
  }

  // 공백 허용, 순수 숫자만 허용 (예: "1234")
  std::optional<int> parseIndex(const std::string& s)
  {
    if (s.empty()) return std::nullopt;

    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j-1]))) --j;
    if (i >= j) return std::nullopt;
    std::string t = s.substr(i, j - i);

    for (char c : t) {
      if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
    }

    try {
      return std::stoi(t);
    } catch (...) {
      return std::nullopt;
    }
  }

private:
  // pubs/subs
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr local_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr ot_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::StateMachine>::SharedPtr state_sub_;
  rclcpp::Time last_ot_update_time_;
  rclcpp::Time last_avoid_update_time_;
  rclcpp::Duration ot_hold_duration_;
  // state
  bool has_global_{false};
  bool has_ot_{false};
  bool has_avoid_{false};
  uint8_t current_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
  int total_{0};
  int waypoint_num_{50};
  double avoid_path_ttl_sec_{0.75};
  std::string global_waypoints_topic_;
  std::string avoid_waypoints_topic_;
  std::string frenet_odometry_topic_;
  std::string state_topic_;
  std::string local_waypoints_topic_;
  std::string local_path_topic_;
  f110_msgs::msg::WpntArray global_wpnts_;
  f110_msgs::msg::OTWpntArray last_ot_;
  f110_msgs::msg::OTWpntArray last_avoid_;

  // param callback handle
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WpntPublisher>());
  rclcpp::shutdown();
  return 0;
}
