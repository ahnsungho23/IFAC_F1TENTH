#include <algorithm>
#include <chrono>
#include <cctype>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/state_machine.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace state_machine
{

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode()
  : Node("state_machine_node")
  {
    declare_parameter<std::string>("state_topic", "/state");
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    declare_parameter<std::string>("avoidance_wpnts_topic", "/planner/avoidance/otwpnts");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("default_state", "global");
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("avoidance_stale_timeout_sec", 0.5);
    declare_parameter<double>("global_stale_timeout_sec", 2.0);
    declare_parameter<double>("frenet_stale_timeout_sec", 0.5);
    declare_parameter<bool>("use_scan_subscription", false);

    state_topic_ = get_parameter("state_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    default_state_name_ = get_parameter("default_state").as_string();
    avoidance_stale_timeout_sec_ = get_nonnegative_parameter("avoidance_stale_timeout_sec");
    global_stale_timeout_sec_ = get_nonnegative_parameter("global_stale_timeout_sec");
    frenet_stale_timeout_sec_ = get_nonnegative_parameter("frenet_stale_timeout_sec");

    double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive. Falling back to 10.0 Hz.");
      publish_rate_hz = 10.0;
    }

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = create_publisher<f110_msgs::msg::StateMachine>(state_topic_, state_qos);

    const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto global_qos = rclcpp::QoS(1).reliable().transient_local();

    frenet_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("frenet_odom_topic").as_string(),
      volatile_qos,
      std::bind(&StateMachineNode::on_frenet_odom, this, std::placeholders::_1));

    global_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      get_parameter("global_waypoints_topic").as_string(),
      global_qos,
      std::bind(&StateMachineNode::on_global_waypoints, this, std::placeholders::_1));

    avoidance_sub_ = create_subscription<f110_msgs::msg::OTWpntArray>(
      get_parameter("avoidance_wpnts_topic").as_string(),
      volatile_qos,
      std::bind(&StateMachineNode::on_avoidance_wpnts, this, std::placeholders::_1));

    if (get_parameter("use_scan_subscription").as_bool()) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("scan_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&StateMachineNode::on_scan, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(period, std::bind(&StateMachineNode::publish_state, this));

    RCLCPP_INFO(
      get_logger(),
      "state_machine_node started. Publishing %s with default_state='%s'.",
      state_topic_.c_str(),
      default_state_name_.c_str());
  }

private:
  double get_nonnegative_parameter(const std::string & name)
  {
    const double value = get_parameter(name).as_double();
    if (value < 0.0) {
      RCLCPP_WARN(get_logger(), "%s must be >= 0.0. Falling back to 0.0.", name.c_str());
      return 0.0;
    }
    return value;
  }

  static std::string normalize_state_name(std::string value)
  {
    value.erase(
      std::remove_if(value.begin(), value.end(), [](unsigned char c) {return std::isspace(c);}),
      value.end());
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  }

  std::optional<uint8_t> parse_state(const std::string & state_name) const
  {
    const auto normalized = normalize_state_name(state_name);
    if (normalized == "global" || normalized == "state_global" || normalized == "0") {
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }
    if (normalized == "avoid" || normalized == "avoidance" || normalized == "state_avoid" ||
      normalized == "1")
    {
      return f110_msgs::msg::StateMachine::STATE_AVOID;
    }
    if (normalized == "overtake" || normalized == "state_overtake" || normalized == "2") {
      return f110_msgs::msg::StateMachine::STATE_OVERTAKE;
    }
    return std::nullopt;
  }

  bool is_fresh(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (timeout_sec <= 0.0) {
      return true;
    }
    return (now() - stamp).seconds() <= timeout_sec;
  }

  bool has_fresh_global() const
  {
    return has_global_ && is_fresh(last_global_time_, global_stale_timeout_sec_);
  }

  bool has_fresh_frenet() const
  {
    return has_frenet_ && is_fresh(last_frenet_time_, frenet_stale_timeout_sec_);
  }

  bool has_fresh_avoidance() const
  {
    return has_avoidance_ && is_fresh(last_avoidance_time_, avoidance_stale_timeout_sec_);
  }

  void on_frenet_odom(const nav_msgs::msg::Odometry::SharedPtr)
  {
    has_frenet_ = true;
    last_frenet_time_ = now();
  }

  void on_global_waypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    has_global_ = !msg->wpnts.empty();
    last_global_time_ = now();
    if (!has_global_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received empty global waypoints. Conservative state output remains available.");
    }
  }

  void on_avoidance_wpnts(const f110_msgs::msg::OTWpntArray::SharedPtr msg)
  {
    has_avoidance_ = !msg->wpnts.empty();
    last_avoidance_time_ = now();
    if (!has_avoidance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received empty avoidance/overtake waypoints. Non-global states will fall back.");
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr)
  {
    has_scan_ = true;
    last_scan_time_ = now();
    // TODO: Add obstacle evidence extraction when the behavior policy is defined.
  }

  uint8_t resolve_requested_state()
  {
    const auto parsed_state = parse_state(default_state_name_);
    if (!parsed_state.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Invalid default_state '%s'. Falling back to STATE_GLOBAL.",
        default_state_name_.c_str());
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    const uint8_t requested_state = parsed_state.value();
    if (requested_state == f110_msgs::msg::StateMachine::STATE_GLOBAL) {
      return requested_state;
    }

    if (!has_fresh_avoidance()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Requested non-global state has no fresh avoidance/overtake path. Falling back to STATE_GLOBAL.");
      return f110_msgs::msg::StateMachine::STATE_GLOBAL;
    }

    return requested_state;
  }

  void publish_state()
  {
    const bool global_ready = has_fresh_global();
    const bool frenet_ready = has_fresh_frenet();
    if (!global_ready || !frenet_ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "State publisher missing fresh inputs: global=%s frenet=%s. Publishing conservative state.",
        global_ready ? "true" : "false",
        frenet_ready ? "true" : "false");
    }

    const uint8_t state = resolve_requested_state();
    f110_msgs::msg::StateMachine msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.state = state;
    state_pub_->publish(msg);

    if (!last_published_state_.has_value() || last_published_state_.value() != state) {
      RCLCPP_INFO(get_logger(), "Published state changed to %u.", state);
      last_published_state_ = state;
    }
  }

  std::string state_topic_;
  std::string frame_id_;
  std::string default_state_name_;
  double avoidance_stale_timeout_sec_{0.5};
  double global_stale_timeout_sec_{2.0};
  double frenet_stale_timeout_sec_{0.5};

  bool has_frenet_{false};
  bool has_global_{false};
  bool has_avoidance_{false};
  bool has_scan_{false};
  rclcpp::Time last_frenet_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoidance_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  std::optional<uint8_t> last_published_state_;

  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_sub_;
  rclcpp::Subscription<f110_msgs::msg::OTWpntArray>::SharedPtr avoidance_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
