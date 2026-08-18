#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "f110_msgs/msg/wpnt_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "global_planning/clcs_frenet_converter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace global_planning
{
namespace
{

enum class ProjectionFailurePolicy
{
  kDropMessage,
  kPublishLastValid,
  kPublishNan
};

ProjectionFailurePolicy parseProjectionFailurePolicy(const std::string & value)
{
  if (value == "publish_last_valid") {
    return ProjectionFailurePolicy::kPublishLastValid;
  }
  if (value == "publish_nan") {
    return ProjectionFailurePolicy::kPublishNan;
  }
  return ProjectionFailurePolicy::kDropMessage;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

std::vector<ReferenceWaypoint> toReferenceWaypoints(const f110_msgs::msg::WpntArray & msg)
{
  std::vector<ReferenceWaypoint> waypoints;
  waypoints.reserve(msg.wpnts.size());
  for (const auto & waypoint : msg.wpnts) {
    waypoints.push_back({waypoint.x_m, waypoint.y_m, waypoint.s_m});
  }
  return waypoints;
}

void clearCovariance(nav_msgs::msg::Odometry & odom)
{
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
}

}  // namespace

class FrenetOdomNode : public rclcpp::Node
{
public:
  FrenetOdomNode()
  : Node("frenet_odom_node")
  {
    declareParameters();
    loadParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&FrenetOdomNode::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      waypoint_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FrenetOdomNode::waypointsCallback, this, std::placeholders::_1));

    frenet_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      frenet_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    if (continuity_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Monotonic s-window tracking enabled: forward=%.2f m back=%.2f m "
        "seed=%.2f m gate=%.2f m reacquire_after=%d misses",
        config_.forward_window, config_.backward_tolerance,
        config_.initial_seed_window, config_.tracked_max_projection_distance,
        config_.reacquire_after_misses);
    }

    RCLCPP_INFO(
      get_logger(),
      "CLCS frenet odom started: odom=%s waypoints=%s output=%s closed_loop=%s "
      "child_frame_id=closest_segment_index velocity_frame=%s failure_policy=%s",
      odom_topic_.c_str(), waypoint_topic_.c_str(), frenet_odom_topic_.c_str(),
      config_.closed_loop ? "true" : "false",
      toString(config_.velocity_frame).c_str(), projection_failure_policy_name_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
    declare_parameter<std::string>("waypoint_topic", "/global_waypoints");
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
    declare_parameter<std::string>("frenet_frame_id", "frenet");
    declare_parameter<std::string>("projection_failure_policy", "drop_message");
    declare_parameter<std::string>("velocity_frame", "body");
    declare_parameter<std::string>("covariance_mode", "zero");

    declare_parameter<bool>("closed_loop", true);
    declare_parameter<bool>("publish_heading_error", true);
    declare_parameter<bool>("publish_frenet_velocity", true);
    declare_parameter<bool>("compatibility_mode", false);
    declare_parameter<bool>("use_path_preprocessing", true);

    declare_parameter<double>("path_change_tolerance", 1.0e-4);
    declare_parameter<double>("duplicate_point_tolerance", 1.0e-3);
    declare_parameter<double>("tangent_epsilon", 0.05);
    declare_parameter<double>("max_projection_distance", 20.0);
    declare_parameter<double>("projection_domain_limit", 20.0);
    declare_parameter<double>("projection_domain_epsilon", 0.1);
    declare_parameter<double>("projection_domain_eps2", 0.0);
    declare_parameter<double>("min_path_length", 0.5);
    declare_parameter<double>("large_gap_factor", 5.0);
    declare_parameter<int>("projection_domain_method", 1);

    // Monotonic s-window tracking (closed-loop wrap).
    declare_parameter<bool>("continuity_enabled", true);
    declare_parameter<double>("forward_window", 1.0);
    declare_parameter<double>("backward_tolerance", 1.0);
    declare_parameter<double>("initial_seed_window", 0.0);
    declare_parameter<double>("tracked_max_projection_distance", 1.5);
    declare_parameter<int>("reacquire_after_misses", 15);

    // Frenet d / vd Output Smoothing Filter
    declare_parameter<bool>("enable_smoothing", true);
    declare_parameter<double>("d_alpha", 0.5);
    // ⚠️ 2026-08-16: 0.3(d_alpha 0.5보다 느림) → d_alpha와 같은 0.5로. v_d는 odom body twist를
    // 기준선 법선에 투영한 기구학적 값이라(수치 미분이 아님) 필터링 이득이 원래 없다 — 그런데
    // d보다 느린 시상수를 주면 d와 v_d가 서로 다른 지연으로 필터링돼 "d의 미분 ≠ v_d"인
    // 운동학적으로 비일관된 신호 쌍이 된다(현재는 소비자가 없어 무해하지만, 훗날 예측
    // d+v_d·Δt 같은 용도로 v_d를 쓰기 시작하면 그대로 문제가 된다). d_alpha와 맞춘다.
    declare_parameter<double>("vd_alpha", 0.5);
  }

  void loadParameters()
  {
    odom_topic_ = get_parameter("odom_topic").as_string();
    waypoint_topic_ = get_parameter("waypoint_topic").as_string();
    frenet_odom_topic_ = get_parameter("frenet_odom_topic").as_string();
    frenet_frame_id_ = get_parameter("frenet_frame_id").as_string();
    projection_failure_policy_name_ = get_parameter("projection_failure_policy").as_string();
    projection_failure_policy_ =
      parseProjectionFailurePolicy(projection_failure_policy_name_);
    covariance_mode_ = get_parameter("covariance_mode").as_string();

    config_.closed_loop = get_parameter("closed_loop").as_bool();
    publish_heading_error_ = get_parameter("publish_heading_error").as_bool();
    config_.publish_frenet_velocity = get_parameter("publish_frenet_velocity").as_bool();
    compatibility_mode_ = get_parameter("compatibility_mode").as_bool();
    use_path_preprocessing_ = get_parameter("use_path_preprocessing").as_bool();

    config_.path_change_tolerance = get_parameter("path_change_tolerance").as_double();
    config_.duplicate_point_tolerance =
      get_parameter("duplicate_point_tolerance").as_double();
    config_.tangent_epsilon = get_parameter("tangent_epsilon").as_double();
    config_.max_projection_distance = get_parameter("max_projection_distance").as_double();
    config_.projection_domain_limit = get_parameter("projection_domain_limit").as_double();
    config_.projection_domain_epsilon = get_parameter("projection_domain_epsilon").as_double();
    config_.projection_domain_eps2 = get_parameter("projection_domain_eps2").as_double();
    config_.min_path_length = get_parameter("min_path_length").as_double();
    config_.large_gap_factor = get_parameter("large_gap_factor").as_double();
    config_.projection_domain_method = get_parameter("projection_domain_method").as_int();
    config_.velocity_frame = parseVelocityFrame(get_parameter("velocity_frame").as_string());

    continuity_enabled_ = get_parameter("continuity_enabled").as_bool();
    config_.forward_window = get_parameter("forward_window").as_double();
    config_.backward_tolerance = get_parameter("backward_tolerance").as_double();
    config_.initial_seed_window = get_parameter("initial_seed_window").as_double();
    config_.tracked_max_projection_distance =
      get_parameter("tracked_max_projection_distance").as_double();
    config_.reacquire_after_misses =
      static_cast<int>(get_parameter("reacquire_after_misses").as_int());

    enable_smoothing_ = get_parameter("enable_smoothing").as_bool();
    d_alpha_ = std::clamp(get_parameter("d_alpha").as_double(), 0.05, 1.0);
    vd_alpha_ = std::clamp(get_parameter("vd_alpha").as_double(), 0.05, 1.0);

    if (!std::isfinite(config_.forward_window) || config_.forward_window <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "forward_window must be > 0 (got %.3f); resetting to 1.0 m.",
        config_.forward_window);
      config_.forward_window = 1.0;
    }
    if (!std::isfinite(config_.backward_tolerance) || config_.backward_tolerance < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "backward_tolerance must be >= 0 (got %.3f); resetting to 1.0 m.",
        config_.backward_tolerance);
      config_.backward_tolerance = 1.0;
    }
    if (config_.reacquire_after_misses < 0) {
      RCLCPP_WARN(
        get_logger(),
        "reacquire_after_misses must be >= 0 (got %d); resetting to 15.",
        config_.reacquire_after_misses);
      config_.reacquire_after_misses = 15;
    }

    if (!use_path_preprocessing_) {
      RCLCPP_WARN(
        get_logger(),
        "use_path_preprocessing=false requested, but safety filtering of invalid and duplicate "
        "points remains enabled because CLCS rejects degenerate paths.");
    }
  }

  void waypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    const auto raw_waypoints = toReferenceWaypoints(*msg);
    if (raw_waypoints.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received %zu waypoint(s); CLCS requires at least 3 valid reference points.",
        raw_waypoints.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      if (!ClcsFrenetConverter::pathChanged(
          last_raw_waypoints_, raw_waypoints, config_.path_change_tolerance))
      {
        return;
      }
    }

    const std::uint64_t next_version = path_version_ + 1;

    ClcsFrenetConverter::Ptr new_converter;
    try {
      new_converter = ClcsFrenetConverter::create(raw_waypoints, config_, next_version);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build CommonRoad-CLCS reference path. Keeping previous valid CLCS if any: %s",
        e.what());
      return;
    }

    const auto & stats = new_converter->stats();
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      converter_ = new_converter;
      last_raw_waypoints_ = raw_waypoints;
      path_version_ = stats.path_version;
      // Progress (s_prev) belongs to the old path; re-acquire on the new one.
      continuity_state_ = ClcsContinuityState{};
      // ⚠️ 2026-08-16: EMA도 함께 리셋해야 한다. 안 하면 새 라인 기준 d가 옛 라인 기준
      // smoothed_d_/smoothed_vd_와 섞여 1~4 사이클(25~100ms) 동안 혼합값이 나온다 — 라인은
      // 자주 재생성되므로(같은 날 하루에도 여러 번) 이 창이 실제로 자주 열린다.
      has_smoothed_state_ = false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Built CLCS path version=%lu input=%zu reference=%zu track_length=%.3f m "
      "build_time=%.3f ms removed_duplicates=%zu invalid=%zu",
      static_cast<unsigned long>(stats.path_version), stats.input_waypoint_count,
      stats.reference_point_count, stats.track_length, stats.build_time_ms,
      stats.removed_duplicate_count, stats.invalid_point_count);

    if (stats.large_gap_count > 0) {
      RCLCPP_WARN(
        get_logger(), "Reference path has %zu unusually large waypoint gap(s).",
        stats.large_gap_count);
    }
    if (stats.self_intersection_count > 0) {
      RCLCPP_WARN(
        get_logger(), "Reference path has %zu possible self-intersection(s).",
        stats.self_intersection_count);
    }
    if (stats.waypoint_s_max_error > 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "Waypoint s_m differs from CLCS geometric arc length. max_error=%.3f m. "
        "Published s uses CLCS geometric arc length.",
        stats.waypoint_s_max_error);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    ClcsFrenetConverter::ConstPtr converter;
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      converter = converter_;
    }

    if (!converter) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for a valid CLCS reference path from %s.", waypoint_topic_.c_str());
      return;
    }

    ClcsConversionInput input;
    input.x = odom->pose.pose.position.x;
    input.y = odom->pose.pose.position.y;
    input.yaw = yawFromQuaternion(odom->pose.pose.orientation);
    input.linear_x = odom->twist.twist.linear.x;
    input.linear_y = odom->twist.twist.linear.y;
    input.yaw_rate = odom->twist.twist.angular.z;

    const auto conversion = continuity_enabled_
      ? converter->convertTracked(input, continuity_state_)
      : converter->convert(input);
    if (conversion.reacquired) {
      RCLCPP_WARN(
        get_logger(),
        "Monotonic s-window re-acquired via global search after %d consecutive "
        "misses (s=%.2f).",
        config_.reacquire_after_misses, conversion.s);
      // d가 전역 재탐색으로 큰 폭 점프하는데 EMA는 옛 값을 물고 있으면 안 되므로 리셋한다.
      has_smoothed_state_ = false;
    }
    if (!conversion.valid) {
      handleProjectionFailure(*odom, conversion);
      return;
    }

    auto output = buildOutputOdometry(*odom, conversion);
    last_valid_odom_ = output;
    has_last_valid_odom_ = true;
    frenet_pub_->publish(output);
  }

  nav_msgs::msg::Odometry buildOutputOdometry(
    const nav_msgs::msg::Odometry & input,
    const ClcsConversionResult & conversion)
  {
    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = frenet_frame_id_;
    output.child_frame_id = std::to_string(conversion.segment_index);
    output.pose.pose.position.x = conversion.s;

    double filtered_d = conversion.d;
    double filtered_vd = conversion.v_d;

    if (enable_smoothing_) {
      if (!has_smoothed_state_) {
        smoothed_d_ = conversion.d;
        smoothed_vd_ = conversion.v_d;
        has_smoothed_state_ = true;
      } else {
        smoothed_d_ = d_alpha_ * conversion.d + (1.0 - d_alpha_) * smoothed_d_;
        smoothed_vd_ = vd_alpha_ * conversion.v_d + (1.0 - vd_alpha_) * smoothed_vd_;
      }
      filtered_d = smoothed_d_;
      filtered_vd = smoothed_vd_;
    }

    output.pose.pose.position.y = filtered_d;
    output.pose.pose.position.z = 0.0;

    if (!compatibility_mode_ && publish_heading_error_) {
      output.pose.pose.orientation = quaternionFromYaw(conversion.heading_error);
    }

    if (!compatibility_mode_ && config_.publish_frenet_velocity) {
      output.twist.twist.linear.x = conversion.v_s;
      output.twist.twist.linear.y = filtered_vd;
      output.twist.twist.angular.z = conversion.yaw_rate;
    }

    if (!compatibility_mode_ && covariance_mode_ != "preserve") {
      clearCovariance(output);
    }
    return output;
  }

  void handleProjectionFailure(
    const nav_msgs::msg::Odometry & input,
    const ClcsConversionResult & conversion)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CLCS projection failed: %s", conversion.error_message.c_str());

    if (projection_failure_policy_ == ProjectionFailurePolicy::kDropMessage) {
      return;
    }

    if (projection_failure_policy_ == ProjectionFailurePolicy::kPublishLastValid) {
      if (!has_last_valid_odom_) {
        return;
      }
      auto output = last_valid_odom_;
      output.header.stamp = input.header.stamp;
      frenet_pub_->publish(output);
      return;
    }

    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = frenet_frame_id_;
    output.child_frame_id = "-1";
    output.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    output.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
    output.pose.pose.position.z = 0.0;
    if (!compatibility_mode_ && covariance_mode_ != "preserve") {
      clearCovariance(output);
    }
    frenet_pub_->publish(output);
  }

  ClcsFrenetConfig config_;
  bool publish_heading_error_{true};
  bool compatibility_mode_{false};
  bool use_path_preprocessing_{true};
  bool continuity_enabled_{true};
  bool has_last_valid_odom_{false};
  bool enable_smoothing_{true};
  double d_alpha_{0.5};
  double vd_alpha_{0.3};
  double smoothed_d_{0.0};
  double smoothed_vd_{0.0};
  bool has_smoothed_state_{false};
  std::uint64_t path_version_{0};

  std::string odom_topic_;
  std::string waypoint_topic_;
  std::string frenet_odom_topic_;
  std::string frenet_frame_id_;
  std::string projection_failure_policy_name_;
  std::string covariance_mode_;
  ProjectionFailurePolicy projection_failure_policy_{ProjectionFailurePolicy::kDropMessage};

  std::mutex converter_mutex_;
  ClcsFrenetConverter::ConstPtr converter_;
  std::vector<ReferenceWaypoint> last_raw_waypoints_;
  ClcsContinuityState continuity_state_;
  nav_msgs::msg::Odometry last_valid_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frenet_pub_;
};

}  // namespace global_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planning::FrenetOdomNode>());
  rclcpp::shutdown();
  return 0;
}
