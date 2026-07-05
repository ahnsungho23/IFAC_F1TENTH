#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace global_planner
{

namespace
{

constexpr double kMinSegmentLengthSquared = 1.0e-12;
constexpr double kSMonotonicEpsilon = 1.0e-6;

double clamp01(const double value)
{
  if (value < 0.0) {
    return 0.0;
  }
  if (value > 1.0) {
    return 1.0;
  }
  return value;
}

double waypoint_distance(
  const f110_msgs::msg::Wpnt & a,
  const f110_msgs::msg::Wpnt & b)
{
  return std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
}

bool has_finite_xy(const f110_msgs::msg::Wpnt & w)
{
  return std::isfinite(w.x_m) && std::isfinite(w.y_m);
}

}  // namespace

struct ProjectionResult
{
  size_t segment_index{0};
  double s{0.0};
  double d{0.0};
  double distance_to_path{0.0};
};

class FrenetOdomNode : public rclcpp::Node
{
public:
  FrenetOdomNode()
  : Node("frenet_odom_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/global_waypoints");
    frenet_odom_topic_ =
      declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
    closed_loop_ = declare_parameter<bool>("closed_loop", true);
    frenet_frame_id_ = declare_parameter<std::string>("frenet_frame_id", "frenet");
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/car_state/frenet/debug");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&FrenetOdomNode::odom_cb, this, std::placeholders::_1));
    wpnt_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      waypoint_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FrenetOdomNode::waypoints_cb, this, std::placeholders::_1));
    out_pub_ = create_publisher<nav_msgs::msg::Odometry>(frenet_odom_topic_, 20);
    if (publish_debug_) {
      debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 20);
    }

    RCLCPP_INFO(
      get_logger(),
      "Frenet odom node started: odom_topic=%s waypoint_topic=%s output_topic=%s "
      "closed_loop=%s",
      odom_topic_.c_str(), waypoint_topic_.c_str(), frenet_odom_topic_.c_str(),
      closed_loop_ ? "true" : "false");
  }

private:
  void waypoints_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    wpnts_ = *msg;
    if (wpnts_.wpnts.size() < 2) {
      has_wpnts_ = false;
      path_length_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received %zu waypoint(s) on %s; at least 2 are required. Frenet odom is not "
        "published.",
        wpnts_.wpnts.size(), waypoint_topic_.c_str());
      return;
    }

    path_length_ = compute_path_length();
    has_wpnts_ = true;
    validate_waypoint_s();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Loaded %zu global waypoints. closed_loop=%s path_length=%.3f m",
      wpnts_.wpnts.size(), closed_loop_ ? "true" : "false", path_length_);
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    if (!has_wpnts_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for at least 2 waypoints on %s before publishing Frenet odom.",
        waypoint_topic_.c_str());
      return;
    }

    const double cx = odom->pose.pose.position.x;
    const double cy = odom->pose.pose.position.y;
    ProjectionResult projection;
    if (!project_to_path(cx, cy, projection)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No usable path segment found on %s; all segments may be zero-length or invalid.",
        waypoint_topic_.c_str());
      return;
    }

    nav_msgs::msg::Odometry out = *odom;
    out.header.frame_id = frenet_frame_id_;
    out.child_frame_id = std::to_string(projection.segment_index);
    out.pose.pose.position.x = projection.s;
    out.pose.pose.position.y = projection.d;
    out.pose.pose.position.z = 0.0;
    out_pub_->publish(out);
    publish_debug(projection);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "segment=%zu s=%.3f d=%.3f distance_to_path=%.3f path_length=%.3f",
      projection.segment_index, projection.s, projection.d, projection.distance_to_path,
      path_length_);
  }

  bool project_to_path(
    const double vehicle_x,
    const double vehicle_y,
    ProjectionResult & result) const
  {
    const auto & points = wpnts_.wpnts;
    if (points.size() < 2) {
      return false;
    }

    const size_t segment_count = closed_loop_ ? points.size() : points.size() - 1;
    double best_distance_squared = std::numeric_limits<double>::max();
    bool found_segment = false;

    for (size_t i = 0; i < segment_count; ++i) {
      const size_t next_index = (i + 1) % points.size();
      const auto & a = points[i];
      const auto & b = points[next_index];

      if (!has_finite_xy(a) || !has_finite_xy(b) || !std::isfinite(a.s_m)) {
        continue;
      }

      const double ab_x = b.x_m - a.x_m;
      const double ab_y = b.y_m - a.y_m;
      const double ab_len_squared = ab_x * ab_x + ab_y * ab_y;
      if (ab_len_squared <= kMinSegmentLengthSquared) {
        continue;
      }

      const double ap_x = vehicle_x - a.x_m;
      const double ap_y = vehicle_y - a.y_m;
      const double t = clamp01((ap_x * ab_x + ap_y * ab_y) / ab_len_squared);
      const double projection_x = a.x_m + t * ab_x;
      const double projection_y = a.y_m + t * ab_y;
      const double dx = vehicle_x - projection_x;
      const double dy = vehicle_y - projection_y;
      const double distance_squared = dx * dx + dy * dy;

      if (distance_squared < best_distance_squared) {
        const double segment_length = std::sqrt(ab_len_squared);
        const double raw_s = a.s_m + t * segment_length;
        const double cross = ab_x * (vehicle_y - projection_y) -
          ab_y * (vehicle_x - projection_x);
        const double distance_to_path = std::sqrt(distance_squared);

        best_distance_squared = distance_squared;
        found_segment = true;
        result.segment_index = i;
        result.s = normalize_s(raw_s);
        result.d = cross < 0.0 ? -distance_to_path : distance_to_path;
        result.distance_to_path = distance_to_path;
      }
    }

    return found_segment;
  }

  double normalize_s(const double s) const
  {
    if (!closed_loop_ || path_length_ <= 0.0) {
      return s;
    }

    double normalized = std::fmod(s, path_length_);
    if (normalized < 0.0) {
      normalized += path_length_;
    }
    return normalized;
  }

  double compute_path_length() const
  {
    const auto & points = wpnts_.wpnts;
    if (points.size() < 2) {
      return 0.0;
    }

    double accumulated_length = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      accumulated_length += waypoint_distance(points[i], points[i + 1]);
    }

    double wrap_length = 0.0;
    if (closed_loop_) {
      wrap_length = waypoint_distance(points.back(), points.front());
      accumulated_length += wrap_length;
    }

    const double last_s = points.back().s_m;
    if (std::isfinite(last_s) && last_s > 0.0) {
      return closed_loop_ ? last_s + wrap_length : last_s;
    }

    return accumulated_length;
  }

  void validate_waypoint_s()
  {
    const auto & points = wpnts_.wpnts;
    size_t non_increasing_count = 0;
    size_t length_mismatch_count = 0;
    double max_length_error = 0.0;

    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const double ds = points[i + 1].s_m - points[i].s_m;
      const double segment_length = waypoint_distance(points[i], points[i + 1]);

      if (!std::isfinite(ds) || ds <= kSMonotonicEpsilon) {
        ++non_increasing_count;
        continue;
      }

      if (segment_length > 0.0) {
        const double length_error = std::fabs(ds - segment_length);
        const double warning_threshold = std::max(0.1, 0.25 * segment_length);
        if (length_error > warning_threshold) {
          ++length_mismatch_count;
          max_length_error = std::max(max_length_error, length_error);
        }
      }
    }

    if (non_increasing_count > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "%zu waypoint s_m interval(s) are not strictly increasing. Frenet s uses each "
        "segment start s_m plus geometric segment interpolation.",
        non_increasing_count);
    }

    if (length_mismatch_count > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "%zu waypoint s_m interval(s) differ from XY segment length. max_error=%.3f m; "
        "calculated s still interpolates using geometric segment length.",
        length_mismatch_count, max_length_error);
    }
  }

  void publish_debug(const ProjectionResult & projection)
  {
    if (!debug_pub_) {
      return;
    }

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      static_cast<double>(projection.segment_index),
      projection.s,
      projection.d,
      projection.distance_to_path,
      path_length_};
    debug_pub_->publish(debug_msg);
  }

  bool has_wpnts_{false};
  bool closed_loop_{true};
  bool publish_debug_{true};
  double path_length_{0.0};
  std::string odom_topic_;
  std::string waypoint_topic_;
  std::string frenet_odom_topic_;
  std::string frenet_frame_id_;
  std::string debug_topic_;
  f110_msgs::msg::WpntArray wpnts_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wpnt_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr out_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::FrenetOdomNode>());
  rclcpp::shutdown();
  return 0;
}
