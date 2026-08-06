#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "readwrite_global_waypoints.hpp"

namespace global_planning
{

class GlobalRepublisherNode : public rclcpp::Node
{
public:
  GlobalRepublisherNode()
  : Node("global_republisher_node")
  {
    declare_parameter("publish_markers", true); /////커(MarkerArray)를 퍼블리시할지 결정합니다. true면 /global_waypoints/markers, /trackbounds/markers 같은 시각화 토픽을 보냄.
    declare_parameter("publish_shortest_path", true); ///최단경로 트래젝토리를 퍼블리시할지 결정합니다. 
    declare_parameter("publish_centerline", false); ////////publish_centerline센터라인 웨이포인트를 퍼블리시할지 결정. true면 /centerline_waypoints(및 마커 옵션 시 /centerline_waypoints/markers) 보냄
    declare_parameter("publish_lattice", false); ///lattice 시각화 토픽을 쓸지 결정합니다. true면 /lattice_viz 퍼블리셔를 활성화합니다
    // Data source: global_waypoints.json is read from <output_base_dir>/<map_name>/.
    // The map name (not a full path) is what the YAML carries, so switching maps only
    // changes `map_name`. `map_path` is an optional explicit directory override.
    declare_parameter("output_base_dir", "offline_trajectory_generator/output");
    declare_parameter("map_name", "");
    declare_parameter("map_path", "");
    declare_parameter("publish_period_sec", 2.0);
    // Marker generation (RViz visualization). The offline generator writes empty
    // marker arrays, so this node builds them from the waypoints instead.
    declare_parameter("marker_frame_id", "map");
    declare_parameter("traj_marker_width", 0.10);
    declare_parameter("trackbound_marker_width", 0.05);

    publish_markers_ = get_parameter("publish_markers").as_bool();
    publish_shortest_path_ = get_parameter("publish_shortest_path").as_bool();
    publish_centerline_ = get_parameter("publish_centerline").as_bool();
    publish_lattice_ = get_parameter("publish_lattice").as_bool();
    marker_frame_id_ = get_parameter("marker_frame_id").as_string();
    traj_marker_width_ = get_parameter("traj_marker_width").as_double();
    trackbound_marker_width_ = get_parameter("trackbound_marker_width").as_double();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    glb_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", latched_qos);
    map_info_pub_ = create_publisher<std_msgs::msg::String>("/map_infos", 10);
    est_lap_time_pub_ = create_publisher<std_msgs::msg::Float32>("/estimated_lap_time", 10);

    if (publish_markers_) {
      glb_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/global_waypoints/markers", 10);
      vis_track_bnds_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/trackbounds/markers", 10);
    }

    if (publish_shortest_path_) {
      glb_sp_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints/shortest_path", 10);
    }

    if (publish_centerline_) {
      centerline_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/centerline_waypoints", 10);
      if (publish_markers_) {
        centerline_markers_pub_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/centerline_waypoints/markers", 10);
      }
    }

    if (publish_lattice_) {
      lattice_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_viz", 10);
    }

    // Resolve the directory holding global_waypoints.json.
    // Priority: explicit map_path override, else <output_base_dir>/<map_name>.
    map_dir_ = get_parameter("map_path").as_string();
    if (map_dir_.empty()) {
      const auto base = get_parameter("output_base_dir").as_string();
      const auto name = get_parameter("map_name").as_string();
      if (!name.empty()) {
        map_dir_ = base.empty() ? name : base + "/" + name;
      }
    }
    if (map_dir_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "no map source: set 'map_name' (with 'output_base_dir') or 'map_path'");
    } else {
      std::string error_msg;
      if (!read_global_waypoints(map_dir_, bundle_, error_msg)) {
        RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
      } else {
        has_bundle_ = true;
        generateMarkers();
        RCLCPP_INFO(get_logger(), "loaded global waypoints from %s", map_dir_.c_str());
      }
    }

    // Atomic in-process swap for the map_creator pipeline: re-read the configured map
    // directory and republish immediately. The caller (map_creator) gates the timing
    // (STATE_GLOBAL, no local commitment, lap boundary); this node only validates data.
    reload_srv_ = create_service<std_srvs::srv::Trigger>(
      "/global_planning/reload_waypoints",
      std::bind(
        &GlobalRepublisherNode::reloadWaypoints, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = get_parameter("publish_period_sec").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&GlobalRepublisherNode::publish_all, this));
  }

private:
  // Fill any marker array that the source JSON left empty by building it from the
  // corresponding waypoint list. Non-empty arrays (if the JSON ever provides them)
  // are left untouched.
  void generateMarkers()
  {
    if (bundle_.global_traj_markers_iqp.markers.empty()) {
      bundle_.global_traj_markers_iqp =
        buildTrajectoryMarkers(bundle_.global_traj_wpnts_iqp, "global_traj_iqp");
    }
    if (bundle_.trackbounds_markers.markers.empty()) {
      bundle_.trackbounds_markers = buildTrackboundMarkers(bundle_.global_traj_wpnts_iqp);
    }
    if (publish_centerline_ && bundle_.centerline_markers.markers.empty()) {
      bundle_.centerline_markers =
        buildPlainLineMarkers(bundle_.centerline_waypoints, "centerline", 0.5f, 0.5f, 0.5f);
    }
  }

  visualization_msgs::msg::Marker makeLineStrip(
    const std::string & ns, const int id, const double width,
    const float r, const float g, const float b, const float a) const
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = marker_frame_id_;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = width;
    m.pose.orientation.w = 1.0;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  // Racing line as a speed-colored LINE_STRIP (green = slow, red = fast).
  visualization_msgs::msg::MarkerArray buildTrajectoryMarkers(
    const f110_msgs::msg::WpntArray & wpnts, const std::string & ns) const
  {
    visualization_msgs::msg::MarkerArray arr;
    if (wpnts.wpnts.empty()) {
      return arr;
    }

    double vmin = std::numeric_limits<double>::max();
    double vmax = std::numeric_limits<double>::lowest();
    for (const auto & w : wpnts.wpnts) {
      vmin = std::min(vmin, w.vx_mps);
      vmax = std::max(vmax, w.vx_mps);
    }
    const double span = (vmax - vmin) > 1e-6 ? (vmax - vmin) : 1.0;

    auto m = makeLineStrip(ns, 0, traj_marker_width_, 1.0f, 1.0f, 1.0f, 1.0f);
    m.points.reserve(wpnts.wpnts.size() + 1);
    m.colors.reserve(wpnts.wpnts.size() + 1);
    for (const auto & w : wpnts.wpnts) {
      geometry_msgs::msg::Point p;
      p.x = w.x_m;
      p.y = w.y_m;
      p.z = 0.0;
      m.points.push_back(p);

      const double t = std::clamp((w.vx_mps - vmin) / span, 0.0, 1.0);
      std_msgs::msg::ColorRGBA c;
      c.r = static_cast<float>(t);
      c.g = static_cast<float>(1.0 - t);
      c.b = 0.0f;
      c.a = 1.0f;
      m.colors.push_back(c);
    }
    // Close the loop back to the first waypoint.
    if (wpnts.wpnts.size() > 2) {
      m.points.push_back(m.points.front());
      m.colors.push_back(m.colors.front());
    }

    arr.markers.push_back(m);
    return arr;
  }

  // Left and right track boundaries derived from d_left/d_right offset along the
  // path normal (psi +/- 90 deg).
  visualization_msgs::msg::MarkerArray buildTrackboundMarkers(
    const f110_msgs::msg::WpntArray & wpnts) const
  {
    visualization_msgs::msg::MarkerArray arr;
    if (wpnts.wpnts.empty()) {
      return arr;
    }

    auto left = makeLineStrip("trackbound_left", 0, trackbound_marker_width_, 0.2f, 0.6f, 1.0f, 1.0f);
    auto right = makeLineStrip("trackbound_right", 1, trackbound_marker_width_, 0.2f, 0.6f, 1.0f, 1.0f);
    left.points.reserve(wpnts.wpnts.size() + 1);
    right.points.reserve(wpnts.wpnts.size() + 1);
    for (const auto & w : wpnts.wpnts) {
      const double s = std::sin(w.psi_rad);
      const double c = std::cos(w.psi_rad);
      geometry_msgs::msg::Point lp;
      lp.x = w.x_m - w.d_left * s;   // left normal = psi + 90 deg -> (-sin, cos)
      lp.y = w.y_m + w.d_left * c;
      lp.z = 0.0;
      geometry_msgs::msg::Point rp;
      rp.x = w.x_m + w.d_right * s;  // right normal = psi - 90 deg -> (sin, -cos)
      rp.y = w.y_m - w.d_right * c;
      rp.z = 0.0;
      left.points.push_back(lp);
      right.points.push_back(rp);
    }
    if (wpnts.wpnts.size() > 2) {
      left.points.push_back(left.points.front());
      right.points.push_back(right.points.front());
    }

    arr.markers.push_back(left);
    arr.markers.push_back(right);
    return arr;
  }

  // Single flat-colored LINE_STRIP (used for the centerline).
  visualization_msgs::msg::MarkerArray buildPlainLineMarkers(
    const f110_msgs::msg::WpntArray & wpnts, const std::string & ns,
    const float r, const float g, const float b) const
  {
    visualization_msgs::msg::MarkerArray arr;
    if (wpnts.wpnts.empty()) {
      return arr;
    }
    auto m = makeLineStrip(ns, 0, trackbound_marker_width_, r, g, b, 1.0f);
    m.points.reserve(wpnts.wpnts.size() + 1);
    for (const auto & w : wpnts.wpnts) {
      geometry_msgs::msg::Point p;
      p.x = w.x_m;
      p.y = w.y_m;
      p.z = 0.0;
      m.points.push_back(p);
    }
    if (wpnts.wpnts.size() > 2) {
      m.points.push_back(m.points.front());
    }
    arr.markers.push_back(m);
    return arr;
  }

  // Data-level validation before a reloaded bundle may replace the live one.
  // Mirrors the strictest downstream requirements (state_machine: strictly
  // increasing finite s_m; local_planning: >=4 waypoints).
  static bool validBundle(const GlobalWaypointBundle & bundle, std::string & why)
  {
    const auto & wpnts = bundle.global_traj_wpnts_iqp.wpnts;
    if (wpnts.size() < 4U) {
      why = "reloaded raceline has fewer than 4 waypoints";
      return false;
    }
    double prev_s = -std::numeric_limits<double>::infinity();
    for (const auto & w : wpnts) {
      if (!std::isfinite(w.s_m) || !std::isfinite(w.x_m) || !std::isfinite(w.y_m) ||
        !std::isfinite(w.psi_rad) || !std::isfinite(w.kappa_radpm) || !std::isfinite(w.vx_mps))
      {
        why = "reloaded raceline contains non-finite fields";
        return false;
      }
      if (w.s_m <= prev_s) {
        why = "reloaded raceline s_m is not strictly increasing";
        return false;
      }
      prev_s = w.s_m;
    }
    return true;
  }

  void reloadWaypoints(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (map_dir_.empty()) {
      response->success = false;
      response->message = "no map directory configured";
      return;
    }
    GlobalWaypointBundle fresh;
    std::string error_msg;
    if (!read_global_waypoints(map_dir_, fresh, error_msg)) {
      response->success = false;
      response->message = error_msg;
      RCLCPP_WARN(get_logger(), "reload rejected: %s", error_msg.c_str());
      return;
    }
    if (!validBundle(fresh, error_msg)) {
      response->success = false;
      response->message = error_msg;
      RCLCPP_WARN(get_logger(), "reload rejected: %s", error_msg.c_str());
      return;
    }
    bundle_ = std::move(fresh);
    has_bundle_ = true;
    generateMarkers();
    publish_all();  // swap immediately; do not wait for the 2 s republish timer
    response->success = true;
    response->message =
      "reloaded " + std::to_string(bundle_.global_traj_wpnts_iqp.wpnts.size()) +
      " waypoints from " + map_dir_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void publish_all()
  {
    if (!has_bundle_) {
      return;
    }

    glb_wpnts_pub_->publish(bundle_.global_traj_wpnts_iqp);
    if (publish_markers_ && glb_markers_pub_) {
      glb_markers_pub_->publish(bundle_.global_traj_markers_iqp);
      vis_track_bnds_pub_->publish(bundle_.trackbounds_markers);
    }

    if (publish_shortest_path_ && glb_sp_wpnts_pub_) {
      glb_sp_wpnts_pub_->publish(bundle_.global_traj_wpnts_sp);
    }

    if (publish_centerline_ && centerline_wpnts_pub_) {
      centerline_wpnts_pub_->publish(bundle_.centerline_waypoints);
      if (publish_markers_ && centerline_markers_pub_) {
        centerline_markers_pub_->publish(bundle_.centerline_markers);
      }
    }

    map_info_pub_->publish(bundle_.map_info_str);
    est_lap_time_pub_->publish(bundle_.est_lap_time);
  }

  bool publish_markers_{true};
  bool publish_shortest_path_{true};
  bool publish_centerline_{false};
  bool publish_lattice_{false};

  std::string marker_frame_id_{"map"};
  double traj_marker_width_{0.10};
  double trackbound_marker_width_{0.05};

  bool has_bundle_{false};
  std::string map_dir_;
  GlobalWaypointBundle bundle_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;

  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr glb_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_track_bnds_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_sp_wpnts_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr est_lap_time_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace global_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_planning::GlobalRepublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
