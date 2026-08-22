#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
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
    // Initial source: <output_base_dir>/<map_name>/global_waypoints.json, with map_path
    // as an optional override. A successful reload switches to reload_map_name without
    // modifying either source file or the YAML.
    declare_parameter("output_base_dir", "offline_trajectory_generator/output");
    declare_parameter("map_name", "");
    declare_parameter("map_path", "");
    declare_parameter("reload_map_name", "obstacle_map");
    // Lap-triggered one-way switch to a pre-generated bundle (e.g. the offline Forza
    // raceline). Empty lap_switch_map_name disables the feature, which is the default:
    // an unconfigured node keeps the existing map -> reload_map_name behavior exactly.
    declare_parameter("lap_switch_map_name", "");
    declare_parameter("lap_switch_count", 11);
    declare_parameter("lap_count_topic", "/lap_count");
    declare_parameter("publish_period_sec", 2.0);

    publish_markers_ = get_parameter("publish_markers").as_bool();
    publish_shortest_path_ = get_parameter("publish_shortest_path").as_bool();
    publish_centerline_ = get_parameter("publish_centerline").as_bool();
    publish_lattice_ = get_parameter("publish_lattice").as_bool();

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
    const auto output_base_dir = get_parameter("output_base_dir").as_string();
    map_dir_ = get_parameter("map_path").as_string();
    if (map_dir_.empty()) {
      const auto name = get_parameter("map_name").as_string();
      if (!name.empty()) {
        map_dir_ = output_base_dir.empty() ? name : output_base_dir + "/" + name;
      }
    }
    reload_map_name_ = get_parameter("reload_map_name").as_string();
    if (!reload_map_name_.empty()) {
      reload_map_dir_ =
        output_base_dir.empty() ? reload_map_name_ : output_base_dir + "/" + reload_map_name_;
    }
    lap_switch_map_name_ = get_parameter("lap_switch_map_name").as_string();
    if (!lap_switch_map_name_.empty()) {
      lap_switch_dir_ = output_base_dir.empty() ?
        lap_switch_map_name_ : output_base_dir + "/" + lap_switch_map_name_;
      lap_switch_count_ = static_cast<int>(get_parameter("lap_switch_count").as_int());
      // Must match lap_counter_node's publisher QoS (KeepLast(1), reliable,
      // transient_local); a volatile subscription would miss the latched value and
      // defer the switch until the next lap actually completes.
      lap_sub_ = create_subscription<std_msgs::msg::Int32>(
        get_parameter("lap_count_topic").as_string(),
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(&GlobalRepublisherNode::onLapCount, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "lap switch armed: at lap %d the source becomes %s",
        lap_switch_count_, lap_switch_dir_.c_str());
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
        RCLCPP_INFO(get_logger(), "loaded global waypoints from %s", map_dir_.c_str());
      }
    }

    // Atomic in-process swap for the map_creator pipeline: load the separately generated
    // reload map and republish immediately. The caller (map_creator) waits for generation
    // completion and the next lap; this node validates data.
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
  // Publish the array as-is, or a single DELETEALL when the source JSON carried no
  // markers. regenerate_obstacle_map (the in-race map_creator path) writes empty
  // arrays on purpose; without the DELETEALL, RViz keeps rendering the previous
  // map's raceline forever because MarkerArray entries are persistent per ns+id.
  void publishMarkers(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
    const visualization_msgs::msg::MarkerArray & markers) const
  {
    if (!markers.markers.empty()) {
      pub->publish(markers);
      return;
    }
    visualization_msgs::msg::MarkerArray clear;
    visualization_msgs::msg::Marker deleter;
    deleter.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.markers.push_back(deleter);
    pub->publish(clear);
  }

  // Data-level validation before a candidate bundle may replace the live one.
  // Shared by the reload service and the lap-count switch, so the wording stays
  // path-neutral.
  // Mirrors the strictest downstream requirements (state_machine: strictly
  // increasing finite s_m; local_planning: >=4 waypoints).
  static bool validBundle(const GlobalWaypointBundle & bundle, std::string & why)
  {
    const auto & wpnts = bundle.global_traj_wpnts_iqp.wpnts;
    if (wpnts.size() < 4U) {
      why = "raceline has fewer than 4 waypoints";
      return false;
    }
    double prev_s = -std::numeric_limits<double>::infinity();
    for (const auto & w : wpnts) {
      if (!std::isfinite(w.s_m) || !std::isfinite(w.x_m) || !std::isfinite(w.y_m) ||
        !std::isfinite(w.psi_rad) || !std::isfinite(w.kappa_radpm) || !std::isfinite(w.vx_mps))
      {
        why = "raceline contains non-finite fields";
        return false;
      }
      if (w.s_m <= prev_s) {
        why = "raceline s_m is not strictly increasing";
        return false;
      }
      prev_s = w.s_m;
    }
    return true;
  }

  // Read + validate + atomically replace the live bundle. Shared by the reload
  // service and the lap-count switch so both apply the same acceptance rules;
  // on any failure the live line is left untouched.
  bool swapTo(const std::string & dir, const std::string & name, std::string & why)
  {
    GlobalWaypointBundle fresh;
    if (!read_global_waypoints(dir, fresh, why)) {
      return false;
    }
    if (!validBundle(fresh, why)) {
      return false;
    }
    bundle_ = std::move(fresh);
    map_dir_ = dir;
    has_bundle_ = true;
    set_parameter(rclcpp::Parameter("map_name", name));
    publish_all();  // swap immediately; do not wait for the 2 s republish timer
    why = "loaded " + std::to_string(bundle_.global_traj_wpnts_iqp.wpnts.size()) +
      " waypoints from " + map_dir_;
    return true;
  }

  void reloadWaypoints(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (reload_map_dir_.empty()) {
      response->success = false;
      response->message = "no reload map configured";
      return;
    }
    std::string msg;
    response->success = swapTo(reload_map_dir_, reload_map_name_, msg);
    response->message = msg;
    if (response->success) {
      RCLCPP_INFO(get_logger(), "reloaded: %s", msg.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "reload rejected: %s", msg.c_str());
    }
  }

  // One-way switch to the pre-generated bundle once the race reaches the configured
  // lap. `>=` so a dropped message cannot skip the switch, and the one-shot flag stops
  // the latched /lap_count value from re-reading the files on every redelivery.
  // A failed attempt leaves the flag clear so the next lap message retries.
  void onLapCount(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (lap_switched_ || msg->data < lap_switch_count_) {
      return;
    }
    std::string why;
    if (!swapTo(lap_switch_dir_, lap_switch_map_name_, why)) {
      RCLCPP_WARN(
        get_logger(), "lap %d switch to %s rejected: %s",
        msg->data, lap_switch_dir_.c_str(), why.c_str());
      return;
    }
    lap_switched_ = true;
    RCLCPP_INFO(get_logger(), "lap %d switch: %s", msg->data, why.c_str());
  }

  void publish_all()
  {
    if (!has_bundle_) {
      return;
    }

    glb_wpnts_pub_->publish(bundle_.global_traj_wpnts_iqp);
    if (publish_markers_ && glb_markers_pub_) {
      publishMarkers(glb_markers_pub_, bundle_.global_traj_markers_iqp);
      publishMarkers(vis_track_bnds_pub_, bundle_.trackbounds_markers);
    }

    if (publish_shortest_path_ && glb_sp_wpnts_pub_) {
      glb_sp_wpnts_pub_->publish(bundle_.global_traj_wpnts_sp);
    }

    if (publish_centerline_ && centerline_wpnts_pub_) {
      centerline_wpnts_pub_->publish(bundle_.centerline_waypoints);
      if (publish_markers_ && centerline_markers_pub_) {
        publishMarkers(centerline_markers_pub_, bundle_.centerline_markers);
      }
    }

    map_info_pub_->publish(bundle_.map_info_str);
    est_lap_time_pub_->publish(bundle_.est_lap_time);
  }

  bool publish_markers_{true};
  bool publish_shortest_path_{true};
  bool publish_centerline_{false};
  bool publish_lattice_{false};

  bool has_bundle_{false};
  std::string map_dir_;
  std::string reload_map_name_;
  std::string reload_map_dir_;
  std::string lap_switch_map_name_;
  std::string lap_switch_dir_;
  int lap_switch_count_{0};
  bool lap_switched_{false};
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
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_sub_;
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
