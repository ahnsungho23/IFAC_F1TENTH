#include "static_obstacle_map/static_obstacle_map_node.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace static_obstacle_map
{

StaticObstacleMapNode::StaticObstacleMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("static_obstacle_map", options)
{
  declareParameters();
  loadParameters();
  memory_.configure(memory_config_);

  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  base_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    base_map_topic_, latched_qos,
    std::bind(&StaticObstacleMapNode::baseMapCallback, this, std::placeholders::_1));
  confirmed_static_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    confirmed_static_obs_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&StaticObstacleMapNode::confirmedStaticCallback, this, std::placeholders::_1));
  if (memory_config_.remove_reclassified_dynamic && !dynamic_obs_topic_.empty()) {
    dynamic_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
      dynamic_obs_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&StaticObstacleMapNode::dynamicCallback, this, std::placeholders::_1));
  }
  output_map_pub_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_topic_, latched_qos);
  if (publish_visualization_) {
    visualization_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, latched_qos);
  }
  reset_service_server_ = create_service<std_srvs::srv::Empty>(
    reset_service_,
    std::bind(
      &StaticObstacleMapNode::resetCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&StaticObstacleMapNode::publishIfDirty, this));

  RCLCPP_INFO(
    get_logger(),
    "static_obstacle_map started (base=%s, confirmed=%s, dynamic=%s, output=%s, "
    "visualization=%s, "
    "association=%.2fm, edge_confirm=%d, edge_tolerance=%.2fm, "
    "max_diagonal=%.2fm, inflation=%.2fm, publish_period=%dms)",
    base_map_topic_.c_str(), confirmed_static_obs_topic_.c_str(),
    memory_config_.remove_reclassified_dynamic ? dynamic_obs_topic_.c_str() : "retraction disabled",
    output_map_topic_.c_str(),
    publish_visualization_ ? visualization_topic_.c_str() : "disabled",
    memory_config_.association_distance_m, memory_config_.edge_confirm_frames,
    memory_config_.edge_match_tolerance_m,
    memory_config_.max_obstacle_diagonal_m, memory_config_.obstacle_inflation_m,
    publish_period_ms_);
}

void StaticObstacleMapNode::declareParameters()
{
  declare_parameter<std::string>("base_map_topic", "/map");
  declare_parameter<std::string>(
    "confirmed_static_obs_topic", "/confirmed_static_obs");
  declare_parameter<std::string>("dynamic_obs_topic", "/opp_obs");
  declare_parameter<std::string>("output_map_topic", "/adaptive_obstacle_map");
  declare_parameter<bool>("publish_visualization", true);
  declare_parameter<std::string>(
    "visualization_topic", "/adaptive_obstacle_map/markers");
  declare_parameter<std::string>(
    "visualization_marker_namespace", "adaptive_static_obstacles");
  declare_parameter<double>("visualization_height_m", 0.15);
  declare_parameter<double>("visualization_color_r", 1.0);
  declare_parameter<double>("visualization_color_g", 0.1);
  declare_parameter<double>("visualization_color_b", 0.1);
  declare_parameter<double>("visualization_color_a", 0.85);
  declare_parameter<std::string>(
    "reset_service", "/static_obstacle_map/reset");
  declare_parameter<int>("publish_period_ms", 100);

  declare_parameter<double>("association_distance_m", 0.30);
  declare_parameter<int>("edge_confirm_frames", 3);
  declare_parameter<double>("edge_match_tolerance_m", 0.05);
  declare_parameter<double>("max_obstacle_diagonal_m", 0.80);
  declare_parameter<double>("obstacle_inflation_m", 0.0);
  declare_parameter<int>("occupied_value", 100);
  declare_parameter<bool>("remove_reclassified_dynamic", false);
  declare_parameter<bool>("clear_on_base_map_geometry_change", true);
}

void StaticObstacleMapNode::loadParameters()
{
  base_map_topic_ = get_parameter("base_map_topic").as_string();
  confirmed_static_obs_topic_ =
    get_parameter("confirmed_static_obs_topic").as_string();
  dynamic_obs_topic_ = get_parameter("dynamic_obs_topic").as_string();
  output_map_topic_ = get_parameter("output_map_topic").as_string();
  publish_visualization_ = get_parameter("publish_visualization").as_bool();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  marker_config_.marker_namespace =
    get_parameter("visualization_marker_namespace").as_string();
  marker_config_.height_m = std::max(
    0.001, get_parameter("visualization_height_m").as_double());
  marker_config_.red = static_cast<float>(
    std::clamp(get_parameter("visualization_color_r").as_double(), 0.0, 1.0));
  marker_config_.green = static_cast<float>(
    std::clamp(get_parameter("visualization_color_g").as_double(), 0.0, 1.0));
  marker_config_.blue = static_cast<float>(
    std::clamp(get_parameter("visualization_color_b").as_double(), 0.0, 1.0));
  marker_config_.alpha = static_cast<float>(
    std::clamp(get_parameter("visualization_color_a").as_double(), 0.0, 1.0));
  reset_service_ = get_parameter("reset_service").as_string();
  publish_period_ms_ = std::max(
    1, static_cast<int>(get_parameter("publish_period_ms").as_int()));

  memory_config_.association_distance_m =
    get_parameter("association_distance_m").as_double();
  memory_config_.edge_confirm_frames = std::max(
    1, static_cast<int>(get_parameter("edge_confirm_frames").as_int()));
  memory_config_.edge_match_tolerance_m = std::max(
    0.0, get_parameter("edge_match_tolerance_m").as_double());
  memory_config_.max_obstacle_diagonal_m =
    get_parameter("max_obstacle_diagonal_m").as_double();
  memory_config_.obstacle_inflation_m =
    get_parameter("obstacle_inflation_m").as_double();
  memory_config_.occupied_value =
    static_cast<int>(get_parameter("occupied_value").as_int());
  memory_config_.remove_reclassified_dynamic =
    get_parameter("remove_reclassified_dynamic").as_bool();
  memory_config_.clear_on_base_map_geometry_change =
    get_parameter("clear_on_base_map_geometry_change").as_bool();
}

void StaticObstacleMapNode::baseMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  const BaseMapUpdate update = memory_.setBaseMap(*message);
  if (!update.accepted) {
    RCLCPP_WARN(
      get_logger(),
      "Rejected invalid base map (%u x %u, resolution=%.6f, data=%zu)",
      message->info.width, message->info.height, message->info.resolution,
      message->data.size());
    return;
  }
  if (update.geometry_changed) {
    RCLCPP_WARN(
      get_logger(),
      "Base-map geometry changed; cleared %zu stored obstacles (clear_on_change=%s).",
      update.cleared_obstacles,
      memory_config_.clear_on_base_map_geometry_change ? "true" : "false");
  } else {
    RCLCPP_INFO_ONCE(
      get_logger(), "Base map received (%u x %u @ %.3f m, frame=%s).",
      message->info.width, message->info.height, message->info.resolution,
      message->header.frame_id.c_str());
  }
  publishMap();
}

void StaticObstacleMapNode::confirmedStaticCallback(
  const f110_msgs::msg::ObstacleArray::SharedPtr message)
{
  const MemoryUpdateStats stats = memory_.updateConfirmed(*message);
  if (stats.rejected > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected %zu confirmed-static entries without valid visible Cartesian AABBs.",
      stats.rejected);
  }
  if (stats.limit_rejected > 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected %zu confirmed obstacle expansion(s) exceeding max diagonal %.3f m.",
      stats.limit_rejected, memory_config_.max_obstacle_diagonal_m);
  }
  if (stats.inserted == 0U && stats.geometry_expanded == 0U) {
    return;
  }
  map_dirty_ = true;
  if (stats.inserted > 0U) {
    RCLCPP_INFO(
      get_logger(), "Persistent static map: +%zu total=%zu.",
      stats.inserted, memory_.obstacles().size());
    publishMap();
  }
}

void StaticObstacleMapNode::dynamicCallback(
  const f110_msgs::msg::ObstacleArray::SharedPtr message)
{
  const MemoryUpdateStats stats = memory_.removeDynamic(*message);
  if (stats.removed == 0U) {
    return;
  }
  RCLCPP_WARN(
    get_logger(),
    "Removed %zu stored obstacle(s) reclassified as dynamic; total=%zu.",
    stats.removed, memory_.obstacles().size());
  publishMap();
}

void StaticObstacleMapNode::resetCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  const std::size_t removed = memory_.clear();
  RCLCPP_INFO(get_logger(), "Persistent static map reset; removed %zu obstacle(s).", removed);
  publishMap();
}

void StaticObstacleMapNode::publishIfDirty()
{
  if (map_dirty_) {
    publishMap();
  }
}

void StaticObstacleMapNode::publishMap()
{
  auto output = memory_.composeMap();
  if (!output.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting for a valid base map before publishing %s.",
      output_map_topic_.c_str());
    return;
  }
  output->header.stamp = now();
  publishVisualization(*output);
  output_map_pub_->publish(std::move(*output));
  map_dirty_ = false;
}

void StaticObstacleMapNode::publishVisualization(
  const nav_msgs::msg::OccupancyGrid & map)
{
  if (!visualization_pub_) {
    return;
  }
  auto markers = buildObstacleMarkers(
    map.header, memory_.obstacles(),
    memory_config_.obstacle_inflation_m, map.info.resolution, marker_config_);
  visualization_pub_->publish(std::move(markers));
}

}  // namespace static_obstacle_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<static_obstacle_map::StaticObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
