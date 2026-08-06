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
  confirmed_static_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    confirmed_static_obs_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&StaticObstacleMapNode::confirmedStaticCallback, this, std::placeholders::_1));
  if (memory_config_.remove_reclassified_dynamic && !dynamic_obs_topic_.empty()) {
    dynamic_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
      dynamic_obs_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&StaticObstacleMapNode::dynamicCallback, this, std::placeholders::_1));
  }
  output_obstacles_pub_ =
    create_publisher<f110_msgs::msg::ObstacleArray>(output_obstacles_topic_, latched_qos);
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
  publishSnapshot();

  RCLCPP_INFO(
    get_logger(),
    "static_obstacle_map started (confirmed=%s, dynamic=%s, output=%s, visualization=%s, "
    "frame=%s, association=%.2fm, edge_confirm=%d, edge_tolerance=%.2fm, "
    "max_diagonal=%.2fm)",
    confirmed_static_obs_topic_.c_str(),
    memory_config_.remove_reclassified_dynamic ? dynamic_obs_topic_.c_str() :
    "retraction disabled",
    output_obstacles_topic_.c_str(),
    publish_visualization_ ? visualization_topic_.c_str() : "disabled",
    frame_id_.c_str(),
    memory_config_.association_distance_m, memory_config_.edge_confirm_frames,
    memory_config_.edge_match_tolerance_m,
    memory_config_.max_obstacle_diagonal_m);
}

void StaticObstacleMapNode::declareParameters()
{
  declare_parameter<std::string>(
    "confirmed_static_obs_topic", "/confirmed_static_obs");
  declare_parameter<std::string>("dynamic_obs_topic", "/opp_obs");
  declare_parameter<std::string>(
    "output_obstacles_topic", "/adaptive_obstacle_map");
  declare_parameter<bool>("publish_visualization", true);
  declare_parameter<std::string>(
    "visualization_topic", "/adaptive_obstacle_map/markers");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<std::string>(
    "visualization_marker_namespace", "adaptive_static_obstacles");
  declare_parameter<double>("visualization_height_m", 0.15);
  declare_parameter<double>("visualization_minimum_footprint_m", 0.025);
  declare_parameter<double>("visualization_color_r", 1.0);
  declare_parameter<double>("visualization_color_g", 0.1);
  declare_parameter<double>("visualization_color_b", 0.1);
  declare_parameter<double>("visualization_color_a", 0.85);
  declare_parameter<std::string>(
    "reset_service", "/static_obstacle_map/reset");

  declare_parameter<double>("association_distance_m", 0.30);
  declare_parameter<int>("edge_confirm_frames", 3);
  declare_parameter<double>("edge_match_tolerance_m", 0.05);
  declare_parameter<double>("max_obstacle_diagonal_m", 0.80);
  declare_parameter<bool>("remove_reclassified_dynamic", false);
}

void StaticObstacleMapNode::loadParameters()
{
  confirmed_static_obs_topic_ =
    get_parameter("confirmed_static_obs_topic").as_string();
  dynamic_obs_topic_ = get_parameter("dynamic_obs_topic").as_string();
  output_obstacles_topic_ =
    get_parameter("output_obstacles_topic").as_string();
  publish_visualization_ = get_parameter("publish_visualization").as_bool();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  marker_config_.marker_namespace =
    get_parameter("visualization_marker_namespace").as_string();
  marker_config_.height_m = std::max(
    0.001, get_parameter("visualization_height_m").as_double());
  visualization_minimum_footprint_m_ = std::max(
    0.001, get_parameter("visualization_minimum_footprint_m").as_double());
  marker_config_.red = static_cast<float>(
    std::clamp(get_parameter("visualization_color_r").as_double(), 0.0, 1.0));
  marker_config_.green = static_cast<float>(
    std::clamp(get_parameter("visualization_color_g").as_double(), 0.0, 1.0));
  marker_config_.blue = static_cast<float>(
    std::clamp(get_parameter("visualization_color_b").as_double(), 0.0, 1.0));
  marker_config_.alpha = static_cast<float>(
    std::clamp(get_parameter("visualization_color_a").as_double(), 0.0, 1.0));
  reset_service_ = get_parameter("reset_service").as_string();

  memory_config_.association_distance_m =
    get_parameter("association_distance_m").as_double();
  memory_config_.edge_confirm_frames = std::max(
    1, static_cast<int>(get_parameter("edge_confirm_frames").as_int()));
  memory_config_.edge_match_tolerance_m = std::max(
    0.0, get_parameter("edge_match_tolerance_m").as_double());
  memory_config_.max_obstacle_diagonal_m =
    get_parameter("max_obstacle_diagonal_m").as_double();
  memory_config_.remove_reclassified_dynamic =
    get_parameter("remove_reclassified_dynamic").as_bool();
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
  if (stats.inserted > 0U) {
    RCLCPP_INFO(
      get_logger(), "Persistent static memory: +%zu total=%zu.",
      stats.inserted, memory_.obstacles().size());
  }
  publishSnapshot();
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
  publishSnapshot();
}

void StaticObstacleMapNode::resetCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  const std::size_t removed = memory_.clear();
  RCLCPP_INFO(
    get_logger(), "Persistent static memory reset; removed %zu obstacle(s).", removed);
  publishSnapshot();
}

void StaticObstacleMapNode::publishSnapshot()
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id_;
  header.stamp = now();
  auto output = memory_.buildObstacleArray(header);
  output_obstacles_pub_->publish(std::move(output));

  if (!visualization_pub_) {
    return;
  }
  auto markers = buildObstacleMarkers(
    header, memory_.obstacles(), 0.0,
    visualization_minimum_footprint_m_, marker_config_);
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
