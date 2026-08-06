#include "static_obstacle_map/obstacle_marker_builder.hpp"

#include <algorithm>

#include <visualization_msgs/msg/marker.hpp>

namespace static_obstacle_map
{

visualization_msgs::msg::MarkerArray buildObstacleMarkers(
  const std_msgs::msg::Header & header,
  const std::vector<StoredObstacle> & obstacles,
  double obstacle_inflation_m,
  double minimum_footprint_m,
  const ObstacleMarkerConfig & config)
{
  using visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray result;
  Marker clear;
  clear.header = header;
  clear.ns = config.marker_namespace;
  clear.action = Marker::DELETEALL;
  result.markers.push_back(clear);

  const double inflation = std::max(0.0, obstacle_inflation_m);
  const double minimum_footprint = std::max(0.001, minimum_footprint_m);
  const double height = std::max(0.001, config.height_m);
  for (const auto & obstacle : obstacles) {
    Marker marker;
    marker.header = header;
    marker.ns = config.marker_namespace;
    marker.id = obstacle.memory_id;
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose.position.x = 0.5 * (obstacle.x_min + obstacle.x_max);
    marker.pose.position.y = 0.5 * (obstacle.y_min + obstacle.y_max);
    marker.pose.position.z = 0.5 * height;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(
      minimum_footprint, obstacle.x_max - obstacle.x_min + 2.0 * inflation);
    marker.scale.y = std::max(
      minimum_footprint, obstacle.y_max - obstacle.y_min + 2.0 * inflation);
    marker.scale.z = height;
    marker.color.r = std::clamp(config.red, 0.0F, 1.0F);
    marker.color.g = std::clamp(config.green, 0.0F, 1.0F);
    marker.color.b = std::clamp(config.blue, 0.0F, 1.0F);
    marker.color.a = std::clamp(config.alpha, 0.0F, 1.0F);
    result.markers.push_back(marker);
  }
  return result;
}

}  // namespace static_obstacle_map
