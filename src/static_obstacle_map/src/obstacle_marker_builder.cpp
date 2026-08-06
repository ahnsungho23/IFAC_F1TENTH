#include "static_obstacle_map/obstacle_marker_builder.hpp"

#include <algorithm>

#include <visualization_msgs/msg/marker.hpp>

namespace static_obstacle_map
{

visualization_msgs::msg::MarkerArray buildObstacleMarkers(
  const std_msgs::msg::Header & header,
  const std::vector<StoredObstacle> & obstacles,
  const ObstacleMarkerConfig & config)
{
  using visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray result;
  Marker clear;
  clear.header = header;
  clear.ns = config.marker_namespace;
  clear.action = Marker::DELETEALL;
  result.markers.push_back(clear);

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
    marker.scale.x = std::max(0.001, obstacle.x_max - obstacle.x_min);
    marker.scale.y = std::max(0.001, obstacle.y_max - obstacle.y_min);
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
