#ifndef STATIC_OBSTACLE_MAP__OBSTACLE_MARKER_BUILDER_HPP_
#define STATIC_OBSTACLE_MAP__OBSTACLE_MARKER_BUILDER_HPP_

#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "static_obstacle_map/static_obstacle_map_memory.hpp"

namespace static_obstacle_map
{

struct ObstacleMarkerConfig
{
  std::string marker_namespace{"persistent_static_obstacles"};
  double height_m{0.15};
  float red{1.0F};
  float green{0.1F};
  float blue{0.1F};
  float alpha{0.85F};
};

visualization_msgs::msg::MarkerArray buildObstacleMarkers(
  const std_msgs::msg::Header & header,
  const std::vector<StoredObstacle> & obstacles,
  const ObstacleMarkerConfig & config);

}  // namespace static_obstacle_map

#endif  // STATIC_OBSTACLE_MAP__OBSTACLE_MARKER_BUILDER_HPP_
