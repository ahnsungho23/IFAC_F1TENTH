// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "obstacle_detector/frenet_marker_builder.hpp"

#include <cmath>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace obstacle_detector
{
namespace
{

constexpr int kLongitudinalSamples = 8;

bool finiteFrenetBounds(const f110_msgs::msg::Obstacle & obstacle)
{
  return std::isfinite(obstacle.s_start) &&
         std::isfinite(obstacle.s_end) &&
         std::isfinite(obstacle.d_right) &&
         std::isfinite(obstacle.d_left) &&
         obstacle.d_right <= obstacle.d_left;
}

double forwardSpan(double start, double end, double track_length)
{
  double span = end - start;
  if (!(track_length > 0.0)) {
    return std::max(0.0, span);
  }
  span = std::fmod(span, track_length);
  if (span < 0.0) {
    span += track_length;
  }
  return span;
}

bool appendBoundaryPoint(
  visualization_msgs::msg::Marker & marker,
  const FrenetProjector & projector,
  double s,
  double d)
{
  geometry_msgs::msg::Point point;
  double yaw = 0.0;
  if (!projector.toCartesian(s, d, point.x, point.y, yaw)) {
    return false;
  }
  point.z = 0.12;
  marker.points.push_back(point);
  return true;
}

}  // namespace

visualization_msgs::msg::MarkerArray buildFrenetObstacleMarkers(
  const f110_msgs::msg::ObstacleArray & obstacles,
  const FrenetProjector & projector,
  const std::string & marker_namespace,
  float red,
  float green,
  float blue)
{
  visualization_msgs::msg::MarkerArray result;

  visualization_msgs::msg::Marker clear;
  clear.header = obstacles.header;
  clear.ns = marker_namespace;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  result.markers.push_back(clear);

  if (!projector.ready()) {
    return result;
  }

  const double track_length = projector.raceline_length();
  for (const auto & obstacle : obstacles.obstacles) {
    if (!finiteFrenetBounds(obstacle)) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = obstacles.header;
    marker.ns = marker_namespace;
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = obstacle.is_visible ? 0.95F : 0.45F;

    const double span = forwardSpan(
      obstacle.s_start, obstacle.s_end, track_length);
    bool valid = true;
    for (int i = 0; i <= kLongitudinalSamples; ++i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(kLongitudinalSamples);
      valid = valid && appendBoundaryPoint(
        marker, projector, obstacle.s_start + ratio * span, obstacle.d_right);
    }
    for (int i = kLongitudinalSamples; i >= 0; --i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(kLongitudinalSamples);
      valid = valid && appendBoundaryPoint(
        marker, projector, obstacle.s_start + ratio * span, obstacle.d_left);
    }
    if (valid && !marker.points.empty()) {
      marker.points.push_back(marker.points.front());
      result.markers.push_back(std::move(marker));
    }
  }
  return result;
}

}  // namespace obstacle_detector
