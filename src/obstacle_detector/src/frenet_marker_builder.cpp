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

// 곡선 장애물 경계를 직선 하나로 단순화하지 않도록 각 종방향 면을 여러 점으로 샘플링한다.
constexpr int kLongitudinalSamples = 8;

// marker를 만들기 전에 네 Frenet 경계가 유한하고 좌우 순서가 올바른지 확인한다.
bool finiteFrenetBounds(const f110_msgs::msg::Obstacle & obstacle)
{
  return std::isfinite(obstacle.s_start) &&
         std::isfinite(obstacle.s_end) &&
         std::isfinite(obstacle.d_right) &&
         std::isfinite(obstacle.d_left) &&
         obstacle.d_right <= obstacle.d_left;
}

// 폐루프 시작점을 지나가는 장애물도 올바른 전방 길이로 표현한다.
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

// Frenet 경계점 하나를 map 좌표로 바꾸어 LINE_STRIP에 추가한다.
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

  // 매 주기 DELETEALL을 먼저 보내 이전 프레임에서 사라진 장애물 marker가 남지 않게 한다.
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
    // 현재 측정된 물체는 진하게, TTL 예측만 남은 물체는 반투명하게 표시한다.
    marker.color.a = obstacle.is_visible ? 0.95F : 0.45F;

    const double span = forwardSpan(
      obstacle.s_start, obstacle.s_end, track_length);
    // 오른쪽 경계를 s 증가 방향으로, 왼쪽 경계를 s 감소 방향으로 따라가 폐곡선을 만든다.
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
