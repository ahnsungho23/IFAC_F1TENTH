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

#ifndef OBSTACLE_DETECTOR__FRENET_MARKER_BUILDER_HPP_
#define OBSTACLE_DETECTOR__FRENET_MARKER_BUILDER_HPP_

#include <string>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "obstacle_detector/frenet_projector.hpp"

namespace obstacle_detector
{

visualization_msgs::msg::MarkerArray buildFrenetObstacleMarkers(
  const f110_msgs::msg::ObstacleArray & obstacles,
  const FrenetProjector & projector,
  const std::string & marker_namespace,
  float red,
  float green,
  float blue);

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__FRENET_MARKER_BUILDER_HPP_
