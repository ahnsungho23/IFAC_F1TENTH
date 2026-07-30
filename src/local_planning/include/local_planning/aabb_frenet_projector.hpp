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

#ifndef LOCAL_PLANNING__AABB_FRENET_PROJECTOR_HPP_
#define LOCAL_PLANNING__AABB_FRENET_PROJECTOR_HPP_

#include <optional>

#include "global_planning/clcs_frenet_converter.hpp"

namespace local_planning
{

struct FrenetAabbBounds
{
  double x_center{0.0};
  double y_center{0.0};
  double s_center{0.0};
  double d_center{0.0};
  double s_start{0.0};
  double s_end{0.0};
  double d_right{0.0};
  double d_left{0.0};
  double closest_abs_d{0.0};
  double diagonal{0.0};
};

std::optional<FrenetAabbBounds> projectCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  double track_length,
  double x_min,
  double x_max,
  double y_min,
  double y_max);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__AABB_FRENET_PROJECTOR_HPP_
