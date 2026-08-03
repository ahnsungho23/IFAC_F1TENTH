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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "global_planning/clcs_frenet_converter.hpp"
#include "obstacle_detector/aabb_frenet_projector.hpp"

namespace obstacle_detector
{
namespace
{

constexpr double kTolerance = 1.0e-6;

global_planning::ClcsFrenetConverter::Ptr makeConverter(
  const std::vector<global_planning::ReferenceWaypoint> & waypoints)
{
  global_planning::ClcsFrenetConfig config;
  config.closed_loop = false;
  return global_planning::ClcsFrenetConverter::create(waypoints, config, 1);
}

TEST(AabbFrenetProjector, PreservesIndependentLongitudinalAndLateralExtents)
{
  const auto converter = makeConverter(
      {
        {0.0, 0.0, 0.0},
        {5.0, 0.0, 5.0},
        {10.0, 0.0, 10.0},
      });

  const auto bounds = projectCartesianAabb(
    *converter, 4.0, 6.0, -0.1, 0.1);

  ASSERT_TRUE(bounds.has_value());
  EXPECT_NEAR(bounds->x_center, 5.0, kTolerance);
  EXPECT_NEAR(bounds->y_center, 0.0, kTolerance);
  EXPECT_NEAR(bounds->s_center, 5.0, kTolerance);
  EXPECT_NEAR(bounds->d_center, 0.0, kTolerance);
  EXPECT_NEAR(bounds->s_start, 4.0, kTolerance);
  EXPECT_NEAR(bounds->s_end, 6.0, kTolerance);
  EXPECT_NEAR(bounds->d_right, -0.1, kTolerance);
  EXPECT_NEAR(bounds->d_left, 0.1, kTolerance);
  EXPECT_NEAR(bounds->diagonal, std::hypot(2.0, 0.2), kTolerance);
  EXPECT_NEAR(bounds->longitudinal_half_extent, 1.0, kTolerance);
}

TEST(AabbFrenetProjector, RotatesAllFourCornersIntoTheLocalTrackFrame)
{
  const auto converter = makeConverter(
      {
        {0.0, 0.0, 0.0},
        {5.0, 5.0, std::sqrt(50.0)},
        {10.0, 10.0, std::sqrt(200.0)},
      });

  const auto bounds = projectCartesianAabb(
    *converter, 4.0, 6.0, 4.9, 5.1);

  ASSERT_TRUE(bounds.has_value());
  const double expected_half_extent = (1.0 + 0.1) / std::sqrt(2.0);
  EXPECT_NEAR(bounds->s_start, bounds->s_center - expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->s_end, bounds->s_center + expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->d_right, bounds->d_center - expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->d_left, bounds->d_center + expected_half_extent, kTolerance);
}

TEST(AabbFrenetProjector, UsesClosestAabbFaceAgainstTheActualCurvedRaceLine)
{
  std::vector<global_planning::ReferenceWaypoint> waypoints;
  double s = 0.0;
  for (int i = 0; i <= 8; ++i) {
    const double angle = static_cast<double>(i) * std::acos(-1.0) / 8.0;
    const double x = std::cos(angle);
    const double y = std::sin(angle);
    if (!waypoints.empty()) {
      s += std::hypot(x - waypoints.back().x, y - waypoints.back().y);
    }
    waypoints.push_back({x, y, s});
  }
  const auto converter = makeConverter(waypoints);

  const auto bounds = projectCartesianAabb(
    *converter, -0.4, 0.4, 0.5, 0.7);

  ASSERT_TRUE(bounds.has_value());
  // The centre tangent at the top of the semicircle would report about 0.3 m. The true nearest
  // point is the perpendicular projection of the AABB's upper-right corner onto the preceding
  // curved-race-line chord.
  const double first_angle = std::acos(-1.0) / 4.0;
  const double second_angle = 3.0 * std::acos(-1.0) / 8.0;
  const double x0 = std::cos(first_angle);
  const double y0 = std::sin(first_angle);
  const double x1 = std::cos(second_angle);
  const double y1 = std::sin(second_angle);
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double t = std::clamp(
    ((0.4 - x0) * dx + (0.7 - y0) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
  const double expected_closest = std::hypot(
    0.4 - (x0 + t * dx), 0.7 - (y0 + t * dy));
  EXPECT_NEAR(bounds->closest_abs_d, expected_closest, kTolerance);
  EXPECT_NEAR(bounds->d_right, expected_closest, kTolerance);
  EXPECT_GT(bounds->d_left, bounds->d_right);
}

TEST(AabbFrenetProjector, RejectsMalformedOrPointSizedAabb)
{
  const auto converter = makeConverter(
      {
        {0.0, 0.0, 0.0},
        {5.0, 0.0, 5.0},
        {10.0, 0.0, 10.0},
      });
  EXPECT_FALSE(projectCartesianAabb(*converter, 2.0, 1.0, -0.1, 0.1));
  EXPECT_FALSE(projectCartesianAabb(*converter, 1.0, 1.0, 0.0, 0.0));
}

}  // namespace
}  // namespace obstacle_detector
