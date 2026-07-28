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

#include <cmath>
#include <cstddef>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace
{

f110_msgs::msg::WpntArray makeStraightReference(
  int count = 300, double spacing = 0.1,
  double left_width = 1.5, double right_width = 1.5)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = 0.0;
    waypoint.psi_rad = 0.0;
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = left_width;
    waypoint.d_right = right_width;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::Obstacle makeObstacle(
  int id, double s, double d_right = -0.20, double d_left = 0.20)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = s;
  obstacle.s_start = s - 0.20;
  obstacle.s_end = s + 0.20;
  obstacle.d_center = 0.5 * (d_right + d_left);
  obstacle.d_right = d_right;
  obstacle.d_left = d_left;
  obstacle.size = 0.40;
  obstacle.is_static = true;
  return obstacle;
}

RacelineSplineParameters testParameters()
{
  RacelineSplineParameters parameters;
  parameters.detection_lookahead_m = 12.0;
  parameters.obstacle_clearance_m = 0.25;
  parameters.vehicle_half_width_m = 0.12;
  parameters.boundary_margin_m = 0.08;
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 50.0;
  return parameters;
}

TEST(RacelineSplinePlanner, RejectsNonMonotonicGlobalReference)
{
  auto reference = makeStraightReference(20);
  reference.wpnts[10].s_m = reference.wpnts[9].s_m;
  RacelineSplinePlanner planner(testParameters());
  std::string error;
  EXPECT_FALSE(planner.setReference(reference, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RacelineSplinePlanner, ShiftsOnlyOrderedGlobalRaceLineSamples)
{
  const auto reference = makeStraightReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_forward = -1.0;
  bool saw_offset = false;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    EXPECT_GT(forward, previous_forward);
    previous_forward = forward;
    const std::size_t reference_index = static_cast<std::size_t>(
      std::llround(waypoint.s_m / 0.1));
    ASSERT_LT(reference_index, reference.wpnts.size());
    const auto & global = reference.wpnts[reference_index];
    EXPECT_NEAR(waypoint.x_m, global.x_m, 1.0e-9);
    EXPECT_NEAR(waypoint.y_m, waypoint.d_m, 1.0e-9);
    EXPECT_EQ(waypoint.s_m, global.s_m);
    saw_offset = saw_offset || std::abs(waypoint.d_m) > 0.20;
  }
  EXPECT_TRUE(saw_offset);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

TEST(RacelineSplinePlanner, UsesRightSideWhenLeftTrackSpaceIsInsufficient)
{
  auto reference = makeStraightReference(300, 0.1, 0.55, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, 0.0);
}

TEST(RacelineSplinePlanner, HonorsCommittedSideWhenItRemainsFeasible)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_TRUE(result.go_left);
}

TEST(RacelineSplinePlanner, IgnoresObstacleWithEnoughRawRacelineClearance)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(4, 7.0, 0.35, 0.55)});
  EXPECT_EQ(result.kind, SplinePlanKind::kNoObstacle) << result.reason;
  EXPECT_TRUE(result.path.wpnts.empty());
}

TEST(RacelineSplinePlanner, BuildsCollisionFreeStopWhenBothSidesAreClosed)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
}

TEST(RacelineSplinePlanner, HandlesObstacleAcrossTrackWrap)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  auto obstacle = makeObstacle(9, 0.30);
  obstacle.s_start = 0.10;
  obstacle.s_end = 0.50;
  const auto result = planner.plan(EgoFrenetState{29.0, 0.0, 2.0}, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  bool wrapped = false;
  double previous = -1.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(29.0, waypoint.s_m);
    EXPECT_GT(forward, previous);
    previous = forward;
    wrapped = wrapped || waypoint.s_m < 1.0;
  }
  EXPECT_TRUE(wrapped);
}

TEST(RacelineSplinePlanner, NeverJumpsToNearbyWrongSnakeBranch)
{
  auto reference = makeStraightReference();
  for (std::size_t i = 150U; i < reference.wpnts.size(); ++i) {
    reference.wpnts[i].x_m = 30.0 - reference.wpnts[i].s_m;
    reference.wpnts[i].y_m = 0.55;
    reference.wpnts[i].psi_rad = 3.14159265358979323846;
  }
  auto parameters = testParameters();
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(11, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LT(waypoint.s_m, 15.0)
      << "planner selected a geometrically nearby but topologically wrong snake branch";
    EXPECT_LT(waypoint.y_m, 0.55)
      << "path must remain an offset of the ordered first race-line branch";
  }
}

}  // namespace
}  // namespace local_planning
