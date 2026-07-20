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

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "local_planning/safe_corridor.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05F;
  grid.info.width = 240;
  grid.info.height = 120;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -3.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  return grid;
}

void fillRectangle(
  nav_msgs::msg::OccupancyGrid & grid,
  const double min_x, const double max_x,
  const double min_y, const double max_y,
  const int8_t value = 100)
{
  const int min_column = std::max(
    0, static_cast<int>(std::floor(
      (min_x - grid.info.origin.position.x) / grid.info.resolution)));
  const int max_column = std::min(
    static_cast<int>(grid.info.width) - 1,
    static_cast<int>(std::floor(
      (max_x - grid.info.origin.position.x) / grid.info.resolution)));
  const int min_row = std::max(
    0, static_cast<int>(std::floor(
      (min_y - grid.info.origin.position.y) / grid.info.resolution)));
  const int max_row = std::min(
    static_cast<int>(grid.info.height) - 1,
    static_cast<int>(std::floor(
      (max_y - grid.info.origin.position.y) / grid.info.resolution)));
  for (int row = min_row; row <= max_row; ++row) {
    for (int column = min_column; column <= max_column; ++column) {
      grid.data[row * grid.info.width + column] = value;
    }
  }
}

std::vector<local_planning::CorridorReferenceSample> makeStraightReferences(
  const double spacing = 0.10, const double d_left = 1.0, const double d_right = 1.0)
{
  std::vector<local_planning::CorridorReferenceSample> references;
  for (int index = 0; index <= 80; ++index) {
    references.push_back(
      {
        index, index, index * spacing, index * spacing, 0.0, 0.0,
        d_left, d_right, 0.5 * spacing});
  }
  return references;
}

local_planning::SafeCorridorConfig makeConfig()
{
  local_planning::SafeCorridorConfig config;
  config.minimum_longitudinal_half_width_m = 0.15;
  config.vehicle_radius_m = 0.20;
  config.path_clearance_margin_m = 0.05;
  config.vehicle_front_extent_m = 0.3302;
  config.vehicle_rear_extent_m = 0.05;
  config.vehicle_width_m = 0.2413;
  config.localization_margin_m = 0.05;
  config.safety_margin_m = 0.05;
  return config;
}

const local_planning::SafeCorridorSample & firstBlocked(
  const local_planning::SafeCorridorResult & result)
{
  const auto iterator = std::find_if(
    result.samples.begin(), result.samples.end(),
    [](const auto & sample) {return sample.center_blocked;});
  EXPECT_NE(iterator, result.samples.end());
  return *iterator;
}

TEST(SafeCorridorTest, DetectsObstacleAttachedToLeftWallAndKeepsRightPassage)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, 0.10, 1.05);
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  EXPECT_FALSE(sample.left_feasible);
  EXPECT_TRUE(sample.right_feasible);
  EXPECT_LT(sample.blocking_raw_min_d, 0.20);
}

TEST(SafeCorridorTest, DetectsObstacleAttachedToRightWallAndKeepsLeftPassage)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, -1.05, -0.10);
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  EXPECT_TRUE(sample.left_feasible);
  EXPECT_FALSE(sample.right_feasible);
  EXPECT_GT(sample.blocking_raw_max_d, -0.20);
}

TEST(SafeCorridorTest, LargeObstacleIsNotDiscardedByComponentArea)
{
  auto grid = makeGrid();
  // 2.4 m^2, larger than the legacy 2.0 m^2 component cutoff.
  fillRectangle(grid, 3.0, 5.0, -0.60, 0.60);
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  EXPECT_TRUE(sample.center_blocked);
  EXPECT_FALSE(sample.left_feasible);
  EXPECT_FALSE(sample.right_feasible);
  EXPECT_FALSE(result.inflated_cell_indices.empty());
  EXPECT_FALSE(firstBlocked(result).inflated_cell_indices.empty());
}

TEST(SafeCorridorTest, DistanceBasedLongitudinalBandFindsThinDiagonalBetweenWaypoints)
{
  auto grid = makeGrid();
  for (int index = 0; index < 9; ++index) {
    const double x = 2.05 + 0.05 * index;
    const double y = -0.20 + 0.05 * index;
    fillRectangle(grid, x, x + 0.01, y, y + 0.01);
  }
  // The diagonal is centered at x=2.25, exactly between 0.5 m-spaced samples.
  // A fixed +/-0.15 m band misses it; half-spacing + footprint support does not.
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences(0.50));
  EXPECT_TRUE(
    std::any_of(
      result.samples.begin(), result.samples.end(),
      [](const auto & sample) {return sample.center_blocked;}));
}

TEST(SafeCorridorTest, PreservesNarrowButFeasibleRightCorridor)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, -0.10, 1.05);
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  double right_width = 0.0;
  for (const auto & interval : sample.feasible_intervals) {
    right_width += std::max(0.0, std::min(0.0, interval.max_d) - interval.min_d);
  }
  EXPECT_TRUE(sample.right_feasible);
  EXPECT_GT(right_width, 0.10);
  EXPECT_LT(right_width, 0.50);
}

TEST(SafeCorridorTest, RectangularFootprintDoesNotUseFrontLengthAsLateralRadius)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, -0.15, 0.15);
  auto config = makeConfig();
  config.vehicle_radius_m = 0.12;
  config.path_clearance_margin_m = 0.03;
  config.localization_margin_m = 0.04;
  config.safety_margin_m = 0.03;
  config.preserve_circular_collision_check = false;

  const auto result = local_planning::SafeCorridorBuilder(config).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  const double expected_lateral_support =
    0.5 * config.vehicle_width_m + config.path_clearance_margin_m +
    config.localization_margin_m + config.safety_margin_m;
  EXPECT_NEAR(sample.track_max_d, 1.0 - expected_lateral_support, 1e-6);
  EXPECT_NEAR(sample.track_min_d, -1.0 + expected_lateral_support, 1e-6);
  EXPECT_TRUE(sample.left_feasible);
  EXPECT_TRUE(sample.right_feasible);
}

TEST(SafeCorridorTest, ReportsBothSidesBlockedExplicitly)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, -0.60, 0.60);
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(
    grid, makeStraightReferences());
  const auto & sample = firstBlocked(result);
  EXPECT_TRUE(sample.feasible_intervals.empty());
  EXPECT_FALSE(sample.left_feasible);
  EXPECT_FALSE(sample.right_feasible);
}

TEST(SafeCorridorTest, AppliesAllUnknownCellPolicies)
{
  auto grid = makeGrid();
  fillRectangle(grid, 3.8, 4.2, -0.10, 0.10, -1);
  auto config = makeConfig();

  config.unknown_policy = local_planning::UnknownCellPolicy::kTreatAsFree;
  const auto free_result = local_planning::SafeCorridorBuilder(config).build(
    grid, makeStraightReferences());
  EXPECT_FALSE(
    std::any_of(
      free_result.samples.begin(), free_result.samples.end(),
      [](const auto & sample) {return sample.center_blocked;}));

  config.unknown_policy = local_planning::UnknownCellPolicy::kTreatAsOccupied;
  const auto occupied_result = local_planning::SafeCorridorBuilder(config).build(
    grid, makeStraightReferences());
  EXPECT_TRUE(firstBlocked(occupied_result).center_blocked);

  config.unknown_policy = local_planning::UnknownCellPolicy::kRejectCandidate;
  const auto reject_result = local_planning::SafeCorridorBuilder(config).build(
    grid, makeStraightReferences());
  EXPECT_TRUE(firstBlocked(reject_result).center_blocked_by_unknown);
}

TEST(SafeCorridorTest, KeepsUnwrappedHorizonAcrossStartFinishIndexWrap)
{
  auto grid = makeGrid();
  fillRectangle(grid, 0.90, 1.10, -0.10, 0.10);
  std::vector<local_planning::CorridorReferenceSample> references{
    {0, 18, 0.0, -1.0, 0.0, 0.0, 1.0, 1.0, 0.5},
    {1, 19, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.5},
    {2, 0, 2.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.5},
    {3, 1, 3.0, 2.0, 0.0, 0.0, 1.0, 1.0, 0.5}};
  const auto result = local_planning::SafeCorridorBuilder(makeConfig()).build(grid, references);
  ASSERT_EQ(result.samples.size(), references.size());
  EXPECT_EQ(result.samples[2].reference.waypoint_index, 0);
  EXPECT_DOUBLE_EQ(result.samples[2].reference.unwrapped_s, 2.0);
  EXPECT_TRUE(result.samples[2].center_blocked);
}

TEST(SafeCorridorTest, IntersectsNarrowPassageAcrossWholeObstacleSpan)
{
  std::vector<local_planning::SafeCorridorSample> samples(3);
  samples[0].feasible_intervals = {{-0.70, -0.20, false}, {0.36, 0.52, false}};
  samples[1].feasible_intervals = {{-0.65, -0.25, false}, {0.40, 0.50, false}};
  samples[2].feasible_intervals = {{-0.60, -0.30, false}, {0.42, 0.48, false}};

  const auto left = local_planning::SafeCorridorBuilder::intersectFeasibleIntervals(
    samples, 0, 2, true);
  const auto right = local_planning::SafeCorridorBuilder::intersectFeasibleIntervals(
    samples, 0, 2, false);
  ASSERT_EQ(left.size(), 1U);
  EXPECT_DOUBLE_EQ(left.front().min_d, 0.42);
  EXPECT_DOUBLE_EQ(left.front().max_d, 0.48);
  ASSERT_EQ(right.size(), 1U);
  EXPECT_DOUBLE_EQ(right.front().min_d, -0.60);
  EXPECT_DOUBLE_EQ(right.front().max_d, -0.30);
}

TEST(SafeCorridorTest, SamplesTargetsInsideNarrowCommonPassage)
{
  const std::vector<local_planning::LateralInterval> passage{{0.42, 0.48, false}};
  const auto targets = local_planning::SafeCorridorBuilder::sampleTargetOffsets(
    passage, 0.65, 4, 0.01);

  ASSERT_GE(targets.size(), 2U);
  for (const double target : targets) {
    EXPECT_GE(target, 0.43 - 1e-9);
    EXPECT_LE(target, 0.47 + 1e-9);
  }
  EXPECT_NEAR(targets.front(), 0.47, 1e-9);
  EXPECT_NE(
    std::find_if(
      targets.begin(), targets.end(),
      [](const double target) {return std::abs(target - 0.45) < 1e-9;}),
    targets.end());
}

TEST(SafeCorridorTest, BuildsPiecewiseProfileWithoutOneCommonOffset)
{
  std::vector<local_planning::SafeCorridorSample> samples(9);
  const std::vector<local_planning::LateralInterval> moving_passage{
    {0.20, 0.35, false}, {0.25, 0.40, false}, {0.35, 0.50, false},
    {0.50, 0.65, false}, {0.65, 0.80, false}, {0.50, 0.65, false},
    {0.35, 0.50, false}, {0.25, 0.40, false}, {0.20, 0.35, false}};
  for (std::size_t index = 0; index < samples.size(); ++index) {
    samples[index].reference.unwrapped_s = 0.1 * static_cast<double>(index);
    samples[index].feasible_intervals = {moving_passage[index]};
  }
  EXPECT_TRUE(
    local_planning::SafeCorridorBuilder::intersectFeasibleIntervals(
      samples, 0, 8, true).empty());

  const auto profiles =
    local_planning::SafeCorridorBuilder::buildCorridorGuidedProfiles(
    samples, 0, 8, true, 0.55, 4, 0.01, 4, 12);
  ASSERT_FALSE(profiles.empty());
  ASSERT_GE(profiles.front().size(), 3U);

  const auto & knots = profiles.front();
  std::size_t segment = 1U;
  for (int index = 0; index <= 8; ++index) {
    while (segment + 1U < knots.size() && index > knots[segment].waypoint_offset) {
      ++segment;
    }
    const auto & left = knots[segment - 1U];
    const auto & right = knots[segment];
    const double ratio = static_cast<double>(index - left.waypoint_offset) /
      static_cast<double>(right.waypoint_offset - left.waypoint_offset);
    const double blend = ratio * ratio * ratio *
      (10.0 + ratio * (-15.0 + 6.0 * ratio));
    const double d = left.d + (right.d - left.d) * blend;
    EXPECT_TRUE(local_planning::SafeCorridorBuilder::corridorContains(samples[index], d));
  }
}

TEST(SafeCorridorTest, RejectsPiecewiseProfileWhenOneSliceBlocksTheSide)
{
  std::vector<local_planning::SafeCorridorSample> samples(3);
  samples[0].feasible_intervals = {{-0.80, -0.30, false}};
  samples[1].feasible_intervals = {{0.20, 0.40, false}};
  samples[2].feasible_intervals = {{-0.75, -0.25, false}};
  EXPECT_TRUE(
    local_planning::SafeCorridorBuilder::buildCorridorGuidedProfiles(
      samples, 0, 2, false, -0.50, 4, 0.01, 2, 8).empty());
}

TEST(SafeCorridorTest, GuidesEntryObstacleAndMergeInsideOneHorizon)
{
  std::vector<local_planning::SafeCorridorSample> samples(9);
  const std::vector<local_planning::LateralInterval> passage{
    {-0.10, 0.10, false}, {-0.10, 0.25, false}, {0.00, 0.40, false},
    {0.25, 0.50, false}, {0.28, 0.52, false}, {0.25, 0.50, false},
    {0.00, 0.45, false}, {-0.10, 0.30, false}, {-0.10, 0.10, false}};
  for (std::size_t index = 0; index < samples.size(); ++index) {
    samples[index].reference.unwrapped_s = 0.1 * static_cast<double>(index);
    samples[index].feasible_intervals = {passage[index]};
  }

  const auto profiles =
    local_planning::SafeCorridorBuilder::buildFullHorizonGuidedProfiles(
    samples, 0, 3, 5, 8, true, 0.0, 0.0, 0.40, 5, 0.01, 2, 12);
  ASSERT_FALSE(profiles.empty());
  const auto & knots = profiles.front();
  // The guided profile must constrain the occupied span, but it deliberately
  // need not place zero-slope knots at both exact collision boundaries. Such
  // boundary knots created curvature cusps on a curved reference line.
  EXPECT_TRUE(
    std::any_of(
      knots.begin(), knots.end(),
      [](const auto & knot) {
        return knot.waypoint_offset >= 3 && knot.waypoint_offset <= 5 && knot.d > 0.0;
      }));
  for (const auto & knot : knots) {
    EXPECT_TRUE(
      local_planning::SafeCorridorBuilder::corridorContains(
        samples[static_cast<std::size_t>(knot.waypoint_offset)], knot.d));
    if (knot.waypoint_offset >= 3 && knot.waypoint_offset <= 5) {
      EXPECT_GE(knot.d, 0.0);
    }
  }
}

}  // namespace
