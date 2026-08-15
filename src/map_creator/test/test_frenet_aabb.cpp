// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "global_planning/clcs_frenet_converter.hpp"
#include "map_creator/frenet_aabb.hpp"

namespace map_creator
{
namespace
{

constexpr double kTolerance = 1.0e-6;
constexpr double kNoBound = 0.0;  // max_center_abs_d <= 0 disables the bound

global_planning::ClcsFrenetConverter::Ptr makeStraightConverter()
{
  global_planning::ClcsFrenetConfig config;
  config.closed_loop = false;
  return global_planning::ClcsFrenetConverter::create(
    {
      {0.0, 0.0, 0.0},
      {5.0, 0.0, 5.0},
      {10.0, 0.0, 10.0},
    },
    config, 1);
}

TEST(FrenetAabb, ProjectsAxisAlignedBoxOnStraightReference)
{
  const auto converter = makeStraightConverter();
  const auto bounds = projectCartesianAabb(*converter, 4.0, 6.0, -0.1, 0.3, kNoBound);

  ASSERT_TRUE(bounds.has_value());
  EXPECT_NEAR(bounds->s_center, 5.0, kTolerance);
  EXPECT_NEAR(bounds->d_center, 0.1, kTolerance);
  EXPECT_NEAR(bounds->s_start, 4.0, kTolerance);
  EXPECT_NEAR(bounds->s_end, 6.0, kTolerance);
  EXPECT_NEAR(bounds->d_right, -0.1, kTolerance);
  EXPECT_NEAR(bounds->d_left, 0.3, kTolerance);
}

TEST(FrenetAabb, RotatesCornersIntoDiagonalTrackFrame)
{
  global_planning::ClcsFrenetConfig config;
  config.closed_loop = false;
  const auto converter = global_planning::ClcsFrenetConverter::create(
    {
      {0.0, 0.0, 0.0},
      {5.0, 5.0, std::sqrt(50.0)},
      {10.0, 10.0, std::sqrt(200.0)},
    },
    config, 1);

  const auto bounds = projectCartesianAabb(*converter, 4.0, 6.0, 4.9, 5.1, kNoBound);

  ASSERT_TRUE(bounds.has_value());
  const double expected_half_extent = (1.0 + 0.1) / std::sqrt(2.0);
  EXPECT_NEAR(bounds->s_start, bounds->s_center - expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->s_end, bounds->s_center + expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->d_right, bounds->d_center - expected_half_extent, kTolerance);
  EXPECT_NEAR(bounds->d_left, bounds->d_center + expected_half_extent, kTolerance);
}

TEST(FrenetAabb, RejectsDegenerateAndNonFiniteAabbs)
{
  const auto converter = makeStraightConverter();

  EXPECT_FALSE(projectCartesianAabb(*converter, 5.0, 5.0, 0.0, 0.0, kNoBound));  // point
  EXPECT_FALSE(projectCartesianAabb(*converter, 6.0, 4.0, -0.1, 0.1, kNoBound));  // min > max
  EXPECT_FALSE(projectCartesianAabb(
      *converter, std::numeric_limits<double>::quiet_NaN(), 6.0, -0.1, 0.1, kNoBound));
}

TEST(FrenetAabb, RejectsCenterBeyondLateralBound)
{
  const auto converter = makeStraightConverter();

  const auto near = projectCartesianAabb(*converter, 4.0, 6.0, 0.9, 1.1, 2.0);
  ASSERT_TRUE(near.has_value());
  EXPECT_NEAR(near->d_center, 1.0, kTolerance);

  EXPECT_FALSE(projectCartesianAabb(*converter, 4.0, 6.0, 2.9, 3.1, 2.0));
}

TEST(FrenetAabb, WrapsSpanAcrossClosedLoopSeam)
{
  // Closed square loop (perimeter 40) starting at (5, 0) so the s=0 seam sits
  // mid-straight (unambiguous +x tangent), not on a 90-degree corner.
  std::vector<global_planning::ReferenceWaypoint> loop;
  double s = 0.0;
  auto push = [&loop, &s](double x, double y) {
      loop.push_back({x, y, s});
      s += 1.0;
    };
  for (double x = 5.0; x < 10.0; x += 1.0) {push(x, 0.0);}
  for (double y = 0.0; y < 10.0; y += 1.0) {push(10.0, y);}
  for (double x = 10.0; x > 0.0; x -= 1.0) {push(x, 10.0);}
  for (double y = 10.0; y > 0.0; y -= 1.0) {push(0.0, y);}
  for (double x = 0.0; x < 5.0; x += 1.0) {push(x, 0.0);}
  global_planning::ClcsFrenetConfig config;
  config.closed_loop = true;
  const auto converter = global_planning::ClcsFrenetConverter::create(loop, config, 1);
  const double track_length = converter->stats().track_length;

  // AABB straddling the s=0 seam at (5, 0).
  const auto bounds = projectCartesianAabb(*converter, 4.5, 5.5, -0.1, 0.1, kNoBound);

  ASSERT_TRUE(bounds.has_value());
  EXPECT_NEAR(bounds->s_start, track_length - 0.5, 1.0e-3);
  EXPECT_NEAR(bounds->s_end, 0.5, 1.0e-3);
  // Painter convention: a wrapped span accumulates track_length until positive.
  double span = bounds->s_end - bounds->s_start;
  while (span < 0.0) {span += track_length;}
  EXPECT_NEAR(span, 1.0, 1.0e-3);
}

}  // namespace
}  // namespace map_creator
