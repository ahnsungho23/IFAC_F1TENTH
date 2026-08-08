#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "global_planning/clcs_frenet_converter.hpp"

namespace
{

using global_planning::ClcsContinuityState;
using global_planning::ClcsConversionInput;
using global_planning::ClcsFrenetConfig;
using global_planning::ClcsFrenetConverter;
using global_planning::ReferenceWaypoint;

constexpr double kTol = 1.0e-2;
constexpr double kPi = 3.14159265358979323846;

ClcsFrenetConfig baseConfig(bool closed_loop = false)
{
  ClcsFrenetConfig config;
  config.closed_loop = closed_loop;
  config.projection_domain_limit = 50.0;
  config.max_projection_distance = 50.0;
  config.projection_domain_eps2 = 0.0;
  config.tangent_epsilon = 0.05;
  config.min_path_length = 0.1;
  config.velocity_frame = global_planning::VelocityFrame::kBody;
  return config;
}

std::vector<ReferenceWaypoint> straightPath()
{
  return {
    {0.0, 0.0, 0.0},
    {5.0, 0.0, 5.0},
    {10.0, 0.0, 10.0},
  };
}

ClcsConversionInput input(double x, double y, double yaw = 0.0)
{
  ClcsConversionInput value;
  value.x = x;
  value.y = y;
  value.yaw = yaw;
  value.linear_x = 1.0;
  value.linear_y = 0.0;
  value.yaw_rate = 0.2;
  return value;
}

// Tight 180 deg hairpin like the ifac_track one: two parallel legs 1.4 m
// apart joined by a ~0.7 m radius right-hand U-turn, ~0.25 m spacing.
// Entry leg: segments 0..19, apex: ~20..27, exit leg: ~28..47.
std::vector<ReferenceWaypoint> hairpinPath()
{
  std::vector<ReferenceWaypoint> path;
  double s = 0.0;
  const auto append = [&path, &s](double x, double y) {
      if (!path.empty()) {
        s += std::hypot(x - path.back().x, y - path.back().y);
      }
      path.push_back({x, y, s});
    };

  // Entry leg: y = 0.7, x = 0 -> 5 (heading +x).
  for (double x = 0.0; x <= 5.0; x += 0.25) {
    append(x, 0.7);
  }
  // Apex: right-hand semicircle around (5, 0) with radius 0.7.
  const double radius = 0.7;
  for (int k = 1; k < 8; ++k) {
    const double angle = (90.0 - 22.5 * k) * kPi / 180.0;
    append(5.0 + radius * std::cos(angle), radius * std::sin(angle));
  }
  // Exit leg: y = -0.7, x = 5 -> 0 (heading -x).
  for (double x = 5.0; x >= 0.0; x -= 0.25) {
    append(x, -0.7);
  }
  return path;
}

// 20 m straight with 1 m spacing so a +-1 m monotonic window covers only a
// few of the 20 segments.
std::vector<ReferenceWaypoint> longStraightPath()
{
  std::vector<ReferenceWaypoint> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back({static_cast<double>(i), 0.0, static_cast<double>(i)});
  }
  return path;
}

std::vector<ReferenceWaypoint> squarePath()
{
  return {
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 10.0},
    {10.0, 10.0, 20.0},
    {0.0, 10.0, 30.0},
  };
}

}  // namespace

TEST(ClcsFrenetConverter, StraightPathCenterLeftRight)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);

  const auto center = converter->convert(input(5.0, 0.0));
  ASSERT_TRUE(center.valid) << center.error_message;
  EXPECT_NEAR(center.s, 5.0, kTol);
  EXPECT_NEAR(center.d, 0.0, kTol);

  const auto left = converter->convert(input(5.0, 2.0));
  ASSERT_TRUE(left.valid) << left.error_message;
  EXPECT_NEAR(left.s, 5.0, kTol);
  EXPECT_NEAR(left.d, 2.0, kTol);

  const auto right = converter->convert(input(5.0, -2.0));
  ASSERT_TRUE(right.valid) << right.error_message;
  EXPECT_NEAR(right.s, 5.0, kTol);
  EXPECT_NEAR(right.d, -2.0, kTol);
}

TEST(ClcsFrenetConverter, ReversedPathFlipsDSign)
{
  const std::vector<ReferenceWaypoint> path = {
    {10.0, 0.0, 0.0},
    {5.0, 0.0, 5.0},
    {0.0, 0.0, 10.0},
  };
  auto converter = ClcsFrenetConverter::create(path, baseConfig(), 1);
  const auto result = converter->convert(input(5.0, 2.0, 3.14159265358979323846));
  ASSERT_TRUE(result.valid) << result.error_message;
  EXPECT_NEAR(result.s, 5.0, kTol);
  EXPECT_NEAR(result.d, -2.0, kTol);
}

TEST(ClcsFrenetConverter, DuplicateWaypointsAreRemoved)
{
  const std::vector<ReferenceWaypoint> path = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {5.0, 0.0, 5.0},
    {10.0, 0.0, 10.0},
  };
  auto converter = ClcsFrenetConverter::create(path, baseConfig(), 7);
  EXPECT_EQ(converter->stats().removed_duplicate_count, 1u);
  const auto result = converter->convert(input(5.0, 1.0));
  ASSERT_TRUE(result.valid) << result.error_message;
  EXPECT_NEAR(result.d, 1.0, kTol);
}

TEST(ClcsFrenetConverter, InvalidPathThrows)
{
  EXPECT_THROW(
    ClcsFrenetConverter::create({}, baseConfig(), 1),
    std::runtime_error);

  EXPECT_THROW(
    ClcsFrenetConverter::create({{0.0, 0.0, 0.0}}, baseConfig(), 1),
    std::runtime_error);

  const std::vector<ReferenceWaypoint> same_points = {
    {1.0, 1.0, 0.0},
    {1.0, 1.0, 0.0},
    {1.0, 1.0, 0.0},
  };
  EXPECT_THROW(
    ClcsFrenetConverter::create(same_points, baseConfig(), 1),
    std::runtime_error);
}

TEST(ClcsFrenetConverter, NonFiniteInputFailsWithoutThrowing)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);
  auto bad_input = input(5.0, 0.0);
  bad_input.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = converter->convert(bad_input);
  EXPECT_FALSE(result.valid);
}

TEST(ClcsFrenetConverter, HeadingErrorAndVelocity)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);

  auto zero = input(5.0, 0.0, 0.0);
  zero.linear_x = 2.0;
  const auto zero_result = converter->convert(zero);
  ASSERT_TRUE(zero_result.valid) << zero_result.error_message;
  EXPECT_NEAR(zero_result.heading_error, 0.0, kTol);
  EXPECT_NEAR(zero_result.v_s, 2.0, kTol);
  EXPECT_NEAR(zero_result.v_d, 0.0, kTol);

  auto left_heading = input(5.0, 0.0, 0.5 * 3.14159265358979323846);
  left_heading.linear_x = 2.0;
  const auto left_result = converter->convert(left_heading);
  ASSERT_TRUE(left_result.valid) << left_result.error_message;
  EXPECT_NEAR(left_result.heading_error, 0.5 * 3.14159265358979323846, kTol);
  EXPECT_NEAR(left_result.v_s, 0.0, kTol);
  EXPECT_NEAR(left_result.v_d, 2.0, kTol);
}

TEST(ClcsFrenetConverter, ClosedLoopSIsWrapped)
{
  const std::vector<ReferenceWaypoint> square = {
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 10.0},
    {10.0, 10.0, 20.0},
    {0.0, 10.0, 30.0},
  };
  auto converter = ClcsFrenetConverter::create(square, baseConfig(true), 1);
  EXPECT_GT(converter->stats().track_length, 39.0);
  EXPECT_LT(converter->stats().track_length, 41.0);

  const auto result = converter->convert(input(0.0, 0.1));
  ASSERT_TRUE(result.valid) << result.error_message;
  EXPECT_GE(result.s, 0.0);
  EXPECT_LT(result.s, converter->stats().track_length);
}

TEST(ClcsFrenetConverter, CartesianFrenetCartesianReconstruction)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);
  const auto result = converter->convert(input(4.0, 1.25));
  ASSERT_TRUE(result.valid) << result.error_message;
  EXPECT_LT(result.reconstruction_error, 1.0e-6);
}

TEST(ClcsFrenetConverter, ConversionBenchmark)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);
  double max_us = 0.0;
  double sum_us = 0.0;

  for (int i = 0; i < 1000; ++i) {
    const auto result = converter->convert(input(1.0 + 0.008 * i, 0.5));
    ASSERT_TRUE(result.valid) << result.error_message;
    max_us = std::max(max_us, result.conversion_time_us);
    sum_us += result.conversion_time_us;
  }

  std::cout << "CLCS conversion benchmark: avg_us=" << sum_us / 1000.0
            << " max_us=" << max_us << std::endl;
}

TEST(ClcsFrenetConverter, TrackedMatchesStatelessOnStraight)
{
  auto converter = ClcsFrenetConverter::create(straightPath(), baseConfig(), 1);
  ClcsContinuityState state;

  // Step 0.4 m so every pose stays inside the +-1 m monotonic window.
  for (double x = 0.5; x < 9.5; x += 0.4) {
    const auto expected = converter->convert(input(x, 0.3));
    const auto tracked = converter->convertTracked(input(x, 0.3), state);
    ASSERT_TRUE(expected.valid) << expected.error_message;
    ASSERT_TRUE(tracked.valid) << tracked.error_message;
    EXPECT_FALSE(tracked.reacquired);
    EXPECT_EQ(tracked.segment_index, expected.segment_index);
    EXPECT_NEAR(tracked.s, expected.s, 1.0e-6);
    EXPECT_NEAR(tracked.d, expected.d, 1.0e-6);
  }
}

TEST(ClcsFrenetConverter, TrackedHairpinRejectsOppositeLeg)
{
  auto converter = ClcsFrenetConverter::create(hairpinPath(), baseConfig(false), 1);

  // Drive forward along the entry leg while the pose drifts from the
  // reference (y = 0.7) past the midline between the legs (y = -0.05). The
  // stateless min-|d| search flips to the exit leg once the pose is closer to
  // it; the tracked conversion must stay on the entry leg because the exit
  // leg is far outside the monotonic s-window.
  ClcsContinuityState state;
  int previous_segment = -1;
  int last_segment = -1;
  for (double x = 0.5; x <= 4.5; x += 0.25) {
    const double y = (x < 1.5) ? (0.7 - 0.75 * (x - 0.5)) : -0.05;
    const auto result = converter->convertTracked(input(x, y), state);
    ASSERT_TRUE(result.valid) << result.error_message;
    EXPECT_LE(result.segment_index, 19) << "flipped off the entry leg at x=" << x;
    if (previous_segment >= 0) {
      EXPECT_GE(result.segment_index, previous_segment)
        << "segment moved backward at x=" << x;
      EXPECT_LE(result.segment_index - previous_segment, 2)
        << "segment jumped at x=" << x;
    }
    previous_segment = result.segment_index;
    last_segment = result.segment_index;
  }
  EXPECT_GE(last_segment, 15);

  // The final pose really is in the ambiguous zone for the stateless search:
  // it picks the opposite (exit) leg.
  const auto stateless = converter->convert(input(4.5, -0.05));
  ASSERT_TRUE(stateless.valid) << stateless.error_message;
  EXPECT_GT(stateless.segment_index, 19)
    << "test pose is not in the ambiguous zone";
}

TEST(ClcsFrenetConverter, TrackedWindowWrapsAcrossSeam)
{
  auto converter = ClcsFrenetConverter::create(squarePath(), baseConfig(true), 1);
  ClcsContinuityState state;

  // Drive down the closing segment (left edge, x=0, heading -y) toward the
  // seam at the origin, stepping < 1 m in arc length each time.
  const auto fix = converter->convertTracked(input(0.05, 3.0), state);
  ASSERT_TRUE(fix.valid) << fix.error_message;
  EXPECT_EQ(fix.segment_index, 3);
  EXPECT_NEAR(fix.s, 37.0, 0.2);

  ASSERT_TRUE(converter->convertTracked(input(0.05, 2.2), state).valid);
  ASSERT_TRUE(converter->convertTracked(input(0.05, 1.4), state).valid);
  const auto near_seam = converter->convertTracked(input(0.05, 0.6), state);
  ASSERT_TRUE(near_seam.valid) << near_seam.error_message;
  EXPECT_EQ(near_seam.segment_index, 3);
  EXPECT_NEAR(near_seam.s, 39.4, 0.2);

  // The +1 m window from s ~ 39.4 must wrap across the seam onto segment 0.
  const auto past_seam = converter->convertTracked(input(0.3, 0.05), state);
  ASSERT_TRUE(past_seam.valid) << past_seam.error_message;
  EXPECT_FALSE(past_seam.reacquired);
  EXPECT_EQ(past_seam.segment_index, 0);
  EXPECT_NEAR(past_seam.s, 0.3, 0.15);

  const auto further = converter->convertTracked(input(1.0, 0.05), state);
  ASSERT_TRUE(further.valid) << further.error_message;
  EXPECT_EQ(further.segment_index, 0);
  EXPECT_NEAR(further.s, 1.0, 0.15);
}

TEST(ClcsFrenetConverter, TrackedFailsClosedOnTeleport)
{
  auto config = baseConfig();
  config.reacquire_after_misses = 0;  // strict skidpad semantics
  auto converter = ClcsFrenetConverter::create(longStraightPath(), config, 1);
  ClcsContinuityState state;

  ASSERT_TRUE(converter->convertTracked(input(2.0, 0.1), state).valid);
  ASSERT_TRUE(state.initialized);
  EXPECT_NEAR(state.s_prev, 2.0, kTol);

  // Teleport far ahead: the window [1, 3] has no candidate at x=15.3 and the
  // converter must fail closed forever (no global fallback).
  for (int i = 1; i <= 3; ++i) {
    const auto lost = converter->convertTracked(input(15.3, 0.1), state);
    EXPECT_FALSE(lost.valid);
    EXPECT_FALSE(lost.reacquired);
    EXPECT_NE(lost.error_message.find("monotonic window"), std::string::npos);
    EXPECT_EQ(state.consecutive_misses, i);
    EXPECT_NEAR(state.s_prev, 2.0, kTol);  // state untouched
  }

  // Returning into the window resumes tracking and clears the miss counter.
  const auto resumed = converter->convertTracked(input(2.5, 0.1), state);
  ASSERT_TRUE(resumed.valid) << resumed.error_message;
  EXPECT_NEAR(resumed.s, 2.5, kTol);
  EXPECT_EQ(state.consecutive_misses, 0);
}

TEST(ClcsFrenetConverter, TrackedReacquiresAfterMisses)
{
  auto config = baseConfig();
  config.reacquire_after_misses = 2;
  auto converter = ClcsFrenetConverter::create(longStraightPath(), config, 1);
  ClcsContinuityState state;

  ASSERT_TRUE(converter->convertTracked(input(2.0, 0.1), state).valid);

  const auto miss = converter->convertTracked(input(15.3, 0.1), state);
  EXPECT_FALSE(miss.valid);
  EXPECT_EQ(state.consecutive_misses, 1);

  // Second consecutive miss reaches the threshold: one loud global re-search.
  const auto reacquired = converter->convertTracked(input(15.3, 0.1), state);
  ASSERT_TRUE(reacquired.valid) << reacquired.error_message;
  EXPECT_TRUE(reacquired.reacquired);
  EXPECT_NEAR(reacquired.s, 15.3, kTol);
  EXPECT_EQ(state.consecutive_misses, 0);

  const auto resumed = converter->convertTracked(input(15.8, 0.1), state);
  ASSERT_TRUE(resumed.valid) << resumed.error_message;
  EXPECT_FALSE(resumed.reacquired);
  EXPECT_NEAR(resumed.s, 15.8, kTol);
}

TEST(ClcsFrenetConverter, TrackedBackwardWithinTolerance)
{
  auto converter = ClcsFrenetConverter::create(longStraightPath(), baseConfig(), 1);
  ClcsContinuityState state;

  ASSERT_TRUE(converter->convertTracked(input(5.0, 0.1), state).valid);

  // 0.8 m backward is inside backward_tolerance = 1.0 m.
  const auto back = converter->convertTracked(input(4.2, 0.1), state);
  ASSERT_TRUE(back.valid) << back.error_message;
  EXPECT_NEAR(back.s, 4.2, kTol);

  // 1.3 m backward from s_prev = 4.2 leaves the window [3.2, 5.2].
  const auto too_far_back = converter->convertTracked(input(2.9, 0.1), state);
  EXPECT_FALSE(too_far_back.valid);
  EXPECT_NEAR(state.s_prev, 4.2, kTol);
}

TEST(ClcsFrenetConverter, TrackedInitialSeedWindow)
{
  auto config = baseConfig();
  config.initial_seed_window = 3.0;
  config.reacquire_after_misses = 0;
  auto converter = ClcsFrenetConverter::create(longStraightPath(), config, 1);
  ClcsContinuityState state;

  // First fix outside [0, 3]: fail closed, no state.
  const auto outside = converter->convertTracked(input(10.0, 0.1), state);
  EXPECT_FALSE(outside.valid);
  EXPECT_NE(outside.error_message.find("initial seed window"), std::string::npos);
  EXPECT_FALSE(state.initialized);

  const auto seeded = converter->convertTracked(input(1.5, 0.1), state);
  ASSERT_TRUE(seeded.valid) << seeded.error_message;
  EXPECT_NEAR(seeded.s, 1.5, kTol);
  EXPECT_TRUE(state.initialized);
}

TEST(ClcsFrenetConverter, TrackedEuclideanGateRejectsFarFix)
{
  auto config = baseConfig();
  config.tracked_max_projection_distance = 0.5;
  config.reacquire_after_misses = 0;
  auto converter = ClcsFrenetConverter::create(straightPath(), config, 1);
  ClcsContinuityState state;

  ASSERT_TRUE(converter->convertTracked(input(5.0, 0.3), state).valid);

  // Inside the s-window but laterally beyond the tracked gate: miss.
  const auto too_far = converter->convertTracked(input(5.3, 0.9), state);
  EXPECT_FALSE(too_far.valid);
  EXPECT_NE(
    too_far.error_message.find("tracked_max_projection_distance"), std::string::npos);
  EXPECT_NEAR(state.s_prev, 5.0, kTol);
}
