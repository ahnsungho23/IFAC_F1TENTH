#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "global_planner/clcs_frenet_converter.hpp"

namespace
{

using global_planner::ClcsConversionInput;
using global_planner::ClcsFrenetConfig;
using global_planner::ClcsFrenetConverter;
using global_planner::ReferenceWaypoint;

constexpr double kTol = 1.0e-2;

ClcsFrenetConfig baseConfig(bool closed_loop = false)
{
  ClcsFrenetConfig config;
  config.closed_loop = closed_loop;
  config.projection_domain_limit = 50.0;
  config.max_projection_distance = 50.0;
  config.projection_domain_eps2 = 0.0;
  config.tangent_epsilon = 0.05;
  config.min_path_length = 0.1;
  config.velocity_frame = global_planner::VelocityFrame::kBody;
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
