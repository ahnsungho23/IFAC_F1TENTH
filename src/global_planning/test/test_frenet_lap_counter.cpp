// Copyright 2026 2026_IFAC contributors

#include <limits>
#include <stdexcept>

#include "global_planning/frenet_lap_counter.hpp"
#include "gtest/gtest.h"

namespace global_planning
{
namespace
{

TEST(FrenetLapCounter, CountsFinishToStartWrap)
{
  FrenetLapCounter counter(10.0, 0.5, 3.0);

  EXPECT_FALSE(counter.update(12.0, 0.0));
  EXPECT_TRUE(counter.update(0.2, 4.0));
  EXPECT_EQ(counter.lapCount(), 1);
}

TEST(FrenetLapCounter, DoesNotCountOrdinaryDecreaseOrRepeatedStartSamples)
{
  FrenetLapCounter counter(10.0, 0.5, 0.0);

  EXPECT_FALSE(counter.update(9.0, 0.0));
  EXPECT_FALSE(counter.update(0.2, 1.0));
  EXPECT_FALSE(counter.update(0.1, 2.0));
  EXPECT_EQ(counter.lapCount(), 0);
}

TEST(FrenetLapCounter, EnforcesMinimumTimeBetweenObservedWraps)
{
  FrenetLapCounter counter(10.0, 0.5, 3.0);

  EXPECT_FALSE(counter.update(12.0, 0.0));
  EXPECT_FALSE(counter.update(0.2, 1.0));
  EXPECT_FALSE(counter.update(12.0, 2.0));
  EXPECT_FALSE(counter.update(0.2, 3.0));
  EXPECT_FALSE(counter.update(12.0, 6.0));
  EXPECT_TRUE(counter.update(0.2, 7.0));
  EXPECT_EQ(counter.lapCount(), 1);
}

TEST(FrenetLapCounter, IgnoresNonFiniteSamples)
{
  FrenetLapCounter counter(10.0, 0.5, 0.0);

  EXPECT_FALSE(counter.update(12.0, 0.0));
  EXPECT_FALSE(counter.update(std::numeric_limits<double>::quiet_NaN(), 1.0));
  EXPECT_TRUE(counter.update(0.2, 2.0));
  EXPECT_EQ(counter.lapCount(), 1);
}

TEST(FrenetLapCounter, ResetsDetectionClockWhenTimeMovesBackward)
{
  FrenetLapCounter counter(10.0, 0.5, 3.0, 2);

  EXPECT_FALSE(counter.update(12.0, 10.0));
  EXPECT_FALSE(counter.update(0.2, 5.0));
  EXPECT_EQ(counter.lapCount(), 2);
  EXPECT_FALSE(counter.update(12.0, 6.0));
  EXPECT_TRUE(counter.update(0.2, 9.0));
  EXPECT_EQ(counter.lapCount(), 3);
}

TEST(FrenetLapCounter, RejectsInvalidConfiguration)
{
  EXPECT_THROW(FrenetLapCounter(0.5, 0.5, 3.0), std::invalid_argument);
  EXPECT_THROW(FrenetLapCounter(10.0, 0.5, -1.0), std::invalid_argument);
  EXPECT_THROW(FrenetLapCounter(10.0, 0.5, 3.0, -1), std::invalid_argument);
}

}  // namespace
}  // namespace global_planning
