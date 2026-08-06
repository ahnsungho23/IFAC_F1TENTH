// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "map_creator/obstacle_ledger.hpp"

namespace
{

f110_msgs::msg::Obstacle makeObstacle(int id, double s, double d, bool is_static = true)
{
  f110_msgs::msg::Obstacle ob;
  ob.id = id;
  ob.s_center = s;
  ob.d_center = d;
  ob.s_start = s - 0.1;
  ob.s_end = s + 0.1;
  ob.d_right = d - 0.1;
  ob.d_left = d + 0.1;
  ob.is_static = is_static;
  return ob;
}

}  // namespace

TEST(ObstacleLedger, MatchesRepeatedObservations)
{
  map_creator::ObstacleLedger ledger;
  ledger.setTrackLength(35.3);
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.addObservations({makeObstacle(1, 10.0, 0.2)}, 0);
  ledger.addObservations({makeObstacle(2, 10.4, 0.15)}, 0);  // same physical obstacle
  ledger.addObservations({makeObstacle(3, 20.0, -0.3)}, 0);  // different obstacle

  EXPECT_EQ(ledger.size(), 2U);
  const auto confirmed = ledger.confirmed(2);
  ASSERT_EQ(confirmed.size(), 1U);
  EXPECT_EQ(confirmed[0].observation_count, 2);
  EXPECT_NEAR(confirmed[0].obstacle.s_center, 10.4, 1e-9);  // latest snapshot kept
}

TEST(ObstacleLedger, WrapAwareMatchingAcrossStartLine)
{
  map_creator::ObstacleLedger ledger;
  ledger.setTrackLength(35.3);
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.addObservations({makeObstacle(1, 35.0, 0.0)}, 0);
  ledger.addObservations({makeObstacle(2, 0.2, 0.0)}, 1);  // wraps: |35.0 - 0.2| = 0.5

  EXPECT_EQ(ledger.size(), 1U);
}

TEST(ObstacleLedger, IgnoresDynamicObstacles)
{
  map_creator::ObstacleLedger ledger;
  ledger.setTrackLength(35.3);
  ledger.addObservations({makeObstacle(1, 5.0, 0.0, false)}, 0);
  EXPECT_TRUE(ledger.empty());
}

TEST(ObstacleLedger, RemovalHysteresis)
{
  map_creator::ObstacleLedger ledger;
  ledger.setTrackLength(35.3);
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.addObservations({makeObstacle(1, 10.0, 0.0)}, 1);
  ledger.addObservations({makeObstacle(2, 20.0, 0.0)}, 1);
  // Lap 2: only the second obstacle is still observed.
  ledger.addObservations({makeObstacle(2, 20.0, 0.0)}, 2);

  EXPECT_TRUE(ledger.removalCandidates(2, 2).empty());   // 1 missed lap only
  const auto removable = ledger.removalCandidates(3, 2);  // 2 missed laps
  ASSERT_EQ(removable.size(), 1U);
  ledger.removeAt(removable);
  EXPECT_EQ(ledger.size(), 1U);
  EXPECT_NEAR(ledger.confirmed(1)[0].obstacle.s_center, 20.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
