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

TEST(ObstacleLedger, MatchesRepeatedObservationsBySuppliedFrenetGeometry)
{
  map_creator::ObstacleLedger ledger;
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.2)}, 0);
  ledger.updateSnapshot({makeObstacle(9, 10.4, 0.15), makeObstacle(3, 20.0, -0.3)}, 0);

  EXPECT_EQ(ledger.size(), 2U);
  const auto snapshot = ledger.snapshot();
  ASSERT_EQ(snapshot.size(), 2U);
  EXPECT_EQ(snapshot[0].observation_count, 2);
  EXPECT_EQ(snapshot[0].obstacle.id, 9);  // latest snapshot kept; ID is not the authority
  EXPECT_NEAR(snapshot[0].obstacle.s_center, 10.4, 1e-9);
  EXPECT_NEAR(snapshot[0].obstacle.d_center, 0.15, 1e-9);
}

TEST(ObstacleLedger, MatchesAcrossStartLineWithSuppliedTrackLength)
{
  map_creator::ObstacleLedger ledger;
  ledger.setTrackLength(35.3);
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.updateSnapshot({makeObstacle(1, 35.0, 0.0)}, 0);
  ledger.updateSnapshot({makeObstacle(2, 0.2, 0.0)}, 1);

  EXPECT_EQ(ledger.size(), 1U);
  EXPECT_EQ(ledger.snapshot()[0].observation_count, 2);
}

TEST(ObstacleLedger, KeepsNearbyEntriesFromSameSnapshotDistinct)
{
  map_creator::ObstacleLedger ledger;

  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.2), makeObstacle(2, 10.0, 0.2)}, 0);

  EXPECT_EQ(ledger.size(), 2U);
}

TEST(ObstacleLedger, DoesNotMatchSameIdOutsideGeometryThresholds)
{
  map_creator::ObstacleLedger ledger;
  ledger.setMatchThresholds(1.0, 0.3);

  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.0)}, 0);
  ledger.updateSnapshot({makeObstacle(1, 20.0, 0.0)}, 1);

  EXPECT_EQ(ledger.size(), 2U);
}

TEST(ObstacleLedger, IgnoresDynamicObstacles)
{
  map_creator::ObstacleLedger ledger;
  ledger.updateSnapshot({makeObstacle(1, 5.0, 0.0, false)}, 0);
  EXPECT_TRUE(ledger.empty());
}

TEST(ObstacleLedger, RemovalHysteresis)
{
  map_creator::ObstacleLedger ledger;

  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.0), makeObstacle(2, 20.0, 0.0)}, 1);
  // Lap 2 authoritative snapshot: only the second obstacle remains.
  ledger.updateSnapshot({makeObstacle(2, 20.0, 0.0)}, 2);

  EXPECT_TRUE(ledger.removalCandidates(2, 2).empty());  // absence just reported
  const auto removable = ledger.removalCandidates(3, 2);  // 1 absent lap only
  EXPECT_TRUE(removable.empty());
  const auto removable_after_hysteresis = ledger.removalCandidates(4, 2);
  ASSERT_EQ(removable_after_hysteresis.size(), 1U);
  ledger.removeAt(removable_after_hysteresis);
  EXPECT_EQ(ledger.size(), 1U);
  EXPECT_NEAR(ledger.snapshot()[0].obstacle.s_center, 20.0, 1e-9);
}

TEST(ObstacleLedger, TransportSilenceDoesNotMeanObstacleAbsence)
{
  map_creator::ObstacleLedger ledger;
  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.0)}, 1);

  EXPECT_TRUE(ledger.removalCandidates(100, 2).empty());
}

TEST(ObstacleLedger, ReappearanceCancelsRemovalHysteresis)
{
  map_creator::ObstacleLedger ledger;
  ledger.updateSnapshot({makeObstacle(1, 10.0, 0.0)}, 1);
  ledger.updateSnapshot({}, 2);
  ledger.updateSnapshot({makeObstacle(1, 10.1, 0.0)}, 3);

  EXPECT_TRUE(ledger.removalCandidates(100, 2).empty());
  ASSERT_EQ(ledger.snapshot().size(), 1U);
  EXPECT_NEAR(ledger.snapshot()[0].obstacle.s_center, 10.1, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
