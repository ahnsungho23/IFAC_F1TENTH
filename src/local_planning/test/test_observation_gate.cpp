// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "local_planning/observation_gate.hpp"

namespace local_planning
{

TEST(ObservationGate, UsesRelaxedNearRequirements)
{
  const auto gate = interpolateObservationGate(2.0, 3.0, 8.0, 1, 3, 0.0, 0.15);
  EXPECT_EQ(gate.observation_count, 1);
  EXPECT_DOUBLE_EQ(gate.minimum_duration_sec, 0.0);
}

TEST(ObservationGate, InterpolatesBetweenNearAndFar)
{
  const auto gate = interpolateObservationGate(5.5, 3.0, 8.0, 1, 3, 0.0, 0.15);
  EXPECT_EQ(gate.observation_count, 2);
  EXPECT_NEAR(gate.minimum_duration_sec, 0.075, 1.0e-12);
}

TEST(ObservationGate, KeepsConservativeFarRequirements)
{
  const auto gate = interpolateObservationGate(10.0, 3.0, 8.0, 1, 3, 0.0, 0.15);
  EXPECT_EQ(gate.observation_count, 3);
  EXPECT_DOUBLE_EQ(gate.minimum_duration_sec, 0.15);
}

}  // namespace local_planning
