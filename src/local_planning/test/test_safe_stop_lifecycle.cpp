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

#include <vector>

#include "local_planning/safe_stop_lifecycle.hpp"

namespace local_planning
{
namespace
{

constexpr double kTrackLength = 100.0;
constexpr double kPassMargin = 0.4;
constexpr double kStoppedSpeed = 0.0625;
constexpr int kReleaseCycles = 8;

SafeStopActivation activation()
{
  SafeStopActivation value;
  value.obstacle_ids = {29};
  value.obstacle_sequence = 41;
  value.obstacle_source_stamp_ns = 10170000000LL;
  value.obstacle_s_start = 15.0;
  value.obstacle_s_end = 15.6;
  value.safe_stop_target_s = 14.6;
  value.activation_ego_s = 10.0;
  value.activation_timestamp_ns = 10170000000LL;
  value.danger_start_distance_m = 5.0;
  value.danger_end_distance_m = 5.6;
  value.safe_stop_target_distance_m = 4.6;
  return value;
}

TEST(SafeStopLifecycle, EmptyDropoutKeepsObstacleAheadLatchedBeyondEightCycles)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 11.0;
  input.ego_speed_mps = 3.0;
  input.static_obstacles_empty = true;

  for (int cycle = 0; cycle < 20; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_FALSE(decision.release_authorized);
    EXPECT_FALSE(decision.raceline_global_handoff_allowed);
    EXPECT_EQ(decision.stopped_clear_count, 0);
  }
  EXPECT_TRUE(lifecycle.active());
}

TEST(SafeStopLifecycle, ReleasesAfterEgoPassesDangerRegionWithMargin)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 16.01;
  input.ego_speed_mps = 1.0;
  input.static_obstacles_empty = true;
  input.obstacle_sequence = 50;

  const auto decision = lifecycle.evaluate(
    input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
  EXPECT_TRUE(decision.obstacle_passed);
  EXPECT_TRUE(decision.release_authorized);
  EXPECT_EQ(decision.release_reason, SafeStopReleaseReason::kObstaclePassed);
  EXPECT_TRUE(decision.raceline_global_handoff_allowed);
}

TEST(SafeStopLifecycle, ReleasesToSelectableHardValidAvoidanceOnlyAfterDebounce)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 11.0;
  input.ego_speed_mps = 2.0;
  input.static_obstacles_empty = false;
  input.hard_valid_avoidance_for_latched_obstacle = true;
  input.state_can_select_avoidance = true;

  for (int cycle = 1; cycle <= kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_EQ(decision.release_authorized, cycle == kReleaseCycles);
    if (cycle == kReleaseCycles) {
      EXPECT_EQ(decision.release_reason, SafeStopReleaseReason::kValidAvoidance);
      EXPECT_FALSE(decision.raceline_global_handoff_allowed);
    }
  }
}

TEST(SafeStopLifecycle, EmptyFramesCannotProveStoppedCorridorClear)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 14.5;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = true;
  input.explicit_forward_corridor_clear = true;

  for (int cycle = 0; cycle < 20; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_FALSE(decision.release_condition_c);
    EXPECT_EQ(decision.stopped_clear_count, 0);
  }
}

TEST(SafeStopLifecycle, ReleasesStoppedVehicleAfterExplicitPersistentClearFrames)
{
  SafeStopLifecycle lifecycle;
  lifecycle.activate(activation());
  SafeStopCycleInput input;
  input.ego_s = 14.5;
  input.ego_speed_mps = 0.0;
  input.static_obstacles_empty = false;
  input.explicit_forward_corridor_clear = true;

  for (int cycle = 1; cycle <= kReleaseCycles; ++cycle) {
    input.obstacle_sequence = 42 + cycle;
    const auto decision = lifecycle.evaluate(
      input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles);
    EXPECT_EQ(decision.release_condition_c, cycle == kReleaseCycles);
    if (cycle == kReleaseCycles) {
      EXPECT_EQ(
        decision.release_reason, SafeStopReleaseReason::kStoppedCorridorClear);
      EXPECT_TRUE(decision.raceline_global_handoff_allowed);
    }
  }
}

TEST(SafeStopLifecycle, DeterministicForRepeatedInputSequence)
{
  std::vector<SafeStopCycleDecision> first;
  std::vector<SafeStopCycleDecision> second;
  for (auto * output : {&first, &second}) {
    SafeStopLifecycle lifecycle;
    lifecycle.activate(activation());
    SafeStopCycleInput input;
    input.ego_s = 11.0;
    input.ego_speed_mps = 2.0;
    input.static_obstacles_empty = false;
    input.hard_valid_avoidance_for_latched_obstacle = true;
    input.state_can_select_avoidance = true;
    for (int cycle = 0; cycle < kReleaseCycles; ++cycle) {
      input.obstacle_sequence = 42 + cycle;
      output->push_back(lifecycle.evaluate(
          input, kTrackLength, kPassMargin, kStoppedSpeed, kReleaseCycles));
    }
  }

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].obstacle_passed, second[index].obstacle_passed);
    EXPECT_EQ(first[index].feasible_avoidance_count, second[index].feasible_avoidance_count);
    EXPECT_EQ(first[index].release_authorized, second[index].release_authorized);
    EXPECT_EQ(first[index].release_reason, second[index].release_reason);
  }
}

}  // namespace
}  // namespace local_planning
