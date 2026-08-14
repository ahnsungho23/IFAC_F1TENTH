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
#include <cstddef>
#include <limits>
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

f110_msgs::msg::WpntArray makeCircularReference(
  int count = 200, double radius = 5.0)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  reference.wpnts.reserve(static_cast<std::size_t>(count));
  const double spacing = 2.0 * M_PI * radius / static_cast<double>(count);
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = radius * std::cos(angle);
    waypoint.y_m = radius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * M_PI;
    waypoint.kappa_radpm = 1.0 / radius;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = 1.5;
    waypoint.d_right = 1.5;
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
  parameters.vehicle_half_width_m = 0.12;
  parameters.safety_margin_m = 0.03;
  parameters.tracking_error_reserve_m = 0.0;
  parameters.maximum_curvature_radpm = 5.0;
  parameters.maximum_curvature_rate_radpm2 = 50.0;
  // Legacy safety-constraint tests isolate one target. Multi-target behavior is exercised by
  // dedicated candidate-generation tests below.
  parameters.target_d_candidate_count = 1;
  return parameters;
}

f110_msgs::msg::WpntArray makeStraightCandidate(
  const f110_msgs::msg::WpntArray & reference,
  double d,
  double yaw,
  std::size_t count = 20U)
{
  f110_msgs::msg::WpntArray path;
  path.header = reference.header;
  count = std::min(count, reference.wpnts.size());
  path.wpnts.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto waypoint = reference.wpnts[index];
    waypoint.id = static_cast<int32_t>(index);
    waypoint.d_m = d;
    waypoint.x_m = reference.wpnts[index].x_m;
    waypoint.y_m = reference.wpnts[index].y_m + d;
    waypoint.psi_rad = yaw;
    waypoint.kappa_radpm = 0.0;
    path.wpnts.push_back(waypoint);
  }
  return path;
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

TEST(RacelineSplinePlanner, P3NonpositiveEntryBoundaryFailsClosedWithoutThrowing)
{
  const auto reference = makeStraightReference(300, 0.1, 1.5, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  P3ShadowResult result;
  EXPECT_NO_THROW(
    result = planner.evaluateP3Shadow(
      EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(41, 0.2)},
      100, 1U, 1U, "NONPOSITIVE_BOUNDARY_TEST"));
  EXPECT_TRUE(result.invoked);
  EXPECT_FALSE(result.would_recover);
  EXPECT_TRUE(result.m1_invoked);
  EXPECT_EQ(result.m0_candidate_count, 0U);
  EXPECT_LE(result.candidate_count, 24U);
  ASSERT_EQ(result.cluster_obstacle_ids.size(), 1U);
  EXPECT_EQ(result.cluster_obstacle_ids.front(), 41);
  EXPECT_TRUE(std::isfinite(result.cluster_start_forward_m));
  EXPECT_TRUE(std::isfinite(result.cluster_end_forward_m));
  EXPECT_LE(result.cluster_start_forward_m, result.cluster_end_forward_m);
}

TEST(RacelineSplinePlanner, RejectsRotatedFootprintWhenCenterlineRemainsInsideLeftBound)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto rotated = makeStraightCandidate(reference, 0.15, 0.40);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(ego, rotated, {}, &reason, &failure));
  EXPECT_EQ(reason, "footprint_track_bound");
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
  EXPECT_EQ(failure.footprint_violation_side, "left");
  EXPECT_GT(failure.centerline_wall_clearance, 0.0);
  EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  EXPECT_GT(failure.wallward_corner_protrusion, 0.0);
  EXPECT_NEAR(failure.heading_relative_to_reference, 0.40, 1.0e-12);
}

TEST(RacelineSplinePlanner, AcceptsHeadingAlignedFootprintAtSameCenterline)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const auto aligned = makeStraightCandidate(reference, 0.15, 0.0);
  std::string reason;
  PathValidationFailure failure;

  EXPECT_TRUE(planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0}, aligned, {}, &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

TEST(RacelineSplinePlanner, DetectsBothLeftAndRightFootprintViolations)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  for (const auto & test : std::vector<std::pair<double, std::string>>{
        {0.15, "left"}, {-0.15, "right"}})
  {
    PathValidationFailure failure;
    EXPECT_FALSE(planner.validatePath(
        ego, makeStraightCandidate(reference, test.first, 0.40), {}, nullptr, &failure));
    EXPECT_EQ(failure.kind, PathValidationFailureKind::kTrackBoundary);
    EXPECT_EQ(failure.footprint_violation_side, test.second);
    EXPECT_GT(failure.centerline_wall_clearance, 0.0);
    EXPECT_LT(failure.rectangular_footprint_wall_clearance, 0.0);
  }
}

TEST(RacelineSplinePlanner, PreservesOtherHardConstraintsAfterFootprintValidation)
{
  const auto reference = makeStraightReference(100, 0.1, 2.0, 2.0);
  auto parameters = testParameters();
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  auto path = makeStraightCandidate(reference, 0.0, 0.0);
  PathValidationFailure failure;

  EXPECT_FALSE(planner.validatePath(
      ego, path, {makeObstacle(501, 1.0)}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);

  path.wpnts.front().kappa_radpm = parameters.maximum_curvature_radpm + 0.1;
  EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);

  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1.0;
  RacelineSplinePlanner rate_planner(parameters);
  ASSERT_TRUE(rate_planner.setReference(reference));
  path = makeStraightCandidate(reference, 0.0, 0.0);
  path.wpnts[2].kappa_radpm = 1.0;
  EXPECT_FALSE(rate_planner.validatePath(ego, path, {}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kGeometry);
}

TEST(RacelineSplinePlanner, FootprintValidationIsBitDeterministic)
{
  const auto reference = makeStraightReference(100, 0.1, 0.30, 0.30);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto path = makeStraightCandidate(reference, -0.15, -0.40);
  PathValidationFailure expected;
  ASSERT_FALSE(planner.validatePath(ego, path, {}, nullptr, &expected));

  for (int repetition = 0; repetition < 10; ++repetition) {
    PathValidationFailure actual;
    EXPECT_FALSE(planner.validatePath(ego, path, {}, nullptr, &actual));
    EXPECT_EQ(actual.kind, expected.kind);
    EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.waypoint_index, expected.waypoint_index);
    EXPECT_EQ(actual.footprint_violation_side, expected.footprint_violation_side);
    EXPECT_EQ(actual.centerline_wall_clearance, expected.centerline_wall_clearance);
    EXPECT_EQ(
      actual.rectangular_footprint_wall_clearance,
      expected.rectangular_footprint_wall_clearance);
    EXPECT_EQ(actual.waypoint_x, expected.waypoint_x);
    EXPECT_EQ(actual.waypoint_y, expected.waypoint_y);
    EXPECT_EQ(actual.waypoint_yaw, expected.waypoint_yaw);
    EXPECT_EQ(
      actual.heading_relative_to_reference,
      expected.heading_relative_to_reference);
    EXPECT_EQ(actual.wallward_corner_protrusion, expected.wallward_corner_protrusion);
  }
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
    EXPECT_EQ(waypoint.vx_mps, global.vx_mps);
    saw_offset = saw_offset || std::abs(waypoint.d_m) > 0.20;
  }
  EXPECT_TRUE(saw_offset);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

TEST(RacelineSplinePlanner, MovesProgressivelyThroughQuinticControlMarkers)
{
  auto parameters = testParameters();
  parameters.transition_distance_scales = {1.0};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(17, 8.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_EQ(result.control_points.size(), 8U);

  const double target = result.target_d;
  ASSERT_GT(target, 0.0);
  EXPECT_NEAR(result.control_points[0].d, 0.0, 1.0e-9);
  EXPECT_GT(result.control_points[1].d, 0.0);
  EXPECT_GT(result.control_points[2].d, result.control_points[1].d);
  EXPECT_LT(result.control_points[2].d, target);
  EXPECT_NEAR(result.control_points[3].d, target, 1.0e-9);
  EXPECT_NEAR(result.control_points[4].d, target, 1.0e-9);
  EXPECT_LT(result.control_points[6].d, result.control_points[5].d);
  EXPECT_GT(result.control_points[6].d, 0.0);
  EXPECT_NEAR(result.control_points[7].d, 0.0, 1.0e-9);
}

TEST(RacelineSplinePlanner, SeparatesAvailableDistanceEntryFromExitScale)
{
  auto parameters = testParameters();
  parameters.pre_apex_distances_m = {4.0, 3.0, 1.5};
  parameters.post_apex_distances_m = {1.5, 3.0, 4.0};
  parameters.entry_transition_fractions = {1.0};
  parameters.transition_distance_scales = {1.0};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(18, 10.0)}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.control_points.empty());

  // Inflated obstacle start is 9.45 m. requested=4.0 m maps monotonically through the
  // pre_apex_far/detection_lookahead ratio, rather than min(requested, available).
  constexpr double kClusterStart = 9.45;
  constexpr double kRequestedEntry = 4.0;
  const double expected_entry = kClusterStart * kRequestedEntry / 12.0;
  EXPECT_NEAR(result.requested_entry_length_m, kRequestedEntry, 1.0e-9);
  EXPECT_NEAR(result.effective_entry_length_m, expected_entry, 1.0e-9);
  EXPECT_NEAR(
    result.control_points.front().forward_s,
    kClusterStart - expected_entry, 1.0e-6);
  // Exit remains an independent post-apex distance.
  EXPECT_NEAR(result.exit_length_m, 4.0, 1.0e-9);
  EXPECT_NEAR(result.merge_s, 14.55, 1.0e-6);
}

TEST(RacelineSplinePlanner, EnumeratesAllSidesTargetsAndTransitionsBeforeRanking)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.entry_transition_fractions = {0.5, 0.75, 1.0};
  parameters.transition_distance_scales = {0.75, 1.0, 1.25};
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(118, 8.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_EQ(result.candidate_audits.size(), 120U);

  const auto selected = std::find_if(
    result.candidate_audits.begin(), result.candidate_audits.end(),
    [](const SplineCandidateAudit & audit) {return audit.selected;});
  ASSERT_NE(selected, result.candidate_audits.end());
  EXPECT_TRUE(selected->feasible);
  EXPECT_EQ(selected->final_rank, 1);
  for (const auto & audit : result.candidate_audits) {
    if (audit.feasible) {
      EXPECT_GE(
        selected->minimum_normalized_safety_slack + 1.0e-6,
        audit.minimum_normalized_safety_slack);
    } else {
      EXPECT_FALSE(audit.rejection_reason.empty());
      EXPECT_EQ(audit.final_rank, -1);
    }
  }
}

TEST(RacelineSplinePlanner, AppendsFullAvailableEntryAfterUnchangedLegacyCandidates)
{
  auto parameters = testParameters();
  parameters.pre_apex_distances_m = {4.0, 3.0, 1.5};
  parameters.entry_transition_fractions = {0.5, 0.75, 1.0};
  parameters.transition_distance_scales = {1.0};
  parameters.maximum_lateral_slope = 100.0;
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(218, 10.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_EQ(result.candidate_audits.size(), 4U);

  constexpr double kAvailableEntry = 9.45;
  const std::vector<double> legacy_fractions{0.5, 0.75, 1.0};
  for (std::size_t index = 0; index < legacy_fractions.size(); ++index) {
    const auto & audit = result.candidate_audits[index];
    const double requested = 4.0 * legacy_fractions[index];
    EXPECT_DOUBLE_EQ(audit.entry_fraction, legacy_fractions[index]);
    EXPECT_NEAR(audit.requested_entry_length_m, requested, 1.0e-12);
    EXPECT_NEAR(
      audit.effective_entry_length_m,
      kAvailableEntry * requested / parameters.detection_lookahead_m, 1.0e-12);
  }

  const auto & full_available = result.candidate_audits.back();
  EXPECT_NEAR(full_available.entry_fraction, 3.0, 1.0e-12);
  EXPECT_NEAR(
    full_available.requested_entry_length_m, parameters.detection_lookahead_m, 1.0e-12);
  EXPECT_NEAR(full_available.effective_entry_length_m, kAvailableEntry, 1.0e-12);
}

TEST(RacelineSplinePlanner, SafetySlackRejectsBarelyWallFeasibleTargetAsBest)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  parameters.entry_transition_fractions = {1.0};
  parameters.transition_distance_scales = {1.0};
  parameters.maximum_lateral_slope = 100.0;
  parameters.maximum_curvature_radpm = 100.0;
  parameters.maximum_curvature_rate_radpm2 = 1000.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.67, 0.67)));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(119, 8.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // Targets are sampled between the obstacle clearance (0.20 + 0.15) and the farthest offset whose
  // FOOTPRINT still fits (0.67 track - 0.12 half width): [0.35, 0.55] in five steps.
  EXPECT_NEAR(result.target_d, 0.50, 1.0e-9);

  const auto selected = std::find_if(
    result.candidate_audits.begin(), result.candidate_audits.end(),
    [](const SplineCandidateAudit & audit) {return audit.selected;});
  ASSERT_NE(selected, result.candidate_audits.end());
  EXPECT_GT(selected->wall_clearance_m, 0.05);
  EXPECT_GT(selected->rectangular_footprint_wall_clearance_m, 0.0);
  // The extreme sample now sits where a vehicle travelling parallel to the reference would just
  // touch the wall. It is still rejected, because the pass is not parallel there: the yaw the
  // maneuver carries pushes a corner past the boundary. Safety-slack scoring must refuse it in
  // favour of a target that keeps real clearance.
  const auto wall_tangent = std::find_if(
    result.candidate_audits.begin(), result.candidate_audits.end(),
    [](const SplineCandidateAudit & audit) {
      return std::abs(audit.target_d - 0.55) < 1.0e-9;
    });
  ASSERT_NE(wall_tangent, result.candidate_audits.end());
  EXPECT_FALSE(wall_tangent->feasible);
  EXPECT_TRUE(wall_tangent->footprint_invalid);
  EXPECT_LT(wall_tangent->rectangular_footprint_wall_clearance_m, 0.0);
  EXPECT_EQ(wall_tangent->rejection_reason, "footprint_track_bound");
  EXPECT_FALSE(wall_tangent->selected);
}

TEST(RacelineSplinePlanner, NominalEntryChangeAlwaysChangesEffectiveGeometry)
{
  auto short_parameters = testParameters();
  short_parameters.entry_transition_fractions = {1.0};
  short_parameters.transition_distance_scales = {1.0};
  short_parameters.pre_apex_distances_m = {4.0, 2.5, 1.0};
  auto long_parameters = short_parameters;
  long_parameters.pre_apex_distances_m = {8.0, 5.0, 2.0};

  RacelineSplinePlanner short_planner(short_parameters);
  RacelineSplinePlanner long_planner(long_parameters);
  ASSERT_TRUE(short_planner.setReference(makeStraightReference()));
  ASSERT_TRUE(long_planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(120, 8.0)};
  const auto short_result = short_planner.plan(ego, obstacles, true, false);
  const auto long_result = long_planner.plan(ego, obstacles, true, false);
  ASSERT_EQ(short_result.kind, SplinePlanKind::kAvoidance) << short_result.reason;
  ASSERT_EQ(long_result.kind, SplinePlanKind::kAvoidance) << long_result.reason;
  EXPECT_DOUBLE_EQ(short_result.requested_entry_length_m, 4.0);
  EXPECT_DOUBLE_EQ(long_result.requested_entry_length_m, 8.0);
  EXPECT_GT(long_result.effective_entry_length_m, short_result.effective_entry_length_m);
  EXPECT_NE(
    long_result.control_points.front().forward_s,
    short_result.control_points.front().forward_s);
  EXPECT_GT(short_result.control_points.front().forward_s, 0.0);
  EXPECT_GT(long_result.control_points.front().forward_s, 0.0);
}

TEST(RacelineSplinePlanner, RepeatedCandidateSelectionIsBitDeterministic)
{
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(121, 8.0)};
  const auto reference = planner.plan(ego, obstacles);
  ASSERT_EQ(reference.kind, SplinePlanKind::kAvoidance) << reference.reason;

  for (int repetition = 0; repetition < 10; ++repetition) {
    const auto repeated = planner.plan(ego, obstacles);
    ASSERT_EQ(repeated.kind, reference.kind);
    ASSERT_EQ(repeated.target_d, reference.target_d);
    ASSERT_EQ(repeated.go_left, reference.go_left);
    ASSERT_EQ(repeated.candidate_audits.size(), reference.candidate_audits.size());
    ASSERT_EQ(repeated.path.wpnts.size(), reference.path.wpnts.size());
    for (std::size_t index = 0; index < reference.path.wpnts.size(); ++index) {
      EXPECT_EQ(repeated.path.wpnts[index].s_m, reference.path.wpnts[index].s_m);
      EXPECT_EQ(repeated.path.wpnts[index].d_m, reference.path.wpnts[index].d_m);
      EXPECT_EQ(repeated.path.wpnts[index].x_m, reference.path.wpnts[index].x_m);
      EXPECT_EQ(repeated.path.wpnts[index].y_m, reference.path.wpnts[index].y_m);
      EXPECT_EQ(
        repeated.path.wpnts[index].kappa_radpm,
        reference.path.wpnts[index].kappa_radpm);
    }
    for (std::size_t index = 0; index < reference.candidate_audits.size(); ++index) {
      EXPECT_EQ(
        repeated.candidate_audits[index].final_rank,
        reference.candidate_audits[index].final_rank);
      EXPECT_EQ(
        repeated.candidate_audits[index].selected,
        reference.candidate_audits[index].selected);
    }
  }
}

TEST(RacelineSplinePlanner, KeepsQuinticAvoidanceValidOnCurvedReference)
{
  auto parameters = testParameters();
  parameters.transition_distance_scales = {1.0};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeCircularReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto obstacle = makeObstacle(19, 8.0);

  const auto result = planner.plan(ego, {obstacle}, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  std::string reason;
  EXPECT_TRUE(planner.validatePath(ego, result.path, {obstacle}, &reason)) << reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_LE(std::abs(waypoint.kappa_radpm), parameters.maximum_curvature_radpm);
  }
}

TEST(RacelineSplinePlanner, AppendsSpeedAwareGlobalTailAfterActualMerge)
{
  auto parameters = testParameters();
  parameters.post_merge_lookahead_m = 2.0;
  parameters.post_merge_min_time_sec = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));

  const EgoFrenetState ego{0.0, 0.0, 6.0};
  const auto result = planner.plan(ego, {makeObstacle(12, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  const double tail_distance =
    planner.forwardDistance(result.merge_s, result.path.wpnts.back().s_m);
  EXPECT_GE(tail_distance, 5.8);
  EXPECT_NEAR(result.path.wpnts.back().d_m, 0.0, 1.0e-6);
}

TEST(RacelineSplinePlanner, BuildsClosedGlobalHandoffWithEgoInStateTail)
{
  const auto reference = makeCircularReference();
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  constexpr double kTailRatio = 0.10;
  const double ego_s = reference.wpnts[37].s_m;
  const auto path = planner.buildGlobalHandoffPath(ego_s, kTailRatio, 2.5);
  ASSERT_EQ(path.wpnts.size(), reference.wpnts.size());

  std::size_t closest_index = 0U;
  double closest_gap = std::numeric_limits<double>::infinity();
  double path_length = 0.0;
  for (std::size_t i = 0; i < path.wpnts.size(); ++i) {
    const double gap = planner.forwardDistance(ego_s, path.wpnts[i].s_m);
    const double circular_gap = std::min(gap, planner.trackLength() - gap);
    if (circular_gap < closest_gap) {
      closest_gap = circular_gap;
      closest_index = i;
    }
    EXPECT_DOUBLE_EQ(path.wpnts[i].d_m, 0.0);
    EXPECT_LE(path.wpnts[i].vx_mps, 2.5);
    if (i > 0U) {
      path_length += std::hypot(
        path.wpnts[i].x_m - path.wpnts[i - 1U].x_m,
        path.wpnts[i].y_m - path.wpnts[i - 1U].y_m);
    }
  }
  const std::size_t tail_count = static_cast<std::size_t>(
    std::ceil(kTailRatio * static_cast<double>(path.wpnts.size())));
  EXPECT_GE(closest_index, path.wpnts.size() - tail_count);

  const double average_spacing =
    path_length / static_cast<double>(path.wpnts.size() - 1U);
  const double closing_gap = std::hypot(
    path.wpnts.front().x_m - path.wpnts.back().x_m,
    path.wpnts.front().y_m - path.wpnts.back().y_m);
  EXPECT_LE(closing_gap, 2.0 * average_spacing);
}

TEST(RacelineSplinePlanner, UsesRightSideWhenLeftTrackSpaceIsInsufficient)
{
  auto reference = makeStraightReference(300, 0.1, 0.45, 1.5);
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, 0.0);
}

TEST(RacelineSplinePlanner, RejectsWallBlockedTargetsBeforeSplineConstruction)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));

  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});

  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_NE(
    result.reason.find("left target d exceeds track bound in obstacle span"),
    std::string::npos);
  EXPECT_NE(
    result.reason.find("right target d exceeds track bound in obstacle span"),
    std::string::npos);
}

TEST(RacelineSplinePlanner, AppliesPhysicalVehicleClearanceOnceToTrackBounds)
{
  // d_left/d_right describe physical boundaries. At heading=0 with half-width 0.12, d=0.23 is
  // exactly tangent for a 0.35 m left boundary and remains valid at numerical tolerance.
  const auto feasible_reference = makeStraightReference(300, 0.1, 0.35, 0.35);
  RacelineSplinePlanner feasible_planner(testParameters());
  ASSERT_TRUE(feasible_planner.setReference(feasible_reference));
  std::string reason;
  EXPECT_TRUE(feasible_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(feasible_reference, 0.23, 0.0), {}, &reason)) << reason;

  const auto blocked_reference = makeStraightReference(300, 0.1, 0.34, 0.34);
  RacelineSplinePlanner blocked_planner(testParameters());
  ASSERT_TRUE(blocked_planner.setReference(blocked_reference));
  EXPECT_FALSE(blocked_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(blocked_reference, 0.23, 0.0), {}, &reason));
}

TEST(RacelineSplinePlanner, BreaksCentredObstacleScoreTieWithTrackHeadroom)
{
  auto reference = makeCircularReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.d_left = 0.9;
  }
  auto parameters = testParameters();
  parameters.target_d_candidate_count = 5;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  // The right side offers more balanced obstacle/wall safety slack than the narrow left side.
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 8.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_FALSE(result.go_left);
  EXPECT_LT(result.target_d, -0.35);
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

TEST(RacelineSplinePlanner, AppliesSingleSafetyMarginToAvoidanceTarget)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + vehicle half-width 0.12 + the only safety margin 0.03
  EXPECT_NEAR(result.target_d, 0.35, 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesTrackingErrorReserveAsSeparateClearanceTerm)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.14;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.29, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.0);

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + half-width 0.12 + safety 0.03 + tracking reserve 0.14
  EXPECT_NEAR(result.target_d, 0.49, 1.0e-9);
}

TEST(RacelineSplinePlanner, BilinearlyInterpolatesTrackingErrorLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.99;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 0.5};
  parameters.tracking_error_lut_values_m = {0.02, 0.04, 0.06, 0.10};

  ASSERT_TRUE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(-1.0, -0.25), 0.055, 1.0e-9);
  EXPECT_NEAR(parameters.trackingErrorReserve(10.0, 2.0), 0.10, 1.0e-9);

  parameters.tracking_error_lut_values_m.pop_back();
  EXPECT_FALSE(parameters.trackingErrorLutValid());
  EXPECT_NEAR(parameters.trackingErrorReserve(1.0, 0.25), 0.99, 1.0e-9);
}

TEST(RacelineSplinePlanner, LimitsAvoidanceSpeedFromVelocityTable)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps =
  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
  {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5};

  ASSERT_TRUE(parameters.avoidanceVelocityLimitValid());
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(6.5, 0.0), 6.5);
  EXPECT_DOUBLE_EQ(parameters.limitedAvoidanceSpeed(3.0, 0.5), 3.0);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.2), 5.754463, 1.0e-6);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.5, 0.5), std::sqrt(14.0), 1.0e-9);
  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(-6.5, -0.9), std::sqrt(7.0 / 0.9), 1.0e-9);

  parameters.avoidance_velocity_limit_lateral_accel_mps2[2] = 7.5;
  EXPECT_FALSE(parameters.avoidanceVelocityLimitValid());
}

TEST(RacelineSplinePlanner, UsesLimitedAvoidanceSpeedForObstacleTrackingLut)
{
  auto parameters = testParameters();
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 4.0, 6.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.04, 0.12};
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};

  EXPECT_NEAR(parameters.limitedAvoidanceSpeed(6.0, 0.5), 2.0, 1.0e-9);
  EXPECT_NEAR(parameters.avoidanceTrackingErrorReserve(6.0, 0.5), 0.02, 1.0e-9);
  EXPECT_NEAR(parameters.obstacleSafetyClearance(6.0, 0.5), 0.17, 1.0e-9);
}

TEST(RacelineSplinePlanner, CapsPublishedAvoidanceWaypointSpeeds)
{
  auto parameters = testParameters();
  parameters.avoidance_velocity_limit_speed_bins_mps = {0.0, 9.0};
  parameters.avoidance_velocity_limit_lateral_accel_mps2 = {2.0, 2.0};
  auto reference = makeStraightReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = 6.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(33, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_LE(
      waypoint.vx_mps * waypoint.vx_mps * std::abs(waypoint.kappa_radpm),
      2.0 + 1.0e-9);
  }
}

TEST(RacelineSplinePlanner, UsesObstacleSpanMaximumTrackingLutReserveForTarget)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.10};
  auto reference = makeStraightReference();
  for (auto & waypoint : reference.wpnts) {
    waypoint.vx_mps = waypoint.s_m >= 6.0 && waypoint.s_m <= 8.0 ? 3.0 : 0.0;
  }

  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(31, 7.0)}, true, false);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  // raw d_left 0.20 + base clearance 0.15 + maximum span LUT reserve 0.10
  EXPECT_NEAR(result.target_d, 0.45, 1.0e-9);
}

TEST(RacelineSplinePlanner, AppliesOnlyWallMarginToTrackBounds)
{
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.0, 0.0, 0.10, 0.10};
  parameters.wall_safety_margin_m = 0.04;
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(3.0, 1.0), 0.04);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.62, 0.62)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  EXPECT_NEAR(feasible.target_d, 0.45, 1.0e-9);

  // 0.60 m of room no longer safe-stops: the strict gate does not fit, but slowing the pass to
  // avoidance_minimum_speed_mps shrinks the reserve enough that a target does. Only a corridor too
  // narrow even at that floor is refused, and the target gate now says so before any spline is
  // built instead of generating candidates the footprint check must throw away.
  RacelineSplinePlanner slowed_planner(parameters);
  ASSERT_TRUE(slowed_planner.setReference(makeStraightReference(300, 0.1, 0.60, 0.60)));
  const auto slowed = slowed_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  ASSERT_EQ(slowed.kind, SplinePlanKind::kAvoidance) << slowed.reason;
  EXPECT_LT(slowed.target_d, 0.45);
  EXPECT_LE(slowed.target_d + parameters.vehicle_half_width_m, 0.60 - 0.04 + 1.0e-9);

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.55, 0.55)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(32, 7.0)}, true, false);
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
  EXPECT_NE(blocked.reason.find("track bound"), std::string::npos) << blocked.reason;
}

TEST(RacelineSplinePlanner, AppliesIndependentWallSafetyMarginOnlyToTrackBounds)
{
  auto parameters = testParameters();
  parameters.wall_safety_margin_m = 0.05;
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), 0.15, 1.0e-9);
  EXPECT_DOUBLE_EQ(parameters.trackBoundaryReserve(2.0, 0.0), 0.05);

  RacelineSplinePlanner feasible_planner(parameters);
  ASSERT_TRUE(feasible_planner.setReference(makeStraightReference(300, 0.1, 0.53, 0.53)));
  const auto feasible = feasible_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  ASSERT_EQ(feasible.kind, SplinePlanKind::kAvoidance) << feasible.reason;
  EXPECT_NEAR(std::abs(feasible.target_d), 0.35, 1.0e-9);

  const auto tangent_reference = makeStraightReference(300, 0.1, 0.40, 0.40);
  RacelineSplinePlanner tangent_planner(parameters);
  ASSERT_TRUE(tangent_planner.setReference(tangent_reference));
  std::string reason;
  EXPECT_TRUE(tangent_planner.validatePath(
      EgoFrenetState{0.0, 0.0, 2.0},
      makeStraightCandidate(tangent_reference, 0.23, 0.0), {}, &reason)) << reason;

  RacelineSplinePlanner blocked_planner(parameters);
  ASSERT_TRUE(blocked_planner.setReference(makeStraightReference(300, 0.1, 0.39, 0.39)));
  const auto blocked = blocked_planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(3, 7.0)});
  EXPECT_EQ(blocked.kind, SplinePlanKind::kSafeStop) << blocked.reason;
}

TEST(RacelineSplinePlanner, RejectsCommittedPathWhenObstacleEnvelopeGrows)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(2, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  std::string reason;
  EXPECT_TRUE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0)}, &reason)) << reason;
  EXPECT_FALSE(
    planner.validatePath(
      EgoFrenetState{0.2, 0.0, 2.0}, committed.path,
      {makeObstacle(2, 7.0, -0.21, 0.21)}, &reason));
}

TEST(RacelineSplinePlanner, UsesSameClearanceForGuardAndRawObstacleInputs)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(23, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  // Soft/hard behavior differs only by the input envelope. Both calls add the same 0.15 m:
  // the uncertainty Guard reaches d=0.21 and collides, while raw geometry reaches d=0.19
  // and remains clear.
  const auto uncertainty_guard = makeObstacle(23, 7.0, -0.20, 0.21);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {uncertainty_guard}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_EQ(failure.obstacle_id, 23);
  EXPECT_TRUE(std::isfinite(failure.waypoint_s));
  EXPECT_TRUE(std::isfinite(failure.waypoint_d));
  EXPECT_NEAR(failure.obstacle_source_d_left, 0.21, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_test_d_left, 0.36, 1.0e-9);
  EXPECT_NEAR(failure.obstacle_clearance, 0.15, 1.0e-9);

  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {makeObstacle(23, 7.0, -0.20, 0.19)},
      &reason, &failure)) << reason;
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kNone);
}

TEST(RacelineSplinePlanner, IgnoresPostMergeTailCollisionForCurrentCommitment)
{
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.5, 3.0, 4.0};
  parameters.transition_distance_scales = {1.0};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto committed = planner.plan(ego, {makeObstacle(24, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const double next_obstacle_s = committed.merge_s + 0.7;
  const auto next_obstacle = makeObstacle(25, next_obstacle_s, -0.20, 0.20);
  std::string reason;
  PathValidationFailure failure;
  EXPECT_FALSE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_GT(failure.waypoint_s, committed.merge_s);

  const double merge_horizon = planner.forwardDistance(ego.s, committed.merge_s);
  EXPECT_TRUE(
    planner.validatePath(
      ego, committed.path, {next_obstacle}, &reason, &failure,
      merge_horizon)) << reason;
}

TEST(RacelineSplinePlanner, StartsNextManeuverContinuouslyFromNonzeroEgoD)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const EgoFrenetState ego{9.05, 0.50, 2.0};
  const auto result = planner.plan(
    ego, {makeObstacle(26, 13.5)}, true, true);
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  ASSERT_FALSE(result.path.wpnts.empty());

  double previous_s = ego.s;
  double previous_d = ego.d;
  for (const auto & waypoint : result.path.wpnts) {
    const double ds = planner.forwardDistance(previous_s, waypoint.s_m);
    ASSERT_GT(ds, 0.0);
    EXPECT_LE(
      std::abs(waypoint.d_m - previous_d) / ds,
      testParameters().maximum_lateral_slope + 1.0e-6);
    previous_s = waypoint.s_m;
    previous_d = waypoint.d_m;
  }
  EXPECT_NEAR(result.path.wpnts.front().d_m, ego.d, 0.02);
}

TEST(RacelineSplinePlanner, DoesNotReverseCommittedSideWhenItBecomesBlocked)
{
  auto reference = makeStraightReference(300, 0.1, 0.34, 1.5);
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto unlocked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, true);
  ASSERT_EQ(unlocked.kind, SplinePlanKind::kAvoidance) << unlocked.reason;
  EXPECT_FALSE(unlocked.go_left);

  const auto locked = planner.plan(ego, {makeObstacle(3, 7.0)}, true, false);
  EXPECT_EQ(locked.kind, SplinePlanKind::kSafeStop) << locked.reason;
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
  EXPECT_TRUE(std::any_of(
      result.path.wpnts.begin(), result.path.wpnts.end(),
      [&parameters](const auto & waypoint) {
        return waypoint.ax_mps2 <
               -0.99 * parameters.safe_stop_deceleration_mps2;
      }));

  const auto repeated = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 7.0, -0.40, 0.40)});
  ASSERT_EQ(repeated.kind, SplinePlanKind::kSafeStop) << repeated.reason;
  ASSERT_EQ(repeated.path.wpnts.size(), result.path.wpnts.size());
  for (std::size_t index = 0; index < result.path.wpnts.size(); ++index) {
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].vx_mps, result.path.wpnts[index].vx_mps);
    EXPECT_DOUBLE_EQ(repeated.path.wpnts[index].ax_mps2, result.path.wpnts[index].ax_mps2);
  }
}

TEST(RacelineSplinePlanner, BuildsSafeStopAtCurrentLateralOffset)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.30, 2.0},
    {makeObstacle(27, 7.0, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_NEAR(waypoint.d_m, 0.30, 1.0e-9);
  }
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
}

TEST(RacelineSplinePlanner, BrakesOnCommittedGeometryBeforeEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto committed = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(28, 7.0)}, true, false);
  ASSERT_EQ(committed.kind, SplinePlanKind::kAvoidance) << committed.reason;

  const auto ego_waypoint = std::find_if(
    committed.path.wpnts.begin(), committed.path.wpnts.end(),
    [](const auto & waypoint) {return waypoint.s_m >= 3.0;});
  ASSERT_NE(ego_waypoint, committed.path.wpnts.end());
  const EgoFrenetState ego{ego_waypoint->s_m, ego_waypoint->d_m, 2.0};
  const auto stop = planner.buildCommittedPathStop(
    ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});

  ASSERT_EQ(stop.kind, SplinePlanKind::kSafeStop) << stop.reason;
  ASSERT_GE(stop.path.wpnts.size(), 2U);
  EXPECT_NEAR(stop.path.wpnts.front().d_m, ego.d, 0.05);
  EXPECT_NEAR(stop.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  EXPECT_TRUE(
    std::any_of(
      stop.path.wpnts.begin(), stop.path.wpnts.end(),
      [](const auto & waypoint) {return std::abs(waypoint.d_m) > 0.10;}));

  auto off_path_ego = ego;
  off_path_ego.d += 0.10;
  const auto discontinuous_stop = planner.buildCommittedPathStop(
    off_path_ego, committed.path, {makeObstacle(29, 9.0, -1.0, 1.0)});
  EXPECT_EQ(discontinuous_stop.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(discontinuous_stop.path.wpnts.empty());
  EXPECT_NE(
    discontinuous_stop.reason.find("discontinuous from the current ego d"),
    std::string::npos);
}

TEST(RacelineSplinePlanner, BuildsPreparationStopForInitialBlockingCluster)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kPreparation) << result.reason;
  ASSERT_GE(result.path.wpnts.size(), 2U);
  EXPECT_EQ(result.obstacle_id, 5);
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
  EXPECT_NEAR(result.path.wpnts.back().vx_mps, 0.0, 1.0e-9);
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
  }
}

TEST(RacelineSplinePlanner, ReportsWholeBlockingClusterInAvoidanceResult)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0},
    {makeObstacle(5, 7.0), makeObstacle(6, 7.6)});

  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  EXPECT_EQ(result.obstacle_ids, (std::vector<int>{5, 6}));
}

TEST(RacelineSplinePlanner, ReportsNearestBlockingClusterIdsForManeuverChaining)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto cluster_ids = planner.blockingClusterIds(
    EgoFrenetState{0.0, 0.0, 2.0},
      {
        makeObstacle(5, 7.0),
        makeObstacle(6, 7.6),
        makeObstacle(9, 10.0),
      });

  EXPECT_EQ(cluster_ids, (std::vector<int>{5, 6}));
}

TEST(RacelineSplinePlanner, RefusesPreparationDelayInsideSafeStopBuffer)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.buildPreparationStop(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.0)});

  EXPECT_EQ(result.kind, SplinePlanKind::kNoSafePath);
  EXPECT_TRUE(result.path.wpnts.empty());
  EXPECT_NE(result.reason.find("inside the safe-stop buffer"), std::string::npos);
}

TEST(RacelineSplinePlanner, AllowsShortValidatedSafeStopPrefix)
{
  auto parameters = testParameters();
  parameters.maximum_target_offset_m = 0.45;
  parameters.minimum_path_points = 8;
  parameters.safe_stop_buffer_m = 0.80;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto result = planner.plan(
    EgoFrenetState{0.0, 0.0, 2.0}, {makeObstacle(5, 1.65, -0.40, 0.40)});
  ASSERT_EQ(result.kind, SplinePlanKind::kSafeStop) << result.reason;
  EXPECT_GE(result.path.wpnts.size(), 2U);
  EXPECT_LT(result.path.wpnts.size(), 8U);
}

TEST(RacelineSplinePlanner, BuildsZeroSpeedEmergencyHold)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  const auto path = planner.buildEmergencyStopPath(EgoFrenetState{2.05, 0.18, 2.0});
  ASSERT_EQ(path.wpnts.size(), 8U);
  for (const auto & waypoint : path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.18);
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(waypoint.ax_mps2, 0.0);
  }
}

TEST(RacelineSplinePlanner, HandlesObstacleAcrossTrackWrap)
{
  RacelineSplinePlanner planner(testParameters());
  ASSERT_TRUE(planner.setReference(makeStraightReference()));
  auto obstacle = makeObstacle(9, 0.30);
  obstacle.s_start = 0.10;
  obstacle.s_end = 0.50;
  constexpr double kEgoS = 24.0;
  const auto result = planner.plan(EgoFrenetState{kEgoS, 0.0, 2.0}, {obstacle});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  bool wrapped = false;
  double previous = -1.0;
  for (const auto & waypoint : result.path.wpnts) {
    const double forward = planner.forwardDistance(kEgoS, waypoint.s_m);
    EXPECT_GT(forward, previous);
    previous = forward;
    wrapped = wrapped || waypoint.s_m < 1.0;
  }
  EXPECT_TRUE(wrapped);
}

TEST(RacelineSplinePlanner, NeverJumpsToNearbyWrongSnakeBranch)
{
  auto reference = makeStraightReference();
  for (std::size_t i = 220U; i < reference.wpnts.size(); ++i) {
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
    EXPECT_LT(waypoint.s_m, 22.0)
      << "planner selected a geometrically nearby but topologically wrong snake branch";
    EXPECT_LT(waypoint.y_m, 0.55)
      << "path must remain an offset of the ordered first race-line branch";
  }
}

// A gap wide enough for the full-speed reserve must not cost any speed: obstacle-free laps and
// roomy avoidances keep the race-line profile.
TEST(RacelineSplinePlanner, WideGapKeepsRaceLineSpeed)
{
  const auto reference = makeStraightReference();
  auto parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.10;
  parameters.avoidance_minimum_speed_mps = 1.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.vx_mps, 3.0);
  }
}

// The lateral-acceleration cap is blind to a straight, so before the gap-driven limit an obstacle
// on a straight was planned at full race-line speed and reserved the widest tracking error the LUT
// has. Squeeze the corridor until only the slow end of the LUT fits and the pass must slow down.
TEST(RacelineSplinePlanner, TightGapOnStraightSlowsDownInsteadOfReservingFullSpeedError)
{
  // 0.40 m of room each side: the race-line-speed reserve (0.30) cannot fit beside the obstacle,
  // the slow-end reserve (0.02) can.
  const auto reference = makeStraightReference(300, 0.1, 0.45, 0.45);
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 1.0;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 1.5, 3.0};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.02, 0.02, 0.02, 0.02, 0.30, 0.30};
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(reference));

  const EgoFrenetState ego{0.0, 0.0, 2.0};
  const auto result = planner.plan(ego, {makeObstacle(7, 7.0, -0.05, 0.05)});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance) << result.reason;

  double slowest = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    slowest = std::min(slowest, waypoint.vx_mps);
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1.0e-9);
    EXPECT_GE(waypoint.vx_mps, parameters.avoidance_minimum_speed_mps - 1.0e-9);
  }
  EXPECT_LT(slowest, 3.0) << "the pass should have been slowed to afford its reserve";
}

// Speed may only be traded for reserve down to the configured floor. Below it the maneuver is
// reported infeasible rather than crawled through, so safe-stop stays the authority.
TEST(RacelineSplinePlanner, GapLimitedSpeedNeverFallsBelowTheFloor)
{
  auto parameters = testParameters();
  parameters.avoidance_minimum_speed_mps = 2.5;
  parameters.tracking_error_lut_speed_bins_mps = {0.0, 6.5};
  parameters.tracking_error_lut_curvature_bins_radpm = {0.0, 1.0};
  parameters.tracking_error_lut_values_m = {0.05, 0.05, 0.30, 0.30};

  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 1.0), 6.0);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, -1.0), 2.5);
  EXPECT_DOUBLE_EQ(parameters.gapLimitedAvoidanceSpeed(2.0, 0.0, -1.0), 2.0);

  const double capped = parameters.gapLimitedAvoidanceSpeed(6.0, 0.0, 0.15);
  EXPECT_GE(capped, 2.5);
  EXPECT_LT(capped, 6.0);
  EXPECT_LE(parameters.trackingErrorReserve(capped, 0.0), 0.15 + 1.0e-6);
}

TEST(RacelineSplinePlanner, MaximumExitLengthCapsCombinedExitScale)
{
  // The slack ranking always prefers the gentlest (longest) exit, and without an absolute cap
  // post_apex_far x transition_long extends the published avoidance path up to ~18 m past the
  // obstacle, deferring the merge back to the global line by that whole tail.
  auto parameters = testParameters();
  parameters.post_apex_distances_m = {1.0, 2.0, 5.0};
  parameters.maximum_exit_length_m = 6.0;
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(0.5), 0.5);
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 6.0 / 5.0);

  parameters.maximum_exit_length_m = 0.0;  // non-positive disables the cap
  EXPECT_DOUBLE_EQ(parameters.cappedCombinedExitScale(3.58), 3.58);
}

TEST(RacelineSplinePlanner, MarginOnlyClusterDegradesToSlowPassInsteadOfSafeStop)
{
  // Track so narrow that both spline sides fail. The obstacle sits entirely left of the line:
  // margin-blocking through the 0.3 m fallback reserve, but its raw envelope plus the physical
  // clearance (0.12 + 0.03) never reaches d = 0, so the line itself stays drivable.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const EgoFrenetState ego{0.0, 0.0, 3.0};
  const auto margin_only = makeObstacle(90, 5.0, 0.20, 0.60);

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 접근 실현성 램프: 자차(3.0 m/s)가 cap(2.0)보다 빠르므로 앞머리는 flat 2.0이 아니라
  // 자차 속도에서 내려오는 프로파일이다. 전 구간 단조 비증가 + 자차 속도 이하이고,
  // 장애물 스팬(및 그 이후)은 cap을 넘지 않아야 한다.
  double previous = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : result.path.wpnts) {
    EXPECT_DOUBLE_EQ(waypoint.d_m, 0.0);
    EXPECT_LE(waypoint.vx_mps, ego.speed + 1e-9);
    EXPECT_LE(waypoint.vx_mps, previous + 1e-9);
    previous = waypoint.vx_mps;
    if (waypoint.s_m >= margin_only.s_start) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  EXPECT_GT(result.path.wpnts.front().vx_mps, 2.5);   // 계단(즉시 2.0) 금지 = 램프 실존
  EXPECT_GT(result.merge_s, margin_only.s_end);

  // Contrast: the same track with a genuinely line-straddling obstacle must still stop.
  const auto physically_blocking = planner.plan(ego, {makeObstacle(91, 5.0)});
  EXPECT_EQ(physically_blocking.kind, SplinePlanKind::kSafeStop);
  EXPECT_FALSE(physically_blocking.margin_pass);
}

TEST(RacelineSplinePlanner, MarginPassApproachRampReachesCapBeforeClusterStart)
{
  // 0814 실차 회귀(run_0814_111210): 4.4 m/s 접근에 flat 2.0 margin pass가 발행돼 계단
  // 감속 → 서비스 브레이크 포화 → 마찰 한계 초과 슬립 → 벽. 램프는 실측 자차 속도에서
  // approach_feasibility_decel로 내려가되 군집 시작 전에 cap에 도달해야 한다.
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  parameters.margin_pass_speed_cap_mps = 2.0;
  parameters.approach_feasibility_decel_mps2 = 2.0;
  RacelineSplinePlanner planner(parameters);
  ASSERT_TRUE(planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto margin_only = makeObstacle(92, 9.0, 0.20, 0.60);
  const EgoFrenetState ego{0.0, 0.0, 4.4};

  const auto result = planner.plan(ego, {margin_only});
  ASSERT_EQ(result.kind, SplinePlanKind::kAvoidance);
  EXPECT_TRUE(result.margin_pass);
  ASSERT_GE(result.path.wpnts.size(), 2U);
  // 필요 감속 (4.4²-2.0²)/(2·~8.5) ≈ 0.9 < 2.0 → 완만한 파라미터 감속이 그대로 쓰이고,
  // cap 도달 지점은 (4.4²-2.0²)/(2·2.0) = 3.84 m — 군집 시작(≈8.5 m)보다 훨씬 앞이다.
  const double reach_cap_at = (ego.speed * ego.speed - 4.0) / (2.0 * 2.0);
  for (const auto & waypoint : result.path.wpnts) {
    // 프로파일 상한(3.0)은 절대 넘지 않는다: 램프값이 그보다 커도 참조 프로파일이 이긴다.
    EXPECT_LE(waypoint.vx_mps, 3.0 + 1e-9);
    if (waypoint.s_m >= reach_cap_at + 0.2) {
      EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
    }
  }
  // 앞머리는 램프를 따른다(참조 프로파일 3.0에 클램프): flat 2.0이 아니어야 한다.
  EXPECT_NEAR(result.path.wpnts.front().vx_mps, 3.0, 1e-9);

  // 비활성(<=0)이면 구 거동(flat cap) 그대로다.
  parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner flat_planner(parameters);
  ASSERT_TRUE(flat_planner.setReference(makeStraightReference(300, 0.1, 0.3, 0.3)));
  const auto flat = flat_planner.plan(ego, {margin_only});
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  for (const auto & waypoint : flat.path.wpnts) {
    EXPECT_LE(waypoint.vx_mps, 2.0 + 1e-9);
  }
}

TEST(RacelineSplinePlanner, AvoidanceApproachBrakesBeforeSpanInsteadOfStepping)
{
  // 회피 스플라인의 접근 구간: 간극/곡률 캡은 스팬 안에서만 작동하므로 접근은 프로파일
  // 속도 그대로다가 스팬 경계에서 계단으로 떨어진다. 후방 제동 램프는 그 계단을 스팬
  // 시작 속도로 미리 내려가는 프로파일로 바꾼다(낮추기만 함). 스팬 내부는 비트 동일.
  auto ramp_parameters = testParameters();
  // 속도에 가파르게 커지는 추적오차 예약 LUT + 좁은 왼쪽 통로: 스팬 안에서 간극 캡이
  // 프로파일(3.0)보다 확실히 낮은 속도를 강제해 "접근 빠름 / 스팬 느림" 계단을 만든다.
  ramp_parameters.tracking_error_lut_speed_bins_mps = {0.0, 2.0, 4.0};
  ramp_parameters.tracking_error_lut_curvature_bins_radpm = {0.0};
  ramp_parameters.tracking_error_lut_values_m = {0.05, 0.08, 0.60};
  ramp_parameters.approach_feasibility_decel_mps2 = 2.0;
  auto flat_parameters = ramp_parameters;
  flat_parameters.approach_feasibility_decel_mps2 = 0.0;
  RacelineSplinePlanner ramp_planner(ramp_parameters);
  RacelineSplinePlanner flat_planner(flat_parameters);
  const auto reference = makeStraightReference(300, 0.1, 0.6, 0.6);
  ASSERT_TRUE(ramp_planner.setReference(reference));
  ASSERT_TRUE(flat_planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.0, 2.0};
  // 비대칭 장애물(오른쪽으로 치우침): 후보 랭킹 1순위(safety slack)에서 왼쪽이 명확히
  // 이기게 해, 램프로 달라지는 2순위(velocity_loss)가 후보 선택을 못 바꾸게 고정한다.
  const std::vector<f110_msgs::msg::Obstacle> obstacles{makeObstacle(93, 8.0, -0.35, 0.05)};

  const auto ramped = ramp_planner.plan(ego, obstacles);
  const auto flat = flat_planner.plan(ego, obstacles);
  ASSERT_EQ(ramped.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(flat.kind, SplinePlanKind::kAvoidance);
  ASSERT_EQ(ramped.path.wpnts.size(), flat.path.wpnts.size());
  const double span_start = obstacles.front().s_start;
  bool lowered_somewhere = false;
  for (std::size_t index = 0; index < ramped.path.wpnts.size(); ++index) {
    const auto & with_ramp = ramped.path.wpnts[index];
    const auto & without = flat.path.wpnts[index];
    ASSERT_DOUBLE_EQ(with_ramp.s_m, without.s_m);
    // 램프는 어디서도 속도를 올리지 않는다.
    EXPECT_LE(with_ramp.vx_mps, without.vx_mps + 1e-9);
    if (with_ramp.s_m >= span_start) {
      // 스팬 및 그 이후는 손대지 않는다(간극/예약 캡 보존).
      EXPECT_DOUBLE_EQ(with_ramp.vx_mps, without.vx_mps);
    } else if (with_ramp.vx_mps < without.vx_mps - 1e-6) {
      lowered_somewhere = true;
    }
  }
  EXPECT_TRUE(lowered_somewhere);
}

TEST(RacelineSplinePlanner, RetentionReserveScaleHoldsCommittedPathThroughEnvelopeGrowth)
{
  // Full clearance = 0.15 (base) + 0.30 (reserve) = 0.45; retention 0.5 keeps 0.15 + 0.15.
  // The obstacle's grown left edge (0.42) violates the full margin against a path at d = 0.8
  // (0.42 + 0.45 > 0.8) but stays inside the retention band (0.42 + 0.30 < 0.8).
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  RacelineSplinePlanner planner(parameters);
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const EgoFrenetState ego{0.0, 0.8, 2.0};
  const auto path = makeStraightCandidate(reference, 0.8, 0.0, 60U);

  const auto grown = makeObstacle(7, 2.0, -0.2, 0.42);
  PathValidationFailure failure;
  EXPECT_FALSE(planner.validatePath(ego, path, {grown}, nullptr, &failure));
  EXPECT_EQ(failure.kind, PathValidationFailureKind::kObstacleCollision);
  EXPECT_TRUE(planner.validatePath(ego, path, {grown}, nullptr, nullptr, std::nullopt, 0.5));

  // Growth past the retention band must still invalidate the committed path.
  const auto beyond_retention = makeObstacle(8, 2.0, -0.2, 0.55);
  EXPECT_FALSE(
    planner.validatePath(ego, path, {beyond_retention}, nullptr, nullptr, std::nullopt, 0.5));

  // The same contract through the P3 suffix validator.
  EXPECT_FALSE(planner.evaluateP3PathCurrent(ego, path, {grown}).hard_valid);
  EXPECT_TRUE(planner.evaluateP3PathCurrent(ego, path, {grown}, 0.5).hard_valid);
}

TEST(RacelineSplinePlanner, LocalizationReserveAddsConstantFloorAndScalesWithRetention)
{
  RacelineSplineParameters parameters = testParameters();
  parameters.tracking_error_reserve_m = 0.30;
  const double base = parameters.obstacleBaseClearance();
  const double without = parameters.obstacleSafetyClearance(2.0, 0.0);
  EXPECT_NEAR(without, base + 0.30, 1e-9);

  parameters.localization_reserve_m = 0.06;
  // Full reserve: the localization floor adds verbatim.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0), base + 0.36, 1e-9);
  // Retention scale halves the WHOLE reserve including the localization floor.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.5), base + 0.18, 1e-9);
  // The base clearance itself is never touched.
  EXPECT_NEAR(parameters.obstacleSafetyClearance(2.0, 0.0, 0.0), base, 1e-9);
  // The gap-limited speed inversion sees the same floor: a gap that admits exactly the
  // tracking reserve no longer fits once the localization floor is added, so the requested
  // speed must drop to the avoidance floor (the constant reserve cannot be shed by slowing).
  const double at_floor = parameters.gapLimitedAvoidanceSpeed(4.0, 0.0, 0.30);
  EXPECT_NEAR(at_floor, parameters.avoidance_minimum_speed_mps, 1e-9);
}

TEST(RacelineSplinePlanner, BuildLastPathBrakeStopsAlongGivenGeometry)
{
  RacelineSplinePlanner planner(testParameters());
  const auto reference = makeStraightReference();
  ASSERT_TRUE(planner.setReference(reference));
  const auto path = makeStraightCandidate(reference, 0.0, 0.0, 60U);
  const EgoFrenetState ego{1.0, 0.0, 2.0};

  const auto braked = planner.buildLastPathBrake(ego, path);
  ASSERT_GE(braked.wpnts.size(), 2U);
  // Stop distance from 2.0 m/s at the default 2.5 m/s^2 deceleration is 0.8 m.
  EXPECT_LE(planner.forwardDistance(ego.s, braked.wpnts.back().s_m), 0.8 + 0.2);
  EXPECT_DOUBLE_EQ(braked.wpnts.back().vx_mps, 0.0);
  for (std::size_t i = 1; i < braked.wpnts.size(); ++i) {
    EXPECT_LE(braked.wpnts[i].vx_mps, braked.wpnts[i - 1].vx_mps + 1e-9);
  }

  f110_msgs::msg::WpntArray empty_path;
  EXPECT_TRUE(planner.buildLastPathBrake(ego, empty_path).wpnts.empty());
}

}  // namespace
}  // namespace local_planning
