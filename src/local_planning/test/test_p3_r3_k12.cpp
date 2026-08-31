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
#include <string>
#include <vector>

#include "local_planning/p3_r3_k12.hpp"

namespace local_planning
{
namespace
{

std::vector<std::string> keys(const std::vector<P3R3K12Factor> & factors)
{
  std::vector<std::string> output;
  for (const auto & factor : factors) {
    output.push_back(factor.configuration_key + "|" + factor.preconstruction_shape_key);
  }
  return output;
}

TEST(P3R3K12, FrozenIdentityAndBudgetsAreImmutable)
{
  EXPECT_STREQ(kP3R3K12MethodName, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12");
  EXPECT_STREQ(
    kP3R3K12MethodSha256,
    "7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc");
  EXPECT_EQ(kP3R3K12CandidateBudget, 12U);
  EXPECT_EQ(kP3R3K12LexicographicQuota, 10U);
  EXPECT_EQ(kP3R3K12CoverageQuota, 2U);
  EXPECT_EQ(kP3R3K12LexicographicQuota + kP3R3K12CoverageQuota, kP3R3K12CandidateBudget);
}

TEST(P3R3K12, GeometryOnlySelectionIsDeterministic)
{
  P3R3K12SideGeometry geometry;
  geometry.go_left = false;
  geometry.outside_is_left = true;
  geometry.ego_d = 0.0;
  geometry.ego_speed = 3.0;
  geometry.cluster_start = 3.0;
  geometry.cluster_end = 4.0;
  geometry.domain_lower = -0.8;
  geometry.domain_upper = -0.2;
  geometry.reference_spacing_m = 0.25;
  geometry.components = {{-0.8, -0.2}};
  geometry.center_values = {-0.5, -0.45, -0.4};
  for (int index = 0; index < 48; ++index) {
    P3R3K12CorridorSample sample;
    sample.station = 0.25 * static_cast<double>(index);
    sample.lower = -0.9;
    sample.upper = 0.9;
    sample.center = 0.0;
    sample.width = 1.8;
    sample.finite = true;
    geometry.reference_samples.push_back(sample);
  }
  const std::vector<P3R3K12ProductionCandidate> production{
    {false, -0.3, -0.4, 0.5, 0.6},
    {false, -0.3, -0.4, 1.0, 1.2}};

  const auto first = selectP3R3K12Factors({geometry}, production);
  const auto second = selectP3R3K12Factors({geometry}, production);
  EXPECT_GT(first.lateral_factor_count, 0U);
  EXPECT_GT(first.pair_priority_count, 0U);
  EXPECT_GE(first.lexicographic.size(), kP3R3K12LexicographicQuota);
  EXPECT_GE(first.coverage.size(), kP3R3K12CoverageQuota);
  EXPECT_EQ(first.lateral_factor_count, second.lateral_factor_count);
  EXPECT_EQ(first.pair_priority_count, second.pair_priority_count);
  EXPECT_EQ(keys(first.lexicographic), keys(second.lexicographic));
  EXPECT_EQ(keys(first.coverage), keys(second.coverage));

}

TEST(P3R3K12, StandaloneDirectSeedSelectionIsDeterministicAndBounded)
{
  P3R3K12SideGeometry geometry;
  geometry.go_left = true;
  geometry.outside_is_left = false;
  geometry.ego_d = 0.05;
  geometry.ego_speed = 3.0;
  geometry.cluster_start = 2.0;
  geometry.cluster_end = 3.0;
  geometry.domain_lower = 0.15;
  geometry.domain_upper = 0.85;
  geometry.bottleneck_center = 0.48;
  geometry.reference_spacing_m = 0.25;
  geometry.entry_scale_min = 0.5145810930150512;
  geometry.entry_scale_max = 1.310934367577036;
  geometry.exit_scale_min = 0.4971684162574945;
  geometry.exit_scale_max = 7.397546202396385;
  geometry.components = {{0.15, 0.85}};
  geometry.center_values = {0.48, 0.50};
  for (int index = 0; index < 48; ++index) {
    P3R3K12CorridorSample sample;
    sample.station = 0.25 * static_cast<double>(index);
    sample.lower = -0.9;
    sample.upper = 0.9;
    sample.center = 0.0;
    sample.width = 1.8;
    sample.finite = true;
    geometry.reference_samples.push_back(sample);
  }

  const auto first = selectP3R3RTStandaloneFactors({geometry}, 128U);
  const auto second = selectP3R3RTStandaloneFactors({geometry}, 128U);
  EXPECT_GT(first.proposed_laterals.size(), 0U);
  EXPECT_LE(first.proposed_laterals.size(), 64U);
  EXPECT_EQ(first.proposed_transitions.size(), 7U);
  EXPECT_LE(first.pair_pool.size(), 128U);
  EXPECT_LE(first.lexicographic.size(), kP3R3K12LexicographicQuota);
  EXPECT_LE(first.coverage.size(), kP3R3K12CoverageQuota);
  EXPECT_EQ(keys(first.proposed_laterals), keys(second.proposed_laterals));
  EXPECT_EQ(keys(first.pair_pool), keys(second.pair_pool));
  EXPECT_EQ(keys(first.lexicographic), keys(second.lexicographic));
  EXPECT_EQ(keys(first.coverage), keys(second.coverage));

  P3R3RTStandaloneOptions v2;
  v2.diversity_policy =
    P3R3RTStandaloneDiversityPolicy::ZERO_INTERFACE_EQUAL_MIN_OUTWARD;
  v2.add_component_half_far002_inward015 = true;
  const auto refined_first = selectP3R3RTStandaloneFactors({geometry}, 128U, nullptr, v2);
  const auto refined_second = selectP3R3RTStandaloneFactors({geometry}, 128U, nullptr, v2);
  EXPECT_LE(refined_first.proposed_laterals.size(), 64U);
  EXPECT_LE(refined_first.proposed_transitions.size(), 7U);
  EXPECT_LE(refined_first.pair_pool.size(), 128U);
  EXPECT_LE(refined_first.lexicographic.size(), kP3R3K12LexicographicQuota);
  EXPECT_LE(refined_first.coverage.size(), kP3R3K12CoverageQuota);
  EXPECT_EQ(keys(refined_first.proposed_laterals), keys(refined_second.proposed_laterals));
  EXPECT_EQ(keys(refined_first.pair_pool), keys(refined_second.pair_pool));
  EXPECT_EQ(keys(refined_first.lexicographic), keys(refined_second.lexicographic));
  EXPECT_EQ(keys(refined_first.coverage), keys(refined_second.coverage));
  EXPECT_TRUE(std::any_of(
      refined_first.proposed_laterals.begin(), refined_first.proposed_laterals.end(),
      [](const auto & row) {
        return row.proposal_operator == "COMPONENT_HALF_FAR002_INWARD015";
      }));
  EXPECT_FALSE(std::any_of(
      refined_first.proposed_laterals.begin(), refined_first.proposed_laterals.end(),
      [](const auto & row) {
        return row.proposal_operator == "COMPONENT_HALF_MID_MINUS_015";
      }));

  for (const auto policy : {
      P3R3RTStandaloneDiversityPolicy::V3_SIDE_BALANCED_DISJOINT,
      P3R3RTStandaloneDiversityPolicy::V3_NORMALIZED_MAXIMIN_DISJOINT})
  {
    P3R3RTStandaloneOptions v3;
    v3.diversity_policy = policy;
    const auto v3_first = selectP3R3RTStandaloneFactors({geometry}, 128U, nullptr, v3);
    const auto v3_second = selectP3R3RTStandaloneFactors({geometry}, 128U, nullptr, v3);
    EXPECT_LE(v3_first.proposed_laterals.size(), 64U);
    EXPECT_LE(v3_first.proposed_transitions.size(), 7U);
    EXPECT_LE(v3_first.pair_pool.size(), 128U);
    EXPECT_LE(v3_first.lexicographic.size(), kP3R3K12LexicographicQuota);
    EXPECT_LE(v3_first.coverage.size(), kP3R3K12CoverageQuota);
    EXPECT_EQ(keys(v3_first.proposed_laterals), keys(v3_second.proposed_laterals));
    EXPECT_EQ(keys(v3_first.pair_pool), keys(v3_second.pair_pool));
    EXPECT_EQ(keys(v3_first.lexicographic), keys(v3_second.lexicographic));
    EXPECT_EQ(keys(v3_first.coverage), keys(v3_second.coverage));
    EXPECT_FALSE(std::any_of(
        v3_first.proposed_laterals.begin(), v3_first.proposed_laterals.end(),
        [](const auto & row) {
          return row.proposal_operator == "COMPONENT_HALF_FAR002_INWARD015";
        }));
  }
}

}  // namespace
}  // namespace local_planning
