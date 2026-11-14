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

#ifndef LOCAL_PLANNING__GQSC_V3_FROZEN_CONTRACT_HPP_
#define LOCAL_PLANNING__GQSC_V3_FROZEN_CONTRACT_HPP_

#include <cstddef>

#include "local_planning/p3_r3_k12.hpp"

namespace local_planning
{

inline constexpr const char * kGqscV3MethodName = "V3_SIDE_BALANCED_DISJOINT";
inline constexpr const char * kGqscV3MethodSha256 =
  "965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780";
inline constexpr const char * kGqscV3PrevalidationLockSha256 =
  "ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185";
inline constexpr std::size_t kGqscV3PairProxyBudget = 128U;
inline constexpr std::size_t kGqscV3ReconstructionBudget = 12U;
inline constexpr std::size_t kGqscV3ValidatorBudget = 12U;

static_assert(kGqscV3ReconstructionBudget == kP3R3K12CandidateBudget);
static_assert(kGqscV3ValidatorBudget == kP3R3K12CandidateBudget);
static_assert(
  kP3R3K12LexicographicQuota + kP3R3K12CoverageQuota == kGqscV3ReconstructionBudget);

inline P3R3RTStandaloneOptions frozenGqscV3Options()
{
  P3R3RTStandaloneOptions options;
  options.diversity_policy =
    P3R3RTStandaloneDiversityPolicy::V3_SIDE_BALANCED_DISJOINT;
  options.add_component_half_far002_inward015 = false;
  options.lexicographic_quota = kP3R3K12LexicographicQuota;
  options.coverage_quota = kP3R3K12CoverageQuota;
  return options;
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__GQSC_V3_FROZEN_CONTRACT_HPP_
