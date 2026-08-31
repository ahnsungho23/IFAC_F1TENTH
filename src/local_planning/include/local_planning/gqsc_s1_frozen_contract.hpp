// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOCAL_PLANNING__GQSC_S1_FROZEN_CONTRACT_HPP_
#define LOCAL_PLANNING__GQSC_S1_FROZEN_CONTRACT_HPP_

#include <cstddef>

#include "local_planning/p3_r3_k12.hpp"

namespace local_planning
{

inline constexpr const char * kGqscS1MethodName = "LEX8_GLOBAL_DISJOINT_COVERAGE4";
inline constexpr const char * kGqscS1MethodSha256 =
  "670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776";
inline constexpr const char * kGqscS1ReferenceV3Sha256 =
  "965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780";
inline constexpr std::size_t kGqscS1PairProxyBudget = 128U;
inline constexpr std::size_t kGqscS1LexicographicQuota = 8U;
inline constexpr std::size_t kGqscS1CoverageQuota = 4U;
inline constexpr std::size_t kGqscS1ReconstructionBudget = 12U;
inline constexpr std::size_t kGqscS1ValidatorBudget = 12U;

static_assert(kGqscS1LexicographicQuota + kGqscS1CoverageQuota ==
  kGqscS1ReconstructionBudget);
static_assert(kGqscS1ReconstructionBudget == kP3R3K12CandidateBudget);
static_assert(kGqscS1ValidatorBudget == kP3R3K12CandidateBudget);

inline P3R3RTStandaloneOptions frozenGqscS1Options()
{
  P3R3RTStandaloneOptions options;
  options.diversity_policy = P3R3RTStandaloneDiversityPolicy::DISJOINT_COVERAGE;
  options.add_component_half_far002_inward015 = false;
  options.lexicographic_quota = kGqscS1LexicographicQuota;
  options.coverage_quota = kGqscS1CoverageQuota;
  return options;
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__GQSC_S1_FROZEN_CONTRACT_HPP_
