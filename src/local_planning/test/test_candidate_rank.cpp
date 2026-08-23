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

// 후보 순위의 단일 정의 계약 (2026-08-21 통합).
//
// 통합 전에는 같은 순위가 P3ShadowEvaluator::betterFeasible 과 plan() 의 better_candidate
// 두 곳에 따로 구현돼 있었고, 실제로 **동률 epsilon 이 1000 배 달랐다**(1e-9 vs 1e-6).
// 여기서는 (a) 순위 항목과 우선순위, (b) 동률 판정 경계, (c) strict weak ordering 성질을
// 고정해, 한쪽만 고쳐서 두 경로가 갈리는 일을 다시 만들지 못하게 한다.

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <vector>

#include "local_planning/candidate_rank.hpp"

namespace local_planning
{
namespace
{

CandidateRankKey makeKey(
  bool exit_reaches, double velocity_loss, double slack, double deviation,
  std::size_t index)
{
  CandidateRankKey key;
  key.exit_reaches_next_obstacle = exit_reaches;
  key.velocity_loss = velocity_loss;
  key.minimum_normalized_safety_slack = slack;
  key.global_path_deviation_m = deviation;
  key.tiebreak_index = index;
  return key;
}

TEST(CandidateRank, ExitReachingNextObstacleIsDemotedFirst)
{
  // 정합성 조건이므로 다른 모든 항목에서 이겨도 강등된다.
  const auto reaching = makeKey(true, 0.0, 1.0, 0.0, 0U);
  const auto clear = makeKey(false, 9.9, -1.0, 9.9, 9U);
  EXPECT_TRUE(betterCandidateRank(clear, reaching));
  EXPECT_FALSE(betterCandidateRank(reaching, clear));
}

TEST(CandidateRank, VelocityOutranksSlack)
{
  // A안(2026-08-16): 하드 게이트를 통과한 뒤라면 여분 slack 보다 속도가 우선한다.
  const auto faster_less_slack = makeKey(false, 1.0, 0.10, 0.0, 0U);
  const auto slower_more_slack = makeKey(false, 2.0, 0.90, 0.0, 1U);
  EXPECT_TRUE(betterCandidateRank(faster_less_slack, slower_more_slack));
}

TEST(CandidateRank, SlackThenDeviationThenGenerationOrder)
{
  const auto more_slack = makeKey(false, 1.0, 0.50, 0.5, 3U);
  const auto less_slack = makeKey(false, 1.0, 0.20, 0.0, 0U);
  EXPECT_TRUE(betterCandidateRank(more_slack, less_slack));

  const auto closer = makeKey(false, 1.0, 0.50, 0.10, 7U);
  const auto farther = makeKey(false, 1.0, 0.50, 0.40, 0U);
  EXPECT_TRUE(betterCandidateRank(closer, farther));

  const auto first_generated = makeKey(false, 1.0, 0.50, 0.10, 2U);
  const auto later_generated = makeKey(false, 1.0, 0.50, 0.10, 5U);
  EXPECT_TRUE(betterCandidateRank(first_generated, later_generated));
  EXPECT_FALSE(betterCandidateRank(later_generated, first_generated));
}

TEST(CandidateRank, TieEpsilonBoundaryIsSingleSourced)
{
  // 통합 전 plan() 은 1e-6 으로 갈랐다: 아래 차이(1e-7)를 동률로 보아 slack 으로
  // 넘겼고, P3 는 velocity 로 순위를 정했다. 통합 후(1e-9)에는 양쪽 모두 velocity 가
  // 결정한다 — 재생 실측에서 종전 거동이 보존되는 쪽이 이 값이다.
  const auto faster_less_slack = makeKey(false, 1.0, 0.10, 0.0, 0U);
  const auto slower_more_slack = makeKey(false, 1.0 + 1.0e-7, 0.90, 0.0, 1U);
  EXPECT_TRUE(betterCandidateRank(faster_less_slack, slower_more_slack))
    << "1e-9 를 넘는 velocity 차이는 유의한 우열로 다룬다";

  // 경계 아래(1e-10)는 동률로 흡수돼 다음 항목(slack)이 결정한다.
  const auto noise_faster_less_slack = makeKey(false, 1.0, 0.10, 0.0, 0U);
  const auto noise_slower_more_slack = makeKey(false, 1.0 + 1.0e-10, 0.90, 0.0, 1U);
  EXPECT_TRUE(betterCandidateRank(noise_slower_more_slack, noise_faster_less_slack));
}

TEST(CandidateRank, IsStrictWeakOrderingUnderSort)
{
  // std::stable_sort 계약 위반(비대칭성·추이성 깨짐)은 UB 라 실차에서 무작위 선택으로
  // 나타난다. 잡음 구간 값을 섞어 넣고 정렬이 성립하는지 확인한다.
  std::mt19937 rng(20260821U);
  std::uniform_int_distribution<int> bucket(0, 3);
  std::vector<CandidateRankKey> keys;
  keys.reserve(200U);
  for (std::size_t index = 0; index < 200U; ++index) {
    const double base = static_cast<double>(bucket(rng));
    const double noise = 1.0e-9 * static_cast<double>(bucket(rng));
    keys.push_back(
      makeKey(
        bucket(rng) == 0, base + noise, 0.1 * bucket(rng) + noise,
        0.2 * bucket(rng) + noise, index));
  }
  for (const auto & first : keys) {
    EXPECT_FALSE(betterCandidateRank(first, first)) << "비반사성";
    for (const auto & second : keys) {
      if (betterCandidateRank(first, second)) {
        EXPECT_FALSE(betterCandidateRank(second, first)) << "비대칭성";
      }
    }
  }
  auto sorted = keys;
  std::stable_sort(sorted.begin(), sorted.end(), betterCandidateRank);
  for (std::size_t index = 1; index < sorted.size(); ++index) {
    EXPECT_FALSE(betterCandidateRank(sorted[index], sorted[index - 1U]))
      << "정렬 후 역전 at " << index;
  }
}

}  // namespace
}  // namespace local_planning
