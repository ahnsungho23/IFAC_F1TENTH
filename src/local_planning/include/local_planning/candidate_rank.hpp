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

#ifndef LOCAL_PLANNING__CANDIDATE_RANK_HPP_
#define LOCAL_PLANNING__CANDIDATE_RANK_HPP_

#include <cmath>
#include <cstddef>

namespace local_planning
{

// 회피 후보의 단일 순위 정의 (2026-08-21 통합).
//
// ■ 왜 하나여야 하나
// 같은 순위가 두 곳에 따로 구현돼 있었다: P3ShadowEvaluator::betterFeasible 와
// RacelineSplinePlanner::plan() 의 better_candidate. 양쪽 주석 모두 "두 순위가 갈리면
// P3 가 고른 것과 다른 경로를 plan() 이 커밋한다"고 경고하면서도 구현은 분리돼 있어,
// 한쪽만 고치면 조용히 갈라지는 구조였다.
//
// ■ 통합하며 실제로 발견한 분기 (수정 전 상태)
// 항목과 순서는 같았지만 **동률 판정 epsilon 이 1000 배 달랐다**: P3 는 1e-9,
// plan() 은 1e-6. 두 값 사이(1e-9 ~ 1e-6)의 차이에 대해 P3 는 "다르다"(그 항목으로
// 순위 결정), plan() 은 "같다"(다음 항목으로 이월)로 서로 다르게 판정했다.
//
// ■ 통합 값으로 1e-9 를 택한 근거 (실측)
// 어느 쪽으로 통합하든 한쪽의 거동은 바뀐다. 그래서 run_20260821_153156 백을 계획
// 노드에 재생해(단일 노드 가드, 동일 발행 부하) 동결본과 비교했다:
//   동일 코드 2회(재현성 바닥)            ot_side 1.53%p / ot_line 0.63%p
//   통합 eps=1e-6                          ot_side 2.39%p / ot_line 2.39%p
//   통합 eps=1e-9                          ot_side 1.37%p / ot_line 1.56%p  ← 바닥과 동급
// 즉 1e-9 로 통합하면 재생 지터 수준 안에서 종전 거동이 보존되고, 1e-6 은 근소 동률
// 구간의 선택을 실제로 조금 바꾼다(안전정지 비중 +2.4%p). 통합의 가치는 "정의가
// 하나"라는 데 있지 특정 값에 있지 않으므로, 거동 보존이 확인된 값을 택한다.
// P3(TEST_ACTIVE 의 실제 선택 주체)가 쓰던 값이기도 하다.
constexpr double kCandidateRankEpsilon = 1.0e-9;

struct CandidateRankKey
{
  // 다음 장애물을 건드리는 exit 은 강등된다. 여유가 아니라 다음 기동의 성립 여부를
  // 좌우하는 정합성 조건이라 최우선이다 (2026-08-16 백: s=31.7 기동의 긴 exit 이
  // s=40.6 장애물을 d=-0.265 로 관통 → 랩당 hard collision 41 회, 25 ms 마다 같은
  // 후보를 재선택하는 무한 재계획).
  bool exit_reaches_next_obstacle{false};
  // 자차 실측 속도에서 후보의 감속 cap까지 남은 거리가 제동거리+제어
  // 응답지연 거리보다 부족한 최대값 [m]. 0이면 seam 실현 가능, 양수면 이미
  // 제동 시작 지점을 놓친 후보다. 하드 기각하지 않고 실현 가능한 후보를 먼저
  // 선택하며, 모두 부족하면 부족량이 작은 경로를 선택해 5C 폴백을 지킨다.
  double ego_braking_distance_deficit_m{0.0};
  // A안 (2026-08-16 사용자 결정): 실현 가능한 후보끼리는 **속도를 slack 보다 먼저**
  // 본다. slack 은 하드 게이트를 통과한 뒤의 여분 마진이고 필요한 여유는 게이트가
  // 이미 보장하므로, 그 위로 더 버는 것보다 빨리 지나가는 편이 낫다. 종전 순서의
  // 실해: 23:41 백 s=33~37 직선에서 늦게 급히 꺾는 경로가 벽 여유로 이겨 곡률
  // 1.38 rad/m → 속도 캡 2.26 m/s, 후방 전파로 구간 평균 2.0 m/s 까지 하락.
  double velocity_loss{0.0};
  double minimum_normalized_safety_slack{0.0};
  double global_path_deviation_m{0.0};
  // 남은 완전 동률은 생성 순서로만 가른다. 최적화 가중치를 더하지 않으면서 대칭
  // 기하에서도 결정적인 선택을 보장한다.
  std::size_t tiebreak_index{0U};
};

// first 가 second 보다 우선하면 true (strict weak ordering).
inline bool betterCandidateRank(const CandidateRankKey & first, const CandidateRankKey & second)
{
  if (first.exit_reaches_next_obstacle != second.exit_reaches_next_obstacle) {
    return second.exit_reaches_next_obstacle;
  }
  const bool first_braking_feasible =
    first.ego_braking_distance_deficit_m <= kCandidateRankEpsilon;
  const bool second_braking_feasible =
    second.ego_braking_distance_deficit_m <= kCandidateRankEpsilon;
  if (first_braking_feasible != second_braking_feasible) {
    return first_braking_feasible;
  }
  const double braking_deficit_delta = first.ego_braking_distance_deficit_m -
    second.ego_braking_distance_deficit_m;
  if (std::abs(braking_deficit_delta) > kCandidateRankEpsilon) {
    return braking_deficit_delta < 0.0;
  }
  const double velocity_delta = first.velocity_loss - second.velocity_loss;
  if (std::abs(velocity_delta) > kCandidateRankEpsilon) {
    return velocity_delta < 0.0;
  }
  const double slack_delta = first.minimum_normalized_safety_slack -
    second.minimum_normalized_safety_slack;
  if (std::abs(slack_delta) > kCandidateRankEpsilon) {
    return slack_delta > 0.0;
  }
  const double deviation_delta = first.global_path_deviation_m -
    second.global_path_deviation_m;
  if (std::abs(deviation_delta) > kCandidateRankEpsilon) {
    return deviation_delta < 0.0;
  }
  return first.tiebreak_index < second.tiebreak_index;
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__CANDIDATE_RANK_HPP_
