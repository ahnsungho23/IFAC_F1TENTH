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

// B1 raw 감속 힌트를 "승격 전"으로 되돌리는 판정 (2026-08-22).
//
// 힌트의 선언된 목적은 **confirmed 승격 지연을 메우는 것**이다: raw 는 4.5~11 m 전방에서
// 이미 잡히는데 승격은 시간 기준이라, 그 사이 접근 속도를 미리 깎아 지연이 잡아먹는 거리를
// 줄인다. 그런데 종전 구현은 승격이 끝나고 회피 기동을 커밋한 뒤에도 같은 장애물에 계속
// 캡을 씌웠다 — 목적을 넘어선 구간이다.
//
// 왜 지금 문제가 되는가: 예약 게이트(obstacle_reserve_mode: none)가 갭 사다리를 끄면
// raw 캡이 **유일하게 남는** 장애물 관련 속도 캡이 된다. run_20260822_015820 실측에서
// 스팬 안 최저 캡의 주체는 갭 69.5% / raw 7.3% 였고, 갭이 빠지면 그 자리를 raw 가 그대로
// 물려받아 통과 속도가 2.80 에 고정된다(같은 백 id12 의 8~12 m 발행값이 정확히 2.80).
//
// 건너뛰는 조건은 **둘 다** 성립할 때뿐이다:
//   ① 그 장애물이 **지금** confirmed 목록에 있다 — 승격이 끝났다.
//   ② **지금 발행하는 회피 경로**가 그 장애물을 책임진다 — 기하가 이미 그것을 피해 간다.
// 하나라도 빠지면 캡은 그대로 걸린다. 특히 confirmed 에서 떨어져 나간(트랙이 끊긴)
// 장애물은 raw 에 남아 있는 한 다시 보호 대상이 된다 — 그 순간이 가장 위험한 순간이다.
//
// ⚠️ ②의 출처는 발행 결과 자신이지 별도 커밋 변수가 아니다. P3(현행 운영)는 활성 기동을
//    새 결과로 만들어 발행하고 옛 P0 커밋 변수를 켜지 않아, 그것을 보면 skip 이 영영
//    걸리지 않는다 (2026-08-22 리플레이 실측: 생략 0회).

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace local_planning
{

// 컨테이너 두 개는 호출부의 벡터를 그대로 받는다(수십 개 규모라 선형 탐색으로 충분하고,
// 집합을 새로 만들면 매 발행마다 할당이 생긴다).
inline bool rawSlowdownHandledByCommitment(
  bool skip_enabled,
  int raw_obstacle_id,
  bool published_result_is_avoidance,
  int published_obstacle_id,
  const std::vector<int> & published_obstacle_ids,
  const std::vector<int> & confirmed_obstacle_ids)
{
  if (!skip_enabled || raw_obstacle_id < 0 || !published_result_is_avoidance) {
    return false;
  }
  const bool handled_here = published_obstacle_id == raw_obstacle_id ||
    std::find(
    published_obstacle_ids.begin(), published_obstacle_ids.end(),
    raw_obstacle_id) != published_obstacle_ids.end();
  if (!handled_here) {
    return false;
  }
  return std::find(
    confirmed_obstacle_ids.begin(), confirmed_obstacle_ids.end(),
    raw_obstacle_id) != confirmed_obstacle_ids.end();
}

// ── 🔵 2026-08-23: 캡을 상수에서 **거리 함수**로 바꾼다 ──────────────────────
//
// 종전: raw 가 trigger 거리(12 m) 안에 잡히면 전방거리와 무관하게 늘 같은 상수(2.8)를
//       씌웠다. 12 m 앞이든 1 m 앞이든 똑같이 2.8 이다.
//
// 왜 바꾸는가 (롤보정 OFF 백 6개, 발동 표본 4,599 실측):
//   · 승격 지연은 옛 설정(10/15 표) 기준으로 잡은 값인데, 현행 검출기는
//     min_hits_confirm 3 / confirmation_window 5 @ 39.3 Hz 라 **p50 0.139 s** 다.
//     그 지연이 먹는 거리는 5.0 m/s 접근에서 0.70 m — 2.8 로 깎아 봐야 0.31 m 를
//     벌 뿐이다. 전방거리 p50 은 3.27 m 이므로 대부분 **너무 일찍, 너무 많이** 깎는다.
//   · 다만 꼬리는 크다(지연 p90 1.46 s). 캡을 없애면 그 경우가 위험해지므로
//     "없애기"가 아니라 "거리에 맞추기"가 맞다.
//
// 형태: 등가속 정지거리 d = v²/(2a) 를 v 에 대해 뒤집는다.
//         v = sqrt(2·a·d)  = 남은 거리 d 안에서 감속을 끝낼 수 있는 최대 속도
//       a 는 새 상수를 만들지 않고 approach_feasibility_decel_mps2(2.0) 를 재사용한다 —
//       이 오버레이의 램프 기울기가 이미 같은 값이라 두 식이 서로 싸우지 않는다.
//
// 바닥(floor)을 두는 이유와 방향:
//       v_cap = max(sqrt(2·a·d), floor)     ← **min 이 아니라 max** 다.
//   · floor 는 종전 상수(raw_slowdown_speed_cap_mps, 2.8)를 그대로 재해석한 것이다.
//   · 교차점 d* = floor²/(2a) = 2.8²/4.0 = **1.96 m**.
//       d < 1.96 m → 바닥이 이긴다 → **종전과 완전히 동일한 거동**
//       d > 1.96 m → 제동식이 이긴다 → 종전보다 **완화**
//   · 즉 이 변경은 **오직 완화만** 한다. 지금보다 느려지거나 더 자주 멈추는 경우가
//     구조적으로 생기지 않는다 — 대회 직전 변경으로서 안전한 방향이다.
//   · raw 는 아직 승격 안 된 검출이다. 오검출일 수 있는 물체 때문에 0.5 m 앞에서
//     1.4 m/s 로 기어가면 안 된다. 진짜 회피·정지는 승격 후 커밋된 기동이 맡는다.
//
// 실측 효과(같은 4,599 표본): d > 1.96 m 인 표본 **69%** 가 완화되고,
//   캡 중앙값 2.80 → **3.53** m/s (p90 5.15). 나머지 31% 는 종전과 같다.
//
// 되돌리기: raw_slowdown_distance_scaled 를 false 로 (런치 인자, 재빌드 불필요).
inline double rawSlowdownSpeedCap(
  bool distance_scaled,
  double floor_cap_mps,
  double decel_mps2,
  double obstacle_front_m)
{
  if (!distance_scaled || !(decel_mps2 > 0.0) ||
    !std::isfinite(obstacle_front_m) || !(obstacle_front_m > 0.0))
  {
    return floor_cap_mps;
  }
  return std::max(floor_cap_mps, std::sqrt(2.0 * decel_mps2 * obstacle_front_m));
}

}  // namespace local_planning
