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

#ifndef LOCAL_PLANNING__MANEUVER_MEMORY_HPP_
#define LOCAL_PLANNING__MANEUVER_MEMORY_HPP_

#include <algorithm>
#include <cmath>

namespace local_planning
{

// 기동 장애물 기억(maneuver_obstacle_rear_s_)의 수명 규칙 (2026-08-22 신설).
//
// ■ 왜 따로 뺐나
// 이 두 판정은 노드 상태에 의존하지 않는 순수 규칙인데, 인라인으로 두었더니 **소비처가
// 셋으로 늘면서 한쪽만 고쳐지는** 구조가 됐다(보류 게이트 / 재회피 차단의 뒤끝 분기 /
// 갱신 함수). 여기 한 곳으로 모아 단위 시험이 노드를 띄우지 않고 직접 검증한다.

// 활성 기동이 없는(= id 목록이 빈) 프레임이 연속 clear_frames 회이면 기억을 비운다.
//
// 🔑 "비어 있다"의 의미가 이 규칙의 전부다. 목록은 P3ManeuverLifecycle 의 record 에서
//    복사되므로 **활성 기동이 없다**는 뜻이지 "이번 프레임 미검출"이 아니다 — 미검출이어도
//    record 에 id 가 남아 있으면 목록에도 남는다. 그러므로 비었을 때 기억을 유지하는 것은
//    정보가 아니라 누수다(run_20260822_002336: 21 m 전에 지나친 entry 가 원형거리 wrap 으로
//    "전방 20.63 m" 가 되어 유령 안전정지 52.6 s 를 만들었다).
// ⚠️ 그럼에도 **한 프레임에 즉시 지우지 않는다.** lifecycle 이 한 콜백만 IDLE 로 튀는 경우가
//    실재하고(2026-08-20 run_062020: 다음 콜백 IDLE → 옛 경로 재발행 → 0.5 s 뒤 벽),
//    그때 기억이 사라지면 그 회귀 보호가 통째로 무효가 된다.
// clear_frames <= 0 이면 항상 false = 종전 거동(누수 포함). empty_frames 는 호출자가 들고
// 있는 카운터로, 비어 있지 않은 프레임에서 0 으로 리셋된다.
inline bool maneuverMemoryShouldClear(
  bool obstacle_ids_empty, int clear_frames, int & empty_frames)
{
  if (!obstacle_ids_empty) {
    empty_frames = 0;
    return false;
  }
  if (clear_frames <= 0) {
    return false;
  }
  ++empty_frames;
  return empty_frames >= clear_frames;
}

// 기억된 뒤끝을 "아직 자차 앞"으로 인정할 상한 [m].
//
// 종전에는 반 바퀴(track/2)만 썼다. 그 heuristic 은 "원형거리로 반 바퀴를 넘으면 이미
// 지나친 것"이라는 뜻인데, 41 m 트랙에서 상한이 20.67 m 라 **21 m 전에 지나친 물체가
// 20.63 m 로 되읽히며 여유 4 cm 로 전방 취급**을 받았다. 계획 지평 밖 물체에는 보류의
// 근거("지금 라인으로 복귀하면 오프셋 0 으로 들이받는다")가 성립하지 않으므로 —
// 그 전에 재계획이 여러 번 돈다 — 지평으로 한 번 더 자른다.
// configured_max_ahead_m 가 유한한 양수가 아니면 종전 거동(반 바퀴)으로 폴백한다.
inline double maneuverMemoryMaxAhead(double track_length_m, double configured_max_ahead_m)
{
  const double half_track = 0.5 * track_length_m;
  if (!std::isfinite(configured_max_ahead_m) || configured_max_ahead_m <= 0.0) {
    return half_track;
  }
  return std::min(half_track, configured_max_ahead_m);
}

}  // namespace local_planning

#endif  // LOCAL_PLANNING__MANEUVER_MEMORY_HPP_
