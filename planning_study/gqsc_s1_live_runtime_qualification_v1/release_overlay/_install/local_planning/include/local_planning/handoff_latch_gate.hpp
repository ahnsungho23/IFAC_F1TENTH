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

#ifndef LOCAL_PLANNING__HANDOFF_LATCH_GATE_HPP_
#define LOCAL_PLANNING__HANDOFF_LATCH_GATE_HPP_

#include <cmath>

namespace local_planning::handoff_latch_gate
{

// 커밋 장애물 뒤끝까지의 원형 전방거리로 글로벌 핸드오프를 막을지 판단한다.
// 모든 입력의 단위는 metre다. 특히 latch_distance_m=8.0은 "8 frame"이 아니라
// 장애물 뒤끝이 자차 전방 8.0 m 이내에 남아 있는 전 구간을 뜻한다.
inline bool blocksGlobalHandoff(
  double rear_forward_m, double half_track_m, double latch_distance_m)
{
  if (!std::isfinite(rear_forward_m) || !std::isfinite(half_track_m) ||
    !std::isfinite(latch_distance_m) || !(half_track_m > 0.0) ||
    !(latch_distance_m > 0.0))
  {
    return false;
  }
  // forwardDistance()는 [0, track_length)다. 반 바퀴보다 멀면 장애물은 이미 자차가
  // 지나친 뒤쪽에 있는 것으로 해석하며, 그 기억으로 핸드오프를 막지 않는다.
  return rear_forward_m <= half_track_m && rear_forward_m <= latch_distance_m;
}

}  // namespace local_planning::handoff_latch_gate

#endif  // LOCAL_PLANNING__HANDOFF_LATCH_GATE_HPP_
