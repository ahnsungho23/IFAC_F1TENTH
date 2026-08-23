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
#include <limits>
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

// 2026-08-23 이전 거리 스케일 A/B를 재현하는 호환 함수다. 반환값은
// applyRawSlowdownProfile의 "장애물 지점 목표속도" 자리에 들어간다. true 모드의
// max(floor,sqrt(2*a*front))은 현재 위치 허용속도를 그 자리에 넣고 profile이 같은 front를
// 다시 적용하므로, 교차점 이후 ego cap을 올바른 제동 envelope보다 sqrt(2)배 부풀린다.
// 운영 기본은 false이며 true는 비교/rollback 재현 외에는 사용하지 않는다. 이 파라미터는
// launch.py의 DeclareLaunchArgument로 노출되지 않고 YAML/ROS parameter override로 바꾼다.
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

inline double rawSlowdownRequiredDistance(
  double ego_speed_mps, double target_speed_mps, double decel_mps2,
  double response_delay_sec, double distance_margin_m)
{
  const double speed = std::isfinite(ego_speed_mps) ? std::max(0.0, ego_speed_mps) : 0.0;
  const double target = std::isfinite(target_speed_mps) ? std::max(0.0, target_speed_mps) : 0.0;
  const double delay = std::isfinite(response_delay_sec) ? std::max(0.0, response_delay_sec) : 0.0;
  const double margin = std::isfinite(distance_margin_m) ? std::max(0.0, distance_margin_m) : 0.0;
  const double excess = speed * speed - target * target;
  if (excess > 0.0 && (!(decel_mps2 > 0.0) || !std::isfinite(decel_mps2))) {
    return std::numeric_limits<double>::infinity();
  }
  const double braking = excess > 0.0 ? excess / (2.0 * decel_mps2) : 0.0;
  return speed * delay + braking + margin;
}

struct RawSlowdownProfileWindow
{
  double front_m{0.0};
  double span_m{0.0};
  double reserved_distance_m{0.0};
};

inline RawSlowdownProfileWindow rawSlowdownProfileWindow(
  double obstacle_front_m, double obstacle_span_m, double ego_speed_mps,
  double response_delay_sec, double distance_margin_m)
{
  const double front = std::isfinite(obstacle_front_m) ? std::max(0.0, obstacle_front_m) : 0.0;
  const double span = std::isfinite(obstacle_span_m) ? std::max(0.0, obstacle_span_m) : 0.0;
  const double speed = std::isfinite(ego_speed_mps) ? std::max(0.0, ego_speed_mps) : 0.0;
  const double delay = std::isfinite(response_delay_sec) ? std::max(0.0, response_delay_sec) : 0.0;
  const double margin = std::isfinite(distance_margin_m) ? std::max(0.0, distance_margin_m) : 0.0;
  const double reserve = std::min(front, speed * delay + margin);
  // 감속 시작 목표를 reserve만큼 당기되 hold 끝(front+span+1 m)은 그대로 둔다.
  return {front - reserve, span + reserve, reserve};
}

}  // namespace local_planning
