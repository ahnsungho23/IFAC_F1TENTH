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

// 보류 게이트의 복구원 선택 (F1, 2026-08-22) — 순수 판정.
//
// holdForManeuverObstacleAhead() 는 "기동 장애물을 아직 안 지났는데 이번 콜백에 쓸 유효한
// P3 출력이 없다" 는 상태에서 불린다. 거기서 무엇을 발행할지 고르는 사다리가 이것이다.
//
// ── 왜 두 번째 단이 필요한가 ────────────────────────────────────────────────
// 종전 사다리는 한 단(committed_result_)뿐이었고, 그 한 단은 **P3 운영에서 구조적으로
// 항상 비어 있다**:
//
//   · 활성 P3 기동은 local_planner_node.cpp 의
//       publishResult(makeP3ActiveResult(lifecycle))
//     로 곧장 발행된다.
//   · committed_result_ 를 채우는 유일한 함수는 commitAvoidance() 인데, 그 호출부는 전부
//     P0/레거시 분기다. P3(TEST_ACTIVE)는 한 번도 거치지 않는다.
//
// 그래서 이 게이트는 사실상 무조건 안전정지였다.
//
// ── 실측 (2026-08-22 실차 run_072312 254.6 s + run_073013 122.7 s) ──────────
//   「P3 기동 종료를 보류한다」            18 회
//     → 그중 안전정지로 간 것                16 회
//        → 사유가 "커밋 경로 없음"           13 회
//        → 나머지 3 회는 committed_result_ 에 글로벌 핸드오프 루프가 들어 있었지만
//          같은 진입 불연속 검사로 떨어졌다.
//
//   그 결과가 경로 붕괴다: 발행 경로 65 점 → 3 점(0.50 m). 컨트롤러의 walk_forward 는
//   열린 경로에서 끝점에 멈추므로 L1 목표가 전방 0.18 m / 횡 0.48 m 가 되고, 요구 횡가속이
//   32.2 m/s² (그립 예산 6.80) 로 튄다 — run_073013 t=93.7, IMU 1.52 g, d 가 −0.05 에서
//   +0.94 로 1.0 m 이탈.
//
// ── 왜 안전한가 ─────────────────────────────────────────────────────────────
// 두 번째 단은 2026-08-20 에 막았던 "옛 기하 무검증 재발행" 이 **아니다**. 호출자는 두
// 후보 모두 지금 이 ego·이 스냅샷으로 validatePath 를 통과시킨 뒤에만 valid 로 넘긴다.
// 그 검사가 진입 불연속 / 전방 점 존재 / 트랙 경계 / 곡률 / 장애물 충돌을 본다.
//
// 순서는 committed 가 먼저다. 그쪽이 더 권위 있는 출처이고, P0 운영에서는 그 단이 먼저
// 잡으므로 거동이 바뀌지 않는다.
//
// ── ⚠️ validatePath 만으로는 부족하다 (2026-08-22, 시험이 잡은 결함) ────────
// 처음 판은 "validatePath 를 매 콜백 통과해야 하니 경로를 소진하면 스스로 떨어진다" 로
// 충분하다고 봤는데 **틀렸다**. validatePath 는 committed 재검증용이라 안에서
//   validateCandidate(ego, path, visible, reason, start_index, 1U, ...)
// 로 최소점수를 **1 로 덮어쓴다**. 거의 끝난 기동을 "점이 몇 개 안 남았다"는 이유로
// 무효화하지 않으려는 의도이고, 완료 판정은 다른 곳이 한다. 그 결과 앞에 1 점만 남은
// 경로도 통과한다 — 실측으로 확인했다(test_raceline_spline.cpp 의
// GuidanceRepublishNeedsItsOwnLengthFloorToTerminate).
//
// 재발행에는 그 완화가 정확히 해롭다. 앞에 1~2 점만 남은 경로를 다시 내보내면 F1 이
// 고치려던 바로 그 실패가 다른 경로로 재현된다 — 컨트롤러의 walk_forward 는 열린 경로에서
// 끝점에 멈추므로, 남은 점이 짧을수록 L1 목표가 코앞에 붙고 요구 횡가속이 튄다.
// 그래서 재발행 후보에만 별도의 기하 하한(GuidanceGeometrySufficient)을 건다. 그 하한이
// 재발행의 자기종료를 보장한다 — 무한히 옛 기하를 붙들 수 없다.

#pragma once

#include <algorithm>
#include <cmath>

namespace local_planning::hold_gate
{

// 이 경로가 컨트롤러에 줄 만큼 남았는가. 보류 게이트의 재발행(F1)과 정지 경로 하한(F3)이
// 같은 질문을 하므로 같은 판정을 쓴다.
//
// ■ 왜 점 수가 아니라 전방 거리인가
// 컨트롤러의 walk_forward 는 열린 경로에서 끝점에 멈춘다. 그러면 L1 목표가 경로 끝점이
// 되고, 필요한 것은 그 끝점까지의 **호 길이**다:
//   L1 = l1_offset + v·l1_speed_gain = 0.6 + 0.32·v   (f1tenth_control 기본값)
// 점 수로는 이것을 못 잰다 — densifyPath() 가 짧은 정지 접두부를 minimum_path_points 까지
// 세분 보간하면서 **기하 구간은 일부러 그대로 두기** 때문이다(2026-08-14, 정지점을 밀지
// 않으려고). 그 경로는 점 수 하한을 만족하면서도 L1 목표가 여전히 코앞이다.
inline bool GuidanceGeometrySufficient(double forward_span_m, double minimum_forward_m)
{
  if (!(minimum_forward_m > 0.0)) {
    return true;   // 하한 꺼짐 = 종전 거동.
  }
  return forward_span_m >= minimum_forward_m;
}

// ── F5 (2026-08-22): 몇 사이클이면 끝날 보류를 래치로 키우지 않는다 ──────────
//
// ■ 무엇을 재는가
// 보류 게이트가 붙잡는 조건은 "기동 장애물의 뒤끝이 아직 앞에 있다" 하나다. 그 조건이
// 물리적으로 해소되는 데 걸리는 시간은 (뒤끝까지 거리)/(현재 속도) 로 바로 나온다.
//
// ■ 실측이 보여 준 불비례 (run_20260822_073013, 자율 98 s)
//     래치 t   속도    뒤끝     통과필요     실제 래치     완전정지   배율
//     34.70   3.96   0.18 m   0.045 s     2.37 s      1.04 s    52x
//     57.95   3.43   0.27 m   0.079 s     2.11 s      0.60 s    27x
//     80.60   3.75   0.18 m   0.048 s     2.78 s      1.76 s    58x
//   보류가 물리적으로 요구한 시간의 합은 1.25 s 인데, 차가 실제로 서 있던 시간은 5.12 s.
//   18 cm / 50 ms 짜리 조건이 2.4 s 완전 정지로 증폭된다. 증폭의 정체는 safe-stop 래치가
//   무거운 상태라는 것이다 — 푸는 조건이 "hard-valid 회피 8회 확인" / "정지한 채 회랑
//   관측" / "위험구간 통과" 셋뿐이고 전부 초 단위다.
//
// ■ 정지 차량은 **명시적으로** 배제한다
//   처음 판은 나눗셈 바닥(0.1 m/s)에 기대어 "속도 0 이면 통과필요시간이 발산하니 알아서
//   걸러진다"고 봤는데 **틀렸다** — 시험이 잡았다. 뒤끝이 아주 작으면(0.02 m) 정지
//   상태에서도 0.2 s 로 계산돼 문턱을 통과한다. 차가 멈춰 있으면 장애물을 영영 못
//   지나가므로, 그 상태에서 회피 기하를 계속 발행하는 것은 정확히 하면 안 되는 일이다.
//   그 경우는 safe-stop 래치의 "정지한 채 회랑이 깨끗함을 관측" 해제 사다리가 담당한다.
//   그러므로 속도 게이트를 따로 둔다. 나눗셈 바닥은 수치 안전용으로만 남긴다.
//
// ■ 노출 상한
//   추정은 매 사이클 다시 계산되므로 감속하면 통과필요시간이 늘어 자연히 꺼진다. 그래도
//   검출 흔들림으로 뒤끝이 계속 작게 보고될 수 있으므로 **누적 보류 시간**으로 한 번 더
//   묶는다. 그 상한을 넘으면 종전대로 래치한다.
inline bool HoldClearsWithinGrace(
  double rear_ahead_m, double speed_mps, double deferred_sec,
  double grace_sec, double max_brief_hold_sec, double min_speed_mps = 0.5)
{
  if (!(grace_sec > 0.0)) {
    return false;   // 하한 꺼짐 = 종전 거동.
  }
  if (!(rear_ahead_m > 0.0) || !std::isfinite(rear_ahead_m) || !std::isfinite(speed_mps)) {
    return false;
  }
  if (!(speed_mps > min_speed_mps)) {
    return false;   // 멈춰 있거나 서는 중 — 래치와 그 해제 사다리에 맡긴다.
  }
  if (deferred_sec > max_brief_hold_sec) {
    return false;   // 너무 오래 붙잡았다. 진짜로 못 지나가는 상황이다.
  }
  const double clear_sec = rear_ahead_m / std::max(speed_mps, 0.1);   // 바닥은 수치 안전용.
  return clear_sec < grace_sec;
}

enum class HoldRecovery
{
  // committed_result_ 를 그대로 재발행한다 (P0 경로).
  kPublishCommitted,
  // 마지막으로 발행된 유효 회피 기하를 재발행한다 (P3 경로 — F1 이 추가한 단).
  kPublishLastGuidance,
  // 둘 다 못 쓴다. 옛 기하를 명령하는 대신 안전정지를 래치한다.
  kSafeStop,
};

// `*_present` 는 "경로에 점이 있는가", `*_valid` 는 "그 경로가 지금 이 ego 로 validatePath 를
// 통과했는가". 비어 있는 경로는 검증조차 하지 않으므로 present=false 면 valid 도 false 여야
// 한다 — 호출자가 그 규약을 지킨다.
//
// `guidance_long_enough` 는 GuidanceGeometrySufficient() 의 결과다. committed 쪽에는 같은
// 하한을 걸지 않는다 — 그쪽은 F1 이전부터 있던 P0 경로이고, 거동을 바꾸지 않는 것이 이
// 변경의 범위다.
inline HoldRecovery SelectHoldRecovery(
  bool committed_present, bool committed_valid,
  bool guidance_present, bool guidance_valid, bool guidance_long_enough)
{
  if (committed_present && committed_valid) {
    return HoldRecovery::kPublishCommitted;
  }
  if (guidance_present && guidance_valid && guidance_long_enough) {
    return HoldRecovery::kPublishLastGuidance;
  }
  return HoldRecovery::kSafeStop;
}

}  // namespace local_planning::hold_gate
