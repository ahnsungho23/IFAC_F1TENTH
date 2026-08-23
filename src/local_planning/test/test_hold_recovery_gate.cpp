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

// 보류 게이트의 복구원 사다리 (F1, 2026-08-22).
//
// 이 테스트가 고정하는 계약:
//   ① 사다리는 **두 단**이다. 두 번째 단(마지막 유효 회피 기하)이 없으면 P3 운영에서 이
//      게이트는 구조적으로 항상 안전정지가 된다 — committed_result_ 는 commitAvoidance()
//      로만 채워지고 그 호출부는 전부 P0/레거시이기 때문이다. 실차에서 「보류」 18 회 중
//      16 회가 안전정지로 갔고 13 회의 사유가 "커밋 경로 없음" 이었다
//      (run_072312 / run_073013, 2026-08-22).
//   ② 순서는 committed 가 먼저다. P0 운영의 거동이 바뀌면 안 된다.
//   ③ **검증을 통과하지 못한 기하는 절대 발행하지 않는다.** 2026-08-20 에 막은 실패 모드가
//      정확히 "옛 기하 무검증 재발행" 이었다 (run_062020 t=538.00, 111 점짜리 지나간 경로
//      → 0.5 s 뒤 벽). 있다는 것만으로는 부족하고 valid 여야 한다.

#include <gtest/gtest.h>

#include <cmath>

#include "local_planning/hold_recovery_gate.hpp"

namespace local_planning::hold_gate
{
namespace
{

// ── ① 두 번째 단이 존재한다 (F1 의 본체) ───────────────────────────────────

TEST(SelectHoldRecovery, EmptyCommittedFallsThroughToTheLastGuidanceInsteadOfStopping)
{
  // 이것이 P3 운영의 상시 상태다: committed_result_ 는 기본생성 그대로고, 마지막으로
  // 발행된 P3 기동만 남아 있다. 종전 사다리는 여기서 곧장 안전정지였다.
  EXPECT_EQ(
    SelectHoldRecovery(false, false, true, true, true),
    HoldRecovery::kPublishLastGuidance);
}

TEST(SelectHoldRecovery, InvalidCommittedAlsoFallsThroughRatherThanStopping)
{
  // committed 에 글로벌 핸드오프 루프가 들어 있었지만 진입 불연속으로 떨어진 경우 —
  // 실차 3 회 (run_073013 t=64.50 / 74.23, run_072312 t=244.27). 그때도 마지막 유효
  // 기하가 살아 있으면 그것을 쓴다.
  EXPECT_EQ(
    SelectHoldRecovery(true, false, true, true, true),
    HoldRecovery::kPublishLastGuidance);
}

// ── ② committed 가 먼저다 ─────────────────────────────────────────────────

TEST(SelectHoldRecovery, ValidCommittedWinsOverTheLastGuidance)
{
  // P0 운영의 거동 보존: 두 후보가 다 유효해도 committed 가 이긴다.
  EXPECT_EQ(
    SelectHoldRecovery(true, true, true, true, true),
    HoldRecovery::kPublishCommitted);
}

TEST(SelectHoldRecovery, ValidCommittedIsUsedEvenWithNoGuidanceAtAll)
{
  EXPECT_EQ(
    SelectHoldRecovery(true, true, false, false, false),
    HoldRecovery::kPublishCommitted);
}

// ── ③ 검증 실패는 발행 금지 ────────────────────────────────────────────────

TEST(SelectHoldRecovery, PresentButInvalidGuidanceStopsInsteadOfRepublishing)
{
  // 🔴 F1 의 안전 경계. 여기서 kPublishLastGuidance 를 돌려주면 2026-08-20 의 벽충돌
  //    (지나간 111 점 경로 재발행) 이 그대로 돌아온다.
  EXPECT_EQ(
    SelectHoldRecovery(false, false, true, false, true),
    HoldRecovery::kSafeStop);
  EXPECT_EQ(
    SelectHoldRecovery(true, false, true, false, true),
    HoldRecovery::kSafeStop);
}

TEST(SelectHoldRecovery, NothingUsableStops)
{
  EXPECT_EQ(
    SelectHoldRecovery(false, false, false, false, false),
    HoldRecovery::kSafeStop);
}

TEST(SelectHoldRecovery, AbsentPathIsNeverPublishedEvenIfMislabelledValid)
{
  // 호출자 규약(present=false 면 valid 도 false)이 깨져도 빈 경로를 내보내지 않는다.
  // 빈 경로 발행은 컨트롤러에서 글로벌 폴백을 유발하고, 그 라인은 장애물을 관통한다.
  EXPECT_EQ(
    SelectHoldRecovery(false, true, false, true, true),
    HoldRecovery::kSafeStop);
  EXPECT_EQ(
    SelectHoldRecovery(false, true, true, true, true),
    HoldRecovery::kPublishLastGuidance);
}

// ── ④ 재발행 기하 하한 ─────────────────────────────────────────────────────
//
// validatePath 는 committed 재검증용이라 minimum_path_points 를 **1 로 덮어쓴다**
// (validateCandidate(..., start_index, 1U, ...)). 그래서 "유효"만으로는 앞에 1 점만 남은
// 경로도 통과한다. 재발행에 그것을 허용하면 F1 이 고치려던 실패가 다른 경로로 그대로
// 재현된다 — 컨트롤러 walk_forward 가 열린 경로의 끝점에 멈추므로 남은 점이 짧을수록 L1
// 목표가 코앞에 붙고 요구 횡가속이 튄다 (실차 최대 32.2 m/s², 그립 예산 6.80).
//
// 이 계약은 시험이 먼저 잡았다: test_raceline_spline.cpp 의
// GuidanceRepublishNeedsItsOwnLengthFloorToTerminate 가 validatePath 단독으로는 "경로
// 끝에 닿을 때까지 계속 유효" 임을 보여 주었고, 그래서 하한을 따로 붙였다.

TEST(GuidanceGeometrySufficient, MeasuresForwardSpanAgainstTheL1Floor)
{
  // 하한 2.5 m = L1(0.6 + 0.32*v) 를 v=5.9 m/s 까지 덮는 값.
  EXPECT_FALSE(GuidanceGeometrySufficient(0.00, 2.5));
  EXPECT_FALSE(GuidanceGeometrySufficient(0.18, 2.5));   // 실차 퇴화 경로 (073013 t=93.7)
  EXPECT_FALSE(GuidanceGeometrySufficient(0.50, 2.5));   // 실차 최소 정지 경로 스팬
  EXPECT_FALSE(GuidanceGeometrySufficient(1.76, 2.5));   // 실차 정지 경로 p50 — 여전히 부족
  EXPECT_TRUE(GuidanceGeometrySufficient(2.50, 2.5));
  EXPECT_TRUE(GuidanceGeometrySufficient(19.3, 2.5));    // 정상 회피 경로
}

TEST(GuidanceGeometrySufficient, ZeroOrNegativeFloorDisablesTheCheck)
{
  // 파라미터를 0 으로 두면 종전(하한 없음) 거동으로 돌아간다 — 되돌릴 길을 남겨 둔다.
  EXPECT_TRUE(GuidanceGeometrySufficient(0.0, 0.0));
  EXPECT_TRUE(GuidanceGeometrySufficient(0.18, 0.0));
  EXPECT_TRUE(GuidanceGeometrySufficient(0.0, -1.0));
  // NaN 하한도 검사를 끄는 쪽으로 떨어져야 한다 — 조용히 모든 경로를 막으면 영구 정지다.
  EXPECT_TRUE(GuidanceGeometrySufficient(0.18, std::nan("")));
}

TEST(SelectHoldRecovery, ValidButTooShortGuidanceStopsInsteadOfRepublishing)
{
  // 🔴 이것이 없으면 재발행이 2~3 점짜리 퇴화 경로를 만들어 낸다.
  EXPECT_EQ(
    SelectHoldRecovery(false, false, true, true, false),
    HoldRecovery::kSafeStop);
}

TEST(SelectHoldRecovery, TheLengthFloorDoesNotApplyToTheCommittedPath)
{
  // committed 는 F1 이전부터 있던 P0 경로다. 하한을 그쪽까지 넓히는 것은 이 변경의
  // 범위가 아니며, 넓히면 P0 거동이 조용히 바뀐다.
  EXPECT_EQ(
    SelectHoldRecovery(true, true, true, true, false),
    HoldRecovery::kPublishCommitted);
}

// ── ⑤ F5: 곧 끝날 보류는 래치로 키우지 않는다 ──────────────────────────────
//
// 실측(run_20260822_073013): 뒤끝 0.18 m / 3.96 m/s = 0.045 s 면 끝날 조건이 2.37 s
// 래치 + 1.04 s 완전정지가 됐다(52x). 자율 98 s 동안 보류가 물리적으로 요구한 시간의
// 합은 1.25 s 인데 실제 정지는 5.12 s 였다.

TEST(HoldClearsWithinGrace, CatchesTheMeasuredCentimetreScaleHolds)
{
  // 실차 세 건. 전부 문턱 0.3 s 안에 들어와야 한다.
  EXPECT_TRUE(HoldClearsWithinGrace(0.18, 3.96, 0.0, 0.3, 0.6));   // t=34.70, 0.045 s
  EXPECT_TRUE(HoldClearsWithinGrace(0.27, 3.43, 0.0, 0.3, 0.6));   // t=57.95, 0.079 s
  EXPECT_TRUE(HoldClearsWithinGrace(0.18, 3.75, 0.0, 0.3, 0.6));   // t=80.60, 0.048 s
  EXPECT_TRUE(HoldClearsWithinGrace(1.02, 4.60, 0.0, 0.3, 0.6));   // t=93.68, 0.222 s
}

TEST(HoldClearsWithinGrace, LeavesGenuinelyLongHoldsToTheLatch)
{
  // t=24.17 (1.40 m / 3.05 = 0.46 s) 와 t=97.81 (1.31 / 3.30 = 0.40 s) 는 문턱 밖이다.
  // 이들은 진짜로 시간이 걸리는 통과라 종전대로 래치가 맞다.
  EXPECT_FALSE(HoldClearsWithinGrace(1.40, 3.05, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(1.31, 3.30, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(8.83, 3.96, 0.0, 0.3, 0.6));   // 072312 t=135.36
}

TEST(HoldClearsWithinGrace, AStoppedCarNeverQualifies)
{
  // 🔴 가장 중요한 경계. 차가 멈춰 있으면 장애물을 **영영** 못 지나간다. 거기서 완화가
  //    켜지면 지나갈 수 없는 장애물 앞에서 회피 기하를 계속 발행하게 된다.
  //    처음엔 나눗셈 바닥(0.1 m/s)만으로 막힌다고 봤는데 뒤끝이 작으면 뚫린다 —
  //    0.02 m / 0.1 = 0.2 s < 0.3. 그래서 속도 게이트를 명시적으로 둔다.
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, 0.0, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, 0.02, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(0.02, 0.0, 0.0, 0.3, 0.6));   // 나눗셈 바닥만으론 뚫린다
  EXPECT_FALSE(HoldClearsWithinGrace(0.02, 0.49, 0.0, 0.3, 0.6));  // 게이트 바로 아래
  // 서는 중이 아니라 실제로 굴러가는 크립이면 통과한다 — 곧 지나가는 게 맞다.
  EXPECT_TRUE(HoldClearsWithinGrace(0.18, 0.7, 0.0, 0.3, 0.6));
  EXPECT_TRUE(HoldClearsWithinGrace(0.02, 0.51, 0.0, 0.3, 0.6));
}

TEST(HoldClearsWithinGrace, ExposureIsBoundedByAccumulatedHoldTime)
{
  // 검출이 흔들려 뒤끝이 계속 작게 보고되면 추정만으로는 안 꺼진다. 누적 보류 시간이
  // 상한을 넘으면 종전대로 래치한다.
  EXPECT_TRUE(HoldClearsWithinGrace(0.18, 3.96, 0.59, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, 3.96, 0.61, 0.3, 0.6));
}

TEST(HoldClearsWithinGrace, ZeroGraceDisablesIt)
{
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, 3.96, 0.0, 0.0, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, 3.96, 0.0, -1.0, 0.6));
}

TEST(HoldClearsWithinGrace, RejectsNonFiniteOrNonPositiveInputs)
{
  EXPECT_FALSE(HoldClearsWithinGrace(0.0, 3.96, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(-0.1, 3.96, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(std::nan(""), 3.96, 0.0, 0.3, 0.6));
  EXPECT_FALSE(HoldClearsWithinGrace(0.18, std::nan(""), 0.0, 0.3, 0.6));
}

// ── 사다리 전수 ────────────────────────────────────────────────────────────

TEST(SelectHoldRecovery, ExhaustiveTruthTable)
{
  // 32 조합 전부. 규칙은 하나다: committed 가 있고 유효하면 그것, 아니면 guidance 가 있고
  // 유효하고 **충분히 길면** 그것, 아니면 정지.
  for (const bool cp : {false, true}) {
    for (const bool cv : {false, true}) {
      for (const bool gp : {false, true}) {
        for (const bool gv : {false, true}) {
          for (const bool gl : {false, true}) {
            const HoldRecovery expected = (cp && cv) ? HoldRecovery::kPublishCommitted :
              ((gp && gv && gl) ? HoldRecovery::kPublishLastGuidance : HoldRecovery::kSafeStop);
            EXPECT_EQ(SelectHoldRecovery(cp, cv, gp, gv, gl), expected)
              << "cp=" << cp << " cv=" << cv << " gp=" << gp << " gv=" << gv << " gl=" << gl;
          }
        }
      }
    }
  }
}

}  // namespace
}  // namespace local_planning::hold_gate
