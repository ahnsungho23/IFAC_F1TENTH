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

// 기동 장애물 기억의 수명 규칙 (2026-08-22).
//
// 이 규칙이 지키는 계약은 둘이다:
//   ① 활성 기동이 없으면 기억을 비운다 — 단 **연속 확인 후에만**.
//      즉시 비우면 2026-08-20 이 막으려던 "한 콜백 IDLE 구멍"이 되살아나고,
//      영영 안 비우면 반 바퀴 뒤 wrap 으로 유령 안전정지가 된다.
//   ② 기억을 "전방"으로 볼 상한은 반 바퀴가 아니라 계획 지평이다.
//      반 바퀴는 41 m 트랙에서 여유가 4 cm 라 지나친 물체가 전방으로 뒤집힌다.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "local_planning/maneuver_memory.hpp"

namespace local_planning
{
namespace
{

TEST(ManeuverMemoryClear, RequiresConsecutiveEmptyFramesBeforeClearing)
{
  int empty_frames = 0;
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames)) << "1 프레임";
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames)) << "2 프레임";
  EXPECT_TRUE(maneuverMemoryShouldClear(true, 3, empty_frames)) << "3 프레임에 비운다";
}

TEST(ManeuverMemoryClear, ANonEmptyFrameBreaksTheStreak)
{
  // 이것이 "한 콜백 IDLE 구멍" 회귀 보호의 본체다: 기동이 살아 있는 프레임이 하나라도
  // 끼면 처음부터 다시 센다.
  int empty_frames = 0;
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames));
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames));
  EXPECT_FALSE(maneuverMemoryShouldClear(false, 3, empty_frames));
  EXPECT_EQ(empty_frames, 0);
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames));
  EXPECT_FALSE(maneuverMemoryShouldClear(true, 3, empty_frames));
  EXPECT_TRUE(maneuverMemoryShouldClear(true, 3, empty_frames));
}

TEST(ManeuverMemoryClear, ZeroFramesKeepsTheLegacyLeakingBehaviour)
{
  // 되돌리기 경로(maneuver_memory_clear_frames: 0)는 **절대** 비우지 않아야 한다.
  int empty_frames = 0;
  for (int i = 0; i < 50; ++i) {
    EXPECT_FALSE(maneuverMemoryShouldClear(true, 0, empty_frames)) << "frame " << i;
  }
  int negative_frames = 0;
  EXPECT_FALSE(maneuverMemoryShouldClear(true, -1, negative_frames));
}

TEST(ManeuverMemoryClear, OneFrameConfigurationClearsImmediately)
{
  int empty_frames = 0;
  EXPECT_TRUE(maneuverMemoryShouldClear(true, 1, empty_frames));
}

TEST(ManeuverMemoryMaxAhead, ClipsTheHalfTrackWrapAmbiguity)
{
  // 실차 트랙: 41.34 m -> 반 바퀴 20.67 m. 21 m 전에 지나친 물체가 20.63 m 로 되읽히던
  // 그 4 cm 구간이 계획 지평(15 m)으로 잘려 사라져야 한다.
  const double kTrack = 41.34;
  const double max_ahead = maneuverMemoryMaxAhead(kTrack, 15.0);
  EXPECT_DOUBLE_EQ(max_ahead, 15.0);
  EXPECT_GT(20.63, max_ahead) << "run_20260822_002336 t=111.67 의 유령이 여전히 전방으로 읽힌다";
  EXPECT_GT(16.15, max_ahead) << "t=112.69 의 같은 유령";
  EXPECT_LT(1.78, max_ahead) << "진짜 근접 보류(최대 1.78 m)는 반드시 살아 있어야 한다";
}

TEST(ManeuverMemoryMaxAhead, NeverExceedsHalfTrack)
{
  // 아주 짧은 트랙에서 지평이 반 바퀴보다 크면 반 바퀴가 이긴다 — 그 위는 원형거리상
  // "뒤"라서 애초에 전방일 수 없다.
  EXPECT_DOUBLE_EQ(maneuverMemoryMaxAhead(20.0, 15.0), 10.0);
  EXPECT_DOUBLE_EQ(maneuverMemoryMaxAhead(41.34, 100.0), 20.67);
}

TEST(ManeuverMemoryMaxAhead, FallsBackToHalfTrackWhenUnset)
{
  // 되돌리기/오설정 경로: 유한한 양수가 아니면 종전 거동(반 바퀴).
  EXPECT_DOUBLE_EQ(maneuverMemoryMaxAhead(41.34, 0.0), 20.67);
  EXPECT_DOUBLE_EQ(maneuverMemoryMaxAhead(41.34, -1.0), 20.67);
  EXPECT_DOUBLE_EQ(
    maneuverMemoryMaxAhead(41.34, std::numeric_limits<double>::quiet_NaN()), 20.67);
  EXPECT_DOUBLE_EQ(
    maneuverMemoryMaxAhead(41.34, std::numeric_limits<double>::infinity()), 20.67);
}

}  // namespace
}  // namespace local_planning
