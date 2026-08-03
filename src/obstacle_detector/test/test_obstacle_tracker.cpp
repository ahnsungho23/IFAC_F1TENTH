#include <gtest/gtest.h>

#include <vector>

#include "obstacle_detector/obstacle_tracker.hpp"

namespace obstacle_detector
{
namespace
{

// 각 테스트에서 형상 차이보다 상태 전이에 집중하도록 만드는 기본 검출값이다.
Detection makeDetection(double s, double d = 0.0)
{
    Detection detection;
    detection.s = s;
    detection.d = d;
    detection.s_half_extent = 0.2;
    detection.d_right_offset = -0.1;
    detection.d_left_offset = 0.1;
    detection.size = 0.4;
    detection.x_min = s - 0.2;
    detection.x_max = s + 0.2;
    detection.y_min = d - 0.2;
    detection.y_max = d + 0.2;
    return detection;
}

// 단위 테스트를 짧고 결정적으로 만들기 위한 추적기 파라미터이다.
TrackerParams testParams()
{
    TrackerParams params;
    params.assoc_use_mahalanobis = false;
    params.assoc_gate = 2.0;
    params.min_hits_confirm = 3;
    params.static_confirm_frames = 3;
    params.dynamic_confirm_frames = 5;
    params.dyn_velocity_mahalanobis_gate = 0.0;
    params.dyn_max_abs_yaw_rate = 1.5;
    params.static_ref_gate = 0.0;
    return params;
}

// 세 번째 관측에서 같은 ID로 임시 정적 발행을 시작하고 연속 저속 관측 뒤 정적으로 확정한다.
TEST(ObstacleTrackerClassification, PublishesAtHitThreeAndKeepsIdThroughStaticPromotion)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int id = tracker.tracks().front().id;
    EXPECT_FALSE(tracker.tracks().front().classified);
    EXPECT_EQ(tracker.tracks().front().motion_class, MotionClass::Pending);

    tracker.update({makeDetection(10.0)}, 0.1);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_FALSE(tracker.tracks().front().classified);
    EXPECT_EQ(tracker.tracks().front().motion_class, MotionClass::Pending);

    tracker.update({makeDetection(10.0)}, 0.2);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_TRUE(tracker.tracks().front().classified);
    EXPECT_EQ(tracker.tracks().front().motion_class, MotionClass::ProvisionalStatic);
    EXPECT_TRUE(tracker.tracks().front().is_static);

    tracker.update({makeDetection(10.0)}, 0.3);
    tracker.update({makeDetection(10.0)}, 0.4);
    tracker.update({makeDetection(10.0)}, 0.5);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_TRUE(tracker.tracks().front().classified);
    EXPECT_EQ(tracker.tracks().front().motion_class, MotionClass::ConfirmedStatic);
    EXPECT_TRUE(tracker.tracks().front().is_static);
}

// 미관측 예측 프레임에서도 마지막 Frenet 종·횡방향 형상은 독립적으로 유지해야 한다.
TEST(ObstacleTrackerGeometry, RetainsIndependentFrenetExtentsDuringPrediction)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    auto detection = makeDetection(10.0, 0.4);
    detection.s_half_extent = 0.30;
    detection.d_right_offset = -0.08;
    detection.d_left_offset = 0.12;
    tracker.update({detection}, 0.0);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().s_half_extent, 0.30);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_right_offset, -0.08);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_left_offset, 0.12);
    EXPECT_TRUE(tracker.tracks().front().is_visible);

    tracker.update({}, 0.1);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().s_half_extent, 0.30);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_right_offset, -0.08);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_left_offset, 0.12);
    EXPECT_FALSE(tracker.tracks().front().is_visible);
}

// 급회전 중의 이동량은 증거로 쌓지 않고, 신뢰 가능한 연속 관측만 동적 승격에 사용한다.
TEST(ObstacleTrackerClassification, RequiresConsecutiveReliableMotionBeforeDynamicPromotion)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    double stamp = 0.0;
    double s = 5.0;
    for (int i = 0; i < 3; ++i)
    {
        tracker.update({makeDetection(s)}, stamp, 0.0, true);
        stamp += 0.1;
        s += 0.1;
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int id = tracker.tracks().front().id;
    EXPECT_EQ(tracker.tracks().front().motion_class, MotionClass::ProvisionalStatic);

    // 뚜렷한 병진 운동도 자차가 급회전 중이면 동적 증거로 누적하면 안 된다.
    for (int i = 0; i < 10; ++i)
    {
        tracker.update({makeDetection(s)}, stamp, 2.0, true);
        stamp += 0.1;
        s += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
        EXPECT_EQ(tracker.tracks().front().id, id);
        EXPECT_NE(tracker.tracks().front().motion_class, MotionClass::Dynamic);
        EXPECT_EQ(tracker.tracks().front().dyn_streak, 0);
    }

    bool became_dynamic = false;
    for (int i = 0; i < 10; ++i)
    {
        tracker.update({makeDetection(s)}, stamp, 0.0, true);
        stamp += 0.1;
        s += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
        EXPECT_EQ(tracker.tracks().front().id, id);
        if (tracker.tracks().front().motion_class == MotionClass::Dynamic)
        {
            became_dynamic = true;
            break;
        }
    }

    EXPECT_TRUE(became_dynamic);
    EXPECT_FALSE(tracker.tracks().front().is_static);
}

}  // namespace
}  // namespace obstacle_detector
