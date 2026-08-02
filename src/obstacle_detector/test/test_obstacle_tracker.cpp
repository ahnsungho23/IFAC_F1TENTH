#include <gtest/gtest.h>

#include <vector>

#include "obstacle_detector/obstacle_tracker.hpp"

namespace obstacle_detector
{
namespace
{

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

    // Even clear translational motion must not accumulate dynamic evidence while ego yaw is rapid.
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
