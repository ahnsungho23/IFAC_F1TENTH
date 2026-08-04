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

TEST(ObstacleTrackerGeometry, SmoothsExtentsFastGrowSlowShrink)
{
    auto params = testParams();
    params.extent_shrink_alpha = 0.25;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    const auto base = makeDetection(10.0);  // d_left_offset 0.1, d_right_offset -0.1
    tracker.update({base}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_left_offset, 0.1);

    // A larger measurement expands the envelope immediately.
    auto bigger = makeDetection(10.0);
    bigger.d_left_offset = 0.5;
    bigger.d_right_offset = -0.5;
    tracker.update({bigger}, 0.1);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_left_offset, 0.5);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_right_offset, -0.5);

    // Smaller measurements shrink by alpha per matched frame instead of snapping back:
    // 0.5 + 0.25 * (0.1 - 0.5) = 0.4, then 0.4 + 0.25 * (0.1 - 0.4) = 0.325.
    tracker.update({base}, 0.2);
    EXPECT_NEAR(tracker.tracks().front().d_left_offset, 0.4, 1.0e-9);
    EXPECT_NEAR(tracker.tracks().front().d_right_offset, -0.4, 1.0e-9);
    tracker.update({base}, 0.3);
    EXPECT_NEAR(tracker.tracks().front().d_left_offset, 0.325, 1.0e-9);
}

TEST(ObstacleTrackerGeometry, LegacyOverwriteWhenShrinkAlphaIsOne)
{
    auto params = testParams();
    params.extent_shrink_alpha = 1.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    auto bigger = makeDetection(10.0);
    bigger.d_left_offset = 0.5;
    tracker.update({bigger}, 0.0);
    tracker.update({makeDetection(10.0)}, 0.1);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().d_left_offset, 0.1);
}

TEST(ObstacleTrackerGeometry, EnvelopeStabilityStreakTracksSettledMeasurements)
{
    auto params = testParams();
    params.envelope_stability_tolerance_m = 0.10;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    // Stable measurements: the streak grows once per matched frame.
    tracker.update({makeDetection(10.0)}, 0.0);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);
    tracker.update({makeDetection(10.02)}, 0.1);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 1);
    tracker.update({makeDetection(10.01)}, 0.2);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 2);

    // A morphing envelope (fan-shaped scatter) resets the streak.
    auto morph = makeDetection(10.0);
    morph.d_left_offset = 0.45;
    morph.d_right_offset = -0.55;
    tracker.update({morph}, 0.3);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);

    // The envelope fast-grew to the morph size; repeating it is stable again.
    tracker.update({morph}, 0.4);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 1);
    // A jumping centre resets the streak.
    tracker.update({makeDetection(10.6)}, 0.5);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);
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

TEST(ObstacleTrackerGeometry, ForwardWindowRejectsOpponentBehindEgoAcrossWrap)
{
    // selectOpponent only considers dynamic objects whose wrap-aware forward distance from the
    // ego falls inside (0, track_length/2). An object just behind the ego wraps to L - eps and
    // must be excluded; this locks the wrap math that predicate relies on.
    std::vector<FrenetProjector::Waypoint> wpnts(2);
    wpnts[0].x = 0.0;
    wpnts[0].s = 0.0;
    wpnts[1].x = 100.0;
    wpnts[1].s = 100.0;
    FrenetProjector frenet;
    frenet.build(wpnts, true, 100.0);
    const double L = frenet.raceline_length();

    const auto forward_distance = [&frenet, L](double s, double ego_s) {
        double ahead = frenet.wrapDelta(s, ego_s);
        if (ahead < 0.0)
        {
            ahead += L;
        }
        return ahead;
    };

    EXPECT_LT(forward_distance(51.0, 50.0), 0.5 * L);    // 1 m ahead: candidate
    EXPECT_GE(forward_distance(49.0, 50.0), 0.5 * L);    // 1 m behind: rejected
    EXPECT_LT(forward_distance(0.5, 99.5), 0.5 * L);     // 1 m ahead across the wrap: candidate
    EXPECT_GE(forward_distance(98.5, 99.5), 0.5 * L);    // 1 m behind across the wrap: rejected
}

}  // namespace
}  // namespace obstacle_detector
