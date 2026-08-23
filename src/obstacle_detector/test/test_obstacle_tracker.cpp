#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
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
    params.confirmation_window = 5;
    params.meas_var_s = 0.001;
    params.meas_var_d = 0.001;
    params.process_var_vs = 0.01;
    params.process_var_vd = 0.01;
    params.dynamic_vote_window = 5;
    params.dynamic_vote_required = 3;
    params.static_vote_window = 15;
    params.static_vote_required = 10;
    params.position_history_size = 15;
    params.static_min_observations = 10;
    params.dynamic_to_static_min_observations = 20;
    params.dynamic_to_static_vote_required = 15;
    // The occlusion hold would keep confirmed static tracks alive across the short miss bursts
    // these tests use to exercise retire/dormant-identity mechanics. Disable it here; the hold
    // has its own dedicated test that re-enables it explicitly.
    params.static_lost_hold_sec = 0.0;
    return params;
}

TEST(ObstacleTrackerClassification, SeparatesThreeOfFiveExistenceFromMotionStatus)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int id = tracker.tracks().front().id;
    EXPECT_EQ(tracker.tracks().front().track_status, TrackStatus::Raw);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Unknown);

    tracker.update({}, 0.1);
    tracker.update({makeDetection(10.0)}, 0.2);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_EQ(tracker.tracks().front().track_status, TrackStatus::Tentative);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Unknown);

    tracker.update({}, 0.3);
    tracker.update({makeDetection(10.0)}, 0.4);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Unknown);
    EXPECT_TRUE(tracker.tracks().front().is_static);
}

TEST(ObstacleTrackerIdentity, ReassociatesSameSpatialClusterAfterKalmanGateRejects)
{
    TrackerParams params = testParams();
    params.assoc_use_mahalanobis = true;
    params.assoc_mahalanobis_gate = 0.01;
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    const int id = tracker.tracks().front().id;
    tracker.update({makeDetection(10.0)}, 0.1);
    tracker.update({makeDetection(10.0)}, 0.2);

    // Centre displacement 0.45 m fails the deliberately strict Mahalanobis gate, but the two
    // 0.4 m-wide Frenet envelopes have only a 0.05 m edge gap and are one spatial cluster.
    tracker.update({makeDetection(10.45)}, 0.3);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_EQ(tracker.lastStats().spatially_reassociated, 1U);
    EXPECT_EQ(tracker.lastStats().spawned, 0U);
}

TEST(ObstacleTrackerIdentity, ReusesPhysicalIdAcrossKalmanTrackLifetimes)
{
    TrackerParams params = testParams();
    params.min_hits_confirm = 1;
    params.confirmation_window = 1;
    params.envelope_stability_frames = 0;
    params.ttl_static = 1;
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    params.physical_id_memory_sec = 2.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int id = tracker.tracks().front().id;
    const int track_uid = tracker.tracks().front().track_uid;

    tracker.update({}, 0.1);
    ASSERT_TRUE(tracker.tracks().empty());
    tracker.update({makeDetection(10.05)}, 0.2);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, id);
    EXPECT_NE(tracker.tracks().front().track_uid, track_uid);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 1U);
}

TEST(ObstacleTrackerIdentity, DormantIdUsesLastMeasurementInsteadOfPredictedPosition)
{
    TrackerParams params = testParams();
    params.min_hits_confirm = 1;
    params.confirmation_window = 1;
    params.envelope_stability_frames = 0;
    params.ttl_static = 2;
    params.ttl_dynamic = 2;
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    params.physical_id_memory_sec = 2.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    tracker.update({makeDetection(11.0)}, 0.1);
    tracker.update({makeDetection(12.0)}, 0.2);
    tracker.update({makeDetection(13.0)}, 0.3);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int physical_id = tracker.tracks().front().id;
    EXPECT_DOUBLE_EQ(tracker.tracks().front().last_measured_s, 13.0);

    // Let the constant-velocity state run far beyond the final measured footprint before the
    // track retires. The dormant identity must remain anchored at the 13.0 m measurement.
    tracker.update({}, 0.8);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_GT(std::abs(tracker.tracks().front().s() - 13.0), 0.5);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().last_measured_s, 13.0);
    tracker.update({}, 0.9);
    ASSERT_TRUE(tracker.tracks().empty());

    tracker.update({makeDetection(13.05)}, 1.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, physical_id);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 1U);
}

TEST(ObstacleTrackerIdentity, StableStaticAnchorIgnoresLateViewpointDrift)
{
    TrackerParams params = testParams();
    params.min_hits_confirm = 1;
    params.confirmation_window = 1;
    params.static_vote_required = 1;
    params.static_min_observations = 1;
    // This test short-circuits every STATIC gate to isolate anchor behaviour, so the observation
    // span and translation gates go too -- a spawn-frame track has no translation history at all,
    // so its proof is NaN. Do not copy this into tests that care about classification timing.
    params.static_min_observation_sec = 0.0;
    params.translation_corroboration_enable = false;
    params.envelope_stability_frames = 0;
    params.ttl_static = 1;
    params.ttl_dynamic = 1;
    params.physical_id_reassociation_gap_s = 0.05;
    params.physical_id_reassociation_gap_d = 0.05;
    params.physical_id_reassociation_gap_map = 0.05;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_TRUE(tracker.tracks().front().stable_identity_anchor.valid);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().stable_identity_anchor.s, 10.0);
    const int physical_id = tracker.tracks().front().id;

    // Later scan faces drift along the same object. The first statistically Static measurement
    // remains the identity anchor even if subsequent motion evidence becomes Dynamic.
    tracker.update({makeDetection(10.2)}, 0.1);
    tracker.update({makeDetection(10.4)}, 0.2);
    tracker.update({makeDetection(10.6)}, 0.3);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().last_measured_s, 10.6);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().stable_identity_anchor.s, 10.0);

    tracker.update({}, 0.4);
    ASSERT_TRUE(tracker.tracks().empty());
    tracker.update({makeDetection(10.0)}, 0.5);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, physical_id);
    EXPECT_TRUE(tracker.tracks().front().stable_identity_anchor.valid);
    EXPECT_DOUBLE_EQ(tracker.tracks().front().stable_identity_anchor.s, 10.0);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 1U);
}

TEST(ObstacleTrackerIdentity, DormantIdCanReassociateByMapAabbWhenFrenetGateFails)
{
    TrackerParams params = testParams();
    params.min_hits_confirm = 1;
    params.confirmation_window = 1;
    params.envelope_stability_frames = 0;
    params.ttl_static = 1;
    params.ttl_dynamic = 1;
    params.physical_id_reassociation_gap_s = 0.05;
    params.physical_id_reassociation_gap_d = 0.05;
    params.physical_id_reassociation_gap_map = 0.05;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const int physical_id = tracker.tracks().front().id;
    tracker.update({}, 0.1);
    ASSERT_TRUE(tracker.tracks().empty());

    Detection curved_projection = makeDetection(12.0);
    curved_projection.x_min = 9.8;
    curved_projection.x_max = 10.2;
    curved_projection.y_min = -0.2;
    curved_projection.y_max = 0.2;
    tracker.update({curved_projection}, 0.2);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().id, physical_id);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 1U);
}

TEST(ObstacleTrackerIdentity, DoesNotReuseRetiredIdForAnotherCluster)
{
    TrackerParams params = testParams();
    params.min_hits_confirm = 1;
    params.confirmation_window = 1;
    params.envelope_stability_frames = 0;
    params.ttl_static = 1;
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    params.physical_id_memory_sec = 2.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    const int old_id = tracker.tracks().front().id;
    tracker.update({}, 0.1);
    tracker.update({makeDetection(11.0)}, 0.2);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_NE(tracker.tracks().front().id, old_id);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 0U);
}

TEST(ObstacleTrackerIdentity, SplitTracksShareOnePhysicalObjectId)
{
    TrackerParams params = testParams();
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    tracker.update({makeDetection(10.0)}, 0.0);
    const int physical_id = tracker.tracks().front().id;
    const int original_track_uid = tracker.tracks().front().track_uid;
    tracker.update({makeDetection(10.0), makeDetection(10.45)}, 0.1);

    ASSERT_EQ(tracker.tracks().size(), 2U);
    EXPECT_EQ(tracker.tracks()[0].id, physical_id);
    EXPECT_EQ(tracker.tracks()[1].id, physical_id);
    EXPECT_NE(tracker.tracks()[0].track_uid, tracker.tracks()[1].track_uid);
    EXPECT_EQ(tracker.tracks()[0].track_uid, original_track_uid);
    EXPECT_EQ(tracker.lastStats().physical_id_reused, 1U);
}

TEST(ObstacleTrackerClassification, RepeatedStableMapMeasurementsBecomeStatic)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    for (int i = 0; i < 14; ++i)
    {
        const double noise = 0.005 * std::sin(static_cast<double>(i));
        tracker.update({makeDetection(10.0 + noise, 0.2 - noise)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Static);
    EXPECT_GE(tracker.tracks().front().static_vote_count, 10);
    EXPECT_LE(tracker.tracks().front().map_position_rms, 0.10);
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

    // A missed measurement breaks the consecutive evidence even though TTL retains the track.
    tracker.update({}, 0.3);
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_FALSE(tracker.tracks().front().is_visible);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);
    tracker.update({makeDetection(10.01)}, 0.4);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 1);

    // B2-① (2026-08-20): 순수 확장(접근 중 보이는 면이 커짐)은 스트릭을 끊지 않는다.
    // 구 대칭 리셋은 성장 프레임마다 발행 게이트를 닫아 두 토픽에 2프레임 구멍을 만들었다
    // (run_192006 id35: 접근 2초 동안 구멍 5회).
    auto morph = makeDetection(10.0);
    morph.d_left_offset = 0.45;
    morph.d_right_offset = -0.55;
    tracker.update({morph}, 0.5);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 2);

    // The envelope fast-grew to the morph size; repeating it is still stable.
    tracker.update({morph}, 0.6);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 3);

    // 수축(부채꼴 산란 유령의 서명)은 종전대로 리셋한다.
    tracker.update({makeDetection(10.0)}, 0.7);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);

    // 다시 커지는 것은 즉시 재적립된다 (fast-grow 와 같은 방향).
    tracker.update({morph}, 0.8);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 1);

    // A jumping centre resets the streak (확장 여부와 무관).
    auto jumped = morph;
    jumped.s = 10.6;
    tracker.update({jumped}, 0.9);
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, 0);
}

TEST(ObstacleTrackerClassification, ConstantMapVelocityBecomesDynamicByRecentVotes)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    double stamp = 0.0;
    double s = 5.0;
    int physical_id = -1;
    bool became_dynamic = false;
    for (int i = 0; i < 30; ++i)
    {
        tracker.update({makeDetection(s)}, stamp);
        stamp += 0.1;
        s += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
        if (physical_id < 0)
        {
            physical_id = tracker.tracks().front().id;
        }
        EXPECT_EQ(tracker.tracks().front().id, physical_id);
        if (tracker.tracks().front().motion_status == MotionStatus::Dynamic)
        {
            became_dynamic = true;
            break;
        }
    }

    EXPECT_TRUE(became_dynamic);
    EXPECT_GE(tracker.tracks().front().dynamic_vote_count, 3);
    EXPECT_FALSE(tracker.tracks().front().is_static);
    EXPECT_EQ(tracker.tracks().front().id, physical_id);
}

TEST(ObstacleTrackerTranslation, GrowthAloneProvesNoTranslation)
{
    // Far edge extends, near edge holds: the box was revealed, not moved.
    EXPECT_DOUBLE_EQ(anchoredAxisTranslation(-15.10, -15.06, -15.10, -14.92), 0.0);
    // Both edges shrink toward each other proves nothing either.
    EXPECT_DOUBLE_EQ(anchoredAxisTranslation(0.0, 1.0, 0.2, 0.8), 0.0);
    // Both edges move the same way: the shared part is proven translation.
    EXPECT_DOUBLE_EQ(anchoredAxisTranslation(0.0, 1.0, 0.3, 1.5), 0.3);
    // Shrinking while moving: only the smaller shared shift is proven (min edge -0.4, max -1.2).
    EXPECT_DOUBLE_EQ(anchoredAxisTranslation(0.0, 1.0, -0.4, -0.2), -0.4);
}

// Replays the ifac_track blind-corner failure: obstacle 0 is occluded until s~4.9, then its
// measured AABB extends away from the ego over ~20 scans while its near faces stay put. The
// centroid travels far enough to drive the map-frame Kalman velocity past the dynamic chi-square
// threshold, which used to move the track out of /static_obs and leave the planner blind.
TEST(ObstacleTrackerTranslation, ProgressiveRevelationKeepsStaticObstacleOutOfDynamic)
{
    const auto revealing = [](double fraction) {
        Detection detection;
        const double x_min = -15.099;
        const double x_max = -15.056 + 0.135 * fraction;
        const double y_min = -0.387;
        const double y_max = -0.132 + 0.065 * fraction;
        detection.x_min = x_min;
        detection.x_max = x_max;
        detection.y_min = y_min;
        detection.y_max = y_max;
        detection.s = 9.0 + 0.5 * (x_min + x_max) - (-15.078);
        detection.d = 0.5 * (y_min + y_max);
        detection.s_half_extent = 0.5 * (x_max - x_min);
        detection.d_right_offset = y_min - detection.d;
        detection.d_left_offset = y_max - detection.d;
        detection.size = std::max(x_max - x_min, y_max - y_min);
        return detection;
    };

    const auto run = [&revealing](bool corroboration_enabled) {
        TrackerParams params = testParams();
        params.translation_corroboration_enable = corroboration_enabled;
        ObstacleTracker tracker;
        tracker.configure(params, nullptr);
        double stamp = 0.0;
        bool became_dynamic = false;
        for (int i = 0; i < 40; ++i)
        {
            const double fraction = std::min(1.0, i / 20.0);
            tracker.update({revealing(fraction)}, stamp);
            stamp += 0.004;  // detector runs at ~250 Hz
            if (tracker.tracks().size() == 1U &&
                tracker.tracks().front().motion_status == MotionStatus::Dynamic)
            {
                became_dynamic = true;
            }
        }
        return became_dynamic;
    };

    EXPECT_TRUE(run(false)) << "regression witness: revelation used to be read as motion";
    EXPECT_FALSE(run(true)) << "a revealed static obstacle must keep its static layer";
}

TEST(ObstacleTrackerTranslation, ProgressiveRevelationProvesNoTranslation)
{
    TrackerParams params = testParams();
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    Detection detection = makeDetection(10.0);
    tracker.update({detection}, 0.0);
    ASSERT_EQ(tracker.tracks().size(), 1U);

    // Only the far x edge is revealed. The safety AABB grows, but no pair of samples proves that
    // the box translated, so no dynamic vote may be cast.
    for (int i = 1; i <= 20; ++i)
    {
        detection.x_max += 0.01;
        detection.s += 0.005;
        detection.s_half_extent += 0.005;
        tracker.update({detection}, 0.01 * i);
    }

    ASSERT_EQ(tracker.tracks().size(), 1U);
    const Track &track = tracker.tracks().front();
    EXPECT_NEAR(track.provable_map_translation_m, 0.0, 1.0e-12);
    EXPECT_NEAR(track.provable_frenet_translation_m, 0.0, 1.0e-12);
    EXPECT_FALSE(track.translation_evidence_persistent);
    EXPECT_NE(track.motion_status, MotionStatus::Dynamic);
}

// Regression for a48b374: the map KF was fed an accumulated two-edge translation instead of the
// raw AABB centre. min(|dmin|,|dmax|) is biased towards zero, so under centimetre edge noise the
// filter read a moving opponent 25-50% too slow and stopped calling it DYNAMIC. Noise is what
// exposes this, so it must be injected -- the noise-free synthetic tests cannot see it.
TEST(ObstacleTrackerTranslation, NoisyEdgesDoNotHideAModeratelyMovingOpponent)
{
    TrackerParams params = testParams();
    params.dynamic_min_translation_m = 0.18;
    params.dynamic_translation_persistence_sec = 0.30;
    params.translation_window_sec = 0.20;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    // Deterministic pseudo-noise on each edge independently, +/- 2 cm.
    uint32_t seed = 12345U;
    const auto noise = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return (static_cast<double>(seed >> 8U) / 16777216.0 - 0.5) * 0.04;
    };

    constexpr double speed = 1.2;   // m/s -- below the 1.5 m/s the old 0.30 m gate demanded
    constexpr double dt = 0.025;    // 40 Hz
    for (int i = 0; i <= 80; ++i)
    {
        const double travelled = speed * dt * i;
        Detection detection = makeDetection(10.0 + travelled);
        detection.x_min = 10.0 + travelled - 0.2 + noise();
        detection.x_max = 10.0 + travelled + 0.2 + noise();
        detection.y_min = -0.2 + noise();
        detection.y_max = 0.2 + noise();
        tracker.update({detection}, dt * i);
    }

    ASSERT_EQ(tracker.tracks().size(), 1U);
    const Track &track = tracker.tracks().front();
    EXPECT_TRUE(track.translation_evidence_persistent);
    EXPECT_EQ(track.motion_status, MotionStatus::Dynamic);
}

// Regression for the run_0814_010624 id42 failure: a one-off localization jump moves both edges
// together, so it passes the two-edge proof. It may not pass the persistence requirement, because
// the proof collapses once the pre-jump samples age out of translation_window_sec.
TEST(ObstacleTrackerTranslation, SingleLocalizationJumpDoesNotProduceDynamicVotes)
{
    TrackerParams params = testParams();
    params.dynamic_min_translation_m = 0.18;
    params.dynamic_translation_persistence_sec = 0.30;
    params.translation_window_sec = 0.20;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    constexpr double dt = 0.025;
    constexpr double jump = 0.35;   // well above dynamic_min_translation_m
    for (int i = 0; i <= 80; ++i)
    {
        const double offset = i >= 20 ? jump : 0.0;
        Detection detection = makeDetection(10.0 + offset);
        detection.x_min = 10.0 + offset - 0.2;
        detection.x_max = 10.0 + offset + 0.2;
        tracker.update({detection}, dt * i);
        EXPECT_NE(tracker.tracks().front().motion_status, MotionStatus::Dynamic)
            << "a single pose jump must never reach DYNAMIC (frame " << i << ")";
    }
}

TEST(ObstacleTrackerTranslation, FrenetEnvelopeCorroboratesTurningOpponent)
{
    TrackerParams params = testParams();
    params.dynamic_min_translation_m = 0.18;
    params.dynamic_translation_persistence_sec = 0.30;
    params.translation_window_sec = 0.20;
    params.translation_history_max_samples = 64;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    constexpr double speed = 1.8;
    constexpr double dt = 0.004;
    constexpr double half_length = 0.28;
    constexpr double half_width = 0.1435;
    constexpr double curvature = 1.316266519;
    constexpr double initial_yaw = 2.0929290258215203;

    double map_proof_at_window_end = std::numeric_limits<double>::quiet_NaN();
    double frenet_proof_at_window_end = std::numeric_limits<double>::quiet_NaN();
    // Run past dynamic_translation_persistence_sec so the sustained proof can actually vote.
    for (int i = 0; i <= 150; ++i)
    {
        const double distance = speed * dt * i;
        const double yaw = initial_yaw + curvature * distance;
        const double center_x =
            (std::sin(yaw) - std::sin(initial_yaw)) / curvature;
        const double center_y =
            (-std::cos(yaw) + std::cos(initial_yaw)) / curvature;
        const double half_x =
            half_length * std::abs(std::cos(yaw)) +
            half_width * std::abs(std::sin(yaw));
        const double half_y =
            half_length * std::abs(std::sin(yaw)) +
            half_width * std::abs(std::cos(yaw));

        Detection detection = makeDetection(5.0 + distance);
        detection.s_half_extent = half_length;
        detection.x_min = center_x - half_x;
        detection.x_max = center_x + half_x;
        detection.y_min = center_y - half_y;
        detection.y_max = center_y + half_y;
        tracker.update({detection}, dt * i);
        if (i == 50)
        {
            map_proof_at_window_end = tracker.tracks().front().provable_map_translation_m;
            frenet_proof_at_window_end =
                tracker.tracks().front().provable_frenet_translation_m;
        }
    }

    ASSERT_EQ(tracker.tracks().size(), 1U);
    const Track &track = tracker.tracks().front();
    // Yaw rotation changes the map AABB's width/height, which cancels part of the two-edge
    // translation. The track-aligned Frenet envelope keeps proving the full arc length, so it is
    // the stronger of the two proofs for a turning object -- that is why it exists.
    EXPECT_GT(frenet_proof_at_window_end, map_proof_at_window_end);
    EXPECT_GT(frenet_proof_at_window_end, params.dynamic_min_translation_m);
    EXPECT_TRUE(track.translation_evidence_persistent);
    EXPECT_EQ(track.motion_status, MotionStatus::Dynamic);
}

// The shipped config/obstacle_detector.yaml values, so these two tests exercise what actually
// runs rather than the deliberately loose testParams().
TrackerParams shippedParams()
{
    TrackerParams p;
    p.meas_var_s = 0.01;
    p.meas_var_d = 0.01;
    p.process_var_vs = 2.0;
    p.process_var_vd = 8.0;
    p.assoc_gate = 0.5;
    p.assoc_use_mahalanobis = true;
    p.min_hits_confirm = 3;
    p.confirmation_window = 5;
    p.static_lost_hold_sec = 5.0;
    p.envelope_stability_tolerance_m = 0.10;
    p.envelope_stability_frames = 2;
    p.dynamic_chi2_threshold = 9.21;
    p.static_chi2_threshold = 5.99;
    p.frenet_dynamic_chi2_threshold = 6.63;
    p.dynamic_vote_window = 5;
    p.dynamic_vote_required = 3;
    p.static_vote_window = 15;
    p.static_vote_required = 10;
    p.position_history_size = 15;
    p.static_min_observations = 10;
    p.static_max_position_rms = 0.10;
    p.static_min_observation_sec = 0.25;
    p.dynamic_to_static_min_observations = 20;
    p.dynamic_to_static_vote_required = 15;
    p.dynamic_to_static_max_position_rms = 0.08;
    p.translation_corroboration_enable = true;
    p.translation_window_sec = 0.20;
    p.translation_history_max_samples = 64;
    p.dynamic_min_translation_m = 0.18;
    p.dynamic_translation_persistence_sec = 0.30;
    return p;
}

Detection makeCarDetection(double x)
{
    Detection detection;
    detection.s = x;
    detection.d = 0.0;
    detection.s_half_extent = 0.28;
    detection.d_right_offset = -0.14;
    detection.d_left_offset = 0.14;
    detection.size = 0.56;
    detection.x_min = x - 0.28;
    detection.x_max = x + 0.28;
    detection.y_min = -0.14;
    detection.y_max = 0.14;
    return detection;
}

// Regression for the 2026-08-20 two-agent run: every STATIC entry gate counted FRAMES. The gym
// bridge publishes scans at 250 Hz (`create_timer(0.004)`), so the 15-frame vote window is 60 ms
// and the 15-sample position RMS spans 3.5 cm at 2 m/s -- while the freshly spawned track still
// carried the zero velocity it was initialized with. The opponent was confirmed STATIC 44 ms after
// track birth and published on /confirmed_static_obs, which local_planning then avoided.
TEST(ObstacleTrackerClassification, MovingOpponentNeverBecomesStaticAtHighScanRate)
{
    ObstacleTracker tracker;
    tracker.configure(shippedParams(), nullptr);

    constexpr double dt = 0.004;   // gym bridge scan timer
    constexpr double speed = 2.0;
    for (int i = 0; i <= 300; ++i)
    {
        tracker.update({makeCarDetection(10.0 + speed * dt * i)}, dt * i);
        ASSERT_EQ(tracker.tracks().size(), 1U);
        EXPECT_NE(tracker.tracks().front().motion_status, MotionStatus::Static)
            << "a 2 m/s opponent must never be confirmed STATIC (frame " << i << ")";
    }
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Dynamic);
}

// The counterpart: the time-based gate must not stop a genuinely stationary obstacle from being
// confirmed. It stays provisional on /static_obs meanwhile, so the delay is on the safe side.
TEST(ObstacleTrackerClassification, StationaryObstacleStillReachesStaticAtHighScanRate)
{
    ObstacleTracker tracker;
    tracker.configure(shippedParams(), nullptr);

    constexpr double dt = 0.004;
    for (int i = 0; i <= 200; ++i)
    {
        tracker.update({makeCarDetection(10.0)}, dt * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Static);
}

TEST(ObstacleTrackerIdentity, SpatialFallbackAlsoReconnectsDynamicTrack)
{
    TrackerParams params = testParams();
    params.assoc_use_mahalanobis = true;
    params.assoc_mahalanobis_gate = 0.5;
    params.physical_id_reassociation_gap_s = 0.10;
    params.physical_id_reassociation_gap_d = 0.05;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    double stamp = 0.0;
    double s = 5.0;
    while (stamp < 3.0)
    {
        tracker.update({makeDetection(s)}, stamp);
        ASSERT_EQ(tracker.tracks().size(), 1U);
        if (tracker.tracks().front().motion_status == MotionStatus::Dynamic)
        {
            break;
        }
        stamp += 0.1;
        s += 0.1;
    }
    ASSERT_EQ(tracker.tracks().front().motion_status, MotionStatus::Dynamic);
    const int physical_id = tracker.tracks().front().id;
    const int track_uid = tracker.tracks().front().track_uid;

    // The expected next centre is s + 0.1. Move the measured scan face another 0.45 m: the
    // strict Kalman gate rejects it, while the two 0.4 m envelopes remain one spatial cluster.
    stamp += 0.1;
    s += 0.55;
    tracker.update({makeDetection(s)}, stamp);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Dynamic);
    EXPECT_EQ(tracker.tracks().front().id, physical_id);
    EXPECT_EQ(tracker.tracks().front().track_uid, track_uid);
    EXPECT_EQ(tracker.lastStats().spatially_reassociated, 1U);
    EXPECT_EQ(tracker.lastStats().spawned, 0U);
}

TEST(ObstacleTrackerClassification, SmallVelocityWithTinyCovarianceIsDynamicEvidence)
{
    const auto params = testParams();
    const Eigen::Vector2d velocity(0.02, 0.0);
    const Eigen::Matrix2d covariance = 1.0e-6 * Eigen::Matrix2d::Identity();
    const auto result = evaluateVelocityEvidence(velocity, covariance, params);

    EXPECT_TRUE(result.covariance_valid);
    EXPECT_GT(result.statistic, params.dynamic_chi2_threshold);
    EXPECT_EQ(result.evidence, MotionEvidence::DynamicEvidence);
}

TEST(ObstacleTrackerClassification, FrenetEvidenceIsDynamicOnly)
{
    TrackerParams params = testParams();
    params.frenet_dynamic_chi2_threshold = 6.63;

    const auto dynamic = evaluateFrenetVelocityEvidence(2.0, 0.10, params);
    EXPECT_TRUE(dynamic.covariance_valid);
    EXPECT_GT(dynamic.statistic, params.frenet_dynamic_chi2_threshold);
    EXPECT_EQ(dynamic.evidence, MotionEvidence::DynamicEvidence);

    // A small Ts is NOT evidence of stationarity: the Frenet filter is driven by the raw AABB
    // centre, which also drifts while a stationary obstacle is progressively revealed.
    const auto stationary = evaluateFrenetVelocityEvidence(0.01, 0.10, params);
    EXPECT_TRUE(stationary.covariance_valid);
    EXPECT_EQ(stationary.evidence, MotionEvidence::Uncertain);
}

TEST(ObstacleTrackerClassification, InvalidFrenetCovarianceProducesUncertainEvidence)
{
    TrackerParams params = testParams();
    const auto result = evaluateFrenetVelocityEvidence(2.0, -1.0, params);
    EXPECT_FALSE(result.covariance_valid);
    EXPECT_EQ(result.evidence, MotionEvidence::Uncertain);
}

TEST(ObstacleTrackerClassification, LargeVelocityWithHugeCovarianceIsNotDynamicEvidence)
{
    const auto params = testParams();
    const Eigen::Vector2d velocity(2.0, 0.0);
    const Eigen::Matrix2d covariance = 100.0 * Eigen::Matrix2d::Identity();
    const auto result = evaluateVelocityEvidence(velocity, covariance, params);

    EXPECT_TRUE(result.covariance_valid);
    EXPECT_LT(result.statistic, params.static_chi2_threshold);
    EXPECT_NE(result.evidence, MotionEvidence::DynamicEvidence);
}

TEST(ObstacleTrackerClassification, SingularCovarianceIsRegularizedWithoutCrash)
{
    const auto params = testParams();
    const auto result = evaluateVelocityEvidence(
        Eigen::Vector2d(0.1, 0.0), Eigen::Matrix2d::Zero(), params);

    EXPECT_TRUE(result.covariance_valid);
    EXPECT_TRUE(std::isfinite(result.statistic));
}

TEST(ObstacleTrackerClassification, NonFiniteCovarianceProducesUncertainEvidence)
{
    const auto params = testParams();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
    covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
    const auto result = evaluateVelocityEvidence(
        Eigen::Vector2d(0.1, 0.0), covariance, params);

    EXPECT_FALSE(result.covariance_valid);
    EXPECT_EQ(result.evidence, MotionEvidence::Uncertain);
}

TEST(ObstacleTrackerClassification, RejectsVoteRequirementsLargerThanWindow)
{
    auto params = testParams();
    params.dynamic_vote_required = params.dynamic_vote_window + 1;
    ObstacleTracker tracker;
    EXPECT_THROW(tracker.configure(params, nullptr), std::invalid_argument);
}

TEST(ObstacleTrackerClassification, PredictionOnlyFrameAddsNoVoteAndDecaysConfidence)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);
    for (int i = 0; i < 14; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().front().motion_status, MotionStatus::Static);
    const std::size_t evidence_size =
        tracker.tracks().front().motion_evidence_history.size();
    const double confidence = tracker.tracks().front().static_confidence;

    tracker.update({}, 1.4);

    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().motion_evidence_history.size(), evidence_size);
    EXPECT_LT(tracker.tracks().front().static_confidence, confidence);
    EXPECT_GT(tracker.tracks().front().time_since_last_measurement, 0.0);
}

TEST(ObstacleTrackerClassification, StoppedDynamicNeedsConservativeStaticReentry)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);
    double stamp = 0.0;
    double position = 5.0;
    for (int i = 0; i < 40; ++i)
    {
        tracker.update({makeDetection(position)}, stamp);
        stamp += 0.1;
        position += 0.1;
        if (tracker.tracks().front().motion_status == MotionStatus::Dynamic)
        {
            break;
        }
    }
    ASSERT_EQ(tracker.tracks().front().motion_status, MotionStatus::Dynamic);

    const double stopped_position = position;
    for (int i = 0; i < 10; ++i)
    {
        tracker.update({makeDetection(stopped_position)}, stamp);
        stamp += 0.1;
    }
    EXPECT_EQ(tracker.tracks().front().motion_status, MotionStatus::Dynamic);

    bool returned_static = false;
    for (int i = 0; i < 60; ++i)
    {
        tracker.update({makeDetection(stopped_position)}, stamp);
        stamp += 0.1;
        if (tracker.tracks().front().motion_status == MotionStatus::Static)
        {
            returned_static = true;
            break;
        }
    }
    EXPECT_TRUE(returned_static);
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

TEST(ObstacleTrackerLifetime, ConfirmedStaticTrackHeldThroughOcclusionForHoldSeconds)
{
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 1.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    // Confirm a stationary track (3-of-5 measurement votes).
    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);
    const int streak_before_occlusion = tracker.tracks().front().envelope_stable_streak;

    // 8 prediction-only frames over 0.8 s: far beyond the 3-frame TTL, inside the 1.0 s hold.
    // The map-fixed object must stay alive with its publish-stability evidence intact.
    for (int i = 0; i < 8; ++i)
    {
        tracker.update({}, 0.4 + 0.1 * i);
        ASSERT_EQ(tracker.tracks().size(), 1U)
            << "confirmed static track retired during the occlusion hold (miss " << i << ")";
    }
    EXPECT_EQ(tracker.tracks().front().envelope_stable_streak, streak_before_occlusion);

    // Once the hold expires the ordinary frame TTL retires the track.
    tracker.update({}, 1.45);
    EXPECT_TRUE(tracker.tracks().empty());
}

TEST(ObstacleTrackerLifetime, FreeSpaceRefutationRetiresHeldEnvelopeAfterConsecutiveScans)
{
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 5.0;
    params.static_hold_freespace_refute_frames = 3;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);

    // Two refuted scans are not enough: a single stray beam through the envelope must not retire a
    // map-fixed object, and the streak resets the moment the scan stops seeing through it.
    const auto refute = [](const Track &) { return true; };
    const auto no_evidence = [](const Track &) { return false; };
    double stamp = 0.4;
    for (int i = 0; i < 2; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, refute);
        stamp += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U) << "retired before the refutation streak (" << i
                                               << ")";
    }
    tracker.update({}, stamp, 0.0, true, false, no_evidence);
    stamp += 0.1;
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().freespace_refute_streak, 0);

    for (int i = 0; i < 2; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, refute);
        stamp += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
    }
    tracker.update({}, stamp, 0.0, true, false, refute);
    EXPECT_TRUE(tracker.tracks().empty())
        << "a held envelope survived " << params.static_hold_freespace_refute_frames
        << " consecutive scans that saw straight through it";
}

TEST(ObstacleTrackerLifetime, OcclusionHoldSurvivesWithoutFreeSpaceEvidence)
{
    // Same hold window and refuter wiring as the retirement test; only the evidence differs. The
    // hold must still cover a plain occlusion, otherwise the refutation would have silently
    // replaced static_lost_hold_sec instead of bounding it.
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 5.0;
    params.static_hold_freespace_refute_frames = 3;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);

    double stamp = 0.4;
    for (int i = 0; i < 10; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, [](const Track &) { return false; });
        stamp += 0.1;
    }
    EXPECT_EQ(tracker.tracks().size(), 1U);
}

TEST(ObstacleTrackerLifetime, TranslatingUnknownTrackIsNotHeldAsMapFixedObject)
{
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.ttl_dynamic = 3;
    params.static_lost_hold_sec = 1.0;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    // A moving object measured while ego motion withholds dynamic votes: it translates provably,
    // yet stays Confirmed+UNKNOWN, which is exactly the state that used to earn a full
    // static_lost_hold_sec of frozen republication on /static_obs.
    double stamp = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        tracker.update({makeDetection(10.0 + 0.15 * i)}, stamp, 0.0, true, true);
        stamp += 0.04;
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    const Track &moving = tracker.tracks().front();
    ASSERT_EQ(moving.track_status, TrackStatus::Confirmed);
    ASSERT_EQ(moving.motion_status, MotionStatus::Unknown);
    ASSERT_TRUE(std::isfinite(moving.provable_translation_m));
    ASSERT_GE(moving.provable_translation_m, params.dynamic_min_translation_m);

    // Occlusion: the ordinary frame TTL must retire it instead of the map-fixed-object hold.
    for (int i = 0; i < 4; ++i)
    {
        tracker.update({}, stamp);
        stamp += 0.04;
    }
    EXPECT_TRUE(tracker.tracks().empty())
        << "a provably translating UNKNOWN track was held as if it were a map-fixed object";
}

TEST(ObstacleTrackerLifetime, HoldFallsBackToPermissiveWhenCorroborationDisabled)
{
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 1.0;
    // Without translation corroboration there is no evidence to judge an UNKNOWN track by, so the
    // hold must keep its previous permissive behaviour rather than silently retiring everything.
    params.translation_corroboration_enable = false;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    double stamp = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        tracker.update({makeDetection(10.0 + 0.15 * i)}, stamp, 0.0, true, true);
        stamp += 0.04;
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);
    ASSERT_EQ(tracker.tracks().front().motion_status, MotionStatus::Unknown);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({}, stamp);
        stamp += 0.04;
    }
    EXPECT_EQ(tracker.tracks().size(), 1U);
}

TEST(ObstacleTrackerClassification, EgoMotionTransientWithholdsDynamicVotes)
{
    // Identical translating cadence; only the ego-motion transient flag differs. The witness run
    // proves the cadence reaches DYNAMIC, so the suppressed run demonstrates the vote hold
    // rather than an inert scenario.
    const auto becomes_dynamic = [](bool ego_motion_transient) {
        ObstacleTracker tracker;
        tracker.configure(testParams(), nullptr);
        double stamp = 0.0;
        double s = 5.0;
        while (stamp < 3.0)
        {
            tracker.update({makeDetection(s)}, stamp, 0.0, true, ego_motion_transient);
            if (!tracker.tracks().empty() &&
                tracker.tracks().front().motion_status == MotionStatus::Dynamic)
            {
                return true;
            }
            stamp += 0.1;
            s += 0.1;
        }
        return false;
    };
    EXPECT_TRUE(becomes_dynamic(false))
        << "regression witness: a 1 m/s translating track must reach DYNAMIC";
    EXPECT_FALSE(becomes_dynamic(true))
        << "dynamic votes must be withheld while ego acceleration is transient";
}

TEST(CrossCovarianceClamp, ClipsToCauchySchwarzBound)
{
    // Multi-member merge: diag comes from member A (per-axis max), cross term from member B,
    // so the raw weighted mean can exceed the bound. sqrt(1.0 * 4.0) = 2.0.
    EXPECT_NEAR(clampCrossCovariance(5.0, 1.0, 4.0), 0.99 * 2.0, 1e-12);
    EXPECT_NEAR(clampCrossCovariance(-5.0, 1.0, 4.0), -0.99 * 2.0, 1e-12);
    // Inside the bound the value passes through untouched (the usual single-member case).
    EXPECT_NEAR(clampCrossCovariance(0.5, 1.0, 4.0), 0.5, 1e-12);
}

TEST(CrossCovarianceClamp, DegenerateInputCollapsesToZero)
{
    // A zero variance mathematically forces cov = 0 (reachable: the merged diagonal is a
    // max-accumulator seeded at 0.0 and every member may hold P(i,i) == 0.0).
    EXPECT_DOUBLE_EQ(clampCrossCovariance(0.3, 0.0, 4.0), 0.0);
    // NaN must never reach the topic. Only the cross term can carry NaN into the helper
    // (the diagonal max-accumulator drops NaN), but the guard covers both operands.
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(std::numeric_limits<double>::quiet_NaN(), 1.0, 4.0), 0.0);
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(0.3, std::numeric_limits<double>::quiet_NaN(), 4.0), 0.0);
    // An infinite variance survives the max-accumulator, so it is a real input here.
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(0.3, std::numeric_limits<double>::infinity(), 4.0), 0.0);
    // Defensive only: mergeLayer() cannot produce a negative variance (max-accumulator seeded
    // at 0.0). This locks the contract for any future caller passing a raw P(i,i).
    EXPECT_DOUBLE_EQ(clampCrossCovariance(0.3, -1.0, 4.0), 0.0);
}

// ------------------------------------------------------------------------------------------------
// 후방 사각 회수 (2026-08-22)
//
// free-space 반증과 같은 자리에 붙는 두 번째 회수 근거다. 반증은 "상자를 관통한 빔"을 요구해
// 시야 안에서만 성립하고, 이쪽은 시야 밖만 맡는다 — 둘은 겹치지 않는다.
// ------------------------------------------------------------------------------------------------

TEST(ObstacleTrackerLifetime, RearBlindRetiresHeldEnvelopeAfterConsecutiveScans)
{
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 5.0;
    params.static_hold_rear_blind_retire_frames = 3;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_EQ(tracker.tracks().front().track_status, TrackStatus::Confirmed);

    const auto blind = [](const Track &) { return true; };
    const auto visible = [](const Track &) { return false; };
    double stamp = 0.4;
    // 두 스캔으로는 부족하다 — TF 한 프레임 흔들림으로 map-fixed 물체를 지우면 안 된다.
    for (int i = 0; i < 2; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, nullptr, blind);
        stamp += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U) << "streak 미달인데 회수됐다 (" << i << ")";
    }
    // 한 프레임이라도 볼 수 있으면 streak 이 끊긴다.
    tracker.update({}, stamp, 0.0, true, false, nullptr, visible);
    stamp += 0.1;
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().rear_blind_streak, 0);

    for (int i = 0; i < 2; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, nullptr, blind);
        stamp += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
    }
    tracker.update({}, stamp, 0.0, true, false, nullptr, blind);
    EXPECT_TRUE(tracker.tracks().empty())
        << "봉투가 " << params.static_hold_rear_blind_retire_frames
        << " 스캔 연속 후방 사각이었는데도 static_lost_hold_sec 를 끝까지 채웠다";
}

TEST(ObstacleTrackerLifetime, RearBlindRetirementIsSkippedWithoutThePredicate)
{
    // 술어를 안 넣으면(또는 frames=0) 종전 순수 차폐 hold 와 비트 단위로 같아야 한다.
    // 이것이 되돌리기 경로(static_hold_rear_blind_retire_enable:=false)의 계약이다.
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 5.0;
    params.static_hold_rear_blind_retire_frames = 3;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);

    double stamp = 0.4;
    for (int i = 0; i < 8; ++i)
    {
        tracker.update({}, stamp);   // 술어 없음
        stamp += 0.1;
    }
    EXPECT_EQ(tracker.tracks().size(), 1U)
        << "술어가 없는데도 회수됐다 — 되돌리기 경로가 종전 거동을 재현하지 못한다";

    // frames=0 도 같은 계약이다.
    TrackerParams disabled = params;
    disabled.static_hold_rear_blind_retire_frames = 0;
    ObstacleTracker off;
    off.configure(disabled, nullptr);
    for (int i = 0; i < 4; ++i)
    {
        off.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(off.tracks().size(), 1U);
    stamp = 0.4;
    const auto blind = [](const Track &) { return true; };
    for (int i = 0; i < 8; ++i)
    {
        off.update({}, stamp, 0.0, true, false, nullptr, blind);
        stamp += 0.1;
    }
    EXPECT_EQ(off.tracks().size(), 1U) << "frames=0 인데 회수됐다";
}

TEST(ObstacleTrackerLifetime, MeasurementResetsTheRearBlindStreak)
{
    // 유령이 다시 보이면(= 측정이 붙으면) 회수 근거가 사라진다. 후진·스핀 복구 경로다.
    TrackerParams params = testParams();
    params.ttl_static = 3;
    params.static_lost_hold_sec = 5.0;
    params.static_hold_rear_blind_retire_frames = 3;
    ObstacleTracker tracker;
    tracker.configure(params, nullptr);

    for (int i = 0; i < 4; ++i)
    {
        tracker.update({makeDetection(10.0)}, 0.1 * i);
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);

    const auto blind = [](const Track &) { return true; };
    double stamp = 0.4;
    for (int i = 0; i < 2; ++i)
    {
        tracker.update({}, stamp, 0.0, true, false, nullptr, blind);
        stamp += 0.1;
    }
    ASSERT_EQ(tracker.tracks().size(), 1U);
    ASSERT_EQ(tracker.tracks().front().rear_blind_streak, 2);

    tracker.update({makeDetection(10.0)}, stamp, 0.0, true, false, nullptr, blind);
    stamp += 0.1;
    ASSERT_EQ(tracker.tracks().size(), 1U);
    EXPECT_EQ(tracker.tracks().front().rear_blind_streak, 0)
        << "측정이 붙었는데 후방 사각 streak 이 남았다";
}

}  // namespace
}  // namespace obstacle_detector
