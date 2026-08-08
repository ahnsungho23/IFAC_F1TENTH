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

TEST(ObstacleTrackerClassification, ConstantMapVelocityBecomesDynamicByRecentVotes)
{
    ObstacleTracker tracker;
    tracker.configure(testParams(), nullptr);

    double stamp = 0.0;
    double s = 5.0;
    bool became_dynamic = false;
    for (int i = 0; i < 30; ++i)
    {
        tracker.update({makeDetection(s)}, stamp);
        stamp += 0.1;
        s += 0.1;
        ASSERT_EQ(tracker.tracks().size(), 1U);
        if (tracker.tracks().front().motion_status == MotionStatus::Dynamic)
        {
            became_dynamic = true;
            break;
        }
    }

    EXPECT_TRUE(became_dynamic);
    EXPECT_GE(tracker.tracks().front().dynamic_vote_count, 3);
    EXPECT_FALSE(tracker.tracks().front().is_static);
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

}  // namespace
}  // namespace obstacle_detector
