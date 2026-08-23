// ================================================================================================
// OBSTACLE TRACKER implementation
// ================================================================================================

#include "obstacle_detector/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace obstacle_detector
{

namespace
{
rclcpp::Clock &warnThrottleClock()
{
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock;
}
}  // namespace

const char *trackStatusName(TrackStatus status)
{
    switch (status)
    {
        case TrackStatus::Raw:
            return "RAW";
        case TrackStatus::Tentative:
            return "TENTATIVE";
        case TrackStatus::Confirmed:
            return "CONFIRMED";
    }
    return "INVALID";
}

const char *motionStatusName(MotionStatus status)
{
    switch (status)
    {
        case MotionStatus::Unknown:
            return "UNKNOWN";
        case MotionStatus::Static:
            return "STATIC";
        case MotionStatus::Dynamic:
            return "DYNAMIC";
    }
    return "INVALID";
}

VelocityEvidenceResult evaluateVelocityEvidence(
    const Eigen::Vector2d &velocity,
    const Eigen::Matrix2d &velocity_covariance,
    const TrackerParams &params)
{
    VelocityEvidenceResult result;
    if (!velocity.allFinite() || !velocity_covariance.allFinite())
    {
        return result;
    }

    Eigen::Matrix2d regularized =
        0.5 * (velocity_covariance + velocity_covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(regularized);
    if (eigen_solver.info() != Eigen::Success || !eigen_solver.eigenvalues().allFinite())
    {
        return result;
    }

    const double epsilon = params.covariance_regularization_epsilon;
    const double minimum_covariance = params.minimum_velocity_covariance;
    const double minimum_eigenvalue = eigen_solver.eigenvalues().minCoeff();
    // A small zero/singular covariance is a numerical condition and can be regularized. A clearly
    // negative eigenvalue means the Kalman covariance is corrupt and must produce UNCERTAIN.
    if (minimum_eigenvalue < -epsilon)
    {
        return result;
    }
    const double diagonal_addition =
        std::max(epsilon, minimum_covariance - minimum_eigenvalue + epsilon);
    regularized.diagonal().array() += diagonal_addition;

    const Eigen::LDLT<Eigen::Matrix2d> solver(regularized);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        return result;
    }
    const Eigen::Vector2d solved = solver.solve(velocity);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        return result;
    }

    result.statistic = std::max(0.0, velocity.dot(solved));
    if (!std::isfinite(result.statistic))
    {
        result.statistic = std::numeric_limits<double>::quiet_NaN();
        return result;
    }
    result.covariance_valid = true;
    if (result.statistic > params.dynamic_chi2_threshold)
    {
        result.evidence = MotionEvidence::DynamicEvidence;
    }
    else if (result.statistic < params.static_chi2_threshold)
    {
        result.evidence = MotionEvidence::StaticEvidence;
    }
    return result;
}

VelocityEvidenceResult evaluateFrenetVelocityEvidence(
    double longitudinal_velocity,
    double longitudinal_velocity_variance,
    const TrackerParams &params)
{
    VelocityEvidenceResult result;
    if (!std::isfinite(longitudinal_velocity) ||
        !std::isfinite(longitudinal_velocity_variance) ||
        longitudinal_velocity_variance < -params.covariance_regularization_epsilon)
    {
        return result;
    }

    const double regularized_variance =
        std::max(longitudinal_velocity_variance, params.minimum_velocity_covariance) +
        params.covariance_regularization_epsilon;
    result.statistic = longitudinal_velocity * longitudinal_velocity / regularized_variance;
    if (!std::isfinite(result.statistic))
    {
        result.statistic = std::numeric_limits<double>::quiet_NaN();
        return result;
    }

    result.covariance_valid = true;
    if (result.statistic > params.frenet_dynamic_chi2_threshold)
    {
        result.evidence = MotionEvidence::DynamicEvidence;
    }
    // No static branch on purpose: this filter is driven by the raw AABB centre in s, so a small
    // Ts is also what a partially revealed stationary obstacle produces. Only the map-frame
    // statistic may certify StaticEvidence.
    return result;
}

double anchoredAxisTranslation(
    double previous_min, double previous_max, double current_min, double current_max)
{
    if (!std::isfinite(previous_min) || !std::isfinite(previous_max) ||
        !std::isfinite(current_min) || !std::isfinite(current_max))
    {
        return 0.0;
    }
    const double delta_min = current_min - previous_min;
    const double delta_max = current_max - previous_max;
    // Only motion shared by both edges is proven translation. One edge moving alone means the box
    // grew or shrank against a stationary opposite face, which is what progressive LiDAR
    // revelation of a stationary obstacle looks like.
    if (delta_min > 0.0 && delta_max > 0.0)
    {
        return std::min(delta_min, delta_max);
    }
    if (delta_min < 0.0 && delta_max < 0.0)
    {
        return std::max(delta_min, delta_max);
    }
    return 0.0;
}

void ObstacleTracker::configure(const TrackerParams &params, const FrenetProjector *frenet)
{
    const auto invalid = [](bool condition, const char *message) {
        if (condition)
        {
            throw std::invalid_argument(message);
        }
    };
    invalid(params.min_hits_confirm <= 0, "min_hits_confirm must be positive");
    invalid(params.confirmation_window <= 0 ||
            params.min_hits_confirm > params.confirmation_window,
            "min_hits_confirm must not exceed confirmation_window");
    invalid(!(params.static_chi2_threshold > 0.0) ||
            !(params.dynamic_chi2_threshold > params.static_chi2_threshold),
            "chi-square thresholds must satisfy 0 < static < dynamic");
    invalid(!(params.frenet_dynamic_chi2_threshold > 0.0),
            "frenet_dynamic_chi2_threshold must be positive");
    invalid(params.dynamic_vote_window <= 0 || params.dynamic_vote_required <= 0 ||
            params.dynamic_vote_required > params.dynamic_vote_window,
            "dynamic vote requirement must fit its window");
    invalid(params.static_vote_window <= 0 || params.static_vote_required <= 0 ||
            params.static_vote_required > params.static_vote_window,
            "static vote requirement must fit its window");
    invalid(params.position_history_size <= 0 || params.static_min_observations <= 0 ||
            params.static_min_observations > params.position_history_size,
            "static observations must fit the position history");
    invalid(!(params.static_max_position_rms > 0.0) ||
            params.dynamic_to_static_min_observations <= 0 ||
            params.dynamic_to_static_vote_required <= 0 ||
            params.dynamic_to_static_vote_required > params.static_vote_window ||
            !(params.dynamic_to_static_max_position_rms > 0.0),
            "dynamic-to-static hysteresis parameters are invalid");
    invalid(!(params.covariance_regularization_epsilon > 0.0) ||
            !(params.minimum_velocity_covariance > 0.0),
            "velocity covariance regularization values must be positive");
    invalid(params.static_score_forgetting_factor < 0.0 ||
            params.static_score_forgetting_factor > 1.0,
            "static_score_forgetting_factor must be in [0, 1]");
    invalid(params.translation_corroboration_enable &&
            (!(params.translation_window_sec > 0.0) ||
             params.translation_history_max_samples < 2 ||
             !(params.dynamic_min_translation_m >= 0.0) ||
             !(params.dynamic_translation_persistence_sec >= 0.0)),
            "translation corroboration parameters are invalid");
    invalid(!(params.static_min_observation_sec >= 0.0),
            "static_min_observation_sec must be non-negative");
    invalid(params.physical_id_reassociation_gap_s < 0.0 ||
            params.physical_id_reassociation_gap_d < 0.0 ||
            params.physical_id_reassociation_gap_map < 0.0 ||
            params.physical_id_memory_sec < 0.0,
            "physical ID reassociation gaps and memory must be non-negative");

    p_ = params;
    frenet_ = frenet;
}

void ObstacleTracker::clear()
{
    // Called when the CLCS reference is rebuilt: existing tracks live in the old s-domain and
    // must not be predicted or published against the new reference.
    tracks_.clear();
    dormant_physical_identities_.clear();
    has_last_stamp_ = false;
}

double ObstacleTracker::frenetDistSquared(double s1, double d1, double s2, double d2) const
{
    const double ds = frenet_ ? frenet_->wrapDelta(s1, s2) : (s1 - s2);
    const double dd = d1 - d2;
    return ds * ds + dd * dd;
}

bool ObstacleTracker::belongsToSamePhysicalCluster(
    double track_s, double track_d, double track_s_half_extent,
    double track_d_right_offset, double track_d_left_offset,
    const Detection &detection) const
{
    const double ds = std::abs(
        frenet_ ? frenet_->wrapDelta(track_s, detection.s) : track_s - detection.s);
    const double gap_s = std::max(
        0.0, ds - std::abs(track_s_half_extent) - std::abs(detection.s_half_extent));

    const double track_right = std::min(
        track_d + track_d_right_offset, track_d + track_d_left_offset);
    const double track_left = std::max(
        track_d + track_d_right_offset, track_d + track_d_left_offset);
    const double detection_right = std::min(
        detection.d + detection.d_right_offset,
        detection.d + detection.d_left_offset);
    const double detection_left = std::max(
        detection.d + detection.d_right_offset,
        detection.d + detection.d_left_offset);
    const double gap_d = std::max(
        {0.0, track_right - detection_left, detection_right - track_left});
    return gap_s <= p_.physical_id_reassociation_gap_s &&
           gap_d <= p_.physical_id_reassociation_gap_d;
}

bool ObstacleTracker::belongsToSameMapCluster(
    const PhysicalIdentityAnchor &anchor, const Detection &detection) const
{
    const bool finite = anchor.valid &&
        std::isfinite(anchor.x_min_map) && std::isfinite(anchor.x_max_map) &&
        std::isfinite(anchor.y_min_map) && std::isfinite(anchor.y_max_map) &&
        std::isfinite(detection.x_min) && std::isfinite(detection.x_max) &&
        std::isfinite(detection.y_min) && std::isfinite(detection.y_max);
    if (!finite || anchor.x_min_map > anchor.x_max_map ||
        anchor.y_min_map > anchor.y_max_map || detection.x_min > detection.x_max ||
        detection.y_min > detection.y_max)
    {
        return false;
    }

    const double gap_x = std::max(
        {0.0, anchor.x_min_map - detection.x_max,
         detection.x_min - anchor.x_max_map});
    const double gap_y = std::max(
        {0.0, anchor.y_min_map - detection.y_max,
         detection.y_min - anchor.y_max_map});
    return std::hypot(gap_x, gap_y) <= p_.physical_id_reassociation_gap_map;
}

double ObstacleTracker::mapCenterDistSquared(
    const PhysicalIdentityAnchor &anchor, const Detection &detection) const
{
    const double anchor_x = 0.5 * (anchor.x_min_map + anchor.x_max_map);
    const double anchor_y = 0.5 * (anchor.y_min_map + anchor.y_max_map);
    const double detection_x = 0.5 * (detection.x_min + detection.x_max);
    const double detection_y = 0.5 * (detection.y_min + detection.y_max);
    const double dx = anchor_x - detection_x;
    const double dy = anchor_y - detection_y;
    return dx * dx + dy * dy;
}

void ObstacleTracker::captureStableIdentityAnchor(
    Track &track, const Detection &detection) const
{
    if (track.stable_identity_anchor.valid ||
        track.track_status != TrackStatus::Confirmed ||
        track.motion_status != MotionStatus::Static)
    {
        return;
    }

    PhysicalIdentityAnchor anchor;
    anchor.valid = true;
    anchor.s = detection.s;
    anchor.d = detection.d;
    anchor.s_half_extent = detection.s_half_extent;
    anchor.d_right_offset = detection.d_right_offset;
    anchor.d_left_offset = detection.d_left_offset;
    anchor.x_min_map = detection.x_min;
    anchor.x_max_map = detection.x_max;
    anchor.y_min_map = detection.y_min;
    anchor.y_max_map = detection.y_max;
    if (belongsToSameMapCluster(anchor, detection))
    {
        track.stable_identity_anchor = anchor;
    }
}

void ObstacleTracker::predict(Track &t, double dt) const
{
    dt = std::clamp(dt, 0.0, p_.dt_max);

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 1) = dt;  // s  += vs*dt
    F(2, 3) = dt;  // d  += vd*dt

    // discrete white-noise acceleration process noise, applied per axis (s,vs) and (d,vd)
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    // s / vs block
    Q(0, 0) = 0.25 * dt4 * p_.process_var_vs;
    Q(0, 1) = 0.5 * dt3 * p_.process_var_vs;
    Q(1, 0) = 0.5 * dt3 * p_.process_var_vs;
    Q(1, 1) = dt2 * p_.process_var_vs;
    // d / vd block
    Q(2, 2) = 0.25 * dt4 * p_.process_var_vd;
    Q(2, 3) = 0.5 * dt3 * p_.process_var_vd;
    Q(3, 2) = 0.5 * dt3 * p_.process_var_vd;
    Q(3, 3) = dt2 * p_.process_var_vd;

    t.x = F * t.x;
    t.P = F * t.P * F.transpose() + Q;
}

void ObstacleTracker::predictMap(Track &t, double dt) const
{
    if (!t.map_filter_initialized)
    {
        return;
    }
    dt = std::clamp(dt, 0.0, p_.dt_max);
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 1) = dt;
    F(2, 3) = dt;

    // Rotate the Frenet tangential/normal acceleration noise into map x/y. This map filter is used
    // only for motion classification; Frenet remains authoritative for association and output.
    double yaw = 0.0;
    double unused_x = 0.0;
    double unused_y = 0.0;
    const bool has_yaw =
        frenet_ && frenet_->toCartesian(t.s(), t.d(), unused_x, unused_y, yaw);
    const double c = has_yaw ? std::cos(yaw) : 1.0;
    const double s = has_yaw ? std::sin(yaw) : 0.0;
    Eigen::Matrix2d rotation;
    rotation << c, -s,
                s, c;
    Eigen::Matrix2d acceleration_covariance =
        rotation *
        (Eigen::Vector2d(p_.process_var_vs, p_.process_var_vd).asDiagonal()) *
        rotation.transpose();

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    const int position_indices[2] = {0, 2};
    const int velocity_indices[2] = {1, 3};
    for (int row = 0; row < 2; ++row)
    {
        for (int column = 0; column < 2; ++column)
        {
            const double covariance = acceleration_covariance(row, column);
            Q(position_indices[row], position_indices[column]) = 0.25 * dt4 * covariance;
            Q(position_indices[row], velocity_indices[column]) = 0.5 * dt3 * covariance;
            Q(velocity_indices[row], position_indices[column]) = 0.5 * dt3 * covariance;
            Q(velocity_indices[row], velocity_indices[column]) = dt2 * covariance;
        }
    }

    t.map_x = F * t.map_x;
    t.map_P = F * t.map_P * F.transpose() + Q;
}

Eigen::Matrix2d ObstacleTracker::measurementCovariance(const Detection &detection) const
{
    const double scale =
        std::isfinite(detection.variance_scale) ? std::max(1.0, detection.variance_scale) : 1.0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = p_.meas_var_s * scale;
    R(1, 1) = p_.meas_var_d * scale;
    return R;
}

Eigen::Matrix2d ObstacleTracker::mapMeasurementCovariance(
    const Track &track, const Detection &detection) const
{
    double yaw = 0.0;
    double unused_x = 0.0;
    double unused_y = 0.0;
    const bool has_yaw =
        frenet_ && frenet_->toCartesian(track.s(), track.d(), unused_x, unused_y, yaw);
    const double c = has_yaw ? std::cos(yaw) : 1.0;
    const double s = has_yaw ? std::sin(yaw) : 0.0;
    Eigen::Matrix2d rotation;
    rotation << c, -s,
                s, c;
    Eigen::Matrix2d covariance =
        rotation * measurementCovariance(detection) * rotation.transpose();
    covariance.diagonal().array() += p_.covariance_regularization_epsilon;
    return covariance;
}

double ObstacleTracker::innovationDistanceSquared(
    const Track &t, const Detection &detection) const
{
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;

    Eigen::Vector2d innovation;
    innovation(0) = frenet_ ?
        frenet_->wrapDelta(detection.s, t.x(0)) : detection.s - t.x(0);
    innovation(1) = detection.d - t.x(2);

    const Eigen::Matrix2d S =
        H * t.P * H.transpose() + measurementCovariance(detection);
    const Eigen::LDLT<Eigen::Matrix2d> solver(S);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector2d solved = solver.solve(innovation);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        return std::numeric_limits<double>::infinity();
    }
    return innovation.dot(solved);
}

double ObstacleTracker::smoothExtent(double previous, double current) const
{
    // Fast-grow/slow-shrink on the offset MAGNITUDE: the envelope expands to a larger
    // measurement immediately but relaxes toward a smaller one over ~1/alpha matched frames.
    // Per-scan AABB flapping (square box vs real shape) then stops flicking the published
    // Frenet envelope across the raceline and back.
    if (std::abs(current) >= std::abs(previous) || p_.extent_shrink_alpha >= 1.0)
    {
        return current;
    }
    if (p_.extent_shrink_alpha <= 0.0)
    {
        return previous;
    }
    return previous + p_.extent_shrink_alpha * (current - previous);
}

void ObstacleTracker::kalmanUpdate(Track &t, const Detection &detection) const
{
    // H selects s (row0) and d (row2)
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;

    const Eigen::Matrix2d R = measurementCovariance(detection);

    // innovation with s-wrap handling
    Eigen::Vector2d z;
    z << detection.s, detection.d;
    Eigen::Vector2d hx;
    hx << t.x(0), t.x(2);
    Eigen::Vector2d y = z - hx;
    if (frenet_)
    {
        y(0) = frenet_->wrapDelta(detection.s, t.x(0));
    }

    Eigen::Matrix2d S = H * t.P * H.transpose() + R;
    // Solve through LDLT like the association gate instead of inverting S blindly: a degraded
    // covariance must skip this update (predicted state kept), not poison the track.
    const Eigen::LDLT<Eigen::Matrix2d> solver(S);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
                             "Kalman update skipped: innovation covariance not positive definite");
        return;
    }
    const Eigen::Matrix<double, 2, 4> solved = solver.solve(H * t.P);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
                             "Kalman update skipped: innovation covariance solve failed");
        return;
    }
    const Eigen::Matrix<double, 4, 2> K = solved.transpose();
    t.x = t.x + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    t.P = (I - K * H) * t.P;
    // The simple covariance form drifts asymmetric over long runs; re-symmetrize it and keep the
    // variances non-negative.
    t.P = 0.5 * (t.P + t.P.transpose());
    for (int i = 0; i < 4; ++i)
    {
        t.P(i, i) = std::max(0.0, t.P(i, i));
    }

    // keep s within [0, length)
    if (frenet_ && frenet_->raceline_length() > 0.0)
    {
        const double L = frenet_->raceline_length();
        t.x(0) = std::fmod(t.x(0), L);
        if (t.x(0) < 0.0)
        {
            t.x(0) += L;
        }
    }
}

bool ObstacleTracker::mapKalmanUpdate(Track &t, const Detection &detection) const
{
    // The measurement is the raw AABB centre. Feeding an accumulated two-edge translation here
    // instead was tried (a48b374) and reverted: min(|dmin|,|dmax|) is biased towards zero, so it
    // under-reported real speed by 25-50% under centimetre edge noise, and being a pure
    // integrator with no re-anchoring it random-walked away from the object under pose jitter.
    // The two-edge proof still guards DYNAMIC entry -- as a vote gate in classify(), not as the
    // filter's measurement.
    const double measurement_x = 0.5 * (detection.x_min + detection.x_max);
    const double measurement_y = 0.5 * (detection.y_min + detection.y_max);
    if (!std::isfinite(measurement_x) || !std::isfinite(measurement_y))
    {
        return false;
    }

    if (!t.map_filter_initialized)
    {
        t.map_x << measurement_x, 0.0, measurement_y, 0.0;
        t.map_P = Eigen::Matrix4d::Zero();
        t.map_P(0, 0) = 0.5;
        t.map_P(1, 1) = 4.0;
        t.map_P(2, 2) = 0.5;
        t.map_P(3, 3) = 4.0;
        t.map_filter_initialized = true;
        return true;
    }

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;
    const Eigen::Vector2d innovation(
        measurement_x - t.map_x(0), measurement_y - t.map_x(2));
    const Eigen::Matrix2d R = mapMeasurementCovariance(t, detection);
    const Eigen::Matrix2d S = H * t.map_P * H.transpose() + R;
    const Eigen::LDLT<Eigen::Matrix2d> solver(S);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
            "Map Kalman update skipped: innovation covariance not positive definite");
        return false;
    }
    const Eigen::Matrix<double, 2, 4> solved = solver.solve(H * t.map_P);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
            "Map Kalman update skipped: innovation covariance solve failed");
        return false;
    }

    const Eigen::Matrix<double, 4, 2> K = solved.transpose();
    t.map_x += K * innovation;
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    t.map_P =
        (identity - K * H) * t.map_P * (identity - K * H).transpose() +
        K * R * K.transpose();
    t.map_P = 0.5 * (t.map_P + t.map_P.transpose());
    return t.map_x.allFinite() && t.map_P.allFinite();
}

int ObstacleTracker::countEvidence(
    const std::deque<MotionEvidence> &history,
    MotionEvidence evidence,
    int window) const
{
    const std::size_t count =
        std::min(history.size(), static_cast<std::size_t>(std::max(0, window)));
    int votes = 0;
    for (std::size_t offset = 0; offset < count; ++offset)
    {
        if (history[history.size() - 1U - offset] == evidence)
        {
            ++votes;
        }
    }
    return votes;
}

double ObstacleTracker::positionRms(
    const std::deque<std::pair<double, double>> &history) const
{
    if (history.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto &position : history)
    {
        mean_x += position.first;
        mean_y += position.second;
    }
    mean_x /= static_cast<double>(history.size());
    mean_y /= static_cast<double>(history.size());

    double squared_displacement_sum = 0.0;
    for (const auto &position : history)
    {
        const double dx = position.first - mean_x;
        const double dy = position.second - mean_y;
        squared_displacement_sum += dx * dx + dy * dy;
    }
    return std::sqrt(squared_displacement_sum / static_cast<double>(history.size()));
}

void ObstacleTracker::updateTrackStatus(Track &t, bool measurement_received) const
{
    t.confirmation_history.push_back(measurement_received);
    while (static_cast<int>(t.confirmation_history.size()) > p_.confirmation_window)
    {
        t.confirmation_history.pop_front();
    }
    if (t.track_status == TrackStatus::Confirmed)
    {
        return;
    }
    const int measurement_votes = static_cast<int>(
        std::count(t.confirmation_history.begin(), t.confirmation_history.end(), true));
    if (measurement_received && measurement_votes >= p_.min_hits_confirm)
    {
        t.track_status = TrackStatus::Confirmed;
    }
    else if (t.hits >= 2)
    {
        t.track_status = TrackStatus::Tentative;
    }
    else
    {
        t.track_status = TrackStatus::Raw;
    }
}

void ObstacleTracker::updateStaticConfidence(
    Track &t, MotionEvidence evidence, bool measurement_received) const
{
    const double forgetting = p_.static_score_forgetting_factor;
    const double update_weight = 1.0 - forgetting;
    double score = forgetting * t.static_confidence;
    if (measurement_received)
    {
        score += 0.20 * update_weight;
        if (evidence == MotionEvidence::StaticEvidence)
        {
            score += 0.55 * update_weight;
        }
        else if (evidence == MotionEvidence::DynamicEvidence)
        {
            score -= 0.80 * update_weight;
        }
        if (std::isfinite(t.map_position_rms))
        {
            score += (t.map_position_rms <= p_.static_max_position_rms ?
                0.25 : -0.25) * update_weight;
        }
    }
    // TODO(perception): subtract free-space contradiction evidence when a ray-traced
    // visibility/free-space contract becomes available. Prediction-only frames only decay.
    t.static_confidence = std::clamp(score, 0.0, 1.0);
}

void ObstacleTracker::updateTranslationEvidence(
    Track &t, const Detection &detection, double stamp) const
{
    if (!p_.translation_corroboration_enable)
    {
        t.provable_map_translation_m = std::numeric_limits<double>::quiet_NaN();
        t.provable_frenet_translation_m = std::numeric_limits<double>::quiet_NaN();
        t.provable_translation_m = std::numeric_limits<double>::quiet_NaN();
        t.translation_corroborated_since = std::numeric_limits<double>::quiet_NaN();
        t.translation_evidence_persistent = false;
        return;
    }

    while (!t.measured_aabb_history.empty() &&
           (stamp - t.measured_aabb_history.front().stamp > p_.translation_window_sec ||
            t.measured_aabb_history.front().stamp > stamp))
    {
        t.measured_aabb_history.pop_front();
    }
    while (static_cast<int>(t.measured_aabb_history.size()) >=
           p_.translation_history_max_samples)
    {
        t.measured_aabb_history.pop_front();
    }

    // Compare against every retained sample rather than only the oldest: the widest separation any
    // pair proves is the strongest translation evidence inside the window, so a genuinely moving
    // object is never hidden by where the window happens to start.
    double widest_map = 0.0;
    double widest_frenet = 0.0;
    for (const MeasuredAabbSample &sample : t.measured_aabb_history)
    {
        const double translation_x = anchoredAxisTranslation(
            sample.x_min, sample.x_max, detection.x_min, detection.x_max);
        const double translation_y = anchoredAxisTranslation(
            sample.y_min, sample.y_max, detection.y_min, detection.y_max);
        widest_map = std::max(widest_map, std::hypot(translation_x, translation_y));

        // Apply the same two-edge proof in the track-aligned Frenet frame. A turning vehicle's
        // map AABB changes width/height as its heading rotates, but its longitudinal Frenet
        // envelope translates coherently. Conversely, one-sided revelation still moves only one
        // edge and proves no translation in either frame.
        const double delta_s_center =
            frenet_ ? frenet_->wrapDelta(detection.s, sample.s) : detection.s - sample.s;
        const double translation_s = anchoredAxisTranslation(
            -sample.s_half_extent, sample.s_half_extent,
            delta_s_center - detection.s_half_extent,
            delta_s_center + detection.s_half_extent);
        const double previous_d_a = sample.d + sample.d_right_offset;
        const double previous_d_b = sample.d + sample.d_left_offset;
        const double current_d_a = detection.d + detection.d_right_offset;
        const double current_d_b = detection.d + detection.d_left_offset;
        const double translation_d = anchoredAxisTranslation(
            std::min(previous_d_a, previous_d_b),
            std::max(previous_d_a, previous_d_b),
            std::min(current_d_a, current_d_b),
            std::max(current_d_a, current_d_b));
        widest_frenet =
            std::max(widest_frenet, std::hypot(translation_s, translation_d));
    }
    if (t.measured_aabb_history.empty())
    {
        t.provable_map_translation_m = std::numeric_limits<double>::quiet_NaN();
        t.provable_frenet_translation_m = std::numeric_limits<double>::quiet_NaN();
        t.provable_translation_m = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
        t.provable_map_translation_m = widest_map;
        t.provable_frenet_translation_m = widest_frenet;
        t.provable_translation_m = std::max(widest_map, widest_frenet);
    }

    // A localization jump translates both edges at once, so it passes the two-edge proof. It only
    // does so while the pre-jump samples stay in the window, however: once they age out the proof
    // collapses back to ~0. A genuinely moving object sustains it indefinitely. Timing the proof
    // therefore separates the two without raising the distance threshold to a value no slow
    // opponent can reach. The requirement is a duration, so it is independent of scan rate.
    const bool corroborated =
        std::isfinite(t.provable_translation_m) &&
        t.provable_translation_m >= p_.dynamic_min_translation_m;
    if (!corroborated)
    {
        t.translation_corroborated_since = std::numeric_limits<double>::quiet_NaN();
    }
    else if (!std::isfinite(t.translation_corroborated_since) ||
             stamp < t.translation_corroborated_since)
    {
        t.translation_corroborated_since = stamp;
    }
    t.translation_evidence_persistent =
        corroborated && std::isfinite(t.translation_corroborated_since) &&
        (stamp - t.translation_corroborated_since) >= p_.dynamic_translation_persistence_sec;

    MeasuredAabbSample sample;
    sample.stamp = stamp;
    sample.x_min = detection.x_min;
    sample.x_max = detection.x_max;
    sample.y_min = detection.y_min;
    sample.y_max = detection.y_max;
    sample.s = detection.s;
    sample.d = detection.d;
    sample.s_half_extent = detection.s_half_extent;
    sample.d_right_offset = detection.d_right_offset;
    sample.d_left_offset = detection.d_left_offset;
    t.measured_aabb_history.push_back(sample);
}

bool ObstacleTracker::holdEligibleWhileUnmeasured(const Track &t) const
{
    if (p_.static_lost_hold_sec <= 0.0 || t.track_status != TrackStatus::Confirmed ||
        !t.is_static)
    {
        return false;
    }
    // The hold's justification is "a map-fixed object cannot change while unobserved". That is
    // proven for a Confirmed STATIC track. It is NOT proven for a Confirmed UNKNOWN one, and
    // `is_static` alone means only "not Dynamic", which every track is before it accumulates
    // dynamic votes. Because a dynamic vote additionally requires dynamic_min_translation_m of
    // corroborated translation over translation_window_sec, EVERY opponent spends at least that
    // window Confirmed-and-Unknown while already published on /static_obs; holding it there turns
    // a briefly occluded opponent into a static_lost_hold_sec ghost at a stale pose.
    if (t.motion_status == MotionStatus::Static)
    {
        return true;
    }
    // Unknown is admitted only on positive evidence that the measured box did not translate.
    // provable_translation_m counts only motion shared by both edges of an axis, so progressive
    // revelation of a stationary obstacle (one edge growing) reads as ~0 and keeps its hold --
    // which is the flicker case static_lost_hold_sec exists for. With corroboration disabled the
    // evidence does not exist at all, so fall back to the previous permissive behaviour instead
    // of silently retiring every Unknown track.
    if (!p_.translation_corroboration_enable)
    {
        return true;
    }
    return std::isfinite(t.provable_translation_m) &&
           t.provable_translation_m < p_.dynamic_min_translation_m;
}

void ObstacleTracker::classify(Track &t, bool measurement_received, double stamp) const
{
    if (t.track_status != TrackStatus::Confirmed)
    {
        t.motion_status = MotionStatus::Unknown;
        t.is_static = true;
        t.motion_status_since = std::numeric_limits<double>::quiet_NaN();
        updateStaticConfidence(t, MotionEvidence::Uncertain, measurement_received);
        return;
    }
    if (!measurement_received || !t.map_filter_initialized)
    {
        updateStaticConfidence(t, MotionEvidence::Uncertain, false);
        t.is_static = t.motion_status != MotionStatus::Dynamic;
        return;
    }
    if (!std::isfinite(t.motion_status_since) || stamp < t.motion_status_since)
    {
        t.motion_status_since = stamp;
    }

    Eigen::Matrix2d velocity_covariance;
    velocity_covariance << t.map_P(1, 1), t.map_P(1, 3),
                           t.map_P(3, 1), t.map_P(3, 3);
    const VelocityEvidenceResult map_evidence = evaluateVelocityEvidence(
        Eigen::Vector2d(t.mapVx(), t.mapVy()), velocity_covariance, p_);
    const VelocityEvidenceResult frenet_evidence = evaluateFrenetVelocityEvidence(
        t.vs(), t.P(1, 1), p_);
    t.velocity_statistic = map_evidence.statistic;
    t.velocity_covariance_valid = map_evidence.covariance_valid;
    t.frenet_velocity_statistic = frenet_evidence.statistic;
    t.frenet_velocity_covariance_valid = frenet_evidence.covariance_valid;

    // Dynamic entry is an OR: Frenet longitudinal motion recovers a turning opponent whose map
    // CV covariance grows through centripetal acceleration. StaticEvidence can only come from the
    // map statistic, because evaluateFrenetVelocityEvidence never reports it -- requiring both
    // views to agree on "stationary" left partially revealed obstacles stuck in UNKNOWN, so they
    // never reached /confirmed_static_obs.
    MotionEvidence combined_evidence = map_evidence.evidence;
    if (frenet_evidence.evidence == MotionEvidence::DynamicEvidence)
    {
        combined_evidence = MotionEvidence::DynamicEvidence;
    }

    // A large map/Frenet velocity statistic is only believed once the measured map AABB or Frenet
    // envelope proves two-edge translation, and holds that proof for longer than the proof window
    // itself. Progressive revelation moves a raw centroid while leaving one edge anchored, and a
    // one-off pose jump moves both edges once; neither survives the persistence requirement, so
    // the uncorroborated vote is downgraded to Uncertain and the track keeps its safety layer
    // instead of vanishing from /static_obs.
    MotionEvidence voted_evidence = combined_evidence;
    t.dynamic_evidence_suppressed = false;
    if (p_.translation_corroboration_enable &&
        voted_evidence == MotionEvidence::DynamicEvidence &&
        !t.translation_evidence_persistent)
    {
        voted_evidence = MotionEvidence::Uncertain;
        t.dynamic_evidence_suppressed = true;
    }
    // Hard braking / launch transients shake the localization pose, and that jitter projects
    // into every track as apparent map-frame translation. A dynamic vote cast in such a frame
    // is not evidence about the obstacle, so it is withheld; static votes and all other
    // bookkeeping continue unchanged, which only delays a real opponent's DYNAMIC entry by the
    // transient's duration.
    if (ego_motion_transient_ && voted_evidence == MotionEvidence::DynamicEvidence)
    {
        voted_evidence = MotionEvidence::Uncertain;
        t.dynamic_evidence_suppressed = true;
    }
    t.motion_evidence_history.push_back(voted_evidence);
    const int history_limit = std::max(p_.dynamic_vote_window, p_.static_vote_window);
    while (static_cast<int>(t.motion_evidence_history.size()) > history_limit)
    {
        t.motion_evidence_history.pop_front();
    }
    t.dynamic_vote_count = countEvidence(
        t.motion_evidence_history, MotionEvidence::DynamicEvidence, p_.dynamic_vote_window);
    t.static_vote_count = countEvidence(
        t.motion_evidence_history, MotionEvidence::StaticEvidence, p_.static_vote_window);
    ++t.motion_observations_since_transition;

    // Every gate below this line except these two counts FRAMES. At 250 Hz (the gym bridge scan
    // timer) the 15-frame vote window is 60 ms and the 15-sample position RMS spans 12 cm of
    // travel even at 2 m/s, so a freshly spawned track -- whose map velocity is still the zero it
    // was initialized with -- satisfied every STATIC condition 44 ms after birth and published a
    // moving opponent on /confirmed_static_obs (2026-08-20 two-agent run). Both conditions here
    // are times or distances, so neither moves with the scan rate:
    //   * the track must hold its current motion status for static_min_observation_sec, which is
    //     longer than translation_window_sec, so the proof below is fully populated;
    //   * that proof must say the measured shape did NOT translate -- the same two-edge evidence
    //     that gates DYNAMIC entry, read in the opposite direction.
    const double motion_status_age =
        std::isfinite(t.motion_status_since) ? stamp - t.motion_status_since : 0.0;
    const bool observation_span_satisfied =
        motion_status_age >= p_.static_min_observation_sec;
    const bool translation_proves_stationary =
        !p_.translation_corroboration_enable ||
        (std::isfinite(t.provable_translation_m) &&
         t.provable_translation_m < p_.dynamic_min_translation_m);
    const bool static_admissible =
        observation_span_satisfied && translation_proves_stationary;

    if (t.dynamic_vote_count >= p_.dynamic_vote_required &&
        t.motion_status != MotionStatus::Dynamic)
    {
        t.motion_status = MotionStatus::Dynamic;
        t.motion_observations_since_transition = 0;
        t.motion_status_since = stamp;
    }
    else if (t.motion_status == MotionStatus::Dynamic)
    {
        const bool conservative_static_reentry =
            static_admissible &&
            t.motion_observations_since_transition >=
                p_.dynamic_to_static_min_observations &&
            t.static_vote_count >= p_.dynamic_to_static_vote_required &&
            static_cast<int>(t.map_position_history.size()) >=
                p_.static_min_observations &&
            std::isfinite(t.map_position_rms) &&
            t.map_position_rms <= p_.dynamic_to_static_max_position_rms;
        if (conservative_static_reentry)
        {
            t.motion_status = MotionStatus::Static;
            t.motion_observations_since_transition = 0;
            t.motion_status_since = stamp;
        }
    }
    else if (t.motion_status == MotionStatus::Unknown)
    {
        const bool static_entry =
            static_admissible &&
            t.static_vote_count >= p_.static_vote_required &&
            static_cast<int>(t.map_position_history.size()) >=
                p_.static_min_observations &&
            std::isfinite(t.map_position_rms) &&
            t.map_position_rms <= p_.static_max_position_rms;
        if (static_entry)
        {
            t.motion_status = MotionStatus::Static;
            t.motion_observations_since_transition = 0;
            t.motion_status_since = stamp;
        }
    }

    updateStaticConfidence(t, voted_evidence, true);
    t.is_static = t.motion_status != MotionStatus::Dynamic;
}

void ObstacleTracker::update(
    const std::vector<Detection> &detections, double stamp,
    double ego_yaw_rate, bool yaw_rate_fresh, bool ego_motion_transient,
    const FreeSpaceRefuter &free_space_refuter,
    const RearBlindPredicate &rear_blind_predicate)
{
    // Retained in the public call signature for source compatibility. Map-frame classification no
    // longer needs an ego-yaw gate because scan points have already been transformed into map.
    (void)ego_yaw_rate;
    (void)yaw_rate_fresh;
    ego_motion_transient_ = ego_motion_transient;
    last_stats_ = TrackerUpdateStats{};

    if (!p_.physical_id_reassociation_enable || p_.physical_id_memory_sec <= 0.0)
    {
        dormant_physical_identities_.clear();
    }
    else
    {
        dormant_physical_identities_.erase(
            std::remove_if(
                dormant_physical_identities_.begin(), dormant_physical_identities_.end(),
                [stamp, this](const DormantPhysicalIdentity &identity) {
                    return !std::isfinite(identity.last_seen_stamp) ||
                           stamp < identity.last_seen_stamp ||
                           stamp - identity.last_seen_stamp > p_.physical_id_memory_sec;
                }),
            dormant_physical_identities_.end());
    }

    double dt = 0.0;
    if (has_last_stamp_)
    {
        dt = stamp - last_stamp_;
        if (dt < 0.0)
        {
            dt = 0.0;
        }
    }
    last_stamp_ = stamp;
    has_last_stamp_ = true;

    // 1) predict all existing tracks to this stamp
    for (auto &t : tracks_)
    {
        // A held confirmed-static track is a map-fixed object with no measurements: propagating
        // the constant-velocity models through the dropout only drifts its state (a noisy
        // velocity estimate integrates over seconds) and inflates its covariance, which (a)
        // breaks re-association on re-detection, spawning a duplicate zombie track, and (b)
        // explodes the published s_var/d_var so downstream uncertainty guards turn a 10 cm
        // sliver into a multi-metre wall (.regression_check2: latched danger span 14.1-20.8 m
        // from a 17.33-17.43 m obstacle). Freeze the state exactly while unobserved; the first
        // prediction-only frame (time_since == 0 here) still propagates normally, so visible
        // tracks are untouched.
        const bool freeze_lost_static =
            holdEligibleWhileUnmeasured(t) && t.time_since_last_measurement > 0.0;
        if (!freeze_lost_static)
        {
            predict(t, dt);
            predictMap(t, dt);
        }
        t.time_since_last_measurement += dt;
        t.is_visible = false;
    }

    // 2) greedy nearest-neighbour association on predicted (s, d)
    const std::size_t nt = tracks_.size();
    const std::size_t nd = detections.size();
    std::vector<int> det_assigned(nd, -1);
    std::vector<int> trk_assigned(nt, -1);

    struct Pair
    {
        std::size_t ti;
        std::size_t di;
        double cost;
    };
    std::vector<Pair> pairs;
    pairs.reserve(nt * nd);
    for (std::size_t ti = 0; ti < nt; ++ti)
    {
        const double gate = tracks_[ti].is_static ? p_.assoc_gate : p_.assoc_gate * p_.aggro_multi;
        const double gate_sq = gate * gate;
        for (std::size_t di = 0; di < nd; ++di)
        {
            ++last_stats_.candidate_pairs;
            const double cost_sq =
                frenetDistSquared(tracks_[ti].x(0), tracks_[ti].x(2), detections[di].s,
                                   detections[di].d);
            if (cost_sq > gate_sq)
            {
                ++last_stats_.euclidean_pair_rejected;
                continue;
            }
            if (p_.assoc_use_mahalanobis)
            {
                const double m2 = innovationDistanceSquared(tracks_[ti], detections[di]);
                if (!std::isfinite(m2) || m2 > p_.assoc_mahalanobis_gate)
                {
                    ++last_stats_.mahalanobis_pair_rejected;
                    continue;
                }
            }
            // Keep the existing Frenet-distance ordering. Mahalanobis is a gate only: adaptive R
            // must not make a noisier far detection win merely because its normalized m2 is small.
            pairs.push_back({ti, di, cost_sq});
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair &a, const Pair &b) { return a.cost < b.cost; });
    for (const auto &pr : pairs)
    {
        if (trk_assigned[pr.ti] >= 0 || det_assigned[pr.di] >= 0)
        {
            continue;
        }
        trk_assigned[pr.ti] = static_cast<int>(pr.di);
        det_assigned[pr.di] = static_cast<int>(pr.ti);
        ++last_stats_.matched;
    }

    // Motion classification selects an output layer; it must never break physical identity.
    // After statistically valid assignments take priority, reconnect every unmatched track whose
    // Frenet envelope still belongs to the same physical cluster, including Dynamic tracks.
    if (p_.physical_id_reassociation_enable)
    {
        std::vector<Pair> spatial_pairs;
        for (std::size_t ti = 0; ti < nt; ++ti)
        {
            if (trk_assigned[ti] >= 0)
            {
                continue;
            }
            for (std::size_t di = 0; di < nd; ++di)
            {
                if (det_assigned[di] >= 0)
                {
                    continue;
                }
                const Track &track = tracks_[ti];
                if (belongsToSamePhysicalCluster(
                        track.s(), track.d(), track.s_half_extent,
                        track.d_right_offset, track.d_left_offset, detections[di]))
                {
                    spatial_pairs.push_back(
                        {ti, di, frenetDistSquared(
                            track.s(), track.d(), detections[di].s, detections[di].d)});
                }
            }
        }
        std::sort(
            spatial_pairs.begin(), spatial_pairs.end(),
            [this](const Pair &a, const Pair &b) {
                if (a.cost != b.cost)
                {
                    return a.cost < b.cost;
                }
                if (tracks_[a.ti].track_uid != tracks_[b.ti].track_uid)
                {
                    return tracks_[a.ti].track_uid < tracks_[b.ti].track_uid;
                }
                return a.di < b.di;
            });
        for (const Pair &pair : spatial_pairs)
        {
            if (trk_assigned[pair.ti] >= 0 || det_assigned[pair.di] >= 0)
            {
                continue;
            }
            trk_assigned[pair.ti] = static_cast<int>(pair.di);
            det_assigned[pair.di] = static_cast<int>(pair.ti);
            ++last_stats_.matched;
            ++last_stats_.spatially_reassociated;
        }
    }

    // 3) update matched tracks
    for (std::size_t ti = 0; ti < nt; ++ti)
    {
        Track &t = tracks_[ti];
        if (trk_assigned[ti] >= 0)
        {
            const Detection &det = detections[trk_assigned[ti]];
            // Envelope-stability streak: compare this measurement with the predicted centre and
            // the retained (smoothed) extents BEFORE updating them. Morphing scatter keeps
            // resetting the streak; stable physical obstacles reach the publish gate
            // within min_hits_confirm frames.
            //
            // B2-① 비대칭 리셋 (2026-08-20, run_192006 접촉 #4 실측): 접근 중 보이는 면이
            // 커지는 것은 진짜 장애물의 정상 현상인데(id35: 0.21→0.45 m 성장), 대칭 리셋은
            // 그 성장 프레임마다 스트릭을 0으로 만들었고 smoothExtent 는 유지 extent 를
            // 즉시 키워 다음 프레임에 복귀시켰다 — 결과는 두 발행 토픽 모두에서 정확히
            // 2 프레임짜리 구멍 5회/2초. 플래너는 그 구멍마다 순간 실명했다. 그래서
            // **확장은 리셋하지 않는다** (smoothExtent 의 "확장은 즉시 반영" 철학과 동일
            // 방향). 산란 유령의 서명인 수축·중심 요동은 종전대로 리셋한다 — 커졌다
            // 작아졌다를 반복하는 산란은 수축 프레임마다 잡힌다.
            const double center_shift = std::sqrt(
                frenetDistSquared(t.s(), t.d(), det.s, det.d));
            // 오프셋은 부호 규약이 축마다 다르므로(d_right_offset 은 음수) smoothExtent 와
            // 같은 **크기(절댓값)** 기준으로 판정한다. 양수 = 수축.
            const double envelope_shrink = std::max(
                {std::abs(t.s_half_extent) - std::abs(det.s_half_extent),
                 std::abs(t.d_right_offset) - std::abs(det.d_right_offset),
                 std::abs(t.d_left_offset) - std::abs(det.d_left_offset)});
            if (center_shift <= p_.envelope_stability_tolerance_m &&
                envelope_shrink <= p_.envelope_stability_tolerance_m)
            {
                ++t.envelope_stable_streak;
            }
            else
            {
                t.envelope_stable_streak = 0;
            }
            // Runs before the retained AABB is overwritten below, so the window still holds the
            // previously measured boxes this detection is compared against.
            updateTranslationEvidence(t, det, stamp);
            kalmanUpdate(t, det);
            const bool map_updated = mapKalmanUpdate(t, det);
            t.hits++;
            t.is_visible = true;
            t.time_since_last_measurement = 0.0;
            t.s_half_extent = smoothExtent(t.s_half_extent, det.s_half_extent);
            t.d_right_offset = smoothExtent(t.d_right_offset, det.d_right_offset);
            t.d_left_offset = smoothExtent(t.d_left_offset, det.d_left_offset);
            t.last_measured_s = det.s;
            t.last_measured_d = det.d;
            t.last_measurement_stamp = stamp;
            t.size = det.size;
            t.x_min_map = det.x_min;
            t.x_max_map = det.x_max;
            t.y_min_map = det.y_min;
            t.y_max_map = det.y_max;
            if (map_updated)
            {
                t.map_position_history.emplace_back(t.mapX(), t.mapY());
                while (static_cast<int>(t.map_position_history.size()) >
                       p_.position_history_size)
                {
                    t.map_position_history.pop_front();
                }
                t.map_position_rms = positionRms(t.map_position_history);
                ++t.total_measurement_updates;
            }
            updateTrackStatus(t, true);
            classify(t, map_updated, stamp);
            if (t.track_status == TrackStatus::Confirmed)
            {
                t.physical_identity_eligible = true;
            }
            captureStableIdentityAnchor(t, det);
            t.freespace_refute_streak = 0;
            t.rear_blind_streak = 0;
            t.ttl = t.is_static ? p_.ttl_static : p_.ttl_dynamic;
        }
        else
        {
            t.ttl--;
            // A confirmed STATIC track is a map-fixed object: its geometry cannot change while
            // unobserved, so an occlusion/FOV dropout is not evidence of disappearance. Hold the
            // track alive and keep its pre-dropout envelope-stability evidence intact for
            // static_lost_hold_sec (scan-stamp time, rate independent). Everything else keeps the
            // original frame-TTL retirement, and the stability streak still resets because a
            // prediction-only frame breaks consecutive-scan evidence.
            const bool hold_candidate =
                holdEligibleWhileUnmeasured(t) &&
                t.time_since_last_measurement < p_.static_lost_hold_sec;
            // 홀드의 근거는 "안 보이니까 차폐됐다"이다. 그런데 그 상자를 관통해 더 먼 곳에서
            // 되돌아온 빔이 있으면 그 공간이 비었다는 적극적 증거이고, 홀드는 뻔히 보이는
            // 자리에 유령을 살려두는 것이 된다. 관통 증거가 연속 N 스캔 쌓이면 즉시 retire한다
            // (반증 자체는 스캔 기하를 가진 node가 제공한다).
            bool freespace_refuted = false;
            if (hold_candidate && free_space_refuter &&
                p_.static_hold_freespace_refute_frames > 0)
            {
                if (free_space_refuter(t))
                {
                    ++t.freespace_refute_streak;
                }
                else
                {
                    t.freespace_refute_streak = 0;
                }
                freespace_refuted =
                    t.freespace_refute_streak >= p_.static_hold_freespace_refute_frames;
            }
            else
            {
                t.freespace_refute_streak = 0;
            }
            // 후방 사각 회수 (2026-08-22). 위 반증은 "상자를 관통한 빔"을 증거로 쓰므로
            // 상자가 시야 안에 있어야만 성립한다. 마지막 실측 봉투가 통째로 FOV 밖으로
            // 넘어가면 그 증거는 영영 못 모으고, hold 는 static_lost_hold_sec 를 끝까지
            // 채운 뒤 어차피 죽는다. 그 사이의 hold 는 갱신 가능성이 0 이라 정보가 없고,
            // 하류(플래너 안전정지 해제 사다리)만 붙잡는다 — 그래서 여기서 끝낸다.
            bool rear_blind_retire = false;
            if (hold_candidate && rear_blind_predicate &&
                p_.static_hold_rear_blind_retire_frames > 0)
            {
                if (rear_blind_predicate(t))
                {
                    ++t.rear_blind_streak;
                }
                else
                {
                    t.rear_blind_streak = 0;
                }
                rear_blind_retire =
                    t.rear_blind_streak >= p_.static_hold_rear_blind_retire_frames;
            }
            else
            {
                t.rear_blind_streak = 0;
            }
            if (hold_candidate && !freespace_refuted && !rear_blind_retire)
            {
                t.ttl = std::max(t.ttl, 1);
            }
            else
            {
                t.envelope_stable_streak = 0;
            }
            if (freespace_refuted || rear_blind_retire)
            {
                // 홀드 없는 track의 frame-TTL 잔여분까지 기다리지 않는다. 반증은 "거기 없다"는
                // 직접 증거이고, 후방 사각은 "여기서는 그 질문에 답할 수단이 영영 없다"는
                // 직접 증거다 — 어느 쪽이든 이번 스캔에서 회수한다.
                t.ttl = 0;
            }
            updateTrackStatus(t, false);
            classify(t, false, stamp);
        }
    }

    // 4) spawn new tracks from unmatched detections
    for (std::size_t di = 0; di < nd; ++di)
    {
        if (det_assigned[di] >= 0)
        {
            continue;
        }
        Track t;
        t.track_uid = next_track_uid_++;
        const Track *active_identity = nullptr;
        double reused_cost = std::numeric_limits<double>::infinity();
        if (p_.physical_id_reassociation_enable)
        {
            // A scan split can leave a second detection after the existing physical track has
            // already received its 1:1 measurement. Give the fragment the same public object ID
            // while keeping a separate internal Kalman-track instance.
            for (const Track &existing : tracks_)
            {
                if (!belongsToSamePhysicalCluster(
                        existing.s(), existing.d(), existing.s_half_extent,
                        existing.d_right_offset, existing.d_left_offset, detections[di]))
                {
                    continue;
                }
                const double cost = frenetDistSquared(
                    existing.s(), existing.d(), detections[di].s, detections[di].d);
                if (cost < reused_cost ||
                    (cost == reused_cost && active_identity != nullptr &&
                    existing.track_uid < active_identity->track_uid))
                {
                    active_identity = &existing;
                    reused_cost = cost;
                }
            }
        }

        std::size_t dormant_identity = dormant_physical_identities_.size();
        if (active_identity == nullptr && p_.physical_id_reassociation_enable)
        {
            for (std::size_t i = 0; i < dormant_physical_identities_.size(); ++i)
            {
                const auto &identity = dormant_physical_identities_[i];
                const bool same_frenet = belongsToSamePhysicalCluster(
                    identity.anchor.s, identity.anchor.d,
                    identity.anchor.s_half_extent,
                    identity.anchor.d_right_offset,
                    identity.anchor.d_left_offset, detections[di]);
                const bool same_map = belongsToSameMapCluster(identity.anchor, detections[di]);
                if (!same_frenet && !same_map)
                {
                    continue;
                }
                double cost = std::numeric_limits<double>::infinity();
                if (same_frenet)
                {
                    cost = frenetDistSquared(
                        identity.anchor.s, identity.anchor.d,
                        detections[di].s, detections[di].d);
                }
                if (same_map)
                {
                    cost = std::min(
                        cost, mapCenterDistSquared(identity.anchor, detections[di]));
                }
                if (cost < reused_cost ||
                    (cost == reused_cost && dormant_identity < dormant_physical_identities_.size() &&
                    identity.id < dormant_physical_identities_[dormant_identity].id))
                {
                    dormant_identity = i;
                    reused_cost = cost;
                }
            }
        }

        if (active_identity != nullptr)
        {
            t.id = active_identity->id;
            t.physical_identity_eligible = active_identity->physical_identity_eligible;
            t.stable_identity_anchor = active_identity->stable_identity_anchor;
            ++last_stats_.physical_id_reused;
        }
        else if (dormant_identity < dormant_physical_identities_.size())
        {
            const DormantPhysicalIdentity reused_identity =
                dormant_physical_identities_[dormant_identity];
            t.id = reused_identity.id;
            t.physical_identity_eligible = true;
            if (reused_identity.stable_anchor)
            {
                t.stable_identity_anchor = reused_identity.anchor;
            }
            dormant_physical_identities_.erase(
                dormant_physical_identities_.begin() +
                static_cast<std::ptrdiff_t>(dormant_identity));
            ++last_stats_.physical_id_reused;
        }
        else
        {
            t.id = next_physical_id_++;
        }
        t.x << detections[di].s, 0.0, detections[di].d, 0.0;
        t.P = Eigen::Matrix4d::Identity();
        t.P(0, 0) = 0.5;
        t.P(1, 1) = 4.0;
        t.P(2, 2) = 0.5;
        t.P(3, 3) = 4.0;
        t.hits = 1;
        t.ttl = p_.ttl_static;
        t.is_static = true;
        t.is_visible = true;
        t.s_half_extent = detections[di].s_half_extent;
        t.d_right_offset = detections[di].d_right_offset;
        t.d_left_offset = detections[di].d_left_offset;
        t.last_measured_s = detections[di].s;
        t.last_measured_d = detections[di].d;
        t.last_measurement_stamp = stamp;
        t.size = detections[di].size;
        t.x_min_map = detections[di].x_min;
        t.x_max_map = detections[di].x_max;
        t.y_min_map = detections[di].y_min;
        t.y_max_map = detections[di].y_max;
        const bool map_updated = mapKalmanUpdate(t, detections[di]);
        if (map_updated)
        {
            t.map_position_history.emplace_back(t.mapX(), t.mapY());
            t.map_position_rms = positionRms(t.map_position_history);
            t.total_measurement_updates = 1;
        }
        updateTrackStatus(t, true);
        classify(t, map_updated, stamp);
        if (t.track_status == TrackStatus::Confirmed)
        {
            t.physical_identity_eligible = true;
        }
        captureStableIdentityAnchor(t, detections[di]);
        tracks_.push_back(std::move(t));
        ++last_stats_.spawned;
    }

    // 5) retire dead tracks
    const std::size_t tracks_before_retirement = tracks_.size();
    if (p_.physical_id_reassociation_enable && p_.physical_id_memory_sec > 0.0)
    {
        for (const Track &track : tracks_)
        {
            if (track.ttl > 0 || !track.physical_identity_eligible)
            {
                continue;
            }
            const bool same_physical_object_still_active = std::any_of(
                tracks_.begin(), tracks_.end(),
                [&track](const Track &other) {
                    return &other != &track && other.ttl > 0 && other.id == track.id;
                });
            if (same_physical_object_still_active)
            {
                continue;
            }
            dormant_physical_identities_.erase(
                std::remove_if(
                    dormant_physical_identities_.begin(), dormant_physical_identities_.end(),
                    [&track](const DormantPhysicalIdentity &identity) {
                        return identity.id == track.id;
                    }),
                dormant_physical_identities_.end());
            DormantPhysicalIdentity identity;
            identity.id = track.id;
            identity.anchor = track.stable_identity_anchor;
            identity.stable_anchor = identity.anchor.valid;
            if (!identity.anchor.valid)
            {
                identity.anchor.valid = true;
                identity.anchor.s = track.last_measured_s;
                identity.anchor.d = track.last_measured_d;
                identity.anchor.s_half_extent = track.s_half_extent;
                identity.anchor.d_right_offset = track.d_right_offset;
                identity.anchor.d_left_offset = track.d_left_offset;
                identity.anchor.x_min_map = track.x_min_map;
                identity.anchor.x_max_map = track.x_max_map;
                identity.anchor.y_min_map = track.y_min_map;
                identity.anchor.y_max_map = track.y_max_map;
            }
            identity.last_seen_stamp = track.last_measurement_stamp;
            dormant_physical_identities_.push_back(identity);
        }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [](const Track &t) { return t.ttl <= 0; }),
                  tracks_.end());
    last_stats_.retired = tracks_before_retirement - tracks_.size();

    // Snapshot the post-update state separately from the event counters above. This prevents a
    // 1-second diagnostics window from misleadingly summing the same live tracks every scan.
    last_stats_.total_tracks = tracks_.size();
    for (const Track &t : tracks_)
    {
        if (t.is_visible)
        {
            ++last_stats_.visible_tracks;
        }
        if (!t.is_visible)
        {
            ++last_stats_.predicted_only;
        }
        if (t.track_status == TrackStatus::Raw)
        {
            ++last_stats_.raw_tracks;
        }
        else if (t.track_status == TrackStatus::Tentative)
        {
            ++last_stats_.tentative_tracks;
        }
        else if (t.motion_status == MotionStatus::Unknown)
        {
            ++last_stats_.motion_unknown;
        }
        else if (t.motion_status == MotionStatus::Static)
        {
            ++last_stats_.motion_static;
        }
        else if (t.motion_status == MotionStatus::Dynamic)
        {
            ++last_stats_.motion_dynamic;
        }
        if (t.track_status == TrackStatus::Confirmed && t.is_visible &&
            !t.velocity_covariance_valid)
        {
            ++last_stats_.invalid_velocity_covariance;
        }
        if (t.dynamic_evidence_suppressed)
        {
            ++last_stats_.translation_suppressed_dynamic_votes;
        }
    }
}

}  // namespace obstacle_detector
