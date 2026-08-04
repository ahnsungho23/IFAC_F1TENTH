// ================================================================================================
// OBSTACLE TRACKER implementation
// ================================================================================================

#include "obstacle_detector/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

void ObstacleTracker::configure(const TrackerParams &params, const FrenetProjector *frenet)
{
    p_ = params;
    frenet_ = frenet;
}

void ObstacleTracker::clear()
{
    // Called when the CLCS reference is rebuilt: existing tracks live in the old s-domain and
    // must not be predicted or published against the new reference.
    tracks_.clear();
    has_last_stamp_ = false;
}

double ObstacleTracker::frenetDistSquared(double s1, double d1, double s2, double d2) const
{
    const double ds = frenet_ ? frenet_->wrapDelta(s1, s2) : (s1 - s2);
    const double dd = d1 - d2;
    return ds * ds + dd * dd;
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

Eigen::Matrix2d ObstacleTracker::measurementCovariance(const Detection &detection) const
{
    const double scale =
        std::isfinite(detection.variance_scale) ? std::max(1.0, detection.variance_scale) : 1.0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = p_.meas_var_s * scale;
    R(1, 1) = p_.meas_var_d * scale;
    return R;
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

double ObstacleTracker::velocityMahalanobisSquared(
    const Track &t, double rel_vs, double rel_vd) const
{
    Eigen::Matrix2d velocity_covariance;
    velocity_covariance << t.P(1, 1), t.P(1, 3),
                           t.P(3, 1), t.P(3, 3);
    const Eigen::LDLT<Eigen::Matrix2d> solver(velocity_covariance);
    if (solver.info() != Eigen::Success || !solver.isPositive())
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
                             "Velocity covariance not positive definite; motion not confident");
        return std::numeric_limits<double>::quiet_NaN();
    }

    const Eigen::Vector2d relative_velocity(rel_vs, rel_vd);
    const Eigen::Vector2d solved = solver.solve(relative_velocity);
    if (solver.info() != Eigen::Success || !solved.allFinite())
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("obstacle_tracker"), warnThrottleClock(), 2000,
                             "Velocity covariance solve failed; motion not confident");
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::max(0.0, relative_velocity.dot(solved));
}

void ObstacleTracker::classify(
    Track &t, double ego_yaw_rate, bool yaw_rate_fresh) const
{
    // Detection confirmation and motion classification deliberately use separate state machines.
    // Hits 1..(min_hits_confirm-1) are completely hidden. The confirming observation publishes the
    // track immediately as provisional static, then later observations collect motion evidence.
    if (!t.classified)
    {
        t.motion_class = MotionClass::Pending;
        t.is_static = true;
        t.dyn_streak = 0;
        t.static_streak = 0;
        t.relative_speed = 0.0;
        t.velocity_mahalanobis_sq = 0.0;
        t.dynamic_motion_reliable = false;
        if (t.is_visible && t.hits >= std::max(1, p_.min_hits_confirm))
        {
            t.classified = true;
            t.motion_class = MotionClass::ProvisionalStatic;
        }
        return;
    }

    // Evidence must come from consecutive measurements, never from prediction-only frames.
    if (!t.is_visible)
    {
        t.dyn_streak = 0;
        t.static_streak = 0;
        t.dynamic_motion_reliable = false;
        t.is_static = t.motion_class != MotionClass::Dynamic;
        return;
    }

    // ---- velocity evidence: flow RELATIVE to the map-flow reference, with hysteresis ----
    const double rel_vs = t.vs() - static_ref_vs_;
    const double rel_vd = t.vd() - static_ref_vd_;
    const double rel_speed = std::hypot(rel_vs, rel_vd);
    t.relative_speed = rel_speed;
    t.velocity_mahalanobis_sq = velocityMahalanobisSquared(t, rel_vs, rel_vd);

    const bool yaw_reliable =
        yaw_rate_fresh && std::isfinite(ego_yaw_rate) &&
        (p_.dyn_max_abs_yaw_rate <= 0.0 ||
         std::abs(ego_yaw_rate) <= p_.dyn_max_abs_yaw_rate);
    const bool velocity_confident =
        p_.dyn_velocity_mahalanobis_gate <= 0.0 ||
        (std::isfinite(t.velocity_mahalanobis_sq) &&
         t.velocity_mahalanobis_sq >= p_.dyn_velocity_mahalanobis_gate);
    const bool vel_dynamic =
        rel_speed > p_.dyn_vel_enter && velocity_confident && yaw_reliable;
    const bool vel_static = rel_speed < p_.dyn_vel_exit;
    t.dynamic_motion_reliable = vel_dynamic;

    // ---- positional-spread vote (ForzaETH style) ----
    bool std_dynamic = false;
    bool std_static = false;
    const bool uses_std_classifier = p_.classifier_mode != ClassifierMode::Velocity;
    if (uses_std_classifier && static_cast<int>(t.hist.size()) >= p_.min_nb_meas)
    {
        double mean_s = 0.0;
        double mean_d = 0.0;
        for (const auto &h : t.hist)
        {
            mean_s += h.first;
            mean_d += h.second;
        }
        mean_s /= static_cast<double>(t.hist.size());
        mean_d /= static_cast<double>(t.hist.size());
        double var_s = 0.0;
        double var_d = 0.0;
        for (const auto &h : t.hist)
        {
            const double es = frenet_ ? frenet_->wrapDelta(h.first, mean_s) : (h.first - mean_s);
            const double ed = h.second - mean_d;
            var_s += es * es;
            var_d += ed * ed;
        }
        var_s /= static_cast<double>(t.hist.size());
        var_d /= static_cast<double>(t.hist.size());
        const double std_s = std::sqrt(var_s);
        const double std_d = std::sqrt(var_d);
        if (std_s < p_.min_std && std_d < p_.min_std)
        {
            std_static = true;
        }
        else if (std_s > p_.max_std || std_d > p_.max_std)
        {
            std_dynamic = true;
        }
    }

    bool dynamic_evidence = false;
    bool static_evidence = false;
    switch (p_.classifier_mode)
    {
        case ClassifierMode::Velocity:
            dynamic_evidence = vel_dynamic;
            static_evidence = vel_static;
            break;
        case ClassifierMode::Std:
            dynamic_evidence = std_dynamic && yaw_reliable;
            static_evidence = std_static;
            break;
        case ClassifierMode::Both:
            dynamic_evidence = vel_dynamic && std_dynamic;
            static_evidence = vel_static || std_static;
            break;
    }

    if (dynamic_evidence)
    {
        ++t.dyn_streak;
        t.static_streak = 0;
    }
    else if (static_evidence)
    {
        ++t.static_streak;
        t.dyn_streak = 0;
    }
    else
    {
        // The hysteresis band, weak velocity confidence, and rapid/stale ego yaw all break a
        // consecutive-evidence streak rather than silently accumulating ambiguous observations.
        t.dyn_streak = 0;
        t.static_streak = 0;
    }

    const int static_frames = std::max(1, p_.static_confirm_frames);
    const int dynamic_frames = std::max(1, p_.dynamic_confirm_frames);
    if (t.motion_class == MotionClass::Dynamic)
    {
        if (t.static_streak >= static_frames)
        {
            t.motion_class = MotionClass::ConfirmedStatic;
            t.static_streak = 0;
        }
    }
    else if (t.dyn_streak >= dynamic_frames)
    {
        t.motion_class = MotionClass::Dynamic;
        t.dyn_streak = 0;
    }
    else if (t.motion_class == MotionClass::ProvisionalStatic &&
             t.static_streak >= static_frames)
    {
        t.motion_class = MotionClass::ConfirmedStatic;
        t.static_streak = 0;
    }

    t.is_static = t.motion_class != MotionClass::Dynamic;
}

void ObstacleTracker::updateStaticReference()
{
    // The map-flow reference velocity is the mean Frenet velocity of the clearly-slow tracks. Walls
    // are already removed by the Layer-1 map filter, so these are stationary obstacles (and any
    // residual static structure) — the layer that flows WITH the map. Their shared apparent
    // velocity is the reference the dynamic opponent is measured against. The gate keeps the
    // fast-moving opponent out of the reference (it must not define its own baseline).
    double sum_vs = 0.0;
    double sum_vd = 0.0;
    int n = 0;
    const double static_ref_gate_sq = p_.static_ref_gate * p_.static_ref_gate;
    for (const auto &t : tracks_)
    {
        const double speed_sq = t.vs() * t.vs() + t.vd() * t.vd();
        if (p_.static_ref_gate > 0.0 && speed_sq < static_ref_gate_sq)
        {
            sum_vs += t.vs();
            sum_vd += t.vd();
            ++n;
        }
    }
    if (n > 0)
    {
        static_ref_vs_ = sum_vs / static_cast<double>(n);
        static_ref_vd_ = sum_vd / static_cast<double>(n);
    }
    else
    {
        static_ref_vs_ = 0.0;
        static_ref_vd_ = 0.0;
    }
}

void ObstacleTracker::update(
    const std::vector<Detection> &detections, double stamp,
    double ego_yaw_rate, bool yaw_rate_fresh)
{
    last_stats_ = TrackerUpdateStats{};

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
        predict(t, dt);
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
            const double center_shift = std::sqrt(
                frenetDistSquared(t.s(), t.d(), det.s, det.d));
            const double envelope_shift = std::max(
                {std::abs(det.s_half_extent - t.s_half_extent),
                 std::abs(det.d_right_offset - t.d_right_offset),
                 std::abs(det.d_left_offset - t.d_left_offset)});
            if (center_shift <= p_.envelope_stability_tolerance_m &&
                envelope_shift <= p_.envelope_stability_tolerance_m)
            {
                ++t.envelope_stable_streak;
            }
            else
            {
                t.envelope_stable_streak = 0;
            }
            kalmanUpdate(t, det);
            t.hits++;
            t.is_visible = true;
            t.s_half_extent = smoothExtent(t.s_half_extent, det.s_half_extent);
            t.d_right_offset = smoothExtent(t.d_right_offset, det.d_right_offset);
            t.d_left_offset = smoothExtent(t.d_left_offset, det.d_left_offset);
            t.size = det.size;
            t.x_min_map = det.x_min;
            t.x_max_map = det.x_max;
            t.y_min_map = det.y_min;
            t.y_max_map = det.y_max;
            if (p_.classifier_mode != ClassifierMode::Velocity)
            {
                t.hist.emplace_back(t.x(0), t.x(2));
                while (static_cast<int>(t.hist.size()) > p_.std_window)
                {
                    t.hist.pop_front();
                }
            }
        }
        else
        {
            t.ttl--;
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
        t.id = next_id_++;
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
        t.size = detections[di].size;
        t.x_min_map = detections[di].x_min;
        t.x_max_map = detections[di].x_max;
        t.y_min_map = detections[di].y_min;
        t.y_max_map = detections[di].y_max;
        if (p_.classifier_mode != ClassifierMode::Velocity)
        {
            t.hist.emplace_back(t.x(0), t.x(2));
        }
        tracks_.push_back(std::move(t));
        ++last_stats_.spawned;
    }

    // 4b) refresh the map-flow reference from slow tracks, then classify every track relative to it
    //     (Layer 2 = flows with the map -> static; Layer 3 = clearly deviates -> dynamic opponent)
    updateStaticReference();
    for (auto &t : tracks_)
    {
        classify(t, ego_yaw_rate, yaw_rate_fresh);
        if (t.is_visible)  // matched or freshly spawned this frame
        {
            t.ttl = t.is_static ? p_.ttl_static : p_.ttl_dynamic;
        }
    }

    // 5) retire dead tracks
    const std::size_t tracks_before_retirement = tracks_.size();
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
        if (t.hits < p_.min_hits_confirm)
        {
            ++last_stats_.hit_confirmation_pending;
        }
        else if (t.motion_class == MotionClass::ProvisionalStatic)
        {
            ++last_stats_.provisional_static;
        }
        else if (t.motion_class == MotionClass::ConfirmedStatic)
        {
            ++last_stats_.confirmed_static;
        }
        else if (t.motion_class == MotionClass::Dynamic)
        {
            ++last_stats_.confirmed_dynamic;
        }
        if (t.is_visible && t.relative_speed > p_.dyn_vel_enter &&
            !t.dynamic_motion_reliable)
        {
            ++last_stats_.dynamic_motion_gated;
        }
    }
}

}  // namespace obstacle_detector
