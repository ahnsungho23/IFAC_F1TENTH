// ================================================================================================
// OBSTACLE TRACKER implementation
// ================================================================================================

#include "opponent_detector/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace opponent_detector
{

void ObstacleTracker::configure(const TrackerParams &params, const FrenetProjector *frenet)
{
    p_ = params;
    frenet_ = frenet;
}

double ObstacleTracker::frenetDist(double s1, double d1, double s2, double d2) const
{
    const double ds = frenet_ ? frenet_->wrapDelta(s1, s2) : (s1 - s2);
    const double dd = d1 - d2;
    return std::hypot(ds, dd);
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
    const double scale = std::max(1.0, detection.variance_scale);
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
    if (solver.info() != Eigen::Success)
    {
        return std::numeric_limits<double>::infinity();
    }
    return innovation.dot(solver.solve(innovation));
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
    Eigen::Matrix<double, 4, 2> K = t.P * H.transpose() * S.inverse();
    t.x = t.x + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    t.P = (I - K * H) * t.P;

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

void ObstacleTracker::classify(Track &t) const
{
    // ---- velocity-based vote (relative to the static field, with hysteresis) ----
    // Walls (removed by the map filter) and stationary obstacles share the static-field velocity
    // (static_ref_*). A track that moves WITH the static field is static; one that clearly deviates
    // is the dynamic opponent. Comparing to the field (not to absolute zero) cancels any common
    // ego-localization drift, so a car-sized stationary obstacle is not mistaken for the opponent.
    bool vel_dynamic = t.is_static ? false : true;  // start from current label
    const double rel_vs = t.vs() - static_ref_vs_;
    const double rel_vd = t.vd() - static_ref_vd_;
    const double rel_speed = std::hypot(rel_vs, rel_vd);
    if (rel_speed > p_.dyn_vel_enter)
    {
        t.dyn_streak++;
        t.static_streak = 0;
    }
    else if (rel_speed < p_.dyn_vel_exit)
    {
        t.static_streak++;
        t.dyn_streak = 0;
    }
    if (t.dyn_streak >= p_.dyn_min_frames)
    {
        vel_dynamic = true;
    }
    if (t.static_streak >= p_.dyn_min_frames)
    {
        vel_dynamic = false;
    }
    // low relative-speed backstop (moving with the static field -> static)
    if (std::abs(rel_vs) < p_.vs_reset && std::abs(rel_vd) < p_.vs_reset)
    {
        vel_dynamic = false;
    }

    // ---- positional-spread vote (ForzaETH style) ----
    bool std_dynamic = !t.is_static;
    if (static_cast<int>(t.hist.size()) >= p_.min_nb_meas)
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
            std_dynamic = false;
        }
        else if (std_s > p_.max_std || std_d > p_.max_std)
        {
            std_dynamic = true;
        }
    }

    switch (p_.classifier_mode)
    {
        case ClassifierMode::Velocity:
            t.is_static = !vel_dynamic;
            break;
        case ClassifierMode::Std:
            t.is_static = !std_dynamic;
            break;
        case ClassifierMode::Both:
            // dynamic only when both agree (conservative)
            t.is_static = !(vel_dynamic && std_dynamic);
            break;
    }
}

void ObstacleTracker::updateStaticReference()
{
    // The static-field velocity is the mean Frenet velocity of the clearly-slow tracks. Walls are
    // already removed by the map filter, so these are stationary obstacles (and any residual static
    // structure); their shared apparent velocity is the reference dynamic tracks are measured
    // against. The gate keeps the fast-moving opponent out of the reference.
    double sum_vs = 0.0;
    double sum_vd = 0.0;
    int n = 0;
    for (const auto &t : tracks_)
    {
        if (t.hits < 1)
        {
            continue;
        }
        if (t.speed() < p_.static_ref_gate)
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

void ObstacleTracker::update(const std::vector<Detection> &detections, double stamp)
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
        for (std::size_t di = 0; di < nd; ++di)
        {
            const double c = frenetDist(tracks_[ti].x(0), tracks_[ti].x(2), detections[di].s,
                                        detections[di].d);
            if (c > gate)
            {
                ++last_stats_.euclidean_rejected;
                continue;
            }
            if (p_.assoc_use_mahalanobis)
            {
                const double m2 = innovationDistanceSquared(tracks_[ti], detections[di]);
                if (!std::isfinite(m2) || m2 > p_.assoc_mahalanobis_gate)
                {
                    ++last_stats_.mahalanobis_rejected;
                    continue;
                }
                pairs.push_back({ti, di, m2});
            }
            else
            {
                pairs.push_back({ti, di, c});
            }
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
    }

    // 3) update matched tracks
    for (std::size_t ti = 0; ti < nt; ++ti)
    {
        Track &t = tracks_[ti];
        if (trk_assigned[ti] >= 0)
        {
            const Detection &det = detections[trk_assigned[ti]];
            kalmanUpdate(t, det);
            ++last_stats_.matched;
            t.hits++;
            t.misses = 0;
            t.is_visible = true;
            t.size = det.size;
            t.x_map = det.x;
            t.y_map = det.y;
            t.x_min_map = det.x_min;
            t.x_max_map = det.x_max;
            t.y_min_map = det.y_min;
            t.y_max_map = det.y_max;
            t.hist.emplace_back(t.x(0), t.x(2));
            while (static_cast<int>(t.hist.size()) > p_.std_window)
            {
                t.hist.pop_front();
            }
        }
        else
        {
            t.misses++;
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
        t.misses = 0;
        t.ttl = p_.ttl_static;
        t.is_static = true;
        t.is_visible = true;
        t.size = detections[di].size;
        t.x_map = detections[di].x;
        t.y_map = detections[di].y;
        t.x_min_map = detections[di].x_min;
        t.x_max_map = detections[di].x_max;
        t.y_min_map = detections[di].y_min;
        t.y_max_map = detections[di].y_max;
        t.hist.emplace_back(t.x(0), t.x(2));
        tracks_.push_back(std::move(t));
        ++last_stats_.spawned;
    }

    // 4b) refresh the static-field reference from slow tracks, then classify every track relative
    //     to it (so stationary obstacles read static and only the deviating opponent reads dynamic)
    updateStaticReference();
    for (auto &t : tracks_)
    {
        classify(t);
        if (t.misses == 0)  // matched or freshly spawned this frame
        {
            t.ttl = t.is_static ? p_.ttl_static : p_.ttl_dynamic;
        }
    }

    // 5) retire dead tracks
    const auto before_retire = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [](const Track &t) { return t.ttl <= 0; }),
                  tracks_.end());
    last_stats_.retired = static_cast<int>(before_retire - tracks_.size());
}

int ObstacleTracker::opponentIndex(double ego_s) const
{
    int best = -1;
    double best_key = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < tracks_.size(); ++i)
    {
        const Track &t = tracks_[i];
        if (t.is_static || t.hits < p_.min_hits_confirm)
        {
            continue;
        }
        double key;
        if (ego_s >= 0.0 && frenet_)
        {
            double ahead = frenet_->wrapDelta(t.x(0), ego_s);
            if (ahead < 0.0)
            {
                ahead += frenet_->raceline_length();  // prefer the one ahead on track
            }
            key = ahead;
        }
        else
        {
            key = t.P(0, 0) + t.P(2, 2);  // lowest positional uncertainty
        }
        if (key < best_key)
        {
            best_key = key;
            best = static_cast<int>(i);
        }
    }
    return best;
}

}  // namespace opponent_detector
