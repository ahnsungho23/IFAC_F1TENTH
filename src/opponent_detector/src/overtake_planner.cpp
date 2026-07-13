// ================================================================================================
// OVERTAKE PLANNER - implementation
// ================================================================================================

#include "opponent_detector/overtake_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace opponent_detector
{

namespace
{

// Shape-preserving monotone cubic Hermite (PCHIP, Fritsch-Carlson). Unlike a natural cubic spline it
// does NOT overshoot: flat 0 knots stay at 0, so the line never swerves the wrong way before the apex.
class Pchip
{
  public:
    bool build(const std::vector<double> &t, const std::vector<double> &v)
    {
        const std::size_t n = t.size();
        if (n < 2 || v.size() != n)
        {
            return false;
        }
        t_ = t;
        v_ = v;
        std::vector<double> h(n - 1), del(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            h[i] = t[i + 1] - t[i];
            if (h[i] <= 0.0)
            {
                return false;
            }
            del[i] = (v[i + 1] - v[i]) / h[i];  // secant slope
        }
        m_.assign(n, 0.0);
        if (n == 2)
        {
            m_[0] = m_[1] = del[0];
            return true;
        }
        // Interior tangents: zero at local extrema / sign changes, else weighted harmonic mean.
        for (std::size_t i = 1; i + 1 < n; ++i)
        {
            if (del[i - 1] * del[i] <= 0.0)
            {
                m_[i] = 0.0;
            }
            else
            {
                const double w1 = 2.0 * h[i] + h[i - 1];
                const double w2 = h[i] + 2.0 * h[i - 1];
                m_[i] = (w1 + w2) / (w1 / del[i - 1] + w2 / del[i]);
            }
        }
        // One-sided, shape-preserving endpoint tangents.
        m_[0] = endpointSlope(h[0], h[1], del[0], del[1]);
        m_[n - 1] = endpointSlope(h[n - 2], h[n - 3], del[n - 2], del[n - 3]);
        return true;
    }

    double eval(double t) const
    {
        const std::size_t n = t_.size();
        if (n == 0)
        {
            return 0.0;
        }
        if (t <= t_.front())
        {
            return v_.front();
        }
        if (t >= t_.back())
        {
            return v_.back();
        }
        std::size_t i = 0;
        while (i + 1 < n && t_[i + 1] < t)
        {
            ++i;
        }
        const double h = t_[i + 1] - t_[i];
        const double s = (t - t_[i]) / h;  // normalized [0,1]
        const double s2 = s * s, s3 = s2 * s;
        // Hermite basis.
        const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        const double h10 = s3 - 2.0 * s2 + s;
        const double h01 = -2.0 * s3 + 3.0 * s2;
        const double h11 = s3 - s2;
        return h00 * v_[i] + h10 * h * m_[i] + h01 * v_[i + 1] + h11 * h * m_[i + 1];
    }

    double tEnd() const { return t_.empty() ? 0.0 : t_.back(); }

  private:
    static double endpointSlope(double h0, double h1, double del0, double del1)
    {
        // Non-centered three-point formula, clamped to preserve shape (avoid endpoint overshoot).
        double m = ((2.0 * h0 + h1) * del0 - h0 * del1) / (h0 + h1);
        if (m * del0 <= 0.0)
        {
            m = 0.0;
        }
        else if (del0 * del1 <= 0.0 && std::abs(m) > std::abs(3.0 * del0))
        {
            m = 3.0 * del0;
        }
        return m;
    }

    std::vector<double> t_, v_, m_;
};

double shortestAngleLerp(double a0, double a1, double frac)
{
    double diff = std::fmod(a1 - a0 + M_PI, 2.0 * M_PI);
    if (diff < 0.0)
    {
        diff += 2.0 * M_PI;
    }
    diff -= M_PI;
    return a0 + diff * frac;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Raceline reference
// ------------------------------------------------------------------------------------------------
void OvertakePlanner::setReference(const f110_msgs::msg::WpntArray &global, double track_length,
                                   double fallback_halfwidth)
{
    ref_.clear();
    ref_.reserve(global.wpnts.size());
    for (const auto &w : global.wpnts)
    {
        Ref r;
        r.s = w.s_m;
        r.x = w.x_m;
        r.y = w.y_m;
        r.psi = w.psi_rad;
        r.vx = w.vx_mps;
        r.kappa = w.kappa_radpm;
        // Racelines exported without bounds carry d_left/d_right = 0; substitute the fallback
        // half-width (same rule as the detector's corridor gate) so the planner has a real track.
        r.dl = (w.d_left > 0.05) ? w.d_left : fallback_halfwidth;
        r.dr = (w.d_right > 0.05) ? w.d_right : fallback_halfwidth;
        ref_.push_back(r);
    }
    std::sort(ref_.begin(), ref_.end(), [](const Ref &a, const Ref &b) { return a.s < b.s; });
    // Prefer the explicit track length; fall back to last-s if unset.
    if (track_length > 1e-3)
    {
        length_ = track_length;
    }
    else if (!ref_.empty())
    {
        length_ = ref_.back().s;
    }
    // A new raceline invalidates any committed maneuver.
    state_ = State::Idle;
    committed_.wpnts.clear();
}

double OvertakePlanner::wrapS(double s) const
{
    if (length_ <= 0.0)
    {
        return s;
    }
    double r = std::fmod(s, length_);
    if (r < 0.0)
    {
        r += length_;
    }
    return r;
}

double OvertakePlanner::fwdDelta(double from, double to) const
{
    return wrapS(to - from);
}

void OvertakePlanner::interp(double s, double &x, double &y, double &psi, double &vx,
                             double &kappa, double &dl, double &dr) const
{
    const std::size_t n = ref_.size();
    const double sw = wrapS(s);
    // Find last index with ref_[i].s <= sw.
    std::size_t i = 0;
    for (std::size_t k = 0; k < n; ++k)
    {
        if (ref_[k].s <= sw)
        {
            i = k;
        }
        else
        {
            break;
        }
    }
    const std::size_t j = (i + 1) % n;
    const double s_i = ref_[i].s;
    const double s_j = (j == 0) ? length_ : ref_[j].s;  // wrap segment ends at the perimeter
    double seg = s_j - s_i;
    double frac = (seg > 1e-6) ? (sw - s_i) / seg : 0.0;
    frac = std::max(0.0, std::min(1.0, frac));

    const Ref &A = ref_[i];
    const Ref &B = ref_[j];
    x = A.x + frac * (B.x - A.x);
    y = A.y + frac * (B.y - A.y);
    vx = A.vx + frac * (B.vx - A.vx);
    kappa = A.kappa + frac * (B.kappa - A.kappa);
    dl = A.dl + frac * (B.dl - A.dl);
    dr = A.dr + frac * (B.dr - A.dr);
    psi = shortestAngleLerp(A.psi, B.psi, frac);
}

void OvertakePlanner::toCartesian(double s, double d, double &x, double &y) const
{
    double x_r, y_r, psi_r, vx, kappa, dl, dr;
    interp(s, x_r, y_r, psi_r, vx, kappa, dl, dr);
    x = x_r - d * std::sin(psi_r);
    y = y_r + d * std::cos(psi_r);
}

double OvertakePlanner::attainableSpeed(const EgoState &ego) const
{
    // The commit decision must not depend on the ego being ALREADY fast (it may be stuck behind
    // the opponent): use the raceline speed at the ego position as the attainable speed, but never
    // less than what the car is actually doing right now.
    double x, y, psi, vx, kappa, dl, dr;
    interp(ego.s, x, y, psi, vx, kappa, dl, dr);
    return std::max(ego.v, vx);
}

// ------------------------------------------------------------------------------------------------
// Path construction (all commit gates live here)
// ------------------------------------------------------------------------------------------------
OvertakePlanner::PlanResult OvertakePlanner::buildPath(const EgoState &ego,
                                                       const OpponentState &opp,
                                                       const OvertakeParams &p,
                                                       int forced_side) const
{
    PlanResult res;
    if (!ready() || ego.s < 0.0 || !opp.valid)
    {
        res.reject = !ready() ? "no raceline" : (ego.s < 0.0 ? "no ego s" : "no opponent");
        return res;
    }

    const bool replanning = (state_ == State::Committed);
    const double ds_opp = fwdDelta(ego.s, opp.s);

    auto fmt = [](double v) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.2f", v);
        return std::string(buf);
    };

    // Trigger window. While replanning mid-maneuver the opponent may legitimately be alongside
    // (ds_opp ~ 0) — only the upper bound applies then.
    if (ds_opp > p.trigger_max_ds)
    {
        res.reject = "out of window: ds=" + fmt(ds_opp) + " > max " + fmt(p.trigger_max_ds);
        return res;
    }
    if (!replanning && ds_opp < p.trigger_min_ds)
    {
        res.reject = "out of window: ds=" + fmt(ds_opp) + " < min " + fmt(p.trigger_min_ds);
        return res;
    }
    // Opponent already behind (wrapped delta > half the track) -> nothing to overtake.
    if (ds_opp > 0.5 * length_)
    {
        res.reject = "opponent behind (ds=" + fmt(ds_opp) + ")";
        return res;
    }

    const double half_sum = p.ego_half_width + p.opponent_half_width;

    // Corridor block test: if the opponent does not overlap the ego corridor it does not block us.
    if (!replanning && std::abs(opp.d) > half_sum + p.block_margin)
    {
        res.reject = "not blocking: |opp_d|=" + fmt(std::abs(opp.d)) + " > " +
                     fmt(half_sum + p.block_margin);
        return res;
    }

    // ---- catchability: relative speed and predicted catch time ----
    const double v_att = attainableSpeed(ego);
    const double v_rel = v_att - opp.vs;
    if (v_rel < p.min_rel_vel)
    {
        res.reject = "rel vel too low: v_rel=" + fmt(v_rel) + " (v_att=" + fmt(v_att) +
                     ", opp_vs=" + fmt(opp.vs) + ") < " + fmt(p.min_rel_vel);
        return res;
    }
    const double t_catch = ds_opp / v_rel;
    if (t_catch > p.max_catch_time)
    {
        res.reject = "catch too slow: t=" + fmt(t_catch) + "s > " + fmt(p.max_catch_time) + "s";
        return res;
    }

    // Steering/curvature limits on the lateral spline. Following the car means matching the path
    // heading (slope dd/ds) AND the path curvature d2d/ds2 — the latter sets the steering angle, so
    // a curvature spike (short ramp) spins the car even when the slope is modest. A PCHIP transition
    // between a flat hold and the apex hold, of amplitude A over arc length L, peaks at
    //   |d'|  = 1.5 * A / L      and      |d''| = 6 * A / L^2 .
    // A ramp must therefore be at least max(1.5*A/slope_lim, sqrt(6*A/kappa_lim)) long; both the
    // reachable apex and the ramp lengths are sized from that, and the fitted spline is verified.
    const double slope_lim = std::max(0.05, p.max_d_slope);
    const double kappa_lim = std::max(1e-3, p.max_path_kappa);
    const double kSlopeGain = 1.5;  // PCHIP flat-to-flat: peak |d'|  = 1.5 * A / L
    const double kCurvGain = 6.0;   // PCHIP flat-to-flat: peak |d''| = 6   * A / L^2
    auto ramp_len = [&](double amp) {
        const double a = std::abs(amp);
        return std::max(kSlopeGain * a / slope_lim, std::sqrt(kCurvGain * a / kappa_lim));
    };

    const double clearance = half_sum + p.lateral_clearance;

    // ---- pass-point prediction: the opponent keeps moving while we close the gap ----
    // Ego travels ds_pass = v_att * t_catch until the pass; apex is placed THERE.
    double ds_pass = std::max(ds_opp, v_att * t_catch);

    // Maneuver extent along the raceline (from ego, in unwrapped t).
    double t_apex0 = std::max(0.3, ds_pass - p.pass_clearance_s);
    double t_apex1 = std::max(t_apex0 + 0.5, ds_pass + p.pass_clearance_s);

    // ---- runway extension: swing out FIRST when the catch point is too close ----
    // While TRAILING close behind (gap ~ ot_trail_gap) the predicted catch point can be nearer
    // than the shortest ramp that reaches a clearing apex under the slope/curvature limits.
    // Rejecting in that case (the old behaviour) locks the planner into trailing forever — no
    // start position ever gets a longer runway. Instead push the apex out to the minimum
    // feasible runway: the ego swings out first (the pre-apex gap hold in the speed profile
    // keeps it from closing on the still-centered opponent), and the pass is predicted from the
    // apex point onwards with the gap it will still be holding there.
    {
        const double amp_max = clearance + std::abs(opp.d - ego.d);
        const double min_runway = ramp_len(amp_max);
        if (t_apex0 < min_runway)
        {
            t_apex0 = min_runway;
            const double gap_hold = std::max(0.5, p.trail_gap);
            const double catch_from_hold =
                std::min(v_att * gap_hold / std::max(0.5, v_rel), p.trigger_max_ds);
            ds_pass = t_apex0 + catch_from_hold;
            t_apex1 = std::max(t_apex0 + 0.5, ds_pass + p.pass_clearance_s);
        }
    }
    const double s_pass = wrapS(ego.s + ds_pass);

    const double t_merge_min = t_apex1 + std::max(0.5, p.post_distance);
    const double t_gate_end = t_merge_min + 1.0;

    // ---- corner gate: no sharp corner anywhere on the (minimal) maneuver ----
    // The slope/curvature-extended merge region beyond t_gate_end is re-checked per apex inside the
    // fit, where a smaller apex can still shorten the maneuver out of the corner.
    {
        const double step = std::max(0.2, p.spline_resolution);
        for (double t = 0.0; t <= t_gate_end + 1e-6; t += step)
        {
            double x, y, psi, vx, kappa, dl, dr;
            interp(ego.s + t, x, y, psi, vx, kappa, dl, dr);
            if (std::abs(kappa) > p.max_kappa)
            {
                res.reject = "corner on maneuver: |kappa|=" + fmt(std::abs(kappa)) + " at s=" +
                             fmt(wrapS(ego.s + t)) + " > " + fmt(p.max_kappa);
                return res;
            }
        }
    }

    // ---- lateral room at the PREDICTED pass location (coarse gate) ----
    double x_o, y_o, psi_o, vx_o, kappa_o, dl_o, dr_o;
    interp(s_pass, x_o, y_o, psi_o, vx_o, kappa_o, dl_o, dr_o);

    // Initial apex offset on a given side, capped by the wall margin at the pass point AND by the
    // lateral distance reachable from the ego offset under the steering/curvature limits before the
    // apex hold begins; false if the side cannot clear the opponent even before fitting.
    // reach solves ramp_len(A) <= t_apex0 for A: min over the slope and curvature bounds.
    auto apex_on_side = [&](bool go_left, double &d_apex) -> bool {
        const double reach = std::min(slope_lim * t_apex0 / kSlopeGain,
                                      kappa_lim * t_apex0 * t_apex0 / kCurvGain);
        if (go_left)
        {
            const double cap = dl_o - p.boundary_margin - p.ego_half_width;
            d_apex = std::min(opp.d + clearance, std::min(cap, ego.d + reach));
        }
        else
        {
            const double cap = -(dr_o - p.boundary_margin - p.ego_half_width);
            d_apex = std::max(opp.d - clearance, std::max(cap, ego.d - reach));
        }
        return std::abs(d_apex - opp.d) >= half_sum;  // still clears the opponent after capping
    };

    const double room_left = (dl_o - p.boundary_margin) - (opp.d + p.opponent_half_width);
    const double room_right = (opp.d - p.opponent_half_width) - (-(dr_o - p.boundary_margin));
    const double min_safe_room = 2.0 * p.ego_half_width + p.lateral_clearance;
    const bool left_ok = room_left >= min_safe_room;
    const bool right_ok = room_right >= min_safe_room;

    // Spline knots for a given apex: CURRENT ego d -> apex hold around the pass -> raceline.
    // Ramp lengths are sized so the PCHIP slope stays within the steering limit.
    auto make_knots = [&](double d_apex, std::vector<double> &tk, std::vector<double> &dk) -> bool {
        tk.clear();
        dk.clear();
        tk.push_back(0.0);
        dk.push_back(ego.d);
        // Stay on the raceline until the entry ramp must begin — but only when the ego actually
        // IS on the raceline; when replanning off-line, ramp directly from the current offset.
        const double ramp_in = std::max(p.pre_distance, ramp_len(d_apex - ego.d));
        const double t_dev = t_apex0 - ramp_in;
        if (t_dev > 0.3 && std::abs(ego.d) < 0.1)
        {
            tk.push_back(t_dev);
            dk.push_back(0.0);
        }
        tk.push_back(t_apex0);
        dk.push_back(d_apex);
        tk.push_back(t_apex1);
        dk.push_back(d_apex);
        // Exit ramp long enough for both the merge slope and curvature to stay within limits.
        const double t_merge = t_apex1 + std::max(std::max(0.5, p.post_distance), ramp_len(d_apex));
        tk.push_back(t_merge);
        dk.push_back(0.0);
        tk.push_back(t_merge + 1.0);  // short flat tail so the merge lands on the raceline
        dk.push_back(0.0);
        // Guard strictly-increasing knots.
        for (std::size_t k = 1; k < tk.size(); ++k)
        {
            if (tk[k] <= tk[k - 1] + 1e-3)
            {
                tk.erase(tk.begin() + k);
                dk.erase(dk.begin() + k);
                --k;
            }
        }
        return tk.size() >= 2;
    };

    // ---- apex fit: the WHOLE spline must stay inside the wall margin ----
    // Clamping samples against the boundary afterwards (old approach) flattened the line along the
    // wall and broke the smooth spline shape. Instead, shrink the apex until the spline itself
    // clears every local bound along the maneuver; reject the side when the shrunken apex can no
    // longer clear the opponent. The published line is then always the untouched spline.
    const double fit_step = std::max(0.05, p.spline_resolution);
    auto fit_side = [&](bool go_left, double &apex_out, Pchip &spline_out,
                        std::string &why) -> bool {
        double apex;
        if (!apex_on_side(go_left, apex))
        {
            why = "apex cannot clear opponent (wall/steering limits)";
            return false;
        }
        const double sgn = go_left ? 1.0 : -1.0;
        // Samples still between the ego's CURRENT offset and the raceline cannot be improved by a
        // smaller apex — only enforce the margin where the path deviates beyond the ego offset.
        const double e_skip = std::max(0.05, sgn * ego.d);
        for (int iter = 0; iter < 8; ++iter)
        {
            std::vector<double> tk, dk;
            if (!make_knots(apex, tk, dk))
            {
                why = "degenerate knots";
                return false;
            }
            Pchip sp;
            if (!sp.build(tk, dk))
            {
                why = "spline build failed";
                return false;
            }
            const double t_total = sp.tEnd();
            // The steering-limited exit ramp can stretch the maneuver beyond the pre-checked
            // minimal extent — re-check the corner gate there (a smaller apex shortens it again).
            if (t_total > t_gate_end + 1e-6)
            {
                bool corner = false;
                const double step = std::max(0.2, p.spline_resolution);
                for (double t = t_gate_end; t <= t_total + 1e-6; t += step)
                {
                    double x, y, psi, vx, kappa, dl_i, dr_i;
                    interp(ego.s + t, x, y, psi, vx, kappa, dl_i, dr_i);
                    if (std::abs(kappa) > p.max_kappa)
                    {
                        corner = true;
                        break;
                    }
                }
                if (corner)
                {
                    apex = sgn * (0.85 * (sgn * apex));
                    if (sgn * (apex - opp.d) < half_sum)
                    {
                        why = "corner inside the steering-limited merge";
                        return false;
                    }
                    continue;
                }
            }
            // Worst-case scale factor so that spline(t) <= local wall cap everywhere; the same
            // sweep also measures the steepest lateral slope AND curvature the spline produces.
            double alpha = 1.0;
            double max_slope = 0.0;
            double max_pcurv = 0.0;  // peak |d''(s)| of the lateral spline
            double d_m1 = 0.0, d_m2 = 0.0;
            int have = 0;
            for (double t = 0.0; t <= t_total + 1e-6; t += fit_step)
            {
                double x, y, psi, vx, kappa, dl_i, dr_i;
                interp(ego.s + t, x, y, psi, vx, kappa, dl_i, dr_i);
                const double d_t = sp.eval(t);
                if (have >= 1)
                {
                    max_slope = std::max(max_slope, std::abs(d_t - d_m1) / fit_step);
                }
                if (have >= 2)
                {
                    const double d2 = (d_t - 2.0 * d_m1 + d_m2) / (fit_step * fit_step);
                    max_pcurv = std::max(max_pcurv, std::abs(d2));
                }
                d_m2 = d_m1;
                d_m1 = d_t;
                ++have;
                const double cap = (go_left ? dl_i : dr_i) - p.boundary_margin - p.ego_half_width;
                const double e = sgn * d_t;
                if (e <= e_skip)
                {
                    continue;
                }
                if (cap <= 1e-3)
                {
                    alpha = 0.0;  // wall margin leaves no track here at all
                    break;
                }
                alpha = std::min(alpha, cap / e);
            }
            if (alpha >= 1.0 - 1e-6)
            {
                // ---- steering limit: cap BOTH the slope (heading) and the curvature (steering
                // angle) the spline demands. Curvature is the one that spins the car in the tight
                // (short-ramp) edge cases, so shrink by whichever bound is violated more. ----
                const double slope_excess = max_slope / (slope_lim * 1.05);
                const double curv_excess = max_pcurv / (kappa_lim * 1.05);
                const double excess = std::max(slope_excess, curv_excess);
                if (excess > 1.0)
                {
                    apex = sgn * (std::max(0.5, 1.0 / excess) * (sgn * apex));
                    if (sgn * (apex - opp.d) < half_sum)
                    {
                        why = "steering/curvature limit shrinks apex below opponent clearance";
                        return false;
                    }
                    continue;
                }
                // Frenet bounds OK — now validate against the LIVE map. The CSV d_left/d_right
                // raycast can miss walls that only exist in the loaded map (obstacle-baked maps),
                // and a path must never cut through real structure. Skip t=0 (the ego's own pose).
                bool hit = false;
                if (occupied_)
                {
                    for (double t = fit_step; t <= t_total + 1e-6; t += fit_step)
                    {
                        double x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i;
                        interp(ego.s + t, x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i);
                        const double d_t = sp.eval(t);
                        if (occupied_(x_r - d_t * std::sin(psi_r), y_r + d_t * std::cos(psi_r)))
                        {
                            hit = true;
                            break;
                        }
                    }
                }
                if (!hit)
                {
                    apex_out = apex;
                    spline_out = sp;
                    return true;
                }
                // Real structure on the line: pull the apex toward the raceline and retry. If the
                // blockage is not apex-driven (e.g. on the ramp from the ego offset), the retries
                // exhaust and the side is rejected — never publish a path through a wall.
                apex = sgn * (0.85 * (sgn * apex));
                if (sgn * (apex - opp.d) < half_sum)
                {
                    why = "map collision on path, no room to shrink";
                    return false;
                }
                continue;
            }
            // Shrink toward the raceline and re-check opponent clearance with the smaller apex.
            apex = sgn * (std::min(alpha, 0.95) * (sgn * apex));
            if (sgn * (apex - opp.d) < half_sum)
            {
                why = "wall margin shrinks apex below opponent clearance";
                return false;
            }
        }
        why = "apex fit did not converge";
        return false;
    };

    bool go_left = false;
    double d_apex = 0.0;
    Pchip spline;
    std::string why_fit;
    const int side = (forced_side != 0) ? forced_side : p.side_mode;
    const std::string room_dbg = "room L=" + fmt(room_left) + " R=" + fmt(room_right) +
                                 " need " + fmt(min_safe_room);
    if (side == 1)
    {
        go_left = true;
        if (!left_ok || !fit_side(true, d_apex, spline, why_fit))
        {
            res.reject = "forced-left infeasible: " + (left_ok ? why_fit : "no room") + " (" +
                         room_dbg + ")";
            return res;
        }
    }
    else if (side == 2)
    {
        go_left = false;
        if (!right_ok || !fit_side(false, d_apex, spline, why_fit))
        {
            res.reject = "forced-right infeasible: " + (right_ok ? why_fit : "no room") + " (" +
                         room_dbg + ")";
            return res;
        }
    }
    else
    {
        if (!left_ok && !right_ok)
        {
            res.reject = "no room either side (" + room_dbg + ")";
            return res;
        }
        go_left = left_ok && (!right_ok || room_left >= room_right);
        const bool first_ok = (go_left ? left_ok : right_ok) &&
                              fit_side(go_left, d_apex, spline, why_fit);
        if (!first_ok)
        {
            const std::string first_why = why_fit.empty() ? "no room" : why_fit;
            go_left = !go_left;
            const bool other_ok = go_left ? left_ok : right_ok;
            if (!other_ok || !fit_side(go_left, d_apex, spline, why_fit))
            {
                res.reject = "no side fits inside wall margin (first side: " + first_why + ") (" +
                             room_dbg + ")";
                return res;
            }
        }
    }

    // ---- sample ONLY the local overtaking segment (ego -> merge + tail) ----
    // The apex fit above already guarantees the spline stays inside the wall margin AND within
    // the steering limit, so the samples are published UNCLAMPED — the line keeps its smooth
    // spline shape (no wall-riding flat segments from a posterior cutoff).
    const double dres = std::max(0.05, p.spline_resolution);
    const double t_total = spline.tEnd();
    std::vector<double> ss, dd, xr, yr, pr, vr, kr, dli, dri;
    for (double t = 0.0; t <= t_total + 1e-6; t += dres)
    {
        const double s_i = wrapS(ego.s + t);
        double x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i;
        interp(s_i, x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i);
        const double d_i = spline.eval(t);
        ss.push_back(s_i);
        dd.push_back(d_i);
        xr.push_back(x_r);
        yr.push_back(y_r);
        pr.push_back(psi_r);
        vr.push_back(vx_r);
        kr.push_back(kappa_r);
        dli.push_back(dl_i);
        dri.push_back(dr_i);
    }
    const std::size_t n = dd.size();
    if (static_cast<int>(n) < p.min_points)
    {
        res.reject = "too few samples (" + std::to_string(n) + ")";
        return res;
    }

    // ---- curvature-limited speed profile with forward/backward accel smoothing ----
    // The lateral spline ADDS curvature (d''(s)) on top of the raceline, so the pass point and the
    // return ramp are tighter than the bare raceline. Two things must hold or the car hits the wall:
    //   (1) at every point v <= sqrt(a_cap / |kappa_total|)  (never corner faster than grip), and
    //   (2) the car must BRAKE before the tight section, not at it — a per-sample cap slows too
    //       late. So the curvature-capped target is passed through a forward+backward longitudinal
    //       acceleration limit (ot_max_long_accel), exactly like an offline speed generator.
    // Lateral-grip capacity a_cap: the raceline speed is feasible by construction, so the grip it
    // already uses (vr^2*|kappa_r|) is proof of LOCAL capacity — a fixed max_lat_accel below that
    // would force the car under the raceline speed even where the spline adds no curvature.
    // a_cap = max(max_lat_accel, vr^2*|kappa_r|): no added curvature -> never slower than the
    // raceline; added curvature -> proportional slowdown v ~ vr*sqrt(kappa_r/kappa_tot).
    // The profile is also anchored to the CURRENT ego speed at the start and eased into the raceline
    // speed at the merge end so the local<->global handoff never steps the setpoint.
    const double v_start = std::max(1.0, ego.v);
    const double blend = std::max(1e-3, p.speed_blend_s);
    const double a_lon = std::max(0.5, p.max_long_accel);

    std::vector<double> kap(n), vtgt(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        double d2 = 0.0;  // d''(s) via central difference (0 at the ends)
        if (i > 0 && i + 1 < n)
        {
            d2 = (dd[i - 1] - 2.0 * dd[i] + dd[i + 1]) / (dres * dres);
        }
        kap[i] = kr[i] + d2;
    }
    // Smooth the numeric curvature (a raw central-difference d'' is noisy -> throttle chatter).
    // The raceline curvature gets the SAME 3-point smoothing so that where the spline adds no
    // curvature ksm == krs exactly and v_phys == vr (zero artificial slowdown at kappa-peak
    // shoulders, where a raw kr[i] under a smoothed kappa_tot would under-report the capacity).
    std::vector<double> ksm(n), krs(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        double acc = kap[i];
        double acc_r = kr[i];
        int cnt = 1;
        if (i > 0) { acc += kap[i - 1]; acc_r += kr[i - 1]; ++cnt; }
        if (i + 1 < n) { acc += kap[i + 1]; acc_r += kr[i + 1]; ++cnt; }
        ksm[i] = acc / cnt;
        krs[i] = acc_r / cnt;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        const double a_cap = std::max(p.max_lat_accel, vr[i] * vr[i] * std::abs(krs[i]));
        const double v_phys = std::sqrt(a_cap / std::max(std::abs(ksm[i]), 1e-3));
        double v = std::min(vr[i] * p.speed_scale, p.v_ceiling);  // overtaking-scaled raceline speed
        const double t_i = static_cast<double>(i) * dres;
        const double w_out = std::min(1.0, std::max(0.0, t_total - t_i) / blend);
        v = vr[i] + w_out * (v - vr[i]);          // ease into the raceline speed at the merge
        v = std::max(std::min(p.v_floor, v_phys), v);  // floor for progress, never above the cap
        vtgt[i] = std::min({v, v_phys, p.v_ceiling});  // hard curvature ceiling
    }
    // Forward pass: bounded acceleration from the current ego speed, plus the PRE-APEX GAP HOLD —
    // while the ego is not yet laterally clear of the opponent it must never close the
    // longitudinal gap below ot_trail_gap (a runway-extended commit from a tight trail would
    // otherwise arrive alongside the still-centered opponent mid-ramp). Time along the profile is
    // integrated as it is built; once the sample's lateral offset clears the opponent the cap
    // lifts and the ego accelerates into the pass.
    const double gap_hold = std::max(0.5, p.trail_gap);
    const double v_opp_hold = std::max(0.0, opp.vs);
    std::vector<double> vprof(n);
    vprof[0] = std::min(vtgt[0], v_start);
    double tau = 0.0;
    for (std::size_t i = 1; i < n; ++i)
    {
        tau += dres / std::max(0.3, vprof[i - 1]);
        const double t_i = static_cast<double>(i) * dres;
        double v_lim = vtgt[i];
        const double gap_i = ds_opp + v_opp_hold * tau - t_i;  // predicted longitudinal gap
        const bool laterally_clear = std::abs(dd[i] - opp.d) >= half_sum + 0.5 * p.lateral_clearance;
        if (!laterally_clear && gap_i > 0.0 && gap_i < gap_hold)
        {
            v_lim = std::min(v_lim, v_opp_hold);
        }
        const double v_acc = std::sqrt(vprof[i - 1] * vprof[i - 1] + 2.0 * a_lon * dres);
        vprof[i] = std::min(v_lim, v_acc);
    }
    // Backward pass: bounded deceleration -> the car brakes BEFORE each tight section.
    for (std::size_t i = n - 1; i-- > 0;)
    {
        const double v_dec = std::sqrt(vprof[i + 1] * vprof[i + 1] + 2.0 * a_lon * dres);
        vprof[i] = std::min(vprof[i], v_dec);
    }

    res.wpnts.wpnts.clear();
    res.wpnts.wpnts.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        const double kappa_tot = ksm[i];
        const double v = vprof[i];
        f110_msgs::msg::Wpnt w;
        w.id = static_cast<int>(i);
        w.s_m = ss[i];
        w.d_m = dd[i];
        w.x_m = xr[i] - dd[i] * std::sin(pr[i]);
        w.y_m = yr[i] + dd[i] * std::cos(pr[i]);
        w.psi_rad = pr[i];
        w.kappa_radpm = kappa_tot;
        w.vx_mps = v;
        w.d_left = dli[i];
        w.d_right = dri[i];
        res.wpnts.wpnts.push_back(w);
    }

    res.ok = true;
    res.go_left = go_left;
    res.merge_s = wrapS(ego.s + t_total);
    res.pass_s = s_pass;
    res.apex_d = d_apex;
    return res;
}

// ------------------------------------------------------------------------------------------------
// Trailing (follow) path: the opponent blocks the corridor but an overtake is not possible.
// Follow the raceline behind it and DECELERATE to its speed at trail_gap — publishing nothing
// would hand control back to the full-speed global raceline and rear-end the opponent.
// ------------------------------------------------------------------------------------------------
OvertakePlanner::PlanResult OvertakePlanner::buildTrailPath(const EgoState &ego,
                                                            const OpponentState &opp,
                                                            const OvertakeParams &p) const
{
    PlanResult res;
    if (!ready() || ego.s < 0.0 || !opp.valid)
    {
        res.reject = "trail: not ready / no opponent";
        return res;
    }
    const double ds_opp = fwdDelta(ego.s, opp.s);
    if (ds_opp > 0.5 * length_)
    {
        res.reject = "trail: opponent behind";
        return res;
    }
    if (ds_opp > p.trigger_max_ds)
    {
        res.reject = "trail: opponent too far ahead";
        return res;
    }
    // Only trail an opponent that actually blocks the corridor (small exit hysteresis so an
    // opponent hovering at the gate edge doesn't flicker trail<->clear every cycle).
    const double half_sum = p.ego_half_width + p.opponent_half_width;
    if (std::abs(opp.d) > half_sum + p.block_margin + (trailing_ ? 0.1 : 0.0))
    {
        res.reject = "trail: opponent not blocking";
        return res;
    }

    // Lateral: ease from the current ego offset back onto the raceline (d = 0) under the same
    // slope/curvature limits as the overtake spline (PCHIP flat-to-flat peak factors).
    const double slope_lim = std::max(0.05, p.max_d_slope);
    const double kappa_lim = std::max(1e-3, p.max_path_kappa);
    const double amp = std::abs(ego.d);
    const double ramp = std::max({1.5 * amp / slope_lim, std::sqrt(6.0 * amp / kappa_lim), 1.0});

    // Path extent: up to the opponent's current s, EXTENDED when the return ramp needs more room —
    // clipping the ramp instead (old behaviour) yanked a far-offset ego (e.g. right after a
    // mid-swing abort) back to the raceline at several times the curvature limit. Geometry past
    // the opponent is safe: the speed profile caps at the opponent speed from t_follow onward, so
    // the ego never drives into the gap. The path is rebuilt every perception cycle anyway.
    const double dres = std::max(0.05, p.spline_resolution);
    const double min_len = std::max(2.0, static_cast<double>(p.min_points + 1) * dres);
    const double t_total = std::max({ds_opp, min_len, ramp / 0.7});
    Pchip spline;
    if (!spline.build({0.0, ramp, t_total}, {ego.d, 0.0, 0.0}))
    {
        res.reject = "trail: spline build failed";
        return res;
    }

    std::vector<double> ss, dd, xr, yr, pr, vr, kr, dli, dri;
    for (double t = 0.0; t <= t_total + 1e-6; t += dres)
    {
        const double s_i = wrapS(ego.s + t);
        double x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i;
        interp(s_i, x_r, y_r, psi_r, vx_r, kappa_r, dl_i, dr_i);
        ss.push_back(s_i);
        dd.push_back(spline.eval(t));
        xr.push_back(x_r);
        yr.push_back(y_r);
        pr.push_back(psi_r);
        vr.push_back(vx_r);
        kr.push_back(kappa_r);
        dli.push_back(dl_i);
        dri.push_back(dr_i);
    }
    const std::size_t n = dd.size();
    if (static_cast<int>(n) < p.min_points)
    {
        res.reject = "trail: too few samples";
        return res;
    }

    // Speed: same curvature-capped + fwd/bwd accel-limited pipeline as the overtake profile, with
    // one extra target — at and beyond the follow point (trail_gap behind the opponent) the speed
    // is the OPPONENT speed. The backward pass then brakes the ego smoothly down to it BEFORE the
    // follow point; if the ego is already inside the gap it holds/undershoots the opponent speed
    // until the gap reopens. No v_floor here: following a slow car must be allowed to be slow.
    const double v_start = std::max(0.5, ego.v);
    const double a_lon = std::max(0.5, p.max_long_accel);
    const double v_opp = std::max(0.0, opp.vs);
    const double t_follow = std::max(0.0, ds_opp - std::max(0.5, p.trail_gap));

    std::vector<double> kap(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        double d2 = 0.0;
        if (i > 0 && i + 1 < n)
        {
            d2 = (dd[i - 1] - 2.0 * dd[i] + dd[i + 1]) / (dres * dres);
        }
        kap[i] = kr[i] + d2;
    }
    std::vector<double> ksm(n), krs(n), vtgt(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        double acc = kap[i];
        double acc_r = kr[i];
        int cnt = 1;
        if (i > 0) { acc += kap[i - 1]; acc_r += kr[i - 1]; ++cnt; }
        if (i + 1 < n) { acc += kap[i + 1]; acc_r += kr[i + 1]; ++cnt; }
        ksm[i] = acc / cnt;
        krs[i] = acc_r / cnt;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        const double a_cap = std::max(p.max_lat_accel, vr[i] * vr[i] * std::abs(krs[i]));
        const double v_phys = std::sqrt(a_cap / std::max(std::abs(ksm[i]), 1e-3));
        double v = std::min({vr[i], v_phys, p.v_ceiling});
        if (static_cast<double>(i) * dres >= t_follow)
        {
            v = std::min(v, v_opp);
        }
        vtgt[i] = v;
    }
    std::vector<double> vprof(n);
    vprof[0] = std::min(vtgt[0], v_start);
    for (std::size_t i = 1; i < n; ++i)
    {
        const double v_acc = std::sqrt(vprof[i - 1] * vprof[i - 1] + 2.0 * a_lon * dres);
        vprof[i] = std::min(vtgt[i], v_acc);
    }
    for (std::size_t i = n - 1; i-- > 0;)
    {
        const double v_dec = std::sqrt(vprof[i + 1] * vprof[i + 1] + 2.0 * a_lon * dres);
        vprof[i] = std::min(vprof[i], v_dec);
    }

    res.wpnts.wpnts.clear();
    res.wpnts.wpnts.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        f110_msgs::msg::Wpnt w;
        w.id = static_cast<int>(i);
        w.s_m = ss[i];
        w.d_m = dd[i];
        w.x_m = xr[i] - dd[i] * std::sin(pr[i]);
        w.y_m = yr[i] + dd[i] * std::cos(pr[i]);
        w.psi_rad = pr[i];
        w.kappa_radpm = ksm[i];
        w.vx_mps = vprof[i];
        w.d_left = dli[i];
        w.d_right = dri[i];
        res.wpnts.wpnts.push_back(w);
    }

    res.ok = true;
    res.merge_s = wrapS(ego.s + t_total);
    res.pass_s = wrapS(opp.s);
    res.apex_d = 0.0;
    return res;
}

// Trailing entry/exit shared by the Idle-reject and Cooldown branches: publish a follow path when
// the opponent blocks but can't be passed, emit ONE empty OT when an active trail ends, otherwise
// leave the decision untouched (silent).
OvertakeDecision OvertakePlanner::trailDecision(const EgoState &ego, const OpponentState &opp,
                                                const OvertakeParams &p, OvertakeDecision dec)
{
    if (p.trail_enabled && opp.valid)
    {
        const PlanResult tr = buildTrailPath(ego, opp, p);
        if (tr.ok)
        {
            if (!trailing_)
            {
                trailing_ = true;
                dec.reason = "trail: overtake not possible, following at opponent speed";
            }
            dec.action = OvertakeDecision::Action::Publish;
            dec.wpnts = tr.wpnts;
            dec.side = "trail";
            return dec;
        }
    }
    if (trailing_)
    {
        trailing_ = false;
        dec.action = OvertakeDecision::Action::Clear;
        dec.reason = "trail end";
    }
    return dec;
}

// ------------------------------------------------------------------------------------------------
// Passing-gap check: has the opponent moved into the gap we committed to pass through?
// ------------------------------------------------------------------------------------------------
bool OvertakePlanner::passingGapLost(const OpponentState &opp, const OvertakeParams &p) const
{
    if (!opp.valid)
    {
        return false;
    }
    // The committed line passes the opponent at apex_d_ on the committed side. The pass is lost
    // when the opponent's CURRENT d leaves less than (half-widths + intrusion margin) of lateral
    // gap to the apex line — i.e. it drifted toward/behind our planned side.
    const double gap = committed_left_ ? (apex_d_ - opp.d) : (opp.d - apex_d_);
    const double needed = p.ego_half_width + p.opponent_half_width +
                          std::max(0.0, p.intrusion_margin - p.lateral_clearance);
    return gap < needed;
}

// ------------------------------------------------------------------------------------------------
// Ego-deviation check: how far is the CAR from the line it committed to?
// ------------------------------------------------------------------------------------------------
double OvertakePlanner::egoPathDeviation(const EgoState &ego) const
{
    // The committed path is only ever refreshed when the OPPONENT drifts; the ego can leave it
    // too (tracking failure, a bump, wall contact). Compare ego.d against the committed waypoint
    // nearest in s; an ego outside the path's s-range altogether is a full deviation.
    if (committed_.wpnts.empty())
    {
        return 0.0;
    }
    double best_ds = 1e9;
    double d_ref = ego.d;
    for (const auto &w : committed_.wpnts)
    {
        const double ds = std::min(fwdDelta(w.s_m, ego.s), fwdDelta(ego.s, w.s_m));
        if (ds < best_ds)
        {
            best_ds = ds;
            d_ref = w.d_m;
        }
    }
    if (best_ds > 2.0)
    {
        return 1e9;  // ego is nowhere on the maneuver (e.g. thrown backwards by a crash)
    }
    return std::abs(ego.d - d_ref);
}

// ------------------------------------------------------------------------------------------------
// State machine
// ------------------------------------------------------------------------------------------------
OvertakeDecision OvertakePlanner::update(const EgoState &ego, const OpponentState &opp,
                                         const OvertakeParams &p, double stamp)
{
    OvertakeDecision dec;

    // Global disable / not ready: if a path was active, clear it once.
    if (!p.enabled || !ready() || ego.s < 0.0)
    {
        if (state_ == State::Committed)
        {
            state_ = State::Cooldown;
            cooldown_until_ = stamp + p.cooldown;
            committed_.wpnts.clear();
            dec.action = OvertakeDecision::Action::Clear;
            dec.reason = "planner disabled / not ready";
        }
        else if (trailing_)
        {
            dec.action = OvertakeDecision::Action::Clear;
            dec.reason = "trail end (planner disabled / not ready)";
        }
        trailing_ = false;
        return dec;
    }

    switch (state_)
    {
        case State::Cooldown:
            if (stamp < cooldown_until_)
            {
                // No overtake retry yet, but a blocking opponent must still be followed —
                // an abort (e.g. passing gap lost) leaves it slow and directly ahead.
                return trailDecision(ego, opp, p, dec);
            }
            state_ = State::Idle;
            [[fallthrough]];

        case State::Idle:
        {
            const PlanResult plan = buildPath(ego, opp, p, 0);
            if (!plan.ok)
            {
                dec.debug = plan.reject;  // surface why the commit gates rejected this cycle
                // Blocked but can't pass -> decelerate and follow instead of going silent.
                return trailDecision(ego, opp, p, dec);
            }
            trailing_ = false;  // commit supersedes an active trail (publishing stays continuous)
            committed_ = plan.wpnts;
            committed_left_ = plan.go_left;
            merge_s_ = plan.merge_s;
            apex_d_ = plan.apex_d;
            commit_stamp_ = stamp;
            plan_opp_s_ = opp.s;
            plan_opp_d_ = opp.d;
            passed_opp_ = false;
            state_ = State::Committed;
            dec.action = OvertakeDecision::Action::Publish;
            dec.wpnts = committed_;
            dec.side = plan.go_left ? "left" : "right";
            dec.reason = "commit";
            return dec;
        }

        case State::Committed:
        {
            auto abort = [&](const std::string &why) {
                state_ = State::Cooldown;
                cooldown_until_ = stamp + p.cooldown;
                committed_.wpnts.clear();
                dec.action = OvertakeDecision::Action::Clear;
                dec.reason = why;
                return dec;
            };

            // ---- pass evidence: the opponent must be OBSERVED behind the ego (sticky) ----
            if (opp.valid)
            {
                const double opp_behind = fwdDelta(opp.s, ego.s);
                if (opp_behind >= p.completion_margin && opp_behind < 0.5 * length_)
                {
                    passed_opp_ = true;
                }
            }

            // ---- completion: pass observed AND the merge point is behind us ----
            // (never assumed from a lost track — a crash that makes the opponent disappear must
            // NOT read as "overtake complete")
            const double to_merge = fwdDelta(ego.s, merge_s_);
            const bool past_merge = to_merge > 0.5 * length_;  // merge point now behind us
            if (past_merge && passed_opp_)
            {
                return abort("overtake complete");
            }

            // ---- hard timeout: never hold a stale maneuver ----
            if (stamp - commit_stamp_ > p.max_duration)
            {
                return abort("max duration exceeded");
            }

            // ---- opponent lost: the committed line is no longer justified ----
            if (!opp.valid)
            {
                return abort(passed_opp_ ? "overtake complete (opponent left behind, track lost)"
                                         : "abort: opponent lost before the pass");
            }

            const double ds_opp = fwdDelta(ego.s, opp.s);
            const bool opp_ahead = ds_opp < 0.5 * length_;
            // Alongside = too late to safely drop back to the global raceline.
            const bool alongside = opp_ahead && ds_opp <= std::max(1.0, p.pass_clearance_s);

            // ---- ego off the committed path: the path must relate to where the car IS ----
            // Everything else only refreshes the plan when the OPPONENT drifts; after a bump,
            // wall contact or plain tracking failure the ego leaves the line and the controller
            // chases a path anchored somewhere else. Replan from the CURRENT pose; a deviation
            // so large the maneuver is meaningless (2x) aborts even alongside — the
            // "never abort alongside" rule assumes the ego is ON the swing line.
            const double dev_lim = std::max(0.05, p.ego_dev_replan);
            const double ego_dev = egoPathDeviation(ego);
            if (ego_dev > 2.0 * dev_lim)
            {
                return abort("ego far off committed path");
            }
            if (ego_dev > dev_lim)
            {
                const PlanResult re = buildPath(ego, opp, p, committed_left_ ? 1 : 2);
                if (re.ok)
                {
                    committed_ = re.wpnts;
                    merge_s_ = re.merge_s;
                    apex_d_ = re.apex_d;
                    plan_opp_s_ = opp.s;
                    plan_opp_d_ = opp.d;
                    dec.action = OvertakeDecision::Action::Publish;
                    dec.wpnts = committed_;
                    dec.side = committed_left_ ? "left" : "right";
                    dec.reason = "replan (ego off path)";
                    return dec;
                }
                if (!alongside)
                {
                    return abort("ego off committed path, replan infeasible: " + re.reject);
                }
                // Alongside with a moderate deviation and no feasible replan: keep the committed
                // line (dropping to global mid-pass steers into the opponent); the 2x bound above
                // still catches a real crash.
            }

            // ---- relative-speed collapse (hysteresis): can no longer finish the pass ----
            if (opp_ahead)
            {
                const double v_rel = attainableSpeed(ego) - opp.vs;
                if (v_rel < p.abort_rel_vel)
                {
                    return abort("relative speed collapsed");
                }
            }

            // ---- opponent moved into the passing gap -> switch side or abort ----
            if (opp_ahead && passingGapLost(opp, p))
            {
                const int other = committed_left_ ? 2 : 1;
                const PlanResult sw = buildPath(ego, opp, p, other);
                if (sw.ok)
                {
                    committed_ = sw.wpnts;
                    committed_left_ = sw.go_left;
                    merge_s_ = sw.merge_s;
                    apex_d_ = sw.apex_d;
                    plan_opp_s_ = opp.s;
                    plan_opp_d_ = opp.d;
                    dec.action = OvertakeDecision::Action::Publish;
                    dec.wpnts = committed_;
                    dec.side = sw.go_left ? "left" : "right";
                    dec.reason = "side switch (passing gap lost)";
                    return dec;
                }
                return abort("passing gap lost, no alternative side");
            }

            if (opp_ahead && !alongside)
            {
                // ---- approach phase: keep judging whether overtaking is still the right call ----
                // Opponent no longer blocks the global corridor -> the maneuver is unnecessary;
                // follow the global raceline instead (small hysteresis vs the commit gate).
                const double half_sum = p.ego_half_width + p.opponent_half_width;
                if (std::abs(opp.d) > half_sum + p.block_margin + 0.1)
                {
                    return abort("opponent cleared the corridor");
                }

                // Re-run ALL commit gates from the current state every cycle. If the environment
                // ahead became unsuitable (corner, no room inside the wall margin, uncatchable),
                // do NOT force the maneuver — drop back to the global raceline while it is still
                // safe to do so.
                const PlanResult chk = buildPath(ego, opp, p, committed_left_ ? 1 : 2);
                if (!chk.ok)
                {
                    const PlanResult sw = buildPath(ego, opp, p, committed_left_ ? 2 : 1);
                    if (sw.ok)
                    {
                        committed_ = sw.wpnts;
                        committed_left_ = sw.go_left;
                        merge_s_ = sw.merge_s;
                        apex_d_ = sw.apex_d;
                        plan_opp_s_ = opp.s;
                        plan_opp_d_ = opp.d;
                        dec.action = OvertakeDecision::Action::Publish;
                        dec.wpnts = committed_;
                        dec.side = sw.go_left ? "left" : "right";
                        dec.reason = "side switch (committed side no longer feasible)";
                        return dec;
                    }
                    return abort("no longer feasible: " + chk.reject);
                }
                // Still feasible: refresh the path only when the opponent drifted, so the
                // published line stays stable frame-to-frame.
                const double drift_s =
                    std::min(fwdDelta(plan_opp_s_, opp.s), fwdDelta(opp.s, plan_opp_s_));
                const double drift_d = std::abs(opp.d - plan_opp_d_);
                if (drift_s > p.replan_ds_threshold || drift_d > p.replan_dd_threshold)
                {
                    committed_ = chk.wpnts;
                    merge_s_ = chk.merge_s;
                    apex_d_ = chk.apex_d;
                    plan_opp_s_ = opp.s;
                    plan_opp_d_ = opp.d;
                    dec.action = OvertakeDecision::Action::Publish;
                    dec.wpnts = committed_;
                    dec.side = committed_left_ ? "left" : "right";
                    dec.reason = "replan (opponent moved)";
                    return dec;
                }
            }
            else if (opp_ahead)
            {
                // ---- alongside: aborting would steer into the opponent — keep the path. ----
                // Only refresh on drift when a same-side replan is still feasible.
                const double drift_s =
                    std::min(fwdDelta(plan_opp_s_, opp.s), fwdDelta(opp.s, plan_opp_s_));
                const double drift_d = std::abs(opp.d - plan_opp_d_);
                if (drift_s > p.replan_ds_threshold || drift_d > p.replan_dd_threshold)
                {
                    const PlanResult re = buildPath(ego, opp, p, committed_left_ ? 1 : 2);
                    if (re.ok)
                    {
                        committed_ = re.wpnts;
                        merge_s_ = re.merge_s;
                        apex_d_ = re.apex_d;
                        plan_opp_s_ = opp.s;
                        plan_opp_d_ = opp.d;
                        dec.action = OvertakeDecision::Action::Publish;
                        dec.wpnts = committed_;
                        dec.side = committed_left_ ? "left" : "right";
                        dec.reason = "replan (opponent moved)";
                        return dec;
                    }
                    // Replan infeasible mid-pass: the passing gap is still open (checked above),
                    // so KEEP the committed path — dropping to global would steer into the opponent.
                }
            }

            // ---- still valid: keep publishing the committed path unchanged ----
            dec.action = OvertakeDecision::Action::Publish;
            dec.wpnts = committed_;
            dec.side = committed_left_ ? "left" : "right";
            return dec;
        }
    }
    return dec;
}

}  // namespace opponent_detector
