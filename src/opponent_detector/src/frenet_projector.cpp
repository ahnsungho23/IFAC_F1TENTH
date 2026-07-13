// ================================================================================================
// FRENET PROJECTOR implementation
// ================================================================================================

#include "opponent_detector/frenet_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace opponent_detector
{

void FrenetProjector::build(std::vector<Waypoint> waypoints, bool closed)
{
    wpnts_ = std::move(waypoints);
    closed_ = closed;
    ready_ = false;
    length_ = 0.0;

    if (wpnts_.size() < 2)
    {
        return;
    }

    // raceline length: last waypoint s plus the closing segment length (for closed loops).
    const Waypoint &front = wpnts_.front();
    const Waypoint &back = wpnts_.back();
    const double closing = std::hypot(front.x - back.x, front.y - back.y);
    length_ = back.s + (closed_ ? closing : 0.0);
    if (length_ <= 0.0)
    {
        length_ = back.s > 0.0 ? back.s : 1.0;
    }
    ready_ = true;
}

double FrenetProjector::wrapDelta(double a, double b) const
{
    double diff = a - b;
    if (!closed_ || length_ <= 0.0)
    {
        return diff;
    }
    while (diff > 0.5 * length_)
    {
        diff -= length_;
    }
    while (diff < -0.5 * length_)
    {
        diff += length_;
    }
    return diff;
}

std::size_t FrenetProjector::nearestIndex(double x, double y) const
{
    std::size_t best = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < wpnts_.size(); ++i)
    {
        const double dx = wpnts_[i].x - x;
        const double dy = wpnts_[i].y - y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2)
        {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

FrenetPoint FrenetProjector::toFrenet(double x, double y) const
{
    FrenetPoint out;
    if (!ready_)
    {
        return out;
    }

    const std::size_t n = wpnts_.size();
    const std::size_t i = nearestIndex(x, y);

    // Evaluate the two segments adjacent to the nearest waypoint and keep the closest projection.
    double best_perp = std::numeric_limits<double>::max();
    bool found = false;

    auto eval_segment = [&](std::size_t ia, std::size_t ib) {
        const Waypoint &a = wpnts_[ia];
        const Waypoint &b = wpnts_[ib];
        const double ax = b.x - a.x;
        const double ay = b.y - a.y;
        const double seg_len2 = ax * ax + ay * ay;
        if (seg_len2 < 1e-9)
        {
            return;
        }
        double t = ((x - a.x) * ax + (y - a.y) * ay) / seg_len2;
        t = std::clamp(t, 0.0, 1.0);
        const double projx = a.x + t * ax;
        const double projy = a.y + t * ay;
        const double perp = std::hypot(x - projx, y - projy);
        if (perp < best_perp)
        {
            best_perp = perp;
            const double seg_len = std::sqrt(seg_len2);
            // signed lateral (left of the segment tangent is positive: cross(tangent, point-a))
            const double cross = (ax * (y - a.y) - ay * (x - a.x)) / seg_len;
            const double ds = wrapDelta(b.s, a.s);
            out.s = a.s + t * ds;
            out.d = cross;
            found = true;
        }
    };

    // previous segment (i-1 -> i) and next segment (i -> i+1), with closed-loop wrap.
    if (i > 0)
    {
        eval_segment(i - 1, i);
    }
    else if (closed_)
    {
        eval_segment(n - 1, 0);
    }
    if (i + 1 < n)
    {
        eval_segment(i, i + 1);
    }
    else if (closed_)
    {
        eval_segment(n - 1, 0);
    }

    if (!found)
    {
        out.s = wpnts_[i].s;
        out.d = 0.0;
    }

    if (closed_ && length_ > 0.0)
    {
        out.s = std::fmod(out.s, length_);
        if (out.s < 0.0)
        {
            out.s += length_;
        }
    }
    return out;
}

void FrenetProjector::boundsAtS(double s, double &d_left, double &d_right, double &vx) const
{
    d_left = 0.0;
    d_right = 0.0;
    vx = 0.0;
    if (!ready_ || wpnts_.empty())
    {
        return;
    }

    // nearest waypoint by s (linear scan; s is monotonic but a scan keeps this robust to gaps).
    std::size_t best = 0;
    double best_ds = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < wpnts_.size(); ++i)
    {
        const double ds = std::abs(wrapDelta(wpnts_[i].s, s));
        if (ds < best_ds)
        {
            best_ds = ds;
            best = i;
        }
    }
    d_left = wpnts_[best].d_left;
    d_right = wpnts_[best].d_right;
    vx = wpnts_[best].vx;
}

}  // namespace opponent_detector
