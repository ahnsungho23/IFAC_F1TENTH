// ================================================================================================
// FRENET PROJECTOR implementation
// ================================================================================================

#include "obstacle_detector/frenet_projector.hpp"

#include <cmath>
#include <limits>

namespace obstacle_detector
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

void FrenetProjector::boundsAtS(double s, double &d_left, double &d_right) const
{
    d_left = 0.0;
    d_right = 0.0;
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
}

}  // namespace obstacle_detector
