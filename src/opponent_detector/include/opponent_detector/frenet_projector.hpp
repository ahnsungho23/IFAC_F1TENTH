// ================================================================================================
// FRENET PROJECTOR - lightweight piecewise-linear Cartesian<->Frenet projection
// ================================================================================================
// The shared frenet_converter package is Python-only (no C++ linkage), and CLAUDE.md requires
// ROS 2 runtime code in C++. The global raceline (/global_waypoints, f110_msgs/WpntArray) is dense
// (~0.1 m spacing) and already carries s_m, x_m, y_m, psi_rad, d_left, d_right, so a segment-wise
// linear projection is accurate enough and avoids a cross-language topic hop.
// ================================================================================================

#ifndef OPPONENT_DETECTOR__FRENET_PROJECTOR_HPP_
#define OPPONENT_DETECTOR__FRENET_PROJECTOR_HPP_

#include <cstddef>
#include <vector>

namespace opponent_detector
{

struct FrenetPoint
{
    double s{0.0};  // arc length along the raceline (m), wraps on [0, raceline_length)
    double d{0.0};  // signed lateral offset (m), left of travel direction positive
};

// Piecewise-linear Frenet projector built from the dense global raceline waypoints (map frame).
class FrenetProjector
{
  public:
    struct Waypoint
    {
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double psi{0.0};
        double d_left{0.0};
        double d_right{0.0};
        double vx{0.0};
    };

    FrenetProjector() = default;

    // Build the projector from waypoints (ordered by increasing s). Needs >= 2 points.
    void build(std::vector<Waypoint> waypoints, bool closed = true);

    bool ready() const { return ready_; }
    double raceline_length() const { return length_; }
    std::size_t size() const { return wpnts_.size(); }

    // Project a Cartesian point (map frame) -> Frenet (s, d).
    FrenetPoint toFrenet(double x, double y) const;

    // Nearest-waypoint track bounds and reference speed at a given s.
    void boundsAtS(double s, double &d_left, double &d_right, double &vx) const;

    // Smallest signed difference (a - b) accounting for the closed-loop wrap.
    double wrapDelta(double a, double b) const;

  private:
    std::size_t nearestIndex(double x, double y) const;

    std::vector<Waypoint> wpnts_;
    double length_{0.0};
    bool closed_{true};
    bool ready_{false};
};

}  // namespace opponent_detector

#endif  // OPPONENT_DETECTOR__FRENET_PROJECTOR_HPP_
