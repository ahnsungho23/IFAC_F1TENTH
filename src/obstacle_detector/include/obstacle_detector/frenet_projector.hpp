// ================================================================================================
// FRENET TRACK HELPER - lightweight boundary lookup and closed-loop s arithmetic
// ================================================================================================
// Accurate Cartesian-to-Frenet projection is provided by global_planning::ClcsFrenetConverter.
// This helper stores only what the detector still needs locally: track length, wrap-aware
// s differences, and nearest-waypoint d_left/d_right boundaries.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_
#define OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_

#include <vector>

namespace obstacle_detector
{

// Track helper built from the ordered global raceline waypoints.
class FrenetProjector
{
  public:
    struct Waypoint
    {
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double d_left{0.0};
        double d_right{0.0};
    };

    FrenetProjector() = default;

    // Build the projector from waypoints (ordered by increasing s). Needs >= 2 points.
    void build(std::vector<Waypoint> waypoints, bool closed = true);

    bool ready() const { return ready_; }
    double raceline_length() const { return length_; }

    // Nearest-waypoint track bounds at a given s.
    void boundsAtS(double s, double &d_left, double &d_right) const;

    // Smallest signed difference (a - b) accounting for the closed-loop wrap.
    double wrapDelta(double a, double b) const;

  private:
    std::vector<Waypoint> wpnts_;
    double length_{0.0};
    bool closed_{true};
    bool ready_{false};
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__FRENET_PROJECTOR_HPP_
