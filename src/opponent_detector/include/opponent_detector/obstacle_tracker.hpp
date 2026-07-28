// ================================================================================================
// OBSTACLE TRACKER - constant-velocity Kalman tracking + static/dynamic classification in Frenet
// ================================================================================================
// State per track: x = [s, vs, d, vd] (constant-velocity model). Measurement: z = [s, d].
// Classification realises the user's plan: identify the DYNAMIC opponent by relative velocity in
// the raceline (Frenet) frame - where ego motion is already removed - with hysteresis and a
// low-speed backstop for robustness. A positional-standard-deviation classifier (ForzaETH style)
// is also available via classifier_mode.
// ================================================================================================

#ifndef OPPONENT_DETECTOR__OBSTACLE_TRACKER_HPP_
#define OPPONENT_DETECTOR__OBSTACLE_TRACKER_HPP_

#include <deque>
#include <vector>

#include <Eigen/Dense>

#include "opponent_detector/frenet_projector.hpp"

namespace opponent_detector
{

// One measurement fed to the tracker (already filtered / Frenet-projected).
struct Detection
{
    double s{0.0};
    double d{0.0};
    double size{0.0};
    double x{0.0};  // cartesian centroid (map frame), kept for visualization
    double y{0.0};
};

enum class ClassifierMode
{
    Velocity,
    Std,
    Both
};

struct TrackerParams
{
    // Kalman noise
    double meas_var_s{0.002};
    double meas_var_d{0.002};
    double process_var_vs{2.0};
    double process_var_vd{8.0};
    // data association
    double assoc_gate{0.5};       // [m] Frenet gate for static / unknown tracks
    double aggro_multi{2.0};      // gate multiplier once a track is dynamic
    // track lifetime
    int ttl_dynamic{40};
    int ttl_static{3};
    int min_hits_confirm{3};      // measurements before a track is reported
    // classification
    ClassifierMode classifier_mode{ClassifierMode::Velocity};
    double dyn_vel_enter{0.5};    // [m/s] speed (relative to the static field) to become dynamic
    double dyn_vel_exit{0.25};    // [m/s] speed to fall back to static (hysteresis)
    int dyn_min_frames{3};        // sustained frames before flipping the label
    double vs_reset{0.1};         // [m/s] longitudinal-speed backstop -> force static
    // static-field reference: walls and stationary obstacles all share one apparent velocity
    // (~0 in the map frame, plus any ego-localization drift). We classify each track by its speed
    // RELATIVE to that reference, so "same velocity as the walls" -> static. The reference is the
    // mean velocity of tracks that are clearly slow (below static_ref_gate), which excludes the
    // moving opponent from contaminating it.
    double static_ref_gate{0.3};  // [m/s] tracks slower than this define the static-field velocity
    int std_window{30};           // history window for the std classifier
    int min_nb_meas{5};           // measurements before the std classifier votes
    double min_std{0.16};         // [m] below on both axes -> static vote
    double max_std{0.20};         // [m] above on either axis -> dynamic
    double dt_max{0.5};           // [s] clamp for prediction step
};

struct Track
{
    int id{0};
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};   // [s, vs, d, vd]
    Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
    int hits{0};
    int misses{0};
    int ttl{0};
    bool is_static{true};
    bool is_visible{false};
    int dyn_streak{0};
    int static_streak{0};
    double size{0.0};
    double x_map{0.0};
    double y_map{0.0};
    std::deque<std::pair<double, double>> hist;  // (s, d) for the std classifier

    double s() const { return x(0); }
    double vs() const { return x(1); }
    double d() const { return x(2); }
    double vd() const { return x(3); }
    double speed() const { return std::hypot(x(1), x(3)); }
};

class ObstacleTracker
{
  public:
    ObstacleTracker() = default;

    void configure(const TrackerParams &params, const FrenetProjector *frenet);

    // Predict all tracks to `stamp`, associate detections, update, spawn/retire, classify.
    void update(const std::vector<Detection> &detections, double stamp);

    const std::vector<Track> &tracks() const { return tracks_; }

    // Index of the "opponent": the confirmed dynamic track closest ahead of ego_s.
    // Returns -1 if none. ego_s < 0 disables the ahead-preference (nearest is used).
    int opponentIndex(double ego_s = -1.0) const;

    // Current static-field reference velocity (Frenet), i.e. the walls'/stationary obstacles'
    // shared apparent velocity that dynamic tracks are measured against.
    double staticRefVs() const { return static_ref_vs_; }
    double staticRefVd() const { return static_ref_vd_; }

  private:
    void predict(Track &t, double dt) const;
    void kalmanUpdate(Track &t, double meas_s, double meas_d) const;
    void updateStaticReference();
    void classify(Track &t) const;
    double frenetDist(double s1, double d1, double s2, double d2) const;

    TrackerParams p_;
    const FrenetProjector *frenet_{nullptr};
    std::vector<Track> tracks_;
    int next_id_{0};
    double last_stamp_{-1.0};
    bool has_last_stamp_{false};
    double static_ref_vs_{0.0};   // static-field reference velocity (Frenet s)
    double static_ref_vd_{0.0};   // static-field reference velocity (Frenet d)
};

}  // namespace opponent_detector

#endif  // OPPONENT_DETECTOR__OBSTACLE_TRACKER_HPP_
