// ================================================================================================
// OBSTACLE TRACKER - constant-velocity Kalman tracking + static/dynamic classification in Frenet
// ================================================================================================
// State per track: x = [s, vs, d, vd] (constant-velocity model). Measurement: z = [s, d].
//
// LAYER 2 / LAYER 3 SPLIT (ego-relative "map-flow" view).
// The classifier separates the two obstacle layers by how each track FLOWS relative to the map:
//   - The walls and stationary obstacles all stream past the moving ego at the same apparent
//     velocity — the "map-flow" reference. In the ego frame this is -v_ego; here it is measured
//     empirically as static_ref_* (the mean Frenet velocity of the clearly-slow tracks), which is
//     equivalent and self-calibrating (it cancels common ego-localization drift and needs no
//     accurate v_ego). In the world-anchored Frenet (s) frame ego motion is already removed, so
//     the map-flow reference sits at ~0.
//   - LAYER 2 (static): a track whose flow matches the map-flow reference (|flow - ref| small).
//   - LAYER 3 (dynamic / opponent): a track that clearly deviates from the map-flow reference. A
//     same-direction opponent flows "slower" than the walls in the ego frame; equivalently it has
//     a non-zero along-track velocity vs in the world frame. Both describe the same test.
// Hysteresis + a low-speed backstop guard the label; a positional-standard-deviation classifier
// (ForzaETH style) is also available via classifier_mode.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
#define OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <deque>
#include <vector>

#include <Eigen/Dense>

#include "obstacle_detector/frenet_projector.hpp"

namespace obstacle_detector
{

// One measurement fed to the tracker (already filtered / Frenet-projected).
struct Detection
{
    double s{0.0};
    double d{0.0};
    // Current Cartesian AABB projected into the local Frenet frame. The longitudinal extent is
    // symmetric around `s`; lateral offsets may be asymmetric after the exact curved-raceline
    // closest-face correction.
    double s_half_extent{0.0};
    double d_right_offset{0.0};
    double d_left_offset{0.0};
    double size{0.0};
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    // Mean LiDAR range of the contributing cluster. Drives the range-scaled confirmation streak.
    double range{0.0};
    // Measurement-covariance multiplier derived from range, cluster density, and fresh ego yaw
    // rate. The base variances remain meas_var_s/meas_var_d in YAML.
    double variance_scale{1.0};
};

enum class ClassifierMode
{
    Velocity,
    Std,
    Both
};

enum class MotionClass
{
    Pending,
    ProvisionalStatic,
    ConfirmedStatic,
    Dynamic
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
    bool assoc_use_mahalanobis{true};
    double assoc_mahalanobis_gate{9.21};  // chi-square gate, 2 DoF (99%)
    // track lifetime
    int ttl_dynamic{40};
    int ttl_static{25};
    // Detection confirmation: consecutive matched frames before a track is reported, scaled by
    // measurement range from confirm_frames_near (range 0) to confirm_frames_far (at
    // confirm_range_max). One missed frame resets the streak, so intermittent flicker never
    // confirms.
    int confirm_frames_near{3};
    int confirm_frames_far{8};
    double confirm_range_max{14.0};   // [m] range where confirm_frames_far fully applies
    // classification
    ClassifierMode classifier_mode{ClassifierMode::Velocity};
    double dyn_vel_enter{0.5};    // [m/s] flow deviation from the map-flow ref to become dynamic
    double dyn_vel_exit{0.25};    // [m/s] flow deviation to fall back to static (hysteresis)
    int static_confirm_frames{3};   // extra low-speed observations after provisional publication
    int dynamic_confirm_frames{25}; // consecutive reliable moving observations before promotion
    double dyn_velocity_mahalanobis_gate{9.21};  // 2-DoF velocity-confidence threshold
    double dyn_max_abs_yaw_rate{1.5};  // [rad/s] freeze dynamic evidence above this ego yaw rate
    // map-flow reference velocity: walls and stationary obstacles all stream past the ego at one
    // shared apparent velocity (the "map flow"; -v_ego in the ego frame, ~0 in world Frenet plus
    // any ego-localization drift). Each track is classified by its flow RELATIVE to this reference,
    // so "flows with the map" -> static (Layer 2) and "clearly deviates" -> dynamic (Layer 3). The
    // reference is the mean velocity of the clearly-slow tracks (below static_ref_gate), which is
    // self-calibrating and structurally excludes the fast opponent from contaminating it.
    double static_ref_gate{0.3};  // [m/s] tracks slower than this define the map-flow reference
    int std_window{30};           // history window for the std classifier
    int min_nb_meas{5};           // measurements before the std classifier votes
    double min_std{0.16};         // [m] below on both axes -> static vote
    double max_std{0.20};         // [m] above on either axis -> dynamic
    double dt_max{0.5};           // [s] clamp for prediction step
    // Per-scan AABB extents flap (square box vs real shape). Smooth their magnitude
    // fast-grow/slow-shrink: expand immediately, relax over ~1/alpha matched frames.
    // 1.0 restores the legacy overwrite behaviour.
    double extent_shrink_alpha{0.25};
    // Static-layer publish stability: count consecutive matched frames whose measured centre
    // and envelope stay within the tolerance of the track. Fan-shaped morphing clusters never
    // settle, so requiring a short streak keeps them unpublished while real obstacles (stable
    // from the first scan) see no added delay beyond the confirmation streak. An unmatched
    // (prediction-only) frame resets the streak to zero.
    double envelope_stability_tolerance_m{0.10};
    int envelope_stability_frames{2};
};

struct Track
{
    int id{0};
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};   // [s, vs, d, vd]
    Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
    int hits{0};
    int consecutive_hits{0};  // matched frames in a row; reset to 0 on a prediction-only frame
    int ttl{0};
    bool is_static{true};
    bool is_visible{false};
    // `classified` means the track has passed the consecutive-frame confirmation and may be
    // published. It first enters ProvisionalStatic so a physical obstacle is available to local
    // planning immediately; motion_class records the separate static/dynamic confidence state.
    bool classified{false};
    MotionClass motion_class{MotionClass::Pending};
    int dyn_streak{0};
    int static_streak{0};
    int envelope_stable_streak{0};  // consecutive matched frames with a settled centre+envelope
    double last_range{0.0};         // [m] last measured cluster range (confirmation scaling)
    double relative_speed{0.0};
    double velocity_mahalanobis_sq{0.0};
    bool dynamic_motion_reliable{false};
    // Most recent measured Frenet footprint, retained relative to the Kalman centre so a
    // predicted-only output can move its last valid shape without claiming a current Cartesian
    // scan footprint.
    double s_half_extent{0.0};
    double d_right_offset{0.0};
    double d_left_offset{0.0};
    double size{0.0};
    // Last measured map-frame Cartesian AABB. It is retained for the next matched update, but
    // consumers must use it only while is_visible=true; prediction updates Frenet state, not this
    // raw scan footprint.
    double x_min_map{0.0};
    double x_max_map{0.0};
    double y_min_map{0.0};
    double y_max_map{0.0};
    std::deque<std::pair<double, double>> hist;  // (s, d) for the std classifier

    double s() const { return x(0); }
    double vs() const { return x(1); }
    double d() const { return x(2); }
    double vd() const { return x(3); }
};

// Per-update tracker diagnostics. Association rejection counters count track-detection candidate
// pairs, while track/classification counters are a snapshot after retirement for the current scan.
struct TrackerUpdateStats
{
    std::size_t candidate_pairs{0};
    std::size_t matched{0};
    std::size_t spawned{0};
    std::size_t retired{0};
    std::size_t euclidean_pair_rejected{0};
    std::size_t mahalanobis_pair_rejected{0};

    std::size_t total_tracks{0};
    std::size_t visible_tracks{0};
    std::size_t hit_confirmation_pending{0};
    std::size_t provisional_static{0};
    std::size_t confirmed_static{0};
    std::size_t confirmed_dynamic{0};
    std::size_t dynamic_motion_gated{0};
};

class ObstacleTracker
{
  public:
    ObstacleTracker() = default;

    void configure(const TrackerParams &params, const FrenetProjector *frenet);

    // Drop every track. Must be called when the Frenet reference is rebuilt, since the old tracks
    // live in the previous s-domain.
    void clear();

    // Predict all tracks to `stamp`, associate detections, update, spawn/retire, classify.
    void update(const std::vector<Detection> &detections, double stamp,
                double ego_yaw_rate = 0.0, bool yaw_rate_fresh = true);

    const std::vector<Track> &tracks() const { return tracks_; }
    const TrackerUpdateStats &lastStats() const { return last_stats_; }

    // Current map-flow reference velocity (Frenet): the walls'/stationary obstacles' shared
    // apparent velocity that the dynamic opponent is measured against (Layer 2 baseline).
    double staticRefVs() const { return static_ref_vs_; }
    double staticRefVd() const { return static_ref_vd_; }

  private:
    void predict(Track &t, double dt) const;
    Eigen::Matrix2d measurementCovariance(const Detection &detection) const;
    double innovationDistanceSquared(const Track &t, const Detection &detection) const;
    double velocityMahalanobisSquared(const Track &t, double rel_vs, double rel_vd) const;
    void kalmanUpdate(Track &t, const Detection &detection) const;
    double smoothExtent(double previous, double current) const;
    void updateStaticReference();
    int requiredConfirmFrames(const Track &t) const;
    void classify(Track &t, double ego_yaw_rate, bool yaw_rate_fresh) const;
    double frenetDistSquared(double s1, double d1, double s2, double d2) const;

    TrackerParams p_;
    const FrenetProjector *frenet_{nullptr};
    std::vector<Track> tracks_;
    int next_id_{0};
    double last_stamp_{-1.0};
    bool has_last_stamp_{false};
    double static_ref_vs_{0.0};   // map-flow reference velocity (Frenet s)
    double static_ref_vd_{0.0};   // map-flow reference velocity (Frenet d)
    TrackerUpdateStats last_stats_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
