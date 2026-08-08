// ================================================================================================
// OBSTACLE TRACKER - Frenet association plus map-frame statistical motion classification
// ================================================================================================
// Association/output state: x = [s, vs, d, vd], measurement z = [s, d].
// Classification-only state: map_x = [x, vx, y, vy], measurement z_map = [x, y].
// Keeping the two filters side-by-side preserves the existing closed-track Frenet association and
// footprint contract while preventing hairpin s-projection changes from deciding motion status.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
#define OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <utility>
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
    // Measurement-covariance multiplier derived from range, cluster density, and fresh ego yaw
    // rate. The base variances remain meas_var_s/meas_var_d in YAML.
    double variance_scale{1.0};
};

enum class TrackStatus
{
    Raw,
    Tentative,
    Confirmed
};

enum class MotionStatus
{
    Unknown,
    Static,
    Dynamic
};

enum class MotionEvidence
{
    StaticEvidence,
    DynamicEvidence,
    Uncertain
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
    // Public obstacle IDs represent physical objects, independently of a Kalman track's lifetime
    // and Unknown/Static/Dynamic motion state. Reconnect matching Frenet envelopes in every motion
    // state and retain a confirmed object's physical ID across a complete track dropout.
    bool physical_id_reassociation_enable{true};
    double physical_id_reassociation_gap_s{0.4};
    double physical_id_reassociation_gap_d{0.3};
    double physical_id_reassociation_gap_map{0.4};
    double physical_id_memory_sec{30.0};
    // track lifetime
    int ttl_dynamic{40};
    int ttl_static{25};
    int min_hits_confirm{3};       // required measurement votes inside confirmation_window
    int confirmation_window{5};    // existence confirmation only; independent of motion
    // Map-frame statistical motion classification.
    double dynamic_chi2_threshold{9.21};
    double static_chi2_threshold{5.99};
    int dynamic_vote_window{5};
    int dynamic_vote_required{3};
    int static_vote_window{15};
    int static_vote_required{10};
    int position_history_size{15};
    int static_min_observations{10};
    double static_max_position_rms{0.10};
    int dynamic_to_static_min_observations{20};
    int dynamic_to_static_vote_required{15};
    double dynamic_to_static_max_position_rms{0.08};
    double covariance_regularization_epsilon{1.0e-6};
    double minimum_velocity_covariance{1.0e-5};
    double static_score_forgetting_factor{0.95};
    double dt_max{0.5};           // [s] clamp for prediction step
    // Per-scan AABB extents flap (square box vs real shape). Smooth their magnitude
    // fast-grow/slow-shrink: expand immediately, relax over ~1/alpha matched frames.
    // 1.0 restores the legacy overwrite behaviour.
    double extent_shrink_alpha{0.25};
    // Static-layer publish stability: count consecutive matched frames whose measured centre
    // and envelope stay within the tolerance of the track. Fan-shaped morphing clusters never
    // settle, so requiring a short streak keeps them unpublished while real obstacles (stable
    // from the first scan) see no added delay beyond min_hits_confirm.
    double envelope_stability_tolerance_m{0.10};
    int envelope_stability_frames{2};
};

struct VelocityEvidenceResult
{
    MotionEvidence evidence{MotionEvidence::Uncertain};
    double statistic{std::numeric_limits<double>::quiet_NaN()};
    bool covariance_valid{false};
};

// Pure utility used by the tracker and covariance edge-case unit tests. It never forms an inverse.
VelocityEvidenceResult evaluateVelocityEvidence(
    const Eigen::Vector2d &velocity,
    const Eigen::Matrix2d &velocity_covariance,
    const TrackerParams &params);

const char *trackStatusName(TrackStatus status);
const char *motionStatusName(MotionStatus status);

struct PhysicalIdentityAnchor
{
    bool valid{false};
    double s{0.0};
    double d{0.0};
    double s_half_extent{0.0};
    double d_right_offset{0.0};
    double d_left_offset{0.0};
    double x_min_map{0.0};
    double x_max_map{0.0};
    double y_min_map{0.0};
    double y_max_map{0.0};
};

struct Track
{
    int id{0};  // persistent physical object ID published on every obstacle topic
    int track_uid{0};  // ephemeral Kalman-track instance ID, never published
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};   // [s, vs, d, vd]
    Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
    // Supplemental classification-only map-frame CV Kalman filter [x, vx, y, vy].
    Eigen::Vector4d map_x{Eigen::Vector4d::Zero()};
    Eigen::Matrix4d map_P{Eigen::Matrix4d::Identity()};
    bool map_filter_initialized{false};
    int hits{0};
    int ttl{0};
    bool is_static{true};
    bool is_visible{false};
    TrackStatus track_status{TrackStatus::Raw};
    MotionStatus motion_status{MotionStatus::Unknown};
    std::deque<bool> confirmation_history;
    std::deque<MotionEvidence> motion_evidence_history;
    std::deque<std::pair<double, double>> map_position_history;
    int dynamic_vote_count{0};
    int static_vote_count{0};
    int motion_observations_since_transition{0};
    int total_measurement_updates{0};
    double map_position_rms{std::numeric_limits<double>::quiet_NaN()};
    double velocity_statistic{std::numeric_limits<double>::quiet_NaN()};
    bool velocity_covariance_valid{false};
    double static_confidence{0.0};
    double time_since_last_measurement{0.0};
    // Raw Frenet centre and timestamp from the most recently associated detection. Unlike x,
    // these values never advance during prediction-only frames and therefore anchor dormant IDs
    // to an observed physical location.
    double last_measured_s{std::numeric_limits<double>::quiet_NaN()};
    double last_measured_d{std::numeric_limits<double>::quiet_NaN()};
    double last_measurement_stamp{std::numeric_limits<double>::quiet_NaN()};
    int envelope_stable_streak{0};  // consecutive matched frames with a settled centre+envelope
    // A confirmed physical ID may be remembered after this Kalman-track instance retires.
    bool physical_identity_eligible{false};
    // Captured once from a statistically Static, existence-confirmed measurement. It stays fixed
    // if later viewpoint changes or false motion evidence move the live Kalman state.
    PhysicalIdentityAnchor stable_identity_anchor;
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

    double s() const { return x(0); }
    double vs() const { return x(1); }
    double d() const { return x(2); }
    double vd() const { return x(3); }
    double mapX() const { return map_x(0); }
    double mapVx() const { return map_x(1); }
    double mapY() const { return map_x(2); }
    double mapVy() const { return map_x(3); }
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
    std::size_t spatially_reassociated{0};
    std::size_t physical_id_reused{0};

    std::size_t total_tracks{0};
    std::size_t visible_tracks{0};
    std::size_t raw_tracks{0};
    std::size_t tentative_tracks{0};
    std::size_t motion_unknown{0};
    std::size_t motion_static{0};
    std::size_t motion_dynamic{0};
    std::size_t invalid_velocity_covariance{0};
    std::size_t predicted_only{0};
};

class ObstacleTracker
{
  public:
    ObstacleTracker() = default;

    void configure(const TrackerParams &params, const FrenetProjector *frenet);

    // Drop every track. Must be called when the CLCS reference is rebuilt, since the old tracks
    // live in the previous s-domain.
    void clear();

    // Predict all tracks to `stamp`, associate detections, update, spawn/retire, classify.
    void update(const std::vector<Detection> &detections, double stamp,
                double ego_yaw_rate = 0.0, bool yaw_rate_fresh = true);

    const std::vector<Track> &tracks() const { return tracks_; }
    const TrackerUpdateStats &lastStats() const { return last_stats_; }

  private:
    struct DormantPhysicalIdentity
    {
        int id{0};
        PhysicalIdentityAnchor anchor;
        bool stable_anchor{false};
        double last_seen_stamp{0.0};
    };

    void predict(Track &t, double dt) const;
    void predictMap(Track &t, double dt) const;
    Eigen::Matrix2d measurementCovariance(const Detection &detection) const;
    Eigen::Matrix2d mapMeasurementCovariance(
        const Track &track, const Detection &detection) const;
    double innovationDistanceSquared(const Track &t, const Detection &detection) const;
    void kalmanUpdate(Track &t, const Detection &detection) const;
    bool mapKalmanUpdate(Track &t, const Detection &detection) const;
    double smoothExtent(double previous, double current) const;
    void updateTrackStatus(Track &t, bool measurement_received) const;
    void classify(Track &t, bool measurement_received) const;
    int countEvidence(
        const std::deque<MotionEvidence> &history,
        MotionEvidence evidence,
        int window) const;
    double positionRms(const std::deque<std::pair<double, double>> &history) const;
    void updateStaticConfidence(
        Track &t, MotionEvidence evidence, bool measurement_received) const;
    double frenetDistSquared(double s1, double d1, double s2, double d2) const;
    bool belongsToSamePhysicalCluster(
        double track_s, double track_d, double track_s_half_extent,
        double track_d_right_offset, double track_d_left_offset,
        const Detection &detection) const;
    bool belongsToSameMapCluster(
        const PhysicalIdentityAnchor &anchor, const Detection &detection) const;
    double mapCenterDistSquared(
        const PhysicalIdentityAnchor &anchor, const Detection &detection) const;
    void captureStableIdentityAnchor(Track &track, const Detection &detection) const;

    TrackerParams p_;
    const FrenetProjector *frenet_{nullptr};
    std::vector<Track> tracks_;
    std::vector<DormantPhysicalIdentity> dormant_physical_identities_;
    int next_track_uid_{0};
    int next_physical_id_{0};
    double last_stamp_{-1.0};
    bool has_last_stamp_{false};
    TrackerUpdateStats last_stats_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
