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
#include <functional>
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
    // A confirmed STATIC track is a map-fixed object: after losing sight (occlusion, FOV,
    // brake-pitch dropouts) keep it alive and publishable for this long, measured on scan
    // stamps, instead of letting the frame TTL retire it in ~0.1 s. 0 disables the hold and
    // restores pure frame-TTL retirement.
    double static_lost_hold_sec{5.0};
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
    // Progressive LiDAR revelation grows a stationary obstacle's measured AABB, which moves its
    // centroid and drives the map-frame Kalman velocity away from zero even though nothing moved.
    // Corroborate every Dynamic vote with translation the AABB edges actually prove: a box has
    // provably translated only where both of its edges on an axis moved the same way, so growth or
    // shrink alone proves nothing. Without that corroboration the vote is Uncertain, never Dynamic.
    bool translation_corroboration_enable{true};
    double translation_window_sec{0.20};
    int translation_history_max_samples{64};
    double dynamic_min_translation_m{0.10};
    // STATIC entry needs only 15 measurement samples, which at 250 Hz is 0.06 s, while DYNAMIC
    // entry additionally has to prove dynamic_min_translation_m of translation. STATIC therefore
    // always wins the race, and a passing opponent that has not yet earned its dynamic votes is
    // frozen into /confirmed_static_obs -- where a Confirmed STATIC track additionally becomes
    // hold-eligible and republishes a ghost at a stale pose for static_lost_hold_sec.
    // The exclusion makes the two entries mutually exclusive on shared evidence instead of
    // equalizing their latency: STATIC entry is denied while the same window carries motion
    // evidence (a corroborated translation or a live dynamic vote), and while the ego-motion
    // transient is withholding dynamic votes -- because a frame whose dynamic evidence is
    // deliberately discarded is not evidence of stillness either. A genuinely map-fixed obstacle
    // produces none of the three (progressive revelation cannot move both edges of an axis the
    // same way), so its detection latency is unchanged.
    bool static_entry_exclusion_enable{true};
    // Upper bound on how long the ego-transient term alone may deny STATIC entry. On a race car
    // |accel| crosses the suppression threshold for most of every braking zone, so an unbounded
    // term would keep newly seen obstacles out of /confirmed_static_obs for seconds at a time --
    // exactly while the car is braking towards them. Past this many seconds of transient-only
    // blocking the term yields: a track that has shown no translation and no dynamic vote for that
    // long is a static obstacle whatever the ego is doing. The translation and dynamic-vote terms
    // are never time-limited; they are evidence about the object itself. 0 disables the bound.
    double static_entry_exclusion_max_transient_sec{1.0};
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
    // static_lost_hold_sec's justification is "unobserved means occluded, and a map-fixed object
    // cannot change while occluded". That is false whenever the scan actively sees THROUGH the
    // held box: beams that traverse its core and return from something further away prove the
    // space is empty, so the hold is keeping a ghost alive in plain view. Retire the track after
    // this many consecutive scans of such free-space refutation (0 disables the refutation and
    // restores the pure occlusion assumption). The refutation itself is supplied by the caller
    // (only the node owns the scan geometry) through update()'s free_space_refuter.
    int static_hold_freespace_refute_frames{3};
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

// Signed translation one axis of an AABB provably underwent. Both edges must move the same way;
// when only one edge moves, the box grew or shrank in place and no translation is proven.
double anchoredAxisTranslation(
    double previous_min, double previous_max, double current_min, double current_max);

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

// One measured map-frame AABB retained inside the translation-corroboration window.
struct MeasuredAabbSample
{
    double stamp{0.0};
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
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
    // Measured map-frame AABBs inside translation_window_sec, and the largest translation any pair
    // of them proves. A stationary obstacle being revealed holds this near zero while its centroid
    // travels several centimetres.
    std::deque<MeasuredAabbSample> measured_aabb_history;
    double provable_translation_m{std::numeric_limits<double>::quiet_NaN()};
    bool dynamic_evidence_suppressed{false};
    // Scan stamp at which the ego-motion transient became the ONLY term denying STATIC entry.
    // NaN whenever some other term also blocks, or nothing blocks.
    double static_entry_transient_block_since{std::numeric_limits<double>::quiet_NaN()};
    double static_confidence{0.0};
    double time_since_last_measurement{0.0};
    // Raw Frenet centre and timestamp from the most recently associated detection. Unlike x,
    // these values never advance during prediction-only frames and therefore anchor dormant IDs
    // to an observed physical location.
    double last_measured_s{std::numeric_limits<double>::quiet_NaN()};
    double last_measured_d{std::numeric_limits<double>::quiet_NaN()};
    double last_measurement_stamp{std::numeric_limits<double>::quiet_NaN()};
    int envelope_stable_streak{0};  // consecutive matched frames with a settled centre+envelope
    // Consecutive unmeasured scans whose beams proved the held envelope's space empty. Any
    // measurement, and any scan that cannot see the box, resets it.
    int freespace_refute_streak{0};
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
    std::size_t translation_suppressed_dynamic_votes{0};
};

class ObstacleTracker
{
  public:
    ObstacleTracker() = default;

    void configure(const TrackerParams &params, const FrenetProjector *frenet);

    // Drop every track. Must be called when the CLCS reference is rebuilt, since the old tracks
    // live in the previous s-domain.
    void clear();

    // Free-space refutation of one held track's last measured map AABB, evaluated against the
    // current scan. Returns true only on positive evidence that the box's space is empty.
    using FreeSpaceRefuter = std::function<bool(const Track &)>;

    // Predict all tracks to `stamp`, associate detections, update, spawn/retire, classify.
    // `ego_motion_transient` marks frames where ego acceleration is transient (hard braking or
    // launch): localization jitter then mimics obstacle translation, so dynamic votes are
    // withheld while it is set. `free_space_refuter` is consulted ONLY for confirmed static
    // tracks that received no measurement this scan and would otherwise be held alive by
    // static_lost_hold_sec; leaving it empty keeps the previous pure-occlusion hold.
    void update(const std::vector<Detection> &detections, double stamp,
                double ego_yaw_rate = 0.0, bool yaw_rate_fresh = true,
                bool ego_motion_transient = false,
                const FreeSpaceRefuter &free_space_refuter = nullptr);

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
    void updateTranslationEvidence(
        Track &t, const Detection &detection, double stamp) const;
    void classify(Track &t, bool measurement_received) const;
    // Single authority for "this unmeasured track may be held/frozen as a map-fixed object".
    // Both the state-freeze and the TTL/publish-evidence hold must ask the same question; two
    // copies of the predicate would drift apart.
    bool holdEligibleWhileUnmeasured(const Track &t) const;
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
    bool ego_motion_transient_{false};
    TrackerUpdateStats last_stats_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_TRACKER_HPP_
