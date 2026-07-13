// ================================================================================================
// OVERTAKE PLANNER - committed spline overtaking maneuver with a continuous validity check
// ================================================================================================
// Replaces the old always-replan AvoidancePlanner. A small state machine decides WHEN a local
// overtaking path exists at all:
//
//   IDLE      : no opponent blocking / not catchable / corner ahead -> publish NOTHING
//   COMMITTED : all gates passed -> a short local spline path (ego -> pass the opponent -> merge
//               back onto the global raceline) is built and re-validated every cycle. The path
//               covers ONLY the overtaking segment, not the whole track.
//   TRAILING  : the opponent BLOCKS the corridor but an overtake is not possible (gates reject,
//               or cooling down after an abort) -> publish a follow path along the raceline that
//               DECELERATES to the opponent speed at ot_trail_gap behind it. Silence here would
//               hand control back to the full-speed global raceline and rear-end the opponent.
//               Rebuilt every cycle (not committed); ends with ONE empty OT when the opponent no
//               longer blocks / is passed / is lost, or seamlessly upgrades to COMMITTED the
//               moment an overtake becomes feasible.
//   (on completion or abort) -> ONE empty OT is emitted so wpnt_publisher falls back to the
//               global raceline, then publishing stops entirely (cooldown, back to IDLE).
//
// Commit gates (all must hold):
//   - opponent is dynamic, ahead, inside the trigger window, and blocks the ego corridor
//   - relative speed (attainable ego speed - opponent vs) >= ot_min_rel_vel  -> catchable
//   - predicted catch time <= ot_max_catch_time_s
//   - raceline curvature stays below ot_max_kappa over the whole maneuver -> no sharp corner
//   - lateral room exists at the PREDICTED pass location (opponent moves; apex is placed where
//     the pass will actually happen, not where the opponent is now)
//   - the spline stays within the steering limit (|dd/ds| <= ot_max_d_slope): ramps are sized
//     for it, the reachable apex is capped by it, and the fitted spline is verified against it
//
// Handoff continuity: the speed profile is blended from the CURRENT ego speed at the path start
// and into the raceline speed at the merge end (ot_speed_blend_s), so switching local<->global
// never steps the speed setpoint (no hard accel/brake at either boundary).
//
// While COMMITTED the opponent keeps moving, so validity is re-judged every cycle:
//   - completion requires the pass to have been OBSERVED (opponent seen behind the ego by
//     completion_margin at least once); a lost opponent track never counts as "complete"
//   - opponent track lost before the pass -> abort (never a false "overtake complete")
//   - while still approaching (not yet alongside) ALL commit gates are re-run every cycle from
//     the current state; if the maneuver is no longer feasible on either side (corner, no room
//     inside the wall margin, uncatchable) -> abort back to the global raceline
//   - opponent intrudes on the committed passing gap -> replan on the other side, else abort
//   - relative speed collapses (ot_abort_rel_vel hysteresis) -> abort (follow instead)
//   - opponent drifted beyond replan thresholds but still passable -> replan from the CURRENT
//     ego (s, d) so the path always starts where the car actually is
//   - once alongside, the path is never aborted for feasibility (dropping to the global line
//     mid-pass would steer into the opponent) — only gap-loss / speed-collapse abort applies
//   - ego reached the merge point AND the pass was observed -> complete
// ================================================================================================

#ifndef OPPONENT_DETECTOR__OVERTAKE_PLANNER_HPP_
#define OPPONENT_DETECTOR__OVERTAKE_PLANNER_HPP_

#include <functional>
#include <string>
#include <vector>

#include <f110_msgs/msg/wpnt_array.hpp>

namespace opponent_detector
{

// All overtake/spline tunables (declared with defaults in the node, overridable from the YAML).
struct OvertakeParams
{
    bool enabled{true};
    // trigger window
    double trigger_min_ds{0.5};       // only consider an opponent at least this far ahead [m]
    double trigger_max_ds{8.0};       // ...and no farther than this ahead [m]
    double block_margin{0.15};        // opponent blocks us when |opp_d| < ego+opp half-widths + this
    // catchability (relative velocity) gates
    double min_rel_vel{0.5};          // commit only when attainable rel. speed >= this [m/s]
    double abort_rel_vel{0.2};        // abort a committed pass when rel. speed drops below [m/s]
    double max_catch_time{6.0};       // commit only when the predicted catch time <= this [s]
    // corner gate
    double max_kappa{0.6};            // no commit if raceline |kappa| exceeds this on the path [1/m]
    // spline shape
    double pre_distance{2.0};         // min ramp length from ego d to the apex offset [m]
    double post_distance{2.0};        // return-to-raceline length after the pass zone [m]
    double pass_clearance_s{1.0};     // hold the apex offset this far around the pass point [m]
    double lateral_clearance{0.30};   // extra gap beyond ego+opponent half-widths at the apex [m]
    double boundary_margin{0.20};     // keep the line at least this far inside the track edge [m]
    double max_d_slope{0.40};         // steering limit: max |dd/ds| of the spline (tan of the
                                      // heading offset vs the raceline); ramps are lengthened and
                                      // the apex flattened to respect it
    double max_path_kappa{0.5};       // curvature limit: max |d2d/ds2| the lateral spline may add
                                      // [1/m]. This is what sets the steering angle — a curvature
                                      // spike spins the car even at modest slope; ramps/apex are
                                      // sized and the fit verified against it
    double fallback_halfwidth{1.5};   // track half-width when waypoint d_left/d_right are unset [m]
    double ego_half_width{0.15};      // ego half-width [m]
    double opponent_half_width{0.25}; // assumed opponent half-width [m]
    double spline_resolution{0.15};   // sample spacing along the path [m]
    int side_mode{0};                 // 0 auto, 1 force-left, 2 force-right
    int min_points{5};                // reject a line shorter than this many samples
    // speed profile (curvature-limited: v = sqrt(a_cap/|kappa_tot|) with
    // a_cap = max(max_lat_accel, raceline vx^2*|kappa_r|) — the raceline's own grip usage is
    // trusted as local capacity, so the cap never forces the car under the raceline speed
    // where the spline adds no curvature; max_lat_accel is the floor grip on straights)
    double max_lat_accel{6.0};
    double max_long_accel{4.0};       // longitudinal accel budget for the fwd/bwd speed smoothing
                                      // [m/s^2] — the car brakes THIS hard before a tight section
    double speed_scale{1.0};
    double v_floor{1.0};
    double v_ceiling{9.0};
    double speed_blend_s{1.5};        // blend length into ego speed (start) / raceline speed (end)
                                      // so the local<->global handoff never steps the setpoint [m]
    // committed-path validity monitoring
    double intrusion_margin{0.5};     // opponent within this lateral distance of the path -> replan/abort [m]
    double replan_ds_threshold{0.5};  // opponent s drift since last plan that forces a replan [m]
    double replan_dd_threshold{0.2};  // opponent d drift since last plan that forces a replan [m]
    double ego_dev_replan{0.4};       // ego lateral deviation from the committed path that forces a
                                      // replan from the CURRENT pose [m]; above 2x the committed
                                      // line is meaningless (crash/slide) and the maneuver aborts
    double completion_margin{1.0};    // ego must be this far past the opponent to complete [m]
    double max_duration{10.0};        // hard cap on a single committed maneuver [s]
    double cooldown{2.0};             // idle time after completion/abort before a new commit [s]
    // trailing (follow mode when the overtake is not possible)
    bool trail_enabled{true};         // decelerate-and-follow local path while blocked but unable
                                      // to pass (instead of silently rejoining the global line)
    double trail_gap{1.5};            // following distance to hold behind the opponent [m], along
                                      // s between vehicle centres; ego speed matches the opponent
                                      // at this point via the fwd/bwd accel-limited profile
};

// Ego / opponent snapshots fed to the state machine each perception cycle.
struct EgoState
{
    double s{-1.0};
    double d{0.0};
    double v{0.0};  // measured ego speed [m/s]
};

struct OpponentState
{
    bool valid{false};
    double s{0.0};
    double d{0.0};
    double vs{0.0};
    double vd{0.0};
};

// What the node should do with the OT topic this cycle.
struct OvertakeDecision
{
    enum class Action
    {
        None,     // publish nothing (IDLE / cooldown) — local path stays silent
        Publish,  // publish the committed overtaking path
        Clear     // publish ONE empty OT (fall back to global), then go silent
    };
    Action action{Action::None};
    f110_msgs::msg::WpntArray wpnts;  // valid when action == Publish
    std::string side;                 // "left" / "right" (overtake) or "trail" when Publish
    std::string reason;               // human-readable state-change reason (for logging)
    std::string debug;                // why the commit gates rejected this cycle (Idle only)
};

class OvertakePlanner
{
  public:
    enum class State
    {
        Idle,
        Committed,
        Cooldown
    };

    // Set the raceline reference (ordered by increasing s). track_length = closed-loop perimeter.
    // fallback_halfwidth replaces unset (~0) waypoint d_left/d_right, matching the detector's
    // fallback_track_halfwidth behaviour — racelines exported without bounds otherwise leave the
    // planner with a zero-width track and every plan rejects with "no room".
    void setReference(const f110_msgs::msg::WpntArray &global, double track_length,
                      double fallback_halfwidth);

    // Occupancy check against the LIVE map (map frame x, y -> true when the path may not go
    // there). The raceline CSV's d_left/d_right raycast can miss structure that only exists in
    // the live /map (e.g. obstacle-baked maps), so every candidate spline is ALSO validated
    // against this checker before it can be committed. Unset = geometry-only validation.
    void setCollisionChecker(std::function<bool(double, double)> checker)
    {
        occupied_ = std::move(checker);
    }

    bool ready() const { return ref_.size() >= 2; }
    double trackLength() const { return length_; }
    State state() const { return state_; }

    // Frenet (s,d) -> map (x,y) using the raceline reference (left-of-travel d positive). For viz.
    void toCartesian(double s, double d, double &x, double &y) const;

    // Advance the state machine one perception cycle. stamp = scan time [s].
    OvertakeDecision update(const EgoState &ego, const OpponentState &opp,
                            const OvertakeParams &p, double stamp);

  private:
    struct Ref
    {
        double s{0.0};
        double x{0.0};
        double y{0.0};
        double psi{0.0};
        double vx{0.0};
        double kappa{0.0};
        double dl{0.0};  // d_left magnitude (left boundary at d = +dl)
        double dr{0.0};  // d_right magnitude (right boundary at d = -dr)
    };

    // Result of one path-construction attempt.
    struct PlanResult
    {
        bool ok{false};
        bool go_left{false};
        f110_msgs::msg::WpntArray wpnts;
        double merge_s{0.0};      // wrapped arc-length where the path rejoins the raceline
        double pass_s{0.0};       // wrapped arc-length of the predicted pass point
        double apex_d{0.0};       // lateral apex offset chosen for the pass
        std::string reject;       // which gate rejected the plan (debug; empty when ok)
    };

    // Build the local overtaking path from the CURRENT ego state; applies every commit gate.
    // forced_side: 0 = auto/param, 1 = left, 2 = right (used for the intrusion side-switch).
    PlanResult buildPath(const EgoState &ego, const OpponentState &opp, const OvertakeParams &p,
                         int forced_side) const;

    // Build the trailing (follow) path: raceline-following lateral ramp from the current ego d,
    // speed profile decelerating to the opponent speed at trail_gap behind it. No commit gates —
    // only requires the opponent ahead within the trigger window and blocking the corridor.
    PlanResult buildTrailPath(const EgoState &ego, const OpponentState &opp,
                              const OvertakeParams &p) const;

    // Publish a trailing path when possible, otherwise close an active trail (ONE empty OT) or
    // stay silent. Shared by the Idle-reject and Cooldown branches of update().
    OvertakeDecision trailDecision(const EgoState &ego, const OpponentState &opp,
                                   const OvertakeParams &p, OvertakeDecision dec);

    // Attainable ego speed used for the catchability prediction (max of measured and raceline vx).
    double attainableSpeed(const EgoState &ego) const;

    // True when the opponent has moved into the passing gap (its d encroaches on the apex side so
    // the planned lateral clearance no longer holds). Evaluated against the committed apex offset
    // and side, NOT the ramping waypoints, so it does not false-trigger before the apex.
    bool passingGapLost(const OpponentState &opp, const OvertakeParams &p) const;

    // Lateral deviation of the CURRENT ego pose from the committed path: |ego.d - d_m| of the
    // committed waypoint nearest in s. Ego outside the path's s-range entirely (behind the start /
    // past the end by more than a small margin) counts as a full deviation (large value).
    double egoPathDeviation(const EgoState &ego) const;

    void interp(double s, double &x, double &y, double &psi, double &vx, double &kappa, double &dl,
                double &dr) const;
    double wrapS(double s) const;
    double fwdDelta(double from, double to) const;  // forward arc distance from -> to (>= 0)

    std::vector<Ref> ref_;
    double length_{0.0};
    std::function<bool(double, double)> occupied_;  // live-map occupancy check (may be empty)

    // ---- state-machine memory ----
    State state_{State::Idle};
    f110_msgs::msg::WpntArray committed_;
    bool committed_left_{false};
    double merge_s_{0.0};
    double apex_d_{0.0};      // committed apex offset (for the passing-gap check)
    double commit_stamp_{0.0};
    double cooldown_until_{0.0};
    double plan_opp_s_{0.0};  // opponent pose when the committed path was (re)built
    double plan_opp_d_{0.0};
    bool passed_opp_{false};  // the opponent was OBSERVED behind the ego (completion evidence)
    bool trailing_{false};    // a trailing (follow) path is currently being published
};

}  // namespace opponent_detector

#endif  // OPPONENT_DETECTOR__OVERTAKE_PLANNER_HPP_
