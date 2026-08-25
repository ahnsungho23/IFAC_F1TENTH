#ifndef STATE_MACHINE__INTERFERENCE_PREDICATE_HPP_
#define STATE_MACHINE__INTERFERENCE_PREDICATE_HPP_

#include <cstdint>

namespace state_machine
{

// Frenet ego state relative to the global raceline (d=0 is the line).
struct InterferenceEgoState
{
  double s{0.0};
  double d{0.0};
  double vs{0.0};
  double track_length{0.0};
};

// Frenet opponent state, including the tracker's cross-covariance terms (cov(s,vs), cov(d,vd)).
// A producer that does not fill s_vs_cov/d_vd_cov (f110_msgs/Obstacle.msg default 0.0) degrades
// to "no cross-correlation", which the propagation formula below already treats correctly as a
// looser but unbiased sigma -- see docs/interference_judgment_migration_proposal.md §3.1/§3.3.
struct InterferenceOpponentState
{
  int32_t id{-1};
  double s_center{0.0};
  double s_start{0.0};
  double s_end{0.0};
  double d_center{0.0};
  double vs{0.0};
  double vd{0.0};
  double s_var{0.0};
  double d_var{0.0};
  double vs_var{0.0};
  double vd_var{0.0};
  double s_vs_cov{0.0};
  double d_vd_cov{0.0};
  double d_left{0.0};
  double d_right{0.0};
  bool is_visible{true};
};

struct InterferenceGeometryConfig
{
  double distance_m{5.0};
  double horizon_sec{1.0};
  double ego_half_width_m{0.16};
  double lateral_margin_m{0.10};
  double ego_front_offset_m{0.25};
};

// Probability that the opponent occupies the ego's global-line band within a 5-point time grid
// over [0, horizon_sec]: p* = max_t P_lat(t). Purely lateral -- does not consider the opponent's
// longitudinal distance/closing speed, only whether it is ahead within half a lap (see the
// front-half-lap gate below). This is one of two independent conditions the caller ORs together
// (see state_machine_node's evaluate_probabilistic_interference()); the other is
// longitudinalGapMeters() below. config.distance_m/ego_front_offset_m are therefore unused here.
// Returns 0.0 for degenerate input (non-positive track_length/horizon, opponent behind or beyond
// half a lap, non-finite geometry).
double interferenceProbability(
  const InterferenceEgoState & ego,
  const InterferenceOpponentState & opponent,
  const InterferenceGeometryConfig & config);

// Longitudinal gap in meters from the ego's front bumper (ego.s + config.ego_front_offset_m) to
// the opponent's center, wrapped forward along the track. This is the second of the two OR'd
// interference conditions (2026-08-25 restore): CRUISE also engages when this gap closes to
// within interference_distance_m, independent of lateral alignment.
// Returns +infinity when gated out (opponent behind ego or beyond half a lap ahead, or
// non-finite/non-positive track_length), so a plain `<= distance_m` check on the result never
// spuriously passes.
double longitudinalGapMeters(
  const InterferenceEgoState & ego,
  const InterferenceOpponentState & opponent,
  const InterferenceGeometryConfig & config);

}  // namespace state_machine

#endif  // STATE_MACHINE__INTERFERENCE_PREDICATE_HPP_
