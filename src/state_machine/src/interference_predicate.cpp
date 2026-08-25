#include "state_machine/interference_predicate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace state_machine
{
namespace
{

// Standard normal CDF via erfc (no external dependency, no libm macro reliance).
double normalCdf(double x)
{
  return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

// P(lo <= X <= hi) for X ~ N(mu, sigma^2). sigma<=0 degenerates to a point mass at mu.
double intervalProbability(double lo, double hi, double mu, double sigma)
{
  if (!(sigma > 1e-9)) {
    return (mu >= lo && mu <= hi) ? 1.0 : 0.0;
  }
  return normalCdf((hi - mu) / sigma) - normalCdf((lo - mu) / sigma);
}

double wrapForward(double from, double to, double track_length)
{
  double delta = std::fmod(to - from, track_length);
  if (delta < 0.0) {
    delta += track_length;
  }
  return delta;
}

// CV time propagation of a tracker 2x2 block's diagonal: var(t) = var + 2*t*cov + t^2*var_rate.
// Floored at 0 so a cov close to its Cauchy-Schwarz bound cannot drive the result negative under
// floating-point rounding.
double propagatedVariance(double var, double cov, double var_rate, double t)
{
  return std::max(0.0, var + 2.0 * t * cov + t * t * var_rate);
}

}  // namespace

double interferenceProbability(
  const InterferenceEgoState & ego,
  const InterferenceOpponentState & opponent,
  const InterferenceGeometryConfig & config)
{
  if (!(ego.track_length > 0.0) || !(config.horizon_sec >= 0.0) ||
    !(config.distance_m > 0.0))
  {
    return 0.0;
  }
  if (!std::isfinite(ego.s) || !std::isfinite(ego.d) || !std::isfinite(ego.vs) ||
    !std::isfinite(opponent.s_center) || !std::isfinite(opponent.d_center) ||
    !std::isfinite(opponent.vs) || !std::isfinite(opponent.vd))
  {
    return 0.0;
  }

  // Front-half-lap gate (same convention as the legacy detector-side judgment): an opponent
  // behind ego or beyond half a lap ahead cannot interfere.
  const double center_ahead = wrapForward(ego.s, opponent.s_center, ego.track_length);
  if (!(center_ahead > 0.0) || !(center_ahead < 0.5 * ego.track_length)) {
    return 0.0;
  }

  const double obstacle_half_width =
    0.5 * std::abs(opponent.d_left - opponent.d_right);
  const double b_eff = std::max(
    0.0, config.ego_half_width_m + config.lateral_margin_m + obstacle_half_width);

  const double d_var = std::max(0.0, opponent.d_var);
  const double vd_var = std::max(0.0, opponent.vd_var);

  // Longitudinal distance/horizon no longer gate interference: any opponent within the
  // front-half-lap gate above triggers purely on lateral corridor overlap.
  static constexpr std::array<double, 5> kGridFractions{0.0, 0.25, 0.5, 0.75, 1.0};
  double p_star = 0.0;
  for (const double fraction : kGridFractions) {
    const double t = fraction * config.horizon_sec;

    // Lateral: CV time propagation, including the tracker's cov(d,vd) cross term.
    const double mu_d = opponent.d_center + opponent.vd * t;
    const double sigma_d = std::sqrt(propagatedVariance(d_var, opponent.d_vd_cov, vd_var, t));
    const double p_lat = intervalProbability(-b_eff, b_eff, mu_d, sigma_d);

    p_star = std::max(p_star, p_lat);
  }
  return p_star;
}

double longitudinalGapMeters(
  const InterferenceEgoState & ego,
  const InterferenceOpponentState & opponent,
  const InterferenceGeometryConfig & config)
{
  constexpr double kGated = std::numeric_limits<double>::infinity();
  if (!(ego.track_length > 0.0) || !std::isfinite(ego.s) || !std::isfinite(opponent.s_center)) {
    return kGated;
  }

  // Same front-half-lap gate as interferenceProbability(): an opponent behind ego or beyond half
  // a lap ahead cannot interfere via either OR'd condition.
  const double center_ahead = wrapForward(ego.s, opponent.s_center, ego.track_length);
  if (!(center_ahead > 0.0) || !(center_ahead < 0.5 * ego.track_length)) {
    return kGated;
  }

  return std::max(0.0, center_ahead - config.ego_front_offset_m);
}

}  // namespace state_machine
