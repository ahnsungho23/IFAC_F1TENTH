#include <gtest/gtest.h>

#include <limits>

#include "state_machine/interference_predicate.hpp"

namespace
{

using state_machine::InterferenceEgoState;
using state_machine::InterferenceGeometryConfig;
using state_machine::InterferenceOpponentState;
using state_machine::interferenceProbability;

InterferenceEgoState defaultEgo()
{
  InterferenceEgoState ego;
  ego.s = 0.0;
  ego.d = 0.0;
  ego.vs = 4.0;
  ego.track_length = 40.0;
  return ego;
}

InterferenceOpponentState defaultOpponent()
{
  InterferenceOpponentState opponent;
  opponent.id = 1;
  opponent.s_center = 3.0;
  opponent.s_start = 2.7;
  opponent.s_end = 3.3;
  opponent.d_center = 0.0;
  opponent.vs = 0.0;
  opponent.vd = 0.0;
  opponent.s_var = 0.0;
  opponent.d_var = 0.0;
  opponent.vs_var = 0.0;
  opponent.vd_var = 0.0;
  opponent.d_left = 0.15;
  opponent.d_right = -0.15;
  opponent.is_visible = true;
  return opponent;
}

InterferenceGeometryConfig defaultConfig()
{
  InterferenceGeometryConfig config;
  config.distance_m = 5.0;
  config.horizon_sec = 1.0;
  config.ego_half_width_m = 0.16;
  config.lateral_margin_m = 0.10;
  config.ego_front_offset_m = 0.25;
  return config;
}

TEST(InterferencePredicate, ZeroVarianceOnLineWithinDistanceIsCertain)
{
  const double p = interferenceProbability(defaultEgo(), defaultOpponent(), defaultConfig());
  EXPECT_NEAR(p, 1.0, 1e-9);
}

TEST(InterferencePredicate, OpponentBeyondDistanceIsNotInterfering)
{
  auto opponent = defaultOpponent();
  opponent.s_center = 12.0;
  opponent.s_start = 11.7;
  opponent.s_end = 12.3;
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_NEAR(p, 0.0, 1e-9);
}

TEST(InterferencePredicate, OpponentOutsideCorridorIsNotInterfering)
{
  auto opponent = defaultOpponent();
  opponent.d_center = 1.5;
  opponent.d_left = 1.65;
  opponent.d_right = 1.35;
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_NEAR(p, 0.0, 1e-9);
}

TEST(InterferencePredicate, OpponentBehindEgoIsNotInterfering)
{
  auto opponent = defaultOpponent();
  opponent.s_center = -3.0;
  opponent.s_start = -3.3;
  opponent.s_end = -2.7;
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_NEAR(p, 0.0, 1e-9);
}

TEST(InterferencePredicate, PositionVarianceSmearsProbabilityBelowOne)
{
  auto opponent = defaultOpponent();
  opponent.s_center = 4.9;
  opponent.s_start = 4.6;
  opponent.s_end = 5.2;
  opponent.s_var = 0.25;  // sigma = 0.5 m, straddles the 5.0 m boundary
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_GT(p, 0.0);
  EXPECT_LT(p, 1.0);
}

TEST(InterferencePredicate, ApproachingOpponentRaisesProbabilityOverHorizon)
{
  // Ego stationary so closing speed comes purely from the opponent. Opponent sits just past the
  // 5 m gate at t=0 (small but nonzero p due to s_var) and only enters the gate within the
  // horizon if it is actually closing -- a stationary opponent (constant gap) never does.
  auto ego = defaultEgo();
  ego.vs = 0.0;
  auto opponent = defaultOpponent();
  opponent.s_center = 6.5;
  opponent.s_start = 6.35;
  opponent.s_end = 6.65;
  opponent.s_var = 0.09;  // sigma = 0.3 m
  opponent.vs = -4.0;     // closing_speed = ego.vs - opponent.vs = 4 m/s
  const double p_closing = interferenceProbability(ego, opponent, defaultConfig());

  auto opponent_stationary = opponent;
  opponent_stationary.vs = 0.0;  // closing_speed = 0 -> gap never shrinks
  const double p_no_closing = interferenceProbability(ego, opponent_stationary, defaultConfig());

  EXPECT_GT(p_closing, p_no_closing);
  EXPECT_LT(p_no_closing, 0.01);
  EXPECT_GT(p_closing, 0.9);
}

TEST(InterferencePredicate, LongitudinalCrossCovarianceShiftsProbability)
{
  // vs_var alone would grow sigma_g^2 by t^2*vs_var regardless of sign; a nonzero s_vs_cov adds
  // the signed 2*t*s_vs_cov term. At the horizon grid's largest t this is the dominant knob, so a
  // positive vs. negative cov (same magnitude, within the Cauchy-Schwarz bound
  // sqrt(s_var*vs_var) = sqrt(0.09*1.0) = 0.3) must move sigma_g -- and hence p* -- apart.
  auto ego = defaultEgo();
  ego.vs = 0.0;
  auto opponent = defaultOpponent();
  opponent.s_center = 6.5;
  opponent.s_start = 6.35;
  opponent.s_end = 6.65;
  opponent.s_var = 0.09;
  opponent.vs_var = 1.0;
  opponent.vs = -4.0;

  auto opponent_pos_cov = opponent;
  opponent_pos_cov.s_vs_cov = 0.25;
  const double p_pos = interferenceProbability(ego, opponent_pos_cov, defaultConfig());

  auto opponent_neg_cov = opponent;
  opponent_neg_cov.s_vs_cov = -0.25;
  const double p_neg = interferenceProbability(ego, opponent_neg_cov, defaultConfig());

  EXPECT_NE(p_pos, p_neg);
}

TEST(InterferencePredicate, LateralCrossCovarianceIsFloorClampedNotNegative)
{
  // d_vd_cov at the Cauchy-Schwarz bound (sqrt(d_var*vd_var)) can drive sigma_d^2 slightly
  // negative under the CV propagation formula for some t; propagatedVariance() must floor it at
  // 0 rather than feeding sqrt() a negative number.
  auto opponent = defaultOpponent();
  opponent.d_var = 0.01;
  opponent.vd_var = 0.01;
  opponent.d_vd_cov = -0.0099;  // just inside sqrt(0.01*0.01) = 0.01
  opponent.vd = 0.5;
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_GE(p, 0.0);
  EXPECT_LE(p, 1.0);
}

TEST(InterferencePredicate, NonFiniteInputReturnsZero)
{
  auto opponent = defaultOpponent();
  opponent.s_center = std::numeric_limits<double>::quiet_NaN();
  const double p = interferenceProbability(defaultEgo(), opponent, defaultConfig());
  EXPECT_EQ(p, 0.0);
}

TEST(InterferencePredicate, ZeroTrackLengthReturnsZero)
{
  auto ego = defaultEgo();
  ego.track_length = 0.0;
  const double p = interferenceProbability(ego, defaultOpponent(), defaultConfig());
  EXPECT_EQ(p, 0.0);
}

}  // namespace
