#include <gtest/gtest.h>

#include "f1tenth_control/cruise_controller.hpp"

namespace
{

f1tenth_control::CruiseControllerConfig defaultConfig()
{
  f1tenth_control::CruiseControllerConfig config;
  config.maximum_speed = 12.0;
  config.emergency_stop_distance = 0.45;
  config.relative_deceleration = 2.5;
  config.proportional_gain = 1.0;
  config.integral_gain = 0.0;
  config.derivative_gain = 0.2;
  config.uncertainty_sigma = 0.0;
  return config;
}

TEST(CruiseController, MatchesOpponentAtDesiredGap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({1.5, 1.5, 4.0, 4.0, 0.0, 0.02});
  EXPECT_NEAR(output.speed_limit, 4.0, 1e-9);
}

TEST(CruiseController, AllowsCatchupButAppliesBrakingDistanceCap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({10.0, 1.5, 5.0, 4.0, 0.0, 0.02});
  EXPECT_GT(output.speed_limit, 4.0);
  EXPECT_LT(output.speed_limit, 9.0);
}

TEST(CruiseController, SlowsWhenClosingInsideDesiredGap)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({1.0, 1.5, 6.0, 4.0, 0.0, 0.02});
  EXPECT_LT(output.speed_limit, 4.0);
  EXPECT_GE(output.speed_limit, 0.0);
}

TEST(CruiseController, StopsInsideEmergencyDistance)
{
  f1tenth_control::CruiseLongitudinalController controller(defaultConfig());
  const auto output = controller.update({0.4, 1.5, 5.0, 3.0, 0.0, 0.02});
  EXPECT_DOUBLE_EQ(output.speed_limit, 0.0);
}

TEST(CruiseController, PositionalUncertaintyReducesUsableGap)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  f1tenth_control::CruiseLongitudinalController controller(config);
  const auto certain = controller.update({2.0, 1.5, 4.0, 4.0, 0.0, 0.02});
  controller.reset();
  const auto uncertain = controller.update({2.0, 1.5, 4.0, 4.0, 0.04, 0.02});
  EXPECT_LT(uncertain.speed_limit, certain.speed_limit);
}

}  // namespace
