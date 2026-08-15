#ifndef F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_
#define F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace f1tenth_control
{

struct CruiseControllerConfig
{
  double maximum_speed{12.0};
  double emergency_stop_distance{0.45};
  double relative_deceleration{2.5};
  double proportional_gain{1.0};
  double integral_gain{0.0};
  double derivative_gain{0.2};
  double integral_limit{2.0};
  double uncertainty_sigma{2.0};
  bool allow_acceleration{true};
};

struct CruiseControllerInput
{
  double gap{0.0};
  double desired_gap{1.5};
  double ego_speed{0.0};
  double opponent_speed{0.0};
  double opponent_s_variance{0.0};
  double dt{0.02};
};

struct CruiseControllerOutput
{
  double speed_limit{0.0};
  double effective_gap{0.0};
  double gap_error{0.0};
  double relative_speed{0.0};
  double gap_integral{0.0};
};

class CruiseLongitudinalController
{
public:
  explicit CruiseLongitudinalController(const CruiseControllerConfig & config);

  CruiseControllerOutput update(const CruiseControllerInput & input);
  void reset();

private:
  CruiseControllerConfig config_;
  double gap_integral_{0.0};
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL__CRUISE_CONTROLLER_HPP_
