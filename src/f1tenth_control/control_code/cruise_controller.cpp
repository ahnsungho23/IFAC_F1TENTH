#include "f1tenth_control/cruise_controller.hpp"

namespace f1tenth_control
{

CruiseLongitudinalController::CruiseLongitudinalController(
  const CruiseControllerConfig & config)
: config_(config)
{
  config_.maximum_speed = std::max(0.0, config_.maximum_speed);
  config_.emergency_stop_distance = std::max(0.0, config_.emergency_stop_distance);
  config_.relative_deceleration = std::max(0.01, config_.relative_deceleration);
  config_.integral_limit = std::max(0.0, config_.integral_limit);
  config_.uncertainty_sigma = std::max(0.0, config_.uncertainty_sigma);
}

CruiseControllerOutput CruiseLongitudinalController::update(
  const CruiseControllerInput & input)
{
  CruiseControllerOutput output;
  const double dt = std::clamp(input.dt, 1e-3, 0.2);
  const double ego_speed = std::max(0.0, input.ego_speed);
  const double opponent_speed = std::max(0.0, input.opponent_speed);
  const double sigma_s = std::sqrt(std::max(0.0, input.opponent_s_variance));

  output.effective_gap = std::max(
    0.0, input.gap - config_.uncertainty_sigma * sigma_s);
  output.gap_error = output.effective_gap - std::max(0.0, input.desired_gap);
  output.relative_speed = opponent_speed - ego_speed;

  gap_integral_ = std::clamp(
    gap_integral_ + output.gap_error * dt,
    -config_.integral_limit, config_.integral_limit);
  output.gap_integral = gap_integral_;

  if (output.effective_gap <= config_.emergency_stop_distance) {
    output.speed_limit = 0.0;
    return output;
  }

  const double feedback_speed =
    opponent_speed +
    config_.proportional_gain * output.gap_error +
    config_.integral_gain * gap_integral_ +
    config_.derivative_gain * output.relative_speed;

  // If the opponent were to brake now, this cap leaves enough relative stopping distance before
  // the emergency boundary. It complements the gap feedback during high closing-speed approaches.
  const double usable_gap =
    std::max(0.0, output.effective_gap - config_.emergency_stop_distance);
  const double braking_speed = std::sqrt(
    opponent_speed * opponent_speed +
    2.0 * config_.relative_deceleration * usable_gap);

  output.speed_limit = std::clamp(
    std::min(feedback_speed, braking_speed), 0.0, config_.maximum_speed);
  if (!config_.allow_acceleration) {
    output.speed_limit = std::min(output.speed_limit, ego_speed);
  }
  return output;
}

void CruiseLongitudinalController::reset()
{
  gap_integral_ = 0.0;
}

}  // namespace f1tenth_control
