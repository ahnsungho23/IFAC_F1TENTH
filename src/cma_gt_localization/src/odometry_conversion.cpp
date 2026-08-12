#include "cma_gt_localization/odometry_conversion.hpp"

#include <algorithm>
#include <stdexcept>

namespace cma_gt_localization
{

TwistMode parseTwistMode(const std::string & value)
{
  if (value == "mcl_compatible") {
    return TwistMode::kMclCompatible;
  }
  if (value == "passthrough") {
    return TwistMode::kPassthrough;
  }
  throw std::invalid_argument(
          "twist_mode must be 'mcl_compatible' or 'passthrough', got: " + value);
}

nav_msgs::msg::Odometry convertGroundTruthOdometry(
  const nav_msgs::msg::Odometry & source,
  const std::string & output_frame,
  const std::string & output_child_frame,
  const TwistMode twist_mode)
{
  nav_msgs::msg::Odometry output;
  output.header = source.header;
  output.header.frame_id = output_frame;
  output.child_frame_id = output_child_frame;
  output.pose.pose = source.pose.pose;

  if (twist_mode == TwistMode::kPassthrough) {
    output.twist.twist = source.twist.twist;
  } else {
    // particle_filter_cpp publishes only current_velocity_ in linear.x on
    // /pf/pose/odom. Preserve that interface so this mode replaces only pose,
    // not downstream velocity semantics.
    output.twist.twist.linear.x = source.twist.twist.linear.x;
  }

  std::fill(output.pose.covariance.begin(), output.pose.covariance.end(), 0.0);
  std::fill(output.twist.covariance.begin(), output.twist.covariance.end(), 0.0);
  return output;
}

}  // namespace cma_gt_localization

