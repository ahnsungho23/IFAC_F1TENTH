#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"

namespace cma_gt_localization
{

enum class TwistMode
{
  kMclCompatible,
  kPassthrough
};

TwistMode parseTwistMode(const std::string & value);

nav_msgs::msg::Odometry convertGroundTruthOdometry(
  const nav_msgs::msg::Odometry & source,
  const std::string & output_frame,
  const std::string & output_child_frame,
  TwistMode twist_mode);

}  // namespace cma_gt_localization

