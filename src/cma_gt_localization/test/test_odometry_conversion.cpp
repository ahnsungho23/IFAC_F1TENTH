#include <gtest/gtest.h>

#include <stdexcept>

#include "cma_gt_localization/odometry_conversion.hpp"

namespace
{

nav_msgs::msg::Odometry sample()
{
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 123;
  message.header.stamp.nanosec = 456789;
  message.header.frame_id = "map";
  message.child_frame_id = "ego_racecar/base_link";
  message.pose.pose.position.x = 1.25;
  message.pose.pose.position.y = -2.5;
  message.pose.pose.orientation.z = 0.25;
  message.pose.pose.orientation.w = 0.9682458365518543;
  message.twist.twist.linear.x = 4.0;
  message.twist.twist.linear.y = 0.2;
  message.twist.twist.angular.z = -0.3;
  message.pose.covariance[0] = 9.0;
  message.twist.covariance[0] = 8.0;
  return message;
}

TEST(OdometryConversion, PreservesTimestampAndPoseExactly)
{
  const auto input = sample();
  const auto output = cma_gt_localization::convertGroundTruthOdometry(
    input, "map", "ego_racecar/base_link",
    cma_gt_localization::TwistMode::kMclCompatible);
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_EQ(output.child_frame_id, "ego_racecar/base_link");
  EXPECT_EQ(output.pose.pose, input.pose.pose);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 4.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(output.twist.covariance[0], 0.0);
}

TEST(OdometryConversion, PassthroughCopiesFullTwist)
{
  const auto input = sample();
  const auto output = cma_gt_localization::convertGroundTruthOdometry(
    input, "map", "ego_racecar/base_link",
    cma_gt_localization::TwistMode::kPassthrough);
  EXPECT_EQ(output.twist.twist, input.twist.twist);
}

TEST(OdometryConversion, RejectsUnknownTwistMode)
{
  EXPECT_THROW(cma_gt_localization::parseTwistMode("other"), std::invalid_argument);
}

}  // namespace
