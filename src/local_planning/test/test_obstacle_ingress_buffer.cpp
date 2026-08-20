// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>

#include "local_planning/detail/obstacle_ingress_buffer.hpp"

namespace local_planning::detail
{
namespace
{

TEST(ObstacleIngressBuffer, KeepsOnlyLatestSnapshotAndItsReceiptTime)
{
  ObstacleIngressBuffer buffer;
  auto first = std::make_shared<f110_msgs::msg::ObstacleArray>();
  auto second = std::make_shared<f110_msgs::msg::ObstacleArray>();
  first->obstacles.resize(1);
  second->obstacles.resize(2);

  EXPECT_EQ(buffer.store(first, rclcpp::Time(10, 0, RCL_ROS_TIME)), 1U);
  EXPECT_EQ(buffer.store(second, rclcpp::Time(12, 345, RCL_ROS_TIME)), 2U);

  const auto snapshot = buffer.latestAfter(0U);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->sequence, 2U);
  EXPECT_EQ(snapshot->message, second);
  EXPECT_EQ(snapshot->receipt_time.nanoseconds(), rclcpp::Time(12, 345).nanoseconds());
  EXPECT_EQ(buffer.latestSequence(), 2U);
}

TEST(ObstacleIngressBuffer, DoesNotReturnAnAlreadyProcessedSnapshot)
{
  ObstacleIngressBuffer buffer;
  auto message = std::make_shared<f110_msgs::msg::ObstacleArray>();

  const auto sequence = buffer.store(message, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(buffer.latestAfter(sequence).has_value());
  EXPECT_FALSE(buffer.latestAfter(sequence + 1U).has_value());
}

}  // namespace
}  // namespace local_planning::detail
