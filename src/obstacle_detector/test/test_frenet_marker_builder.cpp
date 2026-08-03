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

#include <cmath>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include "obstacle_detector/frenet_marker_builder.hpp"
#include "obstacle_detector/frenet_projector.hpp"

namespace obstacle_detector
{
namespace
{

constexpr double kTolerance = 1.0e-9;

// 직선 구간과 마지막-첫점 폐곡선 구간을 함께 시험하는 projector를 만든다.
FrenetProjector makeStraightProjector()
{
  FrenetProjector projector;
  projector.build(
      {
        {0.0, 0.0, 0.0, 2.0, 2.0},
        {10.0, 0.0, 10.0, 2.0, 2.0},
        {20.0, 0.0, 20.0, 2.0, 2.0},
    },
    true);
  return projector;
}

// 일반 d 오프셋과 마지막 waypoint 이후의 폐곡선 보간을 모두 검증한다.
TEST(FrenetProjector, ConvertsOffsetAndClosingSegmentsToCartesian)
{
  auto projector = makeStraightProjector();
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  ASSERT_TRUE(projector.toCartesian(5.0, 0.4, x, y, yaw));
  EXPECT_NEAR(x, 5.0, kTolerance);
  EXPECT_NEAR(y, 0.4, kTolerance);
  EXPECT_NEAR(yaw, 0.0, kTolerance);

  FrenetProjector square;
  square.build(
      {
        {0.0, 0.0, 0.0, 2.0, 2.0},
        {1.0, 0.0, 1.0, 2.0, 2.0},
        {1.0, 1.0, 2.0, 2.0, 2.0},
        {0.0, 1.0, 3.0, 2.0, 2.0},
    },
    true);
  ASSERT_TRUE(square.toCartesian(3.5, 0.2, x, y, yaw));
  EXPECT_NEAR(x, 0.2, kTolerance);
  EXPECT_NEAR(y, 0.5, kTolerance);
  EXPECT_NEAR(yaw, -0.5 * std::acos(-1.0), kTolerance);
}

// Cartesian 메타데이터가 없는 예측 트랙도 최종 Frenet 외곽만으로 그릴 수 있어야 한다.
TEST(FrenetMarkerBuilder, DrawsPublishedFrenetEnvelopeWithoutCartesianMetadata)
{
  const auto projector = makeStraightProjector();
  f110_msgs::msg::ObstacleArray obstacles;
  obstacles.header.frame_id = "map";

  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = 7;
  obstacle.has_cartesian = false;
  obstacle.is_visible = false;
  obstacle.s_start = 4.0;
  obstacle.s_end = 6.0;
  obstacle.d_right = -0.2;
  obstacle.d_left = 0.3;
  obstacles.obstacles.push_back(obstacle);

  const auto markers = buildFrenetObstacleMarkers(
    obstacles, projector, "static_obs_frenet", 0.2F, 0.6F, 1.0F);

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(
    markers.markers.front().action,
    visualization_msgs::msg::Marker::DELETEALL);
  const auto & marker = markers.markers.back();
  EXPECT_EQ(marker.ns, "static_obs_frenet");
  EXPECT_EQ(marker.id, 7);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_FLOAT_EQ(marker.color.a, 0.45F);
  ASSERT_EQ(marker.points.size(), 19U);
  EXPECT_NEAR(marker.points.front().x, 4.0, kTolerance);
  EXPECT_NEAR(marker.points.front().y, -0.2, kTolerance);
  EXPECT_NEAR(marker.points[8].x, 6.0, kTolerance);
  EXPECT_NEAR(marker.points[8].y, -0.2, kTolerance);
  EXPECT_NEAR(marker.points[9].x, 6.0, kTolerance);
  EXPECT_NEAR(marker.points[9].y, 0.3, kTolerance);
  EXPECT_NEAR(marker.points.back().x, marker.points.front().x, kTolerance);
  EXPECT_NEAR(marker.points.back().y, marker.points.front().y, kTolerance);
}

// 빈 계층에서도 DELETEALL을 보내 이전 프레임 marker를 지워야 한다.
TEST(FrenetMarkerBuilder, PublishesDeleteAllWhenTheLayerIsEmpty)
{
  const auto projector = makeStraightProjector();
  f110_msgs::msg::ObstacleArray obstacles;
  obstacles.header.frame_id = "map";
  const auto markers = buildFrenetObstacleMarkers(
    obstacles, projector, "static_obs_frenet", 0.2F, 0.6F, 1.0F);
  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(
    markers.markers.front().action,
    visualization_msgs::msg::Marker::DELETEALL);
}

}  // namespace
}  // namespace obstacle_detector
