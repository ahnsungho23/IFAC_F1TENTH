#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "static_obstacle_map/obstacle_marker_builder.hpp"
#include "static_obstacle_map/static_obstacle_map_memory.hpp"

namespace static_obstacle_map
{
namespace
{

nav_msgs::msg::OccupancyGrid makeMap(
  unsigned int width = 100U, unsigned int height = 100U, double resolution = 0.1)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
  return map;
}

f110_msgs::msg::Obstacle makeStatic(
  int id, double x_min, double x_max, double y_min, double y_max)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.is_static = true;
  obstacle.is_visible = true;
  obstacle.has_cartesian = true;
  obstacle.x_min = x_min;
  obstacle.x_max = x_max;
  obstacle.y_min = y_min;
  obstacle.y_max = y_max;
  obstacle.x_center = 0.5 * (x_min + x_max);
  obstacle.y_center = 0.5 * (y_min + y_max);
  return obstacle;
}

f110_msgs::msg::ObstacleArray arrayWith(const f110_msgs::msg::Obstacle & obstacle)
{
  f110_msgs::msg::ObstacleArray message;
  message.header.frame_id = "map";
  message.obstacles.push_back(obstacle);
  return message;
}

std::size_t mapIndex(
  const nav_msgs::msg::OccupancyGrid & map, unsigned int x, unsigned int y)
{
  return static_cast<std::size_t>(y) * map.info.width + x;
}

TEST(StaticObstacleMapMemory, ClampsEveryInsertionAndUpdateToMaximumDiagonal)
{
  MemoryConfig config;
  config.association_distance_m = 10.0;
  config.edge_confirm_frames = 2;
  config.max_obstacle_diagonal_m = 0.8;
  StaticObstacleMapMemory memory(config);

  auto stats = memory.updateConfirmed(arrayWith(makeStatic(7, -1.0, 1.0, -1.0, 1.0)));
  ASSERT_EQ(stats.inserted, 1U);
  ASSERT_EQ(memory.obstacles().size(), 1U);

  for (int iteration = 0; iteration < 100; ++iteration) {
    const double width = iteration % 2 == 0 ? 4.0 : 0.2;
    const double height = iteration % 2 == 0 ? 0.2 : 4.0;
    stats = memory.updateConfirmed(
      arrayWith(makeStatic(7, -0.5 * width, 0.5 * width, -0.5 * height, 0.5 * height)));
    ASSERT_EQ(stats.updated, 1U);
    ASSERT_EQ(memory.obstacles().size(), 1U);
    const auto & stored = memory.obstacles().front();
    const double diagonal =
      std::hypot(stored.x_max - stored.x_min, stored.y_max - stored.y_min);
    EXPECT_LE(diagonal, config.max_obstacle_diagonal_m + 1.0e-12);
  }
}

TEST(StaticObstacleMapMemory, PreservesFrontCellsAndAddsConsecutivelyObservedGeometry)
{
  MemoryConfig config;
  config.association_distance_m = 0.30;
  config.edge_confirm_frames = 2;
  config.edge_match_tolerance_m = 0.05;
  StaticObstacleMapMemory memory(config);
  auto base_map = makeMap();
  base_map.data[mapIndex(base_map, 0U, 0U)] = 100;
  ASSERT_TRUE(memory.setBaseMap(base_map).accepted);

  memory.updateConfirmed(arrayWith(makeStatic(3, 1.0, 1.2, 1.0, 1.2)));
  auto first = memory.composeMap();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->data[mapIndex(*first, 11U, 11U)], 100);
  EXPECT_EQ(first->data[mapIndex(*first, 0U, 0U)], 100);

  auto update =
    memory.updateConfirmed(arrayWith(makeStatic(3, 1.1, 1.4, 1.0, 1.2)));
  ASSERT_EQ(update.updated, 1U);
  EXPECT_EQ(update.geometry_expanded, 0U);
  update = memory.updateConfirmed(arrayWith(makeStatic(3, 1.1, 1.4, 1.0, 1.2)));
  ASSERT_EQ(update.updated, 1U);
  EXPECT_EQ(update.geometry_expanded, 1U);
  auto second = memory.composeMap();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->data[mapIndex(*second, 11U, 11U)], 100);
  EXPECT_EQ(second->data[mapIndex(*second, 13U, 11U)], 100);
  EXPECT_EQ(second->data[mapIndex(*second, 0U, 0U)], 100);
}

TEST(StaticObstacleMapMemory, RequiresConfiguredConsecutiveSupportForANewEdge)
{
  MemoryConfig config;
  config.edge_confirm_frames = 3;
  config.edge_match_tolerance_m = 0.05;
  StaticObstacleMapMemory memory(config);
  memory.updateConfirmed(arrayWith(makeStatic(9, 1.0, 1.2, 1.0, 1.2)));

  for (int observation = 1; observation <= 2; ++observation) {
    const auto stats =
      memory.updateConfirmed(arrayWith(makeStatic(9, 1.0, 1.4, 1.0, 1.2)));
    EXPECT_EQ(stats.geometry_expanded, 0U);
    ASSERT_EQ(memory.obstacles().size(), 1U);
    EXPECT_DOUBLE_EQ(memory.obstacles().front().x_max, 1.2);
  }

  const auto stats =
    memory.updateConfirmed(arrayWith(makeStatic(9, 1.0, 1.4, 1.0, 1.2)));
  EXPECT_EQ(stats.geometry_expanded, 1U);
  ASSERT_EQ(memory.obstacles().size(), 1U);
  EXPECT_DOUBLE_EQ(memory.obstacles().front().x_min, 1.0);
  EXPECT_DOUBLE_EQ(memory.obstacles().front().x_max, 1.4);
}

TEST(StaticObstacleMapMemory, RejectsOneScanOutwardEdgeOutlier)
{
  MemoryConfig config;
  config.edge_confirm_frames = 3;
  config.edge_match_tolerance_m = 0.05;
  StaticObstacleMapMemory memory(config);
  memory.updateConfirmed(arrayWith(makeStatic(10, 1.0, 1.2, 1.0, 1.2)));

  auto stats =
    memory.updateConfirmed(arrayWith(makeStatic(10, 1.0, 1.6, 1.0, 1.2)));
  EXPECT_EQ(stats.geometry_expanded, 0U);
  stats = memory.updateConfirmed(arrayWith(makeStatic(10, 1.0, 1.2, 1.0, 1.2)));
  EXPECT_EQ(stats.geometry_expanded, 0U);

  ASSERT_EQ(memory.obstacles().size(), 1U);
  EXPECT_DOUBLE_EQ(memory.obstacles().front().x_min, 1.0);
  EXPECT_DOUBLE_EQ(memory.obstacles().front().x_max, 1.2);
}

TEST(StaticObstacleMapMemory, RejectsOversizedUnionWithoutMovingStoredFootprint)
{
  MemoryConfig config;
  config.edge_confirm_frames = 2;
  config.edge_match_tolerance_m = 0.05;
  config.max_obstacle_diagonal_m = 0.5;
  StaticObstacleMapMemory memory(config);
  memory.updateConfirmed(arrayWith(makeStatic(12, 1.0, 1.2, 1.0, 1.2)));

  memory.updateConfirmed(arrayWith(makeStatic(12, 1.1, 1.6, 1.0, 1.2)));
  const auto stats =
    memory.updateConfirmed(arrayWith(makeStatic(12, 1.1, 1.6, 1.0, 1.2)));

  EXPECT_EQ(stats.geometry_expanded, 0U);
  EXPECT_EQ(stats.limit_rejected, 1U);
  ASSERT_EQ(memory.obstacles().size(), 1U);
  const auto & stored = memory.obstacles().front();
  EXPECT_DOUBLE_EQ(stored.x_min, 1.0);
  EXPECT_DOUBLE_EQ(stored.x_max, 1.2);
  EXPECT_DOUBLE_EQ(stored.y_min, 1.0);
  EXPECT_DOUBLE_EQ(stored.y_max, 1.2);
}

TEST(StaticObstacleMapMemory, KeepsConfirmedObjectsAcrossEmptyMessages)
{
  StaticObstacleMapMemory memory;
  memory.updateConfirmed(arrayWith(makeStatic(11, 2.0, 2.2, 3.0, 3.2)));
  ASSERT_EQ(memory.obstacles().size(), 1U);

  f110_msgs::msg::ObstacleArray empty;
  const auto stats = memory.updateConfirmed(empty);
  EXPECT_EQ(stats.inserted, 0U);
  EXPECT_EQ(stats.updated, 0U);
  EXPECT_EQ(stats.removed, 0U);
  EXPECT_EQ(memory.obstacles().size(), 1U);
}

TEST(StaticObstacleMapMemory, RemovesOnlyExplicitSameTrackDynamicReclassification)
{
  StaticObstacleMapMemory memory;
  memory.updateConfirmed(arrayWith(makeStatic(4, 1.0, 1.2, 1.0, 1.2)));
  memory.updateConfirmed(arrayWith(makeStatic(5, 2.0, 2.2, 2.0, 2.2)));
  ASSERT_EQ(memory.obstacles().size(), 2U);

  auto dynamic = makeStatic(4, 1.0, 1.2, 1.0, 1.2);
  dynamic.is_static = false;
  const auto stats = memory.removeDynamic(arrayWith(dynamic));
  EXPECT_EQ(stats.removed, 1U);
  ASSERT_EQ(memory.obstacles().size(), 1U);
  EXPECT_EQ(memory.obstacles().front().source_id, 5);
}

TEST(StaticObstacleMapMemory, RejectsObjectsWithoutCurrentStaticCartesianGeometry)
{
  StaticObstacleMapMemory memory;
  auto invalid = makeStatic(8, 1.0, 1.2, 1.0, 1.2);
  invalid.has_cartesian = false;
  auto stats = memory.updateConfirmed(arrayWith(invalid));
  EXPECT_EQ(stats.rejected, 1U);

  invalid.has_cartesian = true;
  invalid.is_visible = false;
  stats = memory.updateConfirmed(arrayWith(invalid));
  EXPECT_EQ(stats.rejected, 1U);

  invalid.is_visible = true;
  invalid.is_static = false;
  stats = memory.updateConfirmed(arrayWith(invalid));
  EXPECT_EQ(stats.rejected, 1U);
  EXPECT_TRUE(memory.obstacles().empty());
}

TEST(StaticObstacleMapMemory, ClearsMemoryWhenBaseMapGeometryChanges)
{
  StaticObstacleMapMemory memory;
  ASSERT_TRUE(memory.setBaseMap(makeMap()).accepted);
  memory.updateConfirmed(arrayWith(makeStatic(2, 1.0, 1.2, 1.0, 1.2)));
  ASSERT_EQ(memory.obstacles().size(), 1U);

  auto changed = makeMap(80U, 100U, 0.1);
  const BaseMapUpdate update = memory.setBaseMap(changed);
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.geometry_changed);
  EXPECT_EQ(update.cleared_obstacles, 1U);
  EXPECT_TRUE(memory.obstacles().empty());
}

TEST(StaticObstacleMapMemory, RasterizesWithRotatedMapOrigin)
{
  StaticObstacleMapMemory memory;
  auto map = makeMap(10U, 10U, 1.0);
  constexpr double kHalfSqrtTwo = 0.7071067811865476;
  map.info.origin.orientation.z = kHalfSqrtTwo;
  map.info.origin.orientation.w = kHalfSqrtTwo;
  ASSERT_TRUE(memory.setBaseMap(map).accepted);

  // Grid cell (1,1) has local centre (1.5,1.5), which rotates to world (-1.5,1.5).
  memory.updateConfirmed(arrayWith(makeStatic(6, -1.6, -1.4, 1.4, 1.6)));
  const auto output = memory.composeMap();
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->data[mapIndex(*output, 1U, 1U)], 100);
}

TEST(ObstacleMarkerBuilder, PublishesDeleteAllAndInflatedPersistentCubes)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 12;

  StoredObstacle first;
  first.memory_id = 4;
  first.x_min = 1.0;
  first.x_max = 1.2;
  first.y_min = 2.0;
  first.y_max = 2.4;

  ObstacleMarkerConfig config;
  config.marker_namespace = "test_static";
  config.height_m = 0.2;
  config.red = 0.8F;
  config.green = 0.2F;
  config.blue = 0.1F;
  config.alpha = 0.7F;

  const auto markers = buildObstacleMarkers(
    header, {first}, 0.1, 0.05, config);
  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);

  const auto & marker = markers.markers[1];
  EXPECT_EQ(marker.header.frame_id, "map");
  EXPECT_EQ(marker.header.stamp.sec, 12);
  EXPECT_EQ(marker.ns, "test_static");
  EXPECT_EQ(marker.id, 4);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_NEAR(marker.pose.position.x, 1.1, 1.0e-12);
  EXPECT_NEAR(marker.pose.position.y, 2.2, 1.0e-12);
  EXPECT_NEAR(marker.pose.position.z, 0.1, 1.0e-12);
  EXPECT_NEAR(marker.scale.x, 0.4, 1.0e-12);
  EXPECT_NEAR(marker.scale.y, 0.6, 1.0e-12);
  EXPECT_NEAR(marker.scale.z, 0.2, 1.0e-12);
  EXPECT_FLOAT_EQ(marker.color.r, 0.8F);
  EXPECT_FLOAT_EQ(marker.color.g, 0.2F);
  EXPECT_FLOAT_EQ(marker.color.b, 0.1F);
  EXPECT_FLOAT_EQ(marker.color.a, 0.7F);
}

TEST(ObstacleMarkerBuilder, EmptyMemoryStillClearsPreviousRvizMarkers)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const auto markers = buildObstacleMarkers(
    header, {}, 0.0, 0.025, ObstacleMarkerConfig{});
  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
}

}  // namespace
}  // namespace static_obstacle_map
