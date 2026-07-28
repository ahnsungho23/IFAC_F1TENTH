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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "local_planning/local_planner_node.hpp"

using namespace std::chrono_literals;

namespace
{

class FrenetLatticeIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

nav_msgs::msg::OccupancyGrid makeTestMap(const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = stamp;
  map.header.frame_id = "map";
  map.info.resolution = 0.05F;
  map.info.width = 840;
  map.info.height = 120;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = -3.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);

  const auto occupy_rectangle = [&map](
    const double min_x, const double max_x,
    const double min_y, const double max_y)
    {
      const int min_column = std::max(
        0, static_cast<int>(std::floor(
          (min_x - map.info.origin.position.x) / map.info.resolution)));
      const int max_column = std::min(
        static_cast<int>(map.info.width) - 1,
        static_cast<int>(std::floor(
          (max_x - map.info.origin.position.x) / map.info.resolution)));
      const int min_row = std::max(
        0, static_cast<int>(std::floor(
          (min_y - map.info.origin.position.y) / map.info.resolution)));
      const int max_row = std::min(
        static_cast<int>(map.info.height) - 1,
        static_cast<int>(std::floor(
          (max_y - map.info.origin.position.y) / map.info.resolution)));
      for (int row = min_row; row <= max_row; ++row) {
        for (int column = min_column; column <= max_column; ++column) {
          map.data[row * map.info.width + column] = 100;
        }
      }
    };

  // Long components model track walls and are rejected by the component-area
  // classifier. The isolated center component is the static obstacle.
  occupy_rectangle(-0.5, 40.5, 0.95, 1.05);
  occupy_rectangle(-0.5, 40.5, -1.05, -0.95);
  occupy_rectangle(3.8, 4.2, -0.20, 0.20);
  return map;
}

f110_msgs::msg::WpntArray makeStraightGlobalPath(const rclcpp::Time & stamp)
{
  f110_msgs::msg::WpntArray path;
  path.header.stamp = stamp;
  path.header.frame_id = "map";
  // Keep the synthetic loop much longer than the 8 m planning horizon so
  // reaching this obstacle's merge point does not immediately detect the
  // same obstacle as a next-lap target. Alternate waypoint spacing to verify
  // that the Frenet transition is parameterized by physical arc length rather
  // than by the waypoint index.
  constexpr int kWaypointCount = 400;
  path.wpnts.reserve(kWaypointCount);
  double accumulated_s = 0.0;
  for (int i = 0; i < kWaypointCount; ++i) {
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = accumulated_s;
    waypoint.d_m = 0.0;
    waypoint.x_m = waypoint.s_m;
    waypoint.y_m = 0.0;
    waypoint.d_left = 0.90;
    waypoint.d_right = 0.90;
    waypoint.psi_rad = 0.0;
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = 3.0;
    waypoint.ax_mps2 = 0.0;
    path.wpnts.push_back(waypoint);
    accumulated_s += (i % 2 == 0) ? 0.07 : 0.13;
  }
  return path;
}

TEST_F(FrenetLatticeIntegrationTest, PublishesCollisionFreeLatticeAvoidance)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("lookahead_wpnt_num", 180),
      rclcpp::Parameter("detection_lookahead_wpnt_num", 120),
      rclcpp::Parameter("spline_window_margin_wpnts", 70),
      rclcpp::Parameter("vehicle_radius", 0.12),
      rclcpp::Parameter("path_clearance_margin", 0.03),
      rclcpp::Parameter("localization_margin_m", 0.04),
      rclcpp::Parameter("corridor_safety_margin_m", 0.03),
      rclcpp::Parameter("preserve_circular_collision_check", false),
      rclcpp::Parameter("lattice_lateral_samples", 3),
      rclcpp::Parameter("lattice_corridor_target_inset_m", 0.06),
      rclcpp::Parameter("lattice_corridor_beam_width", 8),
      rclcpp::Parameter("lattice_transition_scales", std::vector<double>{1.4, 1.0, 0.75, 1.8}),
      rclcpp::Parameter("lattice_primary_search_budget_ms", 180),
      rclcpp::Parameter("lattice_recovery_search_budget_ms", 240),
      rclcpp::Parameter("obstacle_component_max_area_m2", 0.50),
      rclcpp::Parameter("timer_period_ms", 20)
    });

  auto planner = std::make_shared<local_planning::LocalPlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("frenet_lattice_test_io");
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto global_publisher = io->create_publisher<f110_msgs::msg::WpntArray>(
    "/global_waypoints", latched_qos);
  auto map_publisher = io->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", latched_qos);
  auto odom_publisher = io->create_publisher<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", volatile_qos);

  f110_msgs::msg::OTWpntArray::SharedPtr latest_avoidance;
  std::size_t avoidance_message_count = 0U;
  bool received_empty_avoidance = false;
  auto avoidance_subscription =
    io->create_subscription<f110_msgs::msg::OTWpntArray>(
    "/avoid_waypoints", volatile_qos,
    [&latest_avoidance, &avoidance_message_count, &received_empty_avoidance](
      const f110_msgs::msg::OTWpntArray::SharedPtr message)
    {
      latest_avoidance = message;
      ++avoidance_message_count;
      if (message->wpnts.empty()) {
        received_empty_avoidance = true;
      }
    });
  (void)avoidance_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(io);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto stamp = io->now();
  global_publisher->publish(makeStraightGlobalPath(stamp));
  map_publisher->publish(makeTestMap(stamp));

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 2.0;
  odometry.pose.pose.orientation.w = 1.0;

  // A single out-of-track startup estimate must not create a local path that
  // the controller could chase across the track.
  odom_publisher->publish(odometry);
  const auto invalid_odom_deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < invalid_odom_deadline &&
    (!latest_avoidance || latest_avoidance->ot_line != "invalid_frenet_odom"))
  {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "invalid_frenet_odom");
  EXPECT_TRUE(latest_avoidance->wpnts.empty());

  odometry.pose.pose.position.y = 0.0;

  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < deadline &&
    (!latest_avoidance || latest_avoidance->ot_line != "frenet_lattice_segment"))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_segment");
  EXPECT_TRUE(latest_avoidance->side_switch);
  ASSERT_FALSE(latest_avoidance->wpnts.empty());

  double maximum_lateral_offset = 0.0;
  for (const auto & waypoint : latest_avoidance->wpnts) {
    EXPECT_TRUE(std::isfinite(waypoint.x_m));
    EXPECT_TRUE(std::isfinite(waypoint.y_m));
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_LE(std::abs(waypoint.kappa_radpm), 2.80 + 1e-6);
    maximum_lateral_offset = std::max(maximum_lateral_offset, std::abs(waypoint.d_m));
  }
  EXPECT_GT(maximum_lateral_offset, 0.35);
  for (std::size_t i = 1; i < latest_avoidance->wpnts.size(); ++i) {
    const auto & previous = latest_avoidance->wpnts[i - 1U];
    const auto & current = latest_avoidance->wpnts[i];
    const double ds = std::max(
      1e-3, std::hypot(current.x_m - previous.x_m, current.y_m - previous.y_m));
    const double curvature_rate =
      std::abs(current.kappa_radpm - previous.kappa_radpm) / ds;
    EXPECT_LE(curvature_rate, 18.0 + 1e-6);
  }

  bool found_avoidance_offset = false;
  std::size_t last_offset_index = 0U;
  std::size_t last_plateau_index = 0U;
  double maximum_plateau_speed = 0.0;
  for (std::size_t i = 0; i < latest_avoidance->wpnts.size(); ++i) {
    const auto & waypoint = latest_avoidance->wpnts[i];
    if (std::abs(waypoint.d_m) > 1e-3) {
      found_avoidance_offset = true;
      last_offset_index = i;
    }
    if (std::abs(waypoint.d_m) >= 0.95 * maximum_lateral_offset) {
      last_plateau_index = i;
      maximum_plateau_speed = std::max(maximum_plateau_speed, waypoint.vx_mps);
    }
  }
  ASSERT_TRUE(found_avoidance_offset);
  EXPECT_GE(latest_avoidance->wpnts.size() - last_offset_index - 1U, 10U);

  double maximum_recovery_speed = 0.0;
  for (std::size_t i = last_plateau_index + 1U; i <= last_offset_index; ++i) {
    maximum_recovery_speed = std::max(
      maximum_recovery_speed, latest_avoidance->wpnts[i].vx_mps);
  }
  EXPECT_GT(maximum_recovery_speed, maximum_plateau_speed);

  for (std::size_t i = 0; i + 1U < latest_avoidance->wpnts.size(); ++i) {
    const auto & waypoint = latest_avoidance->wpnts[i];
    EXPECT_LE(waypoint.ax_mps2, 3.0 + 1e-6);
    EXPECT_GE(waypoint.ax_mps2, -6.0 - 1e-6);
  }

  // A short Frenet-odometry gap must not clear a previously collision-checked
  // path. This covers localization jitter without allowing an unbounded stale
  // path to remain active.
  const auto first_avoidance = *latest_avoidance;
  latest_avoidance.reset();
  received_empty_avoidance = false;
  const auto hold_deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < hold_deadline &&
    (!latest_avoidance ||
    latest_avoidance->ot_line != "frenet_lattice_segment_held"))
  {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_segment_held");
  EXPECT_FALSE(latest_avoidance->wpnts.empty()) << latest_avoidance->ot_line;
  EXPECT_FALSE(received_empty_avoidance) << latest_avoidance->ot_line;

  latest_avoidance.reset();
  const auto recovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < recovery_deadline &&
    (!latest_avoidance ||
    latest_avoidance->ot_line != "frenet_lattice_segment"))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_segment");

  std::size_t exact_last_offset_index = 0U;
  for (std::size_t i = 0; i < first_avoidance.wpnts.size(); ++i) {
    if (std::abs(first_avoidance.wpnts[i].d_m) > 1e-9) {
      exact_last_offset_index = i;
    }
  }

  // Moving the planning horizon must not generate a different lateral path
  // for the same static obstacle. The committed path is cropped from the new
  // ego index while overlapping waypoint geometry remains unchanged.
  const std::size_t first_message_count = avoidance_message_count;
  for (const double intermediate_s : {0.5, 1.0}) {
    odometry.pose.pose.position.x = intermediate_s;
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(30ms);
  }
  odometry.pose.pose.position.x = 1.5;
  latest_avoidance.reset();
  const auto commitment_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < commitment_deadline &&
    (!latest_avoidance || avoidance_message_count <= first_message_count ||
    latest_avoidance->wpnts.empty() || latest_avoidance->wpnts.front().id < 14))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(latest_avoidance, nullptr);
  ASSERT_FALSE(latest_avoidance->wpnts.empty());
  EXPECT_EQ(latest_avoidance->ot_side, first_avoidance.ot_side);
  EXPECT_EQ(latest_avoidance->ot_line, first_avoidance.ot_line);
  int overlap_count = 0;
  for (const auto & waypoint : latest_avoidance->wpnts) {
    const auto previous = std::find_if(
      first_avoidance.wpnts.begin(), first_avoidance.wpnts.end(),
      [&waypoint](const auto & value) {return value.id == waypoint.id;});
    if (previous == first_avoidance.wpnts.end()) {
      continue;
    }
    EXPECT_NEAR(waypoint.d_m, previous->d_m, 1e-9);
    EXPECT_NEAR(waypoint.x_m, previous->x_m, 1e-9);
    EXPECT_NEAR(waypoint.y_m, previous->y_m, 1e-9);
    ++overlap_count;
  }
  EXPECT_GT(overlap_count, 20);

  const double merge_s = first_avoidance.wpnts[exact_last_offset_index + 1U].s_m;
  odometry.pose.pose.position.y = 0.30;
  for (double intermediate_s = 2.0; intermediate_s + 0.5 < merge_s;
    intermediate_s += 0.5)
  {
    odometry.pose.pose.position.x = intermediate_s;
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(30ms);
  }
  odometry.pose.pose.position.x = merge_s;
  latest_avoidance.reset();
  received_empty_avoidance = false;
  const auto settle_deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < settle_deadline &&
    (!latest_avoidance || latest_avoidance->wpnts.empty()))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_FALSE(latest_avoidance->wpnts.empty()) << latest_avoidance->ot_line;
  EXPECT_FALSE(received_empty_avoidance) << latest_avoidance->ot_line;

  // The planner may clear the avoidance source only after the real vehicle,
  // not just the geometric path, has settled close to the global d=0 line.
  odometry.pose.pose.position.y = 0.0;
  latest_avoidance.reset();
  received_empty_avoidance = false;
  const auto merge_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < merge_deadline &&
    !received_empty_avoidance)
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(received_empty_avoidance);

  executor.remove_node(io);
  executor.remove_node(planner);
}

TEST_F(FrenetLatticeIntegrationTest, PublishesSafeStopWhenEveryLatticeCandidateFails)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("lookahead_wpnt_num", 80),
      rclcpp::Parameter("detection_lookahead_wpnt_num", 70),
      rclcpp::Parameter("obstacle_component_max_area_m2", 0.50),
      rclcpp::Parameter("lattice_lateral_samples", 1),
      rclcpp::Parameter("lattice_transition_scales", std::vector<double>{0.55}),
      rclcpp::Parameter("lattice_recovery_lateral_samples", 1),
      rclcpp::Parameter(
        "lattice_recovery_transition_scales", std::vector<double>{0.55}),
      rclcpp::Parameter("lattice_recovery_min_transition_wpnts", 10),
      rclcpp::Parameter("lattice_recovery_max_curvature_scale", 1.0),
      rclcpp::Parameter("lattice_max_curvature_radpm", 0.10),
      rclcpp::Parameter("lattice_safe_stop_buffer_wpnts", 8),
      rclcpp::Parameter("timer_period_ms", 20),
      rclcpp::Parameter("global_waypoints_topic", "/safe_stop/global_waypoints"),
      rclcpp::Parameter("map_topic", "/safe_stop/map"),
      rclcpp::Parameter("frenet_odom_topic", "/safe_stop/frenet_odom"),
      rclcpp::Parameter("ot_waypoints_topic", "/safe_stop/avoid_waypoints"),
      rclcpp::Parameter("local_path_topic", "/safe_stop/local_path"),
      rclcpp::Parameter("exact_local_path_topic", "/safe_stop/exact_local_path"),
      rclcpp::Parameter("marker_topic", "/safe_stop/markers")
    });

  auto planner = std::make_shared<local_planning::LocalPlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("frenet_lattice_safe_stop_test_io");
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto global_publisher = io->create_publisher<f110_msgs::msg::WpntArray>(
    "/safe_stop/global_waypoints", latched_qos);
  auto map_publisher = io->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/safe_stop/map", latched_qos);
  auto odom_publisher = io->create_publisher<nav_msgs::msg::Odometry>(
    "/safe_stop/frenet_odom", volatile_qos);

  f110_msgs::msg::OTWpntArray::SharedPtr latest_avoidance;
  auto avoidance_subscription =
    io->create_subscription<f110_msgs::msg::OTWpntArray>(
    "/safe_stop/avoid_waypoints", volatile_qos,
    [&latest_avoidance](const f110_msgs::msg::OTWpntArray::SharedPtr message)
    {
      latest_avoidance = message;
    });
  (void)avoidance_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(io);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto stamp = io->now();
  global_publisher->publish(makeStraightGlobalPath(stamp));
  map_publisher->publish(makeTestMap(stamp));

  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.pose.pose.orientation.w = 1.0;

  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < deadline &&
    (!latest_avoidance || latest_avoidance->ot_line != "frenet_lattice_safe_stop"))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_safe_stop");
  EXPECT_TRUE(latest_avoidance->side_switch);
  ASSERT_GE(latest_avoidance->wpnts.size(), 2U);
  EXPECT_GT(latest_avoidance->wpnts.front().vx_mps, 0.0);
  EXPECT_DOUBLE_EQ(latest_avoidance->wpnts.back().vx_mps, 0.0);
  for (std::size_t i = 0; i < latest_avoidance->wpnts.size(); ++i) {
    EXPECT_LE(latest_avoidance->wpnts[i].vx_mps, 3.0);
    EXPECT_LE(latest_avoidance->wpnts[i].ax_mps2, 0.0);
    if (i > 0U) {
      EXPECT_LE(
        latest_avoidance->wpnts[i].vx_mps,
        latest_avoidance->wpnts[i - 1U].vx_mps + 1e-9);
    }
  }
  EXPECT_LT(latest_avoidance->wpnts.back().x_m, 3.5);

  executor.remove_node(io);
  executor.remove_node(planner);
}

TEST_F(FrenetLatticeIntegrationTest, BrakesOnLastValidatedPathDuringReplanGap)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("lookahead_wpnt_num", 80),
      rclcpp::Parameter("detection_lookahead_wpnt_num", 70),
      rclcpp::Parameter("obstacle_component_max_area_m2", 0.50),
      rclcpp::Parameter("lattice_commit_path_until_clear", false),
      rclcpp::Parameter("lattice_replan_brake_timeout_sec", 2.0),
      rclcpp::Parameter("timer_period_ms", 20),
      rclcpp::Parameter("global_waypoints_topic", "/replan_gap/global_waypoints"),
      rclcpp::Parameter("map_topic", "/replan_gap/map"),
      rclcpp::Parameter("frenet_odom_topic", "/replan_gap/frenet_odom"),
      rclcpp::Parameter("ot_waypoints_topic", "/replan_gap/avoid_waypoints"),
      rclcpp::Parameter("local_path_topic", "/replan_gap/local_path"),
      rclcpp::Parameter("exact_local_path_topic", "/replan_gap/exact_local_path"),
      rclcpp::Parameter("marker_topic", "/replan_gap/markers")
    });

  auto planner = std::make_shared<local_planning::LocalPlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("frenet_lattice_replan_gap_test_io");
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto global_publisher = io->create_publisher<f110_msgs::msg::WpntArray>(
    "/replan_gap/global_waypoints", latched_qos);
  auto map_publisher = io->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/replan_gap/map", latched_qos);
  auto odom_publisher = io->create_publisher<nav_msgs::msg::Odometry>(
    "/replan_gap/frenet_odom", volatile_qos);

  f110_msgs::msg::OTWpntArray::SharedPtr latest_avoidance;
  bool received_empty_during_handoff = false;
  bool watch_handoff = false;
  auto avoidance_subscription =
    io->create_subscription<f110_msgs::msg::OTWpntArray>(
    "/replan_gap/avoid_waypoints", volatile_qos,
    [&latest_avoidance, &received_empty_during_handoff, &watch_handoff](
      const f110_msgs::msg::OTWpntArray::SharedPtr message)
    {
      latest_avoidance = message;
      if (watch_handoff && message->wpnts.empty()) {
        received_empty_during_handoff = true;
      }
    });
  (void)avoidance_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(io);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto stamp = io->now();
  global_publisher->publish(makeStraightGlobalPath(stamp));
  map_publisher->publish(makeTestMap(stamp));
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.y = 0.0;
  odometry.pose.pose.orientation.w = 1.0;

  const auto initial_deadline = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < initial_deadline &&
    (!latest_avoidance || latest_avoidance->ot_line != "frenet_lattice_segment"))
  {
    odometry.header.stamp = io->now();
    odometry.pose.pose.position.x = 0.0;
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(latest_avoidance, nullptr);
  ASSERT_EQ(latest_avoidance->ot_line, "frenet_lattice_segment");
  ASSERT_FALSE(latest_avoidance->wpnts.empty());

  watch_handoff = true;
  latest_avoidance.reset();
  for (const double s : {0.8, 1.6, 2.4, 3.2, 3.6}) {
    odometry.pose.pose.position.x = s;
    for (int repeat = 0; repeat < 3; ++repeat) {
      odometry.header.stamp = io->now();
      odom_publisher->publish(odometry);
      executor.spin_some();
      std::this_thread::sleep_for(25ms);
    }
  }

  const auto handoff_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < handoff_deadline &&
    (!latest_avoidance ||
    latest_avoidance->ot_line != "frenet_lattice_replan_brake"))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_replan_brake");
  EXPECT_FALSE(received_empty_during_handoff);
  ASSERT_GE(latest_avoidance->wpnts.size(), 2U);
  EXPECT_DOUBLE_EQ(latest_avoidance->wpnts.back().vx_mps, 0.0);
  for (std::size_t i = 1; i < latest_avoidance->wpnts.size(); ++i) {
    EXPECT_LE(
      latest_avoidance->wpnts[i].vx_mps,
      latest_avoidance->wpnts[i - 1U].vx_mps + 1e-9);
    EXPECT_LE(latest_avoidance->wpnts[i - 1U].ax_mps2, 1e-9);
  }

  executor.remove_node(io);
  executor.remove_node(planner);
}

TEST_F(FrenetLatticeIntegrationTest, UsesRecoveryLatticeBeforeSafeStop)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("lookahead_wpnt_num", 80),
      rclcpp::Parameter("detection_lookahead_wpnt_num", 70),
      rclcpp::Parameter("obstacle_component_max_area_m2", 0.50),
      rclcpp::Parameter("lattice_lateral_samples", 1),
      rclcpp::Parameter("lattice_transition_scales", std::vector<double>{0.55}),
      rclcpp::Parameter("lattice_max_curvature_radpm", 0.10),
      rclcpp::Parameter("lattice_recovery_lateral_samples", 7),
      rclcpp::Parameter(
        "lattice_recovery_transition_scales", std::vector<double>{1.0, 1.4}),
      rclcpp::Parameter("lattice_recovery_max_curvature_scale", 50.0),
      rclcpp::Parameter("timer_period_ms", 20),
      rclcpp::Parameter("global_waypoints_topic", "/recovery/global_waypoints"),
      rclcpp::Parameter("map_topic", "/recovery/map"),
      rclcpp::Parameter("frenet_odom_topic", "/recovery/frenet_odom"),
      rclcpp::Parameter("ot_waypoints_topic", "/recovery/avoid_waypoints"),
      rclcpp::Parameter("local_path_topic", "/recovery/local_path"),
      rclcpp::Parameter("exact_local_path_topic", "/recovery/exact_local_path"),
      rclcpp::Parameter("marker_topic", "/recovery/markers")
    });

  auto planner = std::make_shared<local_planning::LocalPlannerNode>(options);
  auto io = std::make_shared<rclcpp::Node>("frenet_lattice_recovery_test_io");
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto global_publisher = io->create_publisher<f110_msgs::msg::WpntArray>(
    "/recovery/global_waypoints", latched_qos);
  auto map_publisher = io->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/recovery/map", latched_qos);
  auto odom_publisher = io->create_publisher<nav_msgs::msg::Odometry>(
    "/recovery/frenet_odom", volatile_qos);

  f110_msgs::msg::OTWpntArray::SharedPtr latest_avoidance;
  auto avoidance_subscription =
    io->create_subscription<f110_msgs::msg::OTWpntArray>(
    "/recovery/avoid_waypoints", volatile_qos,
    [&latest_avoidance](const f110_msgs::msg::OTWpntArray::SharedPtr message)
    {
      latest_avoidance = message;
    });
  (void)avoidance_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(io);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto stamp = io->now();
  global_publisher->publish(makeStraightGlobalPath(stamp));
  map_publisher->publish(makeTestMap(stamp));

  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.pose.pose.orientation.w = 1.0;

  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < deadline &&
    (!latest_avoidance || latest_avoidance->ot_line != "frenet_lattice_recovery"))
  {
    odometry.header.stamp = io->now();
    odom_publisher->publish(odometry);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(latest_avoidance, nullptr);
  EXPECT_EQ(latest_avoidance->ot_line, "frenet_lattice_recovery");
  ASSERT_FALSE(latest_avoidance->wpnts.empty());
  double maximum_lateral_offset = 0.0;
  for (const auto & waypoint : latest_avoidance->wpnts) {
    maximum_lateral_offset = std::max(
      maximum_lateral_offset, std::abs(waypoint.d_m));
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_GT(waypoint.vx_mps, 0.0);
  }
  EXPECT_GT(maximum_lateral_offset, 0.35);

  executor.remove_node(io);
  executor.remove_node(planner);
}

}  // namespace
