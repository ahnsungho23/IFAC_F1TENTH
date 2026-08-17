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

// Node-level tests: these exercise LocalPlannerNode through its real ROS interfaces, which is the
// only way to cover the subscription callbacks themselves. The algorithm classes have their own
// unit tests; what is verified here is the wiring those tests cannot see -- which incoming
// messages are accepted as authoritative and which are refused.
//
// Observables come from the P3 cycle diagnostic, which reports the accepted snapshot's identity:
//   obstacle_sequence            advances only when an obstacle array is accepted
//   source_stamp_ns              the accepted array's stamp
//   global_reference_generation  advances only when a NEW reference is adopted

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "local_planning/local_planner_node.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    // These node-level tests publish on the same well-known topics as the other
    // package's node test, and colcon runs packages in parallel. Without an isolated
    // domain they cross-talk (a foreign /global_waypoints made this suite flaky).
    setenv("ROS_DOMAIN_ID", "91", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

f110_msgs::msg::WpntArray ringReference(double d_left = 3.0, double d_right = 3.0)
{
  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  constexpr int kCount = 240;
  constexpr double kRadius = 12.0;
  const double spacing = 2.0 * M_PI * kRadius / static_cast<double>(kCount);
  for (int i = 0; i < kCount; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kCount);
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = i;
    waypoint.s_m = static_cast<double>(i) * spacing;
    waypoint.x_m = kRadius * std::cos(angle);
    waypoint.y_m = kRadius * std::sin(angle);
    waypoint.psi_rad = angle + 0.5 * M_PI;
    waypoint.kappa_radpm = 1.0 / kRadius;
    waypoint.vx_mps = 3.0;
    waypoint.d_left = d_left;
    waypoint.d_right = d_right;
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

f110_msgs::msg::Obstacle validObstacle(int id, double s_center)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = s_center;
  obstacle.s_start = s_center - 0.2;
  obstacle.s_end = s_center + 0.2;
  obstacle.d_right = -0.2;
  obstacle.d_left = 0.2;
  obstacle.d_center = 0.0;
  obstacle.size = 0.4;
  obstacle.is_static = true;
  return obstacle;
}

// Non-finite Frenet bounds are exactly what validFrenetObstacle refuses.
f110_msgs::msg::Obstacle invalidObstacle(int id)
{
  auto obstacle = validObstacle(id, 8.0);
  obstacle.d_left = std::numeric_limits<double>::quiet_NaN();
  obstacle.d_right = std::numeric_limits<double>::quiet_NaN();
  return obstacle;
}

double jsonField(const std::string & json, const std::string & key)
{
  const std::string needle = "\"" + key + "\":";
  const auto position = json.find(needle);
  if (position == std::string::npos) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::strtod(json.c_str() + position + needle.size(), nullptr);
}

class LocalPlannerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"planning_period_ms", 10},
        // Keep the staleness guards out of the way: these tests are about which messages are
        // accepted, not about how long an accepted one survives.
        {"obstacle_stale_timeout_sec", 30.0},
        {"odometry_stale_timeout_sec", 30.0},
        {"p3_mode", std::string("TEST_ACTIVE")},
      });
    planner_ = std::make_shared<local_planning::LocalPlannerNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("local_planner_node_test_helper");

    const auto global_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    waypoints_pub_ =
      helper_->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", global_qos);
    obstacles_pub_ = helper_->create_publisher<f110_msgs::msg::ObstacleArray>(
      "/confirmed_static_obs", volatile_qos);
    odometry_pub_ =
      helper_->create_publisher<nav_msgs::msg::Odometry>("/car_state/frenet/odom", volatile_qos);
    diagnostics_sub_ = helper_->create_subscription<std_msgs::msg::String>(
      "/local_planning/p3_shadow", rclcpp::QoS(1000).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {last_diagnostic_ = message->data;});

    executor_.add_node(planner_);
    executor_.add_node(helper_);
  }

  void TearDown() override
  {
    executor_.remove_node(helper_);
    executor_.remove_node(planner_);
  }

  void spin(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(2));
    }
  }

  void publishOdometry(double s)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = helper_->now();
    odometry.header.frame_id = "map";
    odometry.pose.pose.position.x = s;
    odometry.pose.pose.position.y = 0.0;
    odometry.twist.twist.linear.x = 2.0;
    odometry_pub_->publish(odometry);
  }

  void publishObstacles(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles, std::int32_t stamp_sec)
  {
    f110_msgs::msg::ObstacleArray array;
    array.header.frame_id = "map";
    array.header.stamp.sec = stamp_sec;
    array.obstacles = obstacles;
    obstacles_pub_->publish(array);
  }

  // Drives the node until the diagnostic reports a snapshot carrying this obstacle stamp, so the
  // assertions below never race the planning timer.
  bool waitForAcceptedStamp(std::int32_t stamp_sec, std::chrono::milliseconds timeout)
  {
    const double expected = static_cast<double>(stamp_sec) * 1.0e9;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(2));
      if (!last_diagnostic_.empty() &&
        std::abs(jsonField(last_diagnostic_, "source_stamp_ns") - expected) < 1.0)
      {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<local_planning::LocalPlannerNode> planner_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostics_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string last_diagnostic_;
};


// ── 확정 정적 장애물 기억 ───────────────────────────────────────────────────────────────
//
// 왜 (2026-08-17): 오늘 실패의 전부가 "짧은 지평에서 현재 위치로부터 급하게 계획하기"였다.
// 한 랩 전에 위치를 알면 그 부류가 통째로 사라진다. 대회는 20랩이고 선두 차량이 10랩을
// 완주하면 장애물이 제거되므로 3~10랩이 이득 구간이다.
//
// 계약이 셋이다. 셋 다 깨지면 실차에서 위험하다.

// 1. 기억은 온라인을 **덮어쓰지 않는다**.
//    권한이 둘이면 서로 싸운다(오늘 안전정지 래치와 FSM에서 겪었다). 온라인이 항상 최신이고,
//    기억은 온라인에 없는 것만 채운다.
TEST_F(LocalPlannerNodeTest, RememberedObstaclesNeverOverrideLiveDetection)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  // 같은 자리의 장애물을 두 번 보되, 두 번째는 폭이 다르다. 병합 후에도 **최신** 값이어야 한다.
  auto first = validObstacle(21, 8.0);
  publishObstacles({first}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500))) << last_diagnostic_;

  auto grown = validObstacle(21, 8.0);
  grown.d_left = 0.45;                       // 가까워지며 커진 관측
  grown.d_right = -0.45;
  publishObstacles({grown}, 6);
  ASSERT_TRUE(waitForAcceptedStamp(6, std::chrono::milliseconds(1500))) << last_diagnostic_;

  // 진단의 장애물 목록에 같은 s가 **하나만** 있어야 한다. 기억이 옛 관측을 따로 얹으면 둘이
  // 되고, 그러면 플래너가 유령 장애물을 하나 더 피하려 든다.
  const auto position = last_diagnostic_.find("\"obstacles\"");
  ASSERT_NE(position, std::string::npos) << last_diagnostic_;
  std::size_t count = 0;
  for (std::size_t at = last_diagnostic_.find("\"s_start\"", position);
    at != std::string::npos;
    at = last_diagnostic_.find("\"s_start\"", at + 1))
  {
    ++count;
  }
  EXPECT_EQ(count, 1U)
    << "기억이 온라인 관측 위에 중복으로 얹혔다 — 유령 장애물이 생긴다: " << last_diagnostic_;
}

// 2. 검출이 끊겨도 기억이 장애물을 유지한다.
//    가림(오늘 실측: 폭 0.137 -> 0.473)이나 일시적 미검출로 장애물이 사라지면, 종전에는
//    플래너가 "길이 열렸다"고 보고 커밋을 버렸다. 기억이 그것을 막는다.
TEST_F(LocalPlannerNodeTest, RememberedObstaclesSurviveADetectionDropout)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(31, 9.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500))) << last_diagnostic_;

  // 검출이 한 프레임 끊긴다. 자차는 아직 멀리 있으므로(s=0, 장애물 s=9) "지나쳤다"가 아니다.
  publishObstacles({}, 6);
  ASSERT_TRUE(waitForAcceptedStamp(6, std::chrono::milliseconds(1500))) << last_diagnostic_;

  EXPECT_NE(last_diagnostic_.find("\"s_start\""), std::string::npos)
    << "검출 한 프레임 끊김에 기억이 무너졌다 — 가림 때마다 커밋이 버려진다: "
    << last_diagnostic_;
}

// 3. 시야 확보한 채 지나쳤는데 못 봤으면 **지워야 한다**.
//    규정상 선두 차량이 10랩을 완주하면 장애물이 제거된다. 그 시점은 상대차 진행에 달려 있어
//    우리 랩 카운터로는 맞출 수 없다. 지우지 못하면 레이스 후반 내내 없는 장애물을 피해 돈다.
TEST_F(LocalPlannerNodeTest, RememberedObstaclesAreDroppedAfterPassingWithoutConfirmation)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(41, 3.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500))) << last_diagnostic_;

  // 1회차 — 시야 안(기본 2.0 m)까지 접근했다가 지나친다.
  publishOdometry(2.0);
  publishObstacles({}, 6);
  ASSERT_TRUE(waitForAcceptedStamp(6, std::chrono::milliseconds(1500))) << last_diagnostic_;
  publishOdometry(30.0);
  publishObstacles({}, 7);
  ASSERT_TRUE(waitForAcceptedStamp(7, std::chrono::milliseconds(1500))) << last_diagnostic_;

  // 아직 지우면 안 된다 (removal_passes=2, 2026-08-17). 통과 순간 검출이 한 프레임
  // 끊기는 일은 실제로 일어난다 — 16:17 백 실측으로 제거 6건 중 4건이 이 형태의
  // 오제거였고, 3건은 0.01초 뒤 같은 자리에 다시 생성됐다. 한 번의 미확정으로
  // 지우면 매 랩 기억을 잃고 다시 배우기를 반복한다.
  EXPECT_NE(last_diagnostic_.find("\"s_start\""), std::string::npos)
    << "한 번 못 본 것만으로 기억을 지웠다 — 프레임 한 개 누락에 매 랩 기억이 무너진다: "
    << last_diagnostic_;

  // 2회차 — 다시 접근했다가 지나친다. 이제 제거되어야 한다.
  publishOdometry(2.0);
  publishObstacles({}, 8);
  ASSERT_TRUE(waitForAcceptedStamp(8, std::chrono::milliseconds(1500))) << last_diagnostic_;
  publishOdometry(30.0);
  publishObstacles({}, 9);
  ASSERT_TRUE(waitForAcceptedStamp(9, std::chrono::milliseconds(1500))) << last_diagnostic_;

  EXPECT_EQ(last_diagnostic_.find("\"s_start\""), std::string::npos)
    << "두 번 지나치도록 못 본 장애물이 기억에 남았다 — 제거 후에도 계속 피해 돈다: "
    << last_diagnostic_;
}

// An array whose every entry fails the Frenet validity check is degraded perception. Accepting it
// would store an empty snapshot indistinguishable from the explicitly-empty array that IS allowed
// to erase the retained obstacle memory.
TEST_F(LocalPlannerNodeTest, AllInvalidObstacleArrayRetainsThePreviousSnapshot)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(11, 8.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500)))
    << "the valid array was never accepted; diagnostic=" << last_diagnostic_;
  const double accepted_sequence = jsonField(last_diagnostic_, "obstacle_sequence");
  ASSERT_TRUE(std::isfinite(accepted_sequence));

  publishObstacles({invalidObstacle(12), invalidObstacle(13)}, 9);
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));

  EXPECT_NEAR(jsonField(last_diagnostic_, "source_stamp_ns"), 5.0e9, 1.0)
    << "an all-invalid array advanced the accepted obstacle stamp";
  EXPECT_DOUBLE_EQ(jsonField(last_diagnostic_, "obstacle_sequence"), accepted_sequence)
    << "an all-invalid array advanced the accepted obstacle sequence";
}

// The contrast case: an explicitly empty array is a valid statement that the track is clear and
// must still replace the retained snapshot.
TEST_F(LocalPlannerNodeTest, ExplicitlyEmptyObstacleArrayIsStillAccepted)
{
  waypoints_pub_->publish(ringReference());
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(150));

  publishObstacles({validObstacle(21, 8.0)}, 5);
  ASSERT_TRUE(waitForAcceptedStamp(5, std::chrono::milliseconds(1500)))
    << "the valid array was never accepted; diagnostic=" << last_diagnostic_;

  publishObstacles({}, 9);
  EXPECT_TRUE(waitForAcceptedStamp(9, std::chrono::milliseconds(1500)))
    << "an explicitly empty array was refused; diagnostic=" << last_diagnostic_;
}

// obstacle_detector rebuilds its CLCS when d_left/d_right change. If the planner compared only
// s/x/y the two nodes would run on different track widths after a boundary-only recalibration.
TEST_F(LocalPlannerNodeTest, BoundaryOnlyReferenceChangeIsAdoptedAsNewReference)
{
  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));
  ASSERT_FALSE(last_diagnostic_.empty());
  const double first_generation = jsonField(last_diagnostic_, "global_reference_generation");
  ASSERT_TRUE(std::isfinite(first_generation));

  // Same centreline geometry, different track widths: s/x/y are byte-identical.
  waypoints_pub_->publish(ringReference(1.4, 1.4));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(400));

  EXPECT_GT(jsonField(last_diagnostic_, "global_reference_generation"), first_generation)
    << "a d_left/d_right-only change was treated as the same reference";
}

// The other half of the same contract: an identical retransmission must NOT churn the reference,
// because adopting it would clear the commitment and the P3 lifecycle every time the global
// planner republishes.
TEST_F(LocalPlannerNodeTest, IdenticalReferenceRetransmissionIsIgnored)
{
  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(300));
  ASSERT_FALSE(last_diagnostic_.empty());
  const double first_generation = jsonField(last_diagnostic_, "global_reference_generation");

  waypoints_pub_->publish(ringReference(3.0, 3.0));
  publishOdometry(0.0);
  spin(std::chrono::milliseconds(400));

  EXPECT_DOUBLE_EQ(
    jsonField(last_diagnostic_, "global_reference_generation"), first_generation)
    << "an identical reference retransmission was adopted as a new reference";
}

}  // namespace
