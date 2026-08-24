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

// Node-level tests for ObstacleDetectorNode. The tracker and projector classes have their own
// unit tests; what is covered here is the publish gating inside scanCallback, which only exists
// at the node level.

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <atomic>
#include <tf2_ros/static_transform_broadcaster.h>

#include "obstacle_detector/obstacle_detector_node.hpp"

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
      setenv("ROS_DOMAIN_ID", "92", 1);
      setenv("ROS_LOCALHOST_ONLY", "1", 1);
      rclcpp::init(0, nullptr);
    }
    void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment *const kRclcppEnvironment =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

f110_msgs::msg::WpntArray straightReference()
{
    f110_msgs::msg::WpntArray reference;
    reference.header.frame_id = "map";
    for (int i = 0; i < 200; ++i)
    {
        f110_msgs::msg::Wpnt waypoint;
        waypoint.id = i;
        waypoint.s_m = 0.1 * static_cast<double>(i);
        waypoint.x_m = waypoint.s_m;
        waypoint.y_m = 0.0;
        waypoint.psi_rad = 0.0;
        waypoint.kappa_radpm = 0.0;
        waypoint.vx_mps = 3.0;
        waypoint.d_left = 2.0;
        waypoint.d_right = 2.0;
        reference.wpnts.push_back(waypoint);
    }
    return reference;
}

sensor_msgs::msg::LaserScan scanWithObject(const rclcpp::Time &stamp)
{
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = "laser";
    scan.angle_min = -M_PI_2;
    scan.angle_max = M_PI_2;
    scan.angle_increment = M_PI / 180.0;
    scan.range_min = 0.05;
    scan.range_max = 30.0;
    const int count =
        static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    scan.ranges.assign(static_cast<std::size_t>(count), scan.range_max + 1.0F);
    // A compact return straight ahead: enough beams to survive the cluster-size gate.
    for (int i = 88; i <= 92; ++i)
    {
        scan.ranges[static_cast<std::size_t>(i)] = 5.0F;
    }
    return scan;
}

class ObstacleDetectorNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            // The map filter would need an OccupancyGrid that has nothing to do with this test.
            {"use_map_filter", false},
            {"ego_odom_topic", std::string("/test/ego_odom")},
            {"simulator", false},
        });
        detector_ = std::make_shared<obstacle_detector::ObstacleDetectorNode>(options);
        helper_ = std::make_shared<rclcpp::Node>("obstacle_detector_node_test_helper");

        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = helper_->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "laser";
        transform.transform.rotation.w = 1.0;
        broadcaster_->sendTransform(transform);

        scan_pub_ = helper_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        waypoints_pub_ = helper_->create_publisher<f110_msgs::msg::WpntArray>(
            "/global_waypoints", rclcpp::QoS(1).reliable().transient_local());
        odometry_pub_ =
            helper_->create_publisher<nav_msgs::msg::Odometry>("/test/ego_odom", 10);
        opp_sub_ = helper_->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/opp_obs", 10,
            [this](const f110_msgs::msg::ObstacleArray::SharedPtr) {++opp_messages_;});
        static_sub_ = helper_->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/static_obs", 10,
            [this](const f110_msgs::msg::ObstacleArray::SharedPtr) {++static_messages_;});

        executor_.add_node(detector_);
        executor_.add_node(helper_);
    }

    void TearDown() override
    {
        executor_.remove_node(helper_);
        executor_.remove_node(detector_);
    }

    void spin(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            executor_.spin_some(std::chrono::milliseconds(2));
        }
    }

    void publishScans(int count)
    {
        for (int i = 0; i < count; ++i)
        {
            scan_pub_->publish(scanWithObject(helper_->now()));
            spin(std::chrono::milliseconds(20));
        }
    }

    void publishOdometry()
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = helper_->now();
        odometry.header.frame_id = "map";
        odometry.pose.pose.position.x = 1.0;
        odometry.pose.pose.position.y = 0.0;
        odometry.pose.pose.orientation.w = 1.0;
        odometry_pub_->publish(odometry);
    }

    std::shared_ptr<obstacle_detector::ObstacleDetectorNode> detector_;
    std::shared_ptr<rclcpp::Node> helper_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr opp_sub_;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr static_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    int opp_messages_{0};
    int static_messages_{0};
};

// ego_s_ < 0 means odometry was NEVER received. selectOpponent then has no ahead/behind test at
// all and ranks purely by positional variance, so that state must suppress /opp_obs rather than
// permit it. The static layers do not depend on ego_s_ and must keep ticking meanwhile, which is
// what separates "suppressed correctly" from "the node simply never ran".
TEST_F(ObstacleDetectorNodeTest, OppObsIsSuppressedUntilEgoOdometryArrives)
{
    waypoints_pub_->publish(straightReference());
    spin(std::chrono::milliseconds(200));

    publishScans(6);
    spin(std::chrono::milliseconds(100));

    ASSERT_GT(static_messages_, 0)
        << "the detector never processed a scan, so the /opp_obs assertion would be vacuous";
    EXPECT_EQ(opp_messages_, 0)
        << "/opp_obs was published before any ego odometry was received";

    publishOdometry();
    spin(std::chrono::milliseconds(50));
    publishScans(4);
    spin(std::chrono::milliseconds(100));

    EXPECT_GT(opp_messages_, 0)
        << "/opp_obs stayed suppressed after fresh ego odometry arrived";
}

// ------------------------------------------------------------------------------------------------
// 후방 사각 회수 판정 기하 (2026-08-22)
//
// 이 판정이 지키는 계약은 하나다: **네 꼭짓점이 전부 FOV 밖일 때만** true.
// 하나라도 볼 수 있으면 그건 차폐일 수 있고, 차폐는 static hold 가 존재하는 이유 그 자체다.
// ------------------------------------------------------------------------------------------------

// 실차 라이다(±135°, 1081 빔)를 그대로 쓴다. FOV 를 코드 상수로 두지 않는 것이 설계 요점이라
// 시험도 스캔 헤더로만 FOV 를 준다.
// 순수 기하라 노드 인스턴스가 필요 없다 — static 함수를 직접 부른다.
constexpr auto kInRearBlindCone =
    &obstacle_detector::ObstacleDetectorNode::envelopeInRearBlindCone;

sensor_msgs::msg::LaserScan makeScan(double angle_min, double angle_max)
{
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = static_cast<float>(angle_min);
    scan.angle_max = static_cast<float>(angle_max);
    scan.angle_increment = static_cast<float>((angle_max - angle_min) / 1080.0);
    return scan;
}

// 스캔 원점 기준 (forward, left) 위치에 half 크기의 정사각 봉투를 놓는다. yaw=0 이므로
// map 좌표가 곧 스캔 좌표다.
obstacle_detector::Track makeEnvelope(double forward, double left, double half)
{
    obstacle_detector::Track track;
    track.x_min_map = forward - half;
    track.x_max_map = forward + half;
    track.y_min_map = left - half;
    track.y_max_map = left + half;
    return track;
}

constexpr double kRealScanMin = -2.356194;   // -135.0 deg
constexpr double kRealScanMax = 2.356194;    // +135.0 deg
constexpr double kMargin = 2.0 * M_PI / 180.0;

TEST(RearBlindCone, RetiresTheEnvelopeThatDeadlockedTheRealCar)
{
    // run_20260821_230603 t=56.18 의 유령: 라이다 뒤 1.23 m, 측방 0.55 m. 이 봉투가 사라진
    // 0.22 s 뒤에 안전정지가 풀렸다 — 즉 이 판정이 잡아야 하는 바로 그 형상이다.
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    const auto track = makeEnvelope(-1.23, 0.55, 0.06);
    EXPECT_TRUE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, KeepsAnythingStillInsideTheFieldOfView)
{
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    // 정면
    EXPECT_FALSE(kInRearBlindCone(
        makeEnvelope(3.0, 0.0, 0.15), scan, 0.0, 0.0, 0.0, kMargin));
    // 옆 (bearing 90°)
    EXPECT_FALSE(kInRearBlindCone(
        makeEnvelope(0.0, 2.0, 0.15), scan, 0.0, 0.0, 0.0, kMargin));
    // 뒤쪽이지만 크게 옆으로 벌어져 있어 135° 안쪽 (bearing ≈ 116°)
    EXPECT_FALSE(kInRearBlindCone(
        makeEnvelope(-1.0, 2.0, 0.10), scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, KeepsAnEnvelopeThatOnlyPartiallyLeavesTheFieldOfView)
{
    // 사각 경계에 걸친 봉투 — 꼭짓점 일부만 밖이다. 회수하면 안 된다.
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    const auto track = makeEnvelope(-1.0, 1.0, 0.30);   // 중심 bearing 135°
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, MarginWidensTheFieldOfViewSoItIsConservative)
{
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    // 사각 경계 바로 바깥(≈136.5°)에 아주 작은 봉투를 둔다.
    const double bearing = (136.5) * M_PI / 180.0;
    const double radius = 1.5;
    const auto track = makeEnvelope(radius * std::cos(bearing), radius * std::sin(bearing), 0.01);
    // margin 0 이면 회수 대상
    EXPECT_TRUE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, 0.0));
    // margin 2° 는 FOV 를 넓히므로 아직 회수하지 않는다
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, IsANoOpForA360DegreeScanner)
{
    // 사각이 없는 스캐너에서는 이 회수 근거가 성립하지 않는다.
    const auto scan = makeScan(-M_PI, M_PI);
    const auto track = makeEnvelope(-2.0, 0.0, 0.10);
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, UsesTheScanFramePoseSoEgoYawRotatesTheCone)
{
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    // map 기준 (-2, 0) 의 봉투. yaw=0 이면 정후방이라 사각.
    const auto track = makeEnvelope(-2.0, 0.0, 0.10);
    EXPECT_TRUE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
    // 차가 180° 돌면 같은 봉투가 정면이 된다.
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, M_PI, kMargin));
    // 원점 평행이동도 그대로 따라간다: 라이다가 (-4,0) 이면 그 봉투는 전방 2 m 다.
    EXPECT_FALSE(kInRearBlindCone(track, scan, -4.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, RejectsAnEnvelopeWithoutFiniteGeometry)
{
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    obstacle_detector::Track track = makeEnvelope(-2.0, 0.0, 0.10);
    track.x_min_map = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

TEST(RearBlindCone, KeepsAnEnvelopeSittingOnTheScanOrigin)
{
    // 원점을 덮는 봉투는 bearing 이 수치적으로 무의미하다 — 회수하지 않는다.
    const auto scan = makeScan(kRealScanMin, kRealScanMax);
    const auto track = makeEnvelope(0.0, 0.0, 0.20);
    EXPECT_FALSE(kInRearBlindCone(track, scan, 0.0, 0.0, 0.0, kMargin));
}

}  // namespace

// /initialpose 수동 재배치 리셋 (2026-08-24). 깨진 pose 로 승격된 track 이 재배치 후에도
// hold/메모리로 계속 발행되는 사슬을 검증한다: 재배치 즉시 기존 track 이 전부 소거되고,
// 유예 창 동안은 같은 물체가 계속 보여도 새 track 이 생기지 않아야 하며(발행은 빈 배열로
// 계속되어 플래너 메모리를 지운다), 유예가 끝나면 재승격이 다시 가능해야 한다.
TEST_F(ObstacleDetectorNodeTest, InitialposeClearsTracksAndSuppressesReconfirmation)
{
    std::atomic<int> static_msgs{0};
    std::atomic<int> static_nonempty{0};
    auto content_sub = helper_->create_subscription<f110_msgs::msg::ObstacleArray>(
        "/static_obs", 10,
        [&](const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
            ++static_msgs;
            if (!msg->obstacles.empty())
            {
                ++static_nonempty;
            }
        });

    waypoints_pub_->publish(straightReference());
    publishOdometry();
    spin(std::chrono::milliseconds(50));
    publishScans(12);
    ASSERT_GT(static_nonempty.load(), 0) << "전제: 물체가 track 으로 발행되고 있어야 한다";

    auto initialpose_pub =
        helper_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    geometry_msgs::msg::PoseWithCovarianceStamped reset;
    reset.header.stamp = helper_->now();
    reset.header.frame_id = "map";
    reset.pose.pose.orientation.w = 1.0;

    static_msgs = 0;
    static_nonempty = 0;
    initialpose_pub->publish(reset);
    spin(std::chrono::milliseconds(100));

    // 유예 창(기본 2.0 s) 안에서 같은 물체를 계속 보여준다 — 12스캔 ≈ 0.24 s.
    publishScans(12);
    EXPECT_GT(static_msgs.load(), 0) << "유예 중에도 발행은 계속되어야 한다 (빈 배열)";
    EXPECT_EQ(static_nonempty.load(), 0)
        << "유예 창 안에서는 어떤 track 도 다시 승격되면 안 된다";
}

