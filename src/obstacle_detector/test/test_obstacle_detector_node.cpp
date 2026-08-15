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
#include <cmath>
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
#include <tf2_ros/static_transform_broadcaster.h>

#include "obstacle_detector/obstacle_detector_node.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
    void SetUp() override {rclcpp::init(0, nullptr);}
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

}  // namespace
