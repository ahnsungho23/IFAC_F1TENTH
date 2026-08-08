// Copyright 2026 2026_IFAC contributors

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "global_planning/frenet_lap_counter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace global_planning
{

class LapCounterNode : public rclcpp::Node
{
public:
  LapCounterNode()
  : Node("lap_counter_node")
  {
    const auto odom_topic = declare_parameter<std::string>(
      "frenet_odom_topic", "/car_state/frenet/odom");
    const auto lap_count_topic = declare_parameter<std::string>(
      "lap_count_topic", "/lap_count");
    const double finish_s_min = declare_parameter<double>("finish_s_min", 10.0);
    const double start_s_max = declare_parameter<double>("start_s_max", 0.5);
    const double min_lap_time_sec = declare_parameter<double>("min_lap_time_sec", 3.0);
    const auto initial_lap_count = declare_parameter<std::int64_t>("initial_lap_count", 0);

    if (initial_lap_count < 0 ||
      initial_lap_count > std::numeric_limits<std::int32_t>::max())
    {
      throw std::invalid_argument(
              "initial_lap_count must fit the non-negative std_msgs/msg/Int32 range");
    }

    counter_ = std::make_unique<FrenetLapCounter>(
      finish_s_min, start_s_max, min_lap_time_sec,
      static_cast<std::int32_t>(initial_lap_count));

    const auto count_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    lap_count_pub_ = create_publisher<std_msgs::msg::Int32>(lap_count_topic, count_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&LapCounterNode::onOdometry, this, std::placeholders::_1));

    publishLapCount();
    RCLCPP_INFO(
      get_logger(),
      "Frenet lap counter started: odom=%s output=%s finish_s_min=%.3f "
      "start_s_max=%.3f min_lap_time=%.3f initial_count=%d",
      odom_topic.c_str(), lap_count_topic.c_str(), finish_s_min, start_s_max,
      min_lap_time_sec, counter_->lapCount());
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double s = msg->pose.pose.position.x;
    if (!std::isfinite(s)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite Frenet s value.");
      return;
    }

    const rclcpp::Time message_stamp(msg->header.stamp);
    const double sample_time_sec = message_stamp.nanoseconds() == 0 ?
      now().seconds() : message_stamp.seconds();

    if (!counter_->update(s, sample_time_sec)) {
      return;
    }

    publishLapCount();
    RCLCPP_INFO(get_logger(), "Lap completed: count=%d", counter_->lapCount());
  }

  void publishLapCount()
  {
    std_msgs::msg::Int32 msg;
    msg.data = counter_->lapCount();
    lap_count_pub_->publish(msg);
  }

  std::unique_ptr<FrenetLapCounter> counter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_count_pub_;
};

}  // namespace global_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planning::LapCounterNode>());
  rclcpp::shutdown();
  return 0;
}
