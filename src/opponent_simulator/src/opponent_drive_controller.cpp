#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double distance2d(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

}  // namespace

class OpponentDriveController : public rclcpp::Node
{
public:
  OpponentDriveController()
  : Node("opponent_drive_controller")
  {
    waypoints_topic_ = declare_parameter<std::string>(
      "waypoints_topic", "/centerline_waypoints");
    opponent_odom_topic_ = declare_parameter<std::string>(
      "opponent_odom_topic", "/opp_racecar/odom");
    opponent_drive_topic_ = declare_parameter<std::string>(
      "opponent_drive_topic", "/opp_drive");
    speed_scale_ = declare_parameter<double>("speed_scale", 0.8);
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 2.0);
    wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.33);
    steering_limit_rad_ = declare_parameter<double>("steering_limit_rad", 0.4);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 40.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);
    enabled_ = declare_parameter<bool>("enabled", true);

    validateParameters();

    waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      waypoints_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&OpponentDriveController::onWaypoints, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      opponent_odom_topic_, rclcpp::QoS(10),
      std::bind(&OpponentDriveController::onOdom, this, std::placeholders::_1));
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      opponent_drive_topic_, rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&OpponentDriveController::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Opponent controller started (waypoints=%s, odom=%s, drive=%s, scale=%.2f, rate=%.1fHz)",
      waypoints_topic_.c_str(), opponent_odom_topic_.c_str(), opponent_drive_topic_.c_str(),
      speed_scale_, control_rate_hz_);
  }

private:
  void validateParameters() const
  {
    if (waypoints_topic_.empty() || opponent_odom_topic_.empty() || opponent_drive_topic_.empty()) {
      throw std::invalid_argument("topic parameters must not be empty");
    }
    if (!std::isfinite(speed_scale_) || speed_scale_ < 0.0) {
      throw std::invalid_argument("speed_scale must be finite and non-negative");
    }
    if (!std::isfinite(lookahead_distance_m_) || lookahead_distance_m_ <= 0.0 ||
      !std::isfinite(wheelbase_m_) || wheelbase_m_ <= 0.0 ||
      !std::isfinite(steering_limit_rad_) || steering_limit_rad_ <= 0.0 ||
      !std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("controller geometry, rate, limits, and timeout must be positive");
    }
  }

  void onWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    if (msg->wpnts.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring empty waypoint array on %s",
        waypoints_topic_.c_str());
      return;
    }
    waypoints_ = msg->wpnts;
    RCLCPP_INFO(get_logger(), "Received %zu opponent waypoints", waypoints_.size());
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    opponent_x_ = msg->pose.pose.position.x;
    opponent_y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    opponent_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_odom_time_ = now();
    has_odom_ = true;
  }

  void controlLoop()
  {
    if (!enabled_ || waypoints_.empty() || !has_odom_) {
      return;
    }
    if ((now() - last_odom_time_).seconds() > odom_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Opponent odometry is stale; suppressing /opp_drive");
      return;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      const double distance = distance2d(
        opponent_x_, opponent_y_, waypoints_[index].x_m, waypoints_[index].y_m);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = index;
      }
    }

    std::size_t lookahead_index = nearest_index;
    double accumulated_distance = 0.0;
    for (std::size_t offset = 0; offset < waypoints_.size(); ++offset) {
      const std::size_t current = (nearest_index + offset) % waypoints_.size();
      const std::size_t next = (current + 1) % waypoints_.size();
      accumulated_distance += distance2d(
        waypoints_[current].x_m, waypoints_[current].y_m,
        waypoints_[next].x_m, waypoints_[next].y_m);
      lookahead_index = next;
      if (accumulated_distance >= lookahead_distance_m_) {
        break;
      }
    }

    const auto & target = waypoints_[lookahead_index];
    const double dx = target.x_m - opponent_x_;
    const double dy = target.y_m - opponent_y_;
    const double lookahead = std::hypot(dx, dy);
    if (lookahead < 1e-6) {
      return;
    }

    const double alpha = normalizeAngle(std::atan2(dy, dx) - opponent_yaw_);
    const double steering = std::clamp(
      std::atan2(2.0 * wheelbase_m_ * std::sin(alpha), lookahead),
      -steering_limit_rad_, steering_limit_rad_);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command.drive.speed = std::max(0.0, static_cast<double>(target.vx_mps) * speed_scale_);
    command.drive.steering_angle = steering;
    drive_pub_->publish(command);
  }

  std::string waypoints_topic_;
  std::string opponent_odom_topic_;
  std::string opponent_drive_topic_;
  double speed_scale_{0.8};
  double lookahead_distance_m_{2.0};
  double wheelbase_m_{0.33};
  double steering_limit_rad_{0.4};
  double control_rate_hz_{40.0};
  double odom_timeout_sec_{0.5};
  bool enabled_{true};

  std::vector<f110_msgs::msg::Wpnt> waypoints_;
  bool has_odom_{false};
  double opponent_x_{0.0};
  double opponent_y_{0.0};
  double opponent_yaw_{0.0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpponentDriveController>());
  rclcpp::shutdown();
  return 0;
}
