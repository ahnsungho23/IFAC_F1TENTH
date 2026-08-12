#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "cma_gt_localization/odometry_conversion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cma_gt_localization
{

class GroundTruthLocalizationBridge : public rclcpp::Node
{
public:
  GroundTruthLocalizationBridge()
  : Node("gt_localization_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/ego_racecar/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/pf/pose/odom");
    expected_input_frame_ = declare_parameter<std::string>("expected_input_frame", "map");
    expected_input_child_frame_ = declare_parameter<std::string>(
      "expected_input_child_frame", "ego_racecar/base_link");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    output_child_frame_ = declare_parameter<std::string>(
      "output_child_frame", "ego_racecar/base_link");
    reject_frame_mismatch_ = declare_parameter<bool>("reject_frame_mismatch", true);
    const auto twist_mode_name = declare_parameter<std::string>(
      "twist_mode", "mcl_compatible");
    const auto qos_depth = declare_parameter<int>("qos_depth", 10);

    if (input_topic_.empty() || output_topic_.empty() || input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must be non-empty and different");
    }
    if (qos_depth <= 0) {
      throw std::invalid_argument("qos_depth must be positive");
    }
    twist_mode_ = parseTwistMode(twist_mode_name);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth))).reliable();
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, qos,
      std::bind(&GroundTruthLocalizationBridge::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "CMA-only GT localization bridge: %s [%s -> %s] -> %s [%s -> %s], "
      "twist_mode=%s, publishes_tf=false",
      input_topic_.c_str(), expected_input_frame_.c_str(),
      expected_input_child_frame_.c_str(), output_topic_.c_str(), output_frame_.c_str(),
      output_child_frame_.c_str(), twist_mode_name.c_str());
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const bool frame_matches =
      message->header.frame_id == expected_input_frame_ &&
      message->child_frame_id == expected_input_child_frame_;
    if (!frame_matches) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GT odometry frame mismatch: received [%s -> %s], expected [%s -> %s]",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        expected_input_frame_.c_str(), expected_input_child_frame_.c_str());
      if (reject_frame_mismatch_) {
        return;
      }
    }

    publisher_->publish(convertGroundTruthOdometry(
      *message, output_frame_, output_child_frame_, twist_mode_));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string expected_input_frame_;
  std::string expected_input_child_frame_;
  std::string output_frame_;
  std::string output_child_frame_;
  bool reject_frame_mismatch_{true};
  TwistMode twist_mode_{TwistMode::kMclCompatible};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace cma_gt_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<cma_gt_localization::GroundTruthLocalizationBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gt_localization_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

