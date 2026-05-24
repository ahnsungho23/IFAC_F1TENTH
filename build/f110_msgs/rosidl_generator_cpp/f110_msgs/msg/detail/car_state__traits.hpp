// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/car_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'slam_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'ekf_twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarState & msg,
  std::ostream & out)
{
  out << "{";
  // member: slam_pose
  {
    out << "slam_pose: ";
    to_flow_style_yaml(msg.slam_pose, out);
    out << ", ";
  }

  // member: ekf_twist
  {
    out << "ekf_twist: ";
    to_flow_style_yaml(msg.ekf_twist, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: slam_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slam_pose:\n";
    to_block_style_yaml(msg.slam_pose, out, indentation + 2);
  }

  // member: ekf_twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ekf_twist:\n";
    to_block_style_yaml(msg.ekf_twist, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace f110_msgs

namespace rosidl_generator_traits
{

[[deprecated("use f110_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const f110_msgs::msg::CarState & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::CarState & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::CarState>()
{
  return "f110_msgs::msg::CarState";
}

template<>
inline const char * name<f110_msgs::msg::CarState>()
{
  return "f110_msgs/msg/CarState";
}

template<>
struct has_fixed_size<f110_msgs::msg::CarState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<f110_msgs::msg::CarState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<f110_msgs::msg::CarState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE__TRAITS_HPP_
