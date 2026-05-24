// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/CarStateStamped.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/car_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'state'
#include "f110_msgs/msg/detail/car_state__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarStateStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    to_flow_style_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarStateStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state:\n";
    to_block_style_yaml(msg.state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarStateStamped & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::CarStateStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::CarStateStamped & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::CarStateStamped>()
{
  return "f110_msgs::msg::CarStateStamped";
}

template<>
inline const char * name<f110_msgs::msg::CarStateStamped>()
{
  return "f110_msgs/msg/CarStateStamped";
}

template<>
struct has_fixed_size<f110_msgs::msg::CarStateStamped>
  : std::integral_constant<bool, has_fixed_size<f110_msgs::msg::CarState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<f110_msgs::msg::CarStateStamped>
  : std::integral_constant<bool, has_bounded_size<f110_msgs::msg::CarState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<f110_msgs::msg::CarStateStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__TRAITS_HPP_
