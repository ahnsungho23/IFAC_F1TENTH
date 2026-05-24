// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/GapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__GAP_DATA__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__GAP_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/gap_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GapData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: gap_diff
  {
    out << "gap_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.gap_diff, out);
    out << ", ";
  }

  // member: vs_diff
  {
    out << "vs_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.vs_diff, out);
    out << ", ";
  }

  // member: gap_int
  {
    out << "gap_int: ";
    rosidl_generator_traits::value_to_yaml(msg.gap_int, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GapData & msg,
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

  // member: gap_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gap_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.gap_diff, out);
    out << "\n";
  }

  // member: vs_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vs_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.vs_diff, out);
    out << "\n";
  }

  // member: gap_int
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gap_int: ";
    rosidl_generator_traits::value_to_yaml(msg.gap_int, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GapData & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::GapData & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::GapData & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::GapData>()
{
  return "f110_msgs::msg::GapData";
}

template<>
inline const char * name<f110_msgs::msg::GapData>()
{
  return "f110_msgs/msg/GapData";
}

template<>
struct has_fixed_size<f110_msgs::msg::GapData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<f110_msgs::msg::GapData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<f110_msgs::msg::GapData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__GAP_DATA__TRAITS_HPP_
