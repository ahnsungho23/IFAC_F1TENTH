// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/PidData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PID_DATA__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__PID_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/pid_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PidData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: should
  {
    out << "should: ";
    rosidl_generator_traits::value_to_yaml(msg.should, out);
    out << ", ";
  }

  // member: actual
  {
    out << "actual: ";
    rosidl_generator_traits::value_to_yaml(msg.actual, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: d_value
  {
    out << "d_value: ";
    rosidl_generator_traits::value_to_yaml(msg.d_value, out);
    out << ", ";
  }

  // member: i_value
  {
    out << "i_value: ";
    rosidl_generator_traits::value_to_yaml(msg.i_value, out);
    out << ", ";
  }

  // member: input
  {
    out << "input: ";
    rosidl_generator_traits::value_to_yaml(msg.input, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PidData & msg,
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

  // member: should
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "should: ";
    rosidl_generator_traits::value_to_yaml(msg.should, out);
    out << "\n";
  }

  // member: actual
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual: ";
    rosidl_generator_traits::value_to_yaml(msg.actual, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: d_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_value: ";
    rosidl_generator_traits::value_to_yaml(msg.d_value, out);
    out << "\n";
  }

  // member: i_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_value: ";
    rosidl_generator_traits::value_to_yaml(msg.i_value, out);
    out << "\n";
  }

  // member: input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input: ";
    rosidl_generator_traits::value_to_yaml(msg.input, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PidData & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::PidData & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::PidData & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::PidData>()
{
  return "f110_msgs::msg::PidData";
}

template<>
inline const char * name<f110_msgs::msg::PidData>()
{
  return "f110_msgs/msg/PidData";
}

template<>
struct has_fixed_size<f110_msgs::msg::PidData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<f110_msgs::msg::PidData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<f110_msgs::msg::PidData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__PID_DATA__TRAITS_HPP_
