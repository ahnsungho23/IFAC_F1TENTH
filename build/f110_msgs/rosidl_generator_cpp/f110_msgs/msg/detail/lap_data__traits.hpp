// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__LAP_DATA__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__LAP_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/lap_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LapData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: lap_count
  {
    out << "lap_count: ";
    rosidl_generator_traits::value_to_yaml(msg.lap_count, out);
    out << ", ";
  }

  // member: lap_time
  {
    out << "lap_time: ";
    rosidl_generator_traits::value_to_yaml(msg.lap_time, out);
    out << ", ";
  }

  // member: average_lateral_error_to_global_waypoints
  {
    out << "average_lateral_error_to_global_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.average_lateral_error_to_global_waypoints, out);
    out << ", ";
  }

  // member: max_lateral_error_to_global_waypoints
  {
    out << "max_lateral_error_to_global_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.max_lateral_error_to_global_waypoints, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LapData & msg,
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

  // member: lap_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lap_count: ";
    rosidl_generator_traits::value_to_yaml(msg.lap_count, out);
    out << "\n";
  }

  // member: lap_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lap_time: ";
    rosidl_generator_traits::value_to_yaml(msg.lap_time, out);
    out << "\n";
  }

  // member: average_lateral_error_to_global_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "average_lateral_error_to_global_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.average_lateral_error_to_global_waypoints, out);
    out << "\n";
  }

  // member: max_lateral_error_to_global_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_lateral_error_to_global_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.max_lateral_error_to_global_waypoints, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LapData & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::LapData & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::LapData & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::LapData>()
{
  return "f110_msgs::msg::LapData";
}

template<>
inline const char * name<f110_msgs::msg::LapData>()
{
  return "f110_msgs/msg/LapData";
}

template<>
struct has_fixed_size<f110_msgs::msg::LapData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<f110_msgs::msg::LapData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<f110_msgs::msg::LapData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__LAP_DATA__TRAITS_HPP_
