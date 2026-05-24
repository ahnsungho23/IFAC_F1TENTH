// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/OTWpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/ot_wpnt_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'last_switch_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'wpnts'
#include "f110_msgs/msg/detail/wpnt__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OTWpntArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: last_switch_time
  {
    out << "last_switch_time: ";
    to_flow_style_yaml(msg.last_switch_time, out);
    out << ", ";
  }

  // member: side_switch
  {
    out << "side_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.side_switch, out);
    out << ", ";
  }

  // member: ot_side
  {
    out << "ot_side: ";
    rosidl_generator_traits::value_to_yaml(msg.ot_side, out);
    out << ", ";
  }

  // member: ot_line
  {
    out << "ot_line: ";
    rosidl_generator_traits::value_to_yaml(msg.ot_line, out);
    out << ", ";
  }

  // member: wpnts
  {
    if (msg.wpnts.size() == 0) {
      out << "wpnts: []";
    } else {
      out << "wpnts: [";
      size_t pending_items = msg.wpnts.size();
      for (auto item : msg.wpnts) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OTWpntArray & msg,
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

  // member: last_switch_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_switch_time:\n";
    to_block_style_yaml(msg.last_switch_time, out, indentation + 2);
  }

  // member: side_switch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "side_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.side_switch, out);
    out << "\n";
  }

  // member: ot_side
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ot_side: ";
    rosidl_generator_traits::value_to_yaml(msg.ot_side, out);
    out << "\n";
  }

  // member: ot_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ot_line: ";
    rosidl_generator_traits::value_to_yaml(msg.ot_line, out);
    out << "\n";
  }

  // member: wpnts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wpnts.size() == 0) {
      out << "wpnts: []\n";
    } else {
      out << "wpnts:\n";
      for (auto item : msg.wpnts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OTWpntArray & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::OTWpntArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::OTWpntArray & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::OTWpntArray>()
{
  return "f110_msgs::msg::OTWpntArray";
}

template<>
inline const char * name<f110_msgs::msg::OTWpntArray>()
{
  return "f110_msgs/msg/OTWpntArray";
}

template<>
struct has_fixed_size<f110_msgs::msg::OTWpntArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<f110_msgs::msg::OTWpntArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<f110_msgs::msg::OTWpntArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__TRAITS_HPP_
