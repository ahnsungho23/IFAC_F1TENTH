// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPP_WPNT__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__OPP_WPNT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/opp_wpnt__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OppWpnt & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: s_m
  {
    out << "s_m: ";
    rosidl_generator_traits::value_to_yaml(msg.s_m, out);
    out << ", ";
  }

  // member: d_m
  {
    out << "d_m: ";
    rosidl_generator_traits::value_to_yaml(msg.d_m, out);
    out << ", ";
  }

  // member: x_m
  {
    out << "x_m: ";
    rosidl_generator_traits::value_to_yaml(msg.x_m, out);
    out << ", ";
  }

  // member: y_m
  {
    out << "y_m: ";
    rosidl_generator_traits::value_to_yaml(msg.y_m, out);
    out << ", ";
  }

  // member: proj_vs_mps
  {
    out << "proj_vs_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.proj_vs_mps, out);
    out << ", ";
  }

  // member: vd_mps
  {
    out << "vd_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.vd_mps, out);
    out << ", ";
  }

  // member: d_var
  {
    out << "d_var: ";
    rosidl_generator_traits::value_to_yaml(msg.d_var, out);
    out << ", ";
  }

  // member: vs_var
  {
    out << "vs_var: ";
    rosidl_generator_traits::value_to_yaml(msg.vs_var, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OppWpnt & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: s_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_m: ";
    rosidl_generator_traits::value_to_yaml(msg.s_m, out);
    out << "\n";
  }

  // member: d_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_m: ";
    rosidl_generator_traits::value_to_yaml(msg.d_m, out);
    out << "\n";
  }

  // member: x_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_m: ";
    rosidl_generator_traits::value_to_yaml(msg.x_m, out);
    out << "\n";
  }

  // member: y_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_m: ";
    rosidl_generator_traits::value_to_yaml(msg.y_m, out);
    out << "\n";
  }

  // member: proj_vs_mps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proj_vs_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.proj_vs_mps, out);
    out << "\n";
  }

  // member: vd_mps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vd_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.vd_mps, out);
    out << "\n";
  }

  // member: d_var
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_var: ";
    rosidl_generator_traits::value_to_yaml(msg.d_var, out);
    out << "\n";
  }

  // member: vs_var
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vs_var: ";
    rosidl_generator_traits::value_to_yaml(msg.vs_var, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OppWpnt & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::OppWpnt & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::OppWpnt & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::OppWpnt>()
{
  return "f110_msgs::msg::OppWpnt";
}

template<>
inline const char * name<f110_msgs::msg::OppWpnt>()
{
  return "f110_msgs/msg/OppWpnt";
}

template<>
struct has_fixed_size<f110_msgs::msg::OppWpnt>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<f110_msgs::msg::OppWpnt>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<f110_msgs::msg::OppWpnt>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__OPP_WPNT__TRAITS_HPP_
