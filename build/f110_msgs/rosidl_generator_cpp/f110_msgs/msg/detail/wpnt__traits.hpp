// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/Wpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__WPNT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/wpnt__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Wpnt & msg,
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

  // member: d_right
  {
    out << "d_right: ";
    rosidl_generator_traits::value_to_yaml(msg.d_right, out);
    out << ", ";
  }

  // member: d_left
  {
    out << "d_left: ";
    rosidl_generator_traits::value_to_yaml(msg.d_left, out);
    out << ", ";
  }

  // member: psi_rad
  {
    out << "psi_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.psi_rad, out);
    out << ", ";
  }

  // member: kappa_radpm
  {
    out << "kappa_radpm: ";
    rosidl_generator_traits::value_to_yaml(msg.kappa_radpm, out);
    out << ", ";
  }

  // member: vx_mps
  {
    out << "vx_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.vx_mps, out);
    out << ", ";
  }

  // member: ax_mps2
  {
    out << "ax_mps2: ";
    rosidl_generator_traits::value_to_yaml(msg.ax_mps2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wpnt & msg,
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

  // member: d_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_right: ";
    rosidl_generator_traits::value_to_yaml(msg.d_right, out);
    out << "\n";
  }

  // member: d_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_left: ";
    rosidl_generator_traits::value_to_yaml(msg.d_left, out);
    out << "\n";
  }

  // member: psi_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "psi_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.psi_rad, out);
    out << "\n";
  }

  // member: kappa_radpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kappa_radpm: ";
    rosidl_generator_traits::value_to_yaml(msg.kappa_radpm, out);
    out << "\n";
  }

  // member: vx_mps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx_mps: ";
    rosidl_generator_traits::value_to_yaml(msg.vx_mps, out);
    out << "\n";
  }

  // member: ax_mps2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ax_mps2: ";
    rosidl_generator_traits::value_to_yaml(msg.ax_mps2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wpnt & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::Wpnt & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::Wpnt & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::Wpnt>()
{
  return "f110_msgs::msg::Wpnt";
}

template<>
inline const char * name<f110_msgs::msg::Wpnt>()
{
  return "f110_msgs/msg/Wpnt";
}

template<>
struct has_fixed_size<f110_msgs::msg::Wpnt>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<f110_msgs::msg::Wpnt>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<f110_msgs::msg::Wpnt>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__WPNT__TRAITS_HPP_
