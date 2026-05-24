// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Obstacle & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: s_start
  {
    out << "s_start: ";
    rosidl_generator_traits::value_to_yaml(msg.s_start, out);
    out << ", ";
  }

  // member: s_end
  {
    out << "s_end: ";
    rosidl_generator_traits::value_to_yaml(msg.s_end, out);
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

  // member: is_actually_a_gap
  {
    out << "is_actually_a_gap: ";
    rosidl_generator_traits::value_to_yaml(msg.is_actually_a_gap, out);
    out << ", ";
  }

  // member: s_center
  {
    out << "s_center: ";
    rosidl_generator_traits::value_to_yaml(msg.s_center, out);
    out << ", ";
  }

  // member: d_center
  {
    out << "d_center: ";
    rosidl_generator_traits::value_to_yaml(msg.d_center, out);
    out << ", ";
  }

  // member: size
  {
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << ", ";
  }

  // member: vs
  {
    out << "vs: ";
    rosidl_generator_traits::value_to_yaml(msg.vs, out);
    out << ", ";
  }

  // member: vd
  {
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << ", ";
  }

  // member: s_var
  {
    out << "s_var: ";
    rosidl_generator_traits::value_to_yaml(msg.s_var, out);
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
    out << ", ";
  }

  // member: vd_var
  {
    out << "vd_var: ";
    rosidl_generator_traits::value_to_yaml(msg.vd_var, out);
    out << ", ";
  }

  // member: is_static
  {
    out << "is_static: ";
    rosidl_generator_traits::value_to_yaml(msg.is_static, out);
    out << ", ";
  }

  // member: is_visible
  {
    out << "is_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.is_visible, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Obstacle & msg,
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

  // member: s_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_start: ";
    rosidl_generator_traits::value_to_yaml(msg.s_start, out);
    out << "\n";
  }

  // member: s_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_end: ";
    rosidl_generator_traits::value_to_yaml(msg.s_end, out);
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

  // member: is_actually_a_gap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_actually_a_gap: ";
    rosidl_generator_traits::value_to_yaml(msg.is_actually_a_gap, out);
    out << "\n";
  }

  // member: s_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_center: ";
    rosidl_generator_traits::value_to_yaml(msg.s_center, out);
    out << "\n";
  }

  // member: d_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_center: ";
    rosidl_generator_traits::value_to_yaml(msg.d_center, out);
    out << "\n";
  }

  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << "\n";
  }

  // member: vs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vs: ";
    rosidl_generator_traits::value_to_yaml(msg.vs, out);
    out << "\n";
  }

  // member: vd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << "\n";
  }

  // member: s_var
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_var: ";
    rosidl_generator_traits::value_to_yaml(msg.s_var, out);
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

  // member: vd_var
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vd_var: ";
    rosidl_generator_traits::value_to_yaml(msg.vd_var, out);
    out << "\n";
  }

  // member: is_static
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_static: ";
    rosidl_generator_traits::value_to_yaml(msg.is_static, out);
    out << "\n";
  }

  // member: is_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.is_visible, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Obstacle & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::Obstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::Obstacle & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::Obstacle>()
{
  return "f110_msgs::msg::Obstacle";
}

template<>
inline const char * name<f110_msgs::msg::Obstacle>()
{
  return "f110_msgs/msg/Obstacle";
}

template<>
struct has_fixed_size<f110_msgs::msg::Obstacle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<f110_msgs::msg::Obstacle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<f110_msgs::msg::Obstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
