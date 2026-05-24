// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/opponent_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'oppwpnts'
#include "f110_msgs/msg/detail/opp_wpnt__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OpponentTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: lap_count
  {
    out << "lap_count: ";
    rosidl_generator_traits::value_to_yaml(msg.lap_count, out);
    out << ", ";
  }

  // member: opp_is_on_trajectory
  {
    out << "opp_is_on_trajectory: ";
    rosidl_generator_traits::value_to_yaml(msg.opp_is_on_trajectory, out);
    out << ", ";
  }

  // member: oppwpnts
  {
    if (msg.oppwpnts.size() == 0) {
      out << "oppwpnts: []";
    } else {
      out << "oppwpnts: [";
      size_t pending_items = msg.oppwpnts.size();
      for (auto item : msg.oppwpnts) {
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
  const OpponentTrajectory & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
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

  // member: opp_is_on_trajectory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "opp_is_on_trajectory: ";
    rosidl_generator_traits::value_to_yaml(msg.opp_is_on_trajectory, out);
    out << "\n";
  }

  // member: oppwpnts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.oppwpnts.size() == 0) {
      out << "oppwpnts: []\n";
    } else {
      out << "oppwpnts:\n";
      for (auto item : msg.oppwpnts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OpponentTrajectory & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::OpponentTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::OpponentTrajectory & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::OpponentTrajectory>()
{
  return "f110_msgs::msg::OpponentTrajectory";
}

template<>
inline const char * name<f110_msgs::msg::OpponentTrajectory>()
{
  return "f110_msgs/msg/OpponentTrajectory";
}

template<>
struct has_fixed_size<f110_msgs::msg::OpponentTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<f110_msgs::msg::OpponentTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<f110_msgs::msg::OpponentTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__TRAITS_HPP_
