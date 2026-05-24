// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__TRAITS_HPP_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "f110_msgs/msg/detail/proj_opp_traj__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'detections'
#include "f110_msgs/msg/detail/proj_opp_point__traits.hpp"

namespace f110_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ProjOppTraj & msg,
  std::ostream & out)
{
  out << "{";
  // member: lapcount
  {
    out << "lapcount: ";
    rosidl_generator_traits::value_to_yaml(msg.lapcount, out);
    out << ", ";
  }

  // member: nrofpoints
  {
    out << "nrofpoints: ";
    rosidl_generator_traits::value_to_yaml(msg.nrofpoints, out);
    out << ", ";
  }

  // member: opp_is_on_trajectory
  {
    out << "opp_is_on_trajectory: ";
    rosidl_generator_traits::value_to_yaml(msg.opp_is_on_trajectory, out);
    out << ", ";
  }

  // member: detections
  {
    if (msg.detections.size() == 0) {
      out << "detections: []";
    } else {
      out << "detections: [";
      size_t pending_items = msg.detections.size();
      for (auto item : msg.detections) {
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
  const ProjOppTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lapcount
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lapcount: ";
    rosidl_generator_traits::value_to_yaml(msg.lapcount, out);
    out << "\n";
  }

  // member: nrofpoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nrofpoints: ";
    rosidl_generator_traits::value_to_yaml(msg.nrofpoints, out);
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

  // member: detections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detections.size() == 0) {
      out << "detections: []\n";
    } else {
      out << "detections:\n";
      for (auto item : msg.detections) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ProjOppTraj & msg, bool use_flow_style = false)
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
  const f110_msgs::msg::ProjOppTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  f110_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use f110_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const f110_msgs::msg::ProjOppTraj & msg)
{
  return f110_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<f110_msgs::msg::ProjOppTraj>()
{
  return "f110_msgs::msg::ProjOppTraj";
}

template<>
inline const char * name<f110_msgs::msg::ProjOppTraj>()
{
  return "f110_msgs/msg/ProjOppTraj";
}

template<>
struct has_fixed_size<f110_msgs::msg::ProjOppTraj>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<f110_msgs::msg::ProjOppTraj>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<f110_msgs::msg::ProjOppTraj>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__TRAITS_HPP_
