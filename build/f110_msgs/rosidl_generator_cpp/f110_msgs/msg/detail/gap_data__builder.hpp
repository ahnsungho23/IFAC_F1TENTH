// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/GapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__GAP_DATA__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__GAP_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/gap_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_GapData_gap_int
{
public:
  explicit Init_GapData_gap_int(::f110_msgs::msg::GapData & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::GapData gap_int(::f110_msgs::msg::GapData::_gap_int_type arg)
  {
    msg_.gap_int = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::GapData msg_;
};

class Init_GapData_vs_diff
{
public:
  explicit Init_GapData_vs_diff(::f110_msgs::msg::GapData & msg)
  : msg_(msg)
  {}
  Init_GapData_gap_int vs_diff(::f110_msgs::msg::GapData::_vs_diff_type arg)
  {
    msg_.vs_diff = std::move(arg);
    return Init_GapData_gap_int(msg_);
  }

private:
  ::f110_msgs::msg::GapData msg_;
};

class Init_GapData_gap_diff
{
public:
  explicit Init_GapData_gap_diff(::f110_msgs::msg::GapData & msg)
  : msg_(msg)
  {}
  Init_GapData_vs_diff gap_diff(::f110_msgs::msg::GapData::_gap_diff_type arg)
  {
    msg_.gap_diff = std::move(arg);
    return Init_GapData_vs_diff(msg_);
  }

private:
  ::f110_msgs::msg::GapData msg_;
};

class Init_GapData_header
{
public:
  Init_GapData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GapData_gap_diff header(::f110_msgs::msg::GapData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GapData_gap_diff(msg_);
  }

private:
  ::f110_msgs::msg::GapData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::GapData>()
{
  return f110_msgs::msg::builder::Init_GapData_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__GAP_DATA__BUILDER_HPP_
