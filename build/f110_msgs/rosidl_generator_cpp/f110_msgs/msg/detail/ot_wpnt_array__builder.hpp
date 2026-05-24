// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/OTWpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/ot_wpnt_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_OTWpntArray_wpnts
{
public:
  explicit Init_OTWpntArray_wpnts(::f110_msgs::msg::OTWpntArray & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::OTWpntArray wpnts(::f110_msgs::msg::OTWpntArray::_wpnts_type arg)
  {
    msg_.wpnts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

class Init_OTWpntArray_ot_line
{
public:
  explicit Init_OTWpntArray_ot_line(::f110_msgs::msg::OTWpntArray & msg)
  : msg_(msg)
  {}
  Init_OTWpntArray_wpnts ot_line(::f110_msgs::msg::OTWpntArray::_ot_line_type arg)
  {
    msg_.ot_line = std::move(arg);
    return Init_OTWpntArray_wpnts(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

class Init_OTWpntArray_ot_side
{
public:
  explicit Init_OTWpntArray_ot_side(::f110_msgs::msg::OTWpntArray & msg)
  : msg_(msg)
  {}
  Init_OTWpntArray_ot_line ot_side(::f110_msgs::msg::OTWpntArray::_ot_side_type arg)
  {
    msg_.ot_side = std::move(arg);
    return Init_OTWpntArray_ot_line(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

class Init_OTWpntArray_side_switch
{
public:
  explicit Init_OTWpntArray_side_switch(::f110_msgs::msg::OTWpntArray & msg)
  : msg_(msg)
  {}
  Init_OTWpntArray_ot_side side_switch(::f110_msgs::msg::OTWpntArray::_side_switch_type arg)
  {
    msg_.side_switch = std::move(arg);
    return Init_OTWpntArray_ot_side(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

class Init_OTWpntArray_last_switch_time
{
public:
  explicit Init_OTWpntArray_last_switch_time(::f110_msgs::msg::OTWpntArray & msg)
  : msg_(msg)
  {}
  Init_OTWpntArray_side_switch last_switch_time(::f110_msgs::msg::OTWpntArray::_last_switch_time_type arg)
  {
    msg_.last_switch_time = std::move(arg);
    return Init_OTWpntArray_side_switch(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

class Init_OTWpntArray_header
{
public:
  Init_OTWpntArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OTWpntArray_last_switch_time header(::f110_msgs::msg::OTWpntArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OTWpntArray_last_switch_time(msg_);
  }

private:
  ::f110_msgs::msg::OTWpntArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::OTWpntArray>()
{
  return f110_msgs::msg::builder::Init_OTWpntArray_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__BUILDER_HPP_
