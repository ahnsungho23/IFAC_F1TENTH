// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/WpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT_ARRAY__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__WPNT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/wpnt_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_WpntArray_wpnts
{
public:
  explicit Init_WpntArray_wpnts(::f110_msgs::msg::WpntArray & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::WpntArray wpnts(::f110_msgs::msg::WpntArray::_wpnts_type arg)
  {
    msg_.wpnts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::WpntArray msg_;
};

class Init_WpntArray_header
{
public:
  Init_WpntArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WpntArray_wpnts header(::f110_msgs::msg::WpntArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WpntArray_wpnts(msg_);
  }

private:
  ::f110_msgs::msg::WpntArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::WpntArray>()
{
  return f110_msgs::msg::builder::Init_WpntArray_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__WPNT_ARRAY__BUILDER_HPP_
