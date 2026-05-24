// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/CarStateStamped.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/car_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_CarStateStamped_state
{
public:
  explicit Init_CarStateStamped_state(::f110_msgs::msg::CarStateStamped & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::CarStateStamped state(::f110_msgs::msg::CarStateStamped::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::CarStateStamped msg_;
};

class Init_CarStateStamped_header
{
public:
  Init_CarStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarStateStamped_state header(::f110_msgs::msg::CarStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CarStateStamped_state(msg_);
  }

private:
  ::f110_msgs::msg::CarStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::CarStateStamped>()
{
  return f110_msgs::msg::builder::Init_CarStateStamped_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__BUILDER_HPP_
