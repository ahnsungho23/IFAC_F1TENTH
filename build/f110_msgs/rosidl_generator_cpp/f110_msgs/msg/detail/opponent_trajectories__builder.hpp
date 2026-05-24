// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/OpponentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/opponent_trajectories__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_OpponentTrajectories_trajectories
{
public:
  explicit Init_OpponentTrajectories_trajectories(::f110_msgs::msg::OpponentTrajectories & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::OpponentTrajectories trajectories(::f110_msgs::msg::OpponentTrajectories::_trajectories_type arg)
  {
    msg_.trajectories = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectories msg_;
};

class Init_OpponentTrajectories_header
{
public:
  Init_OpponentTrajectories_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpponentTrajectories_trajectories header(::f110_msgs::msg::OpponentTrajectories::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpponentTrajectories_trajectories(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectories msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::OpponentTrajectories>()
{
  return f110_msgs::msg::builder::Init_OpponentTrajectories_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__BUILDER_HPP_
