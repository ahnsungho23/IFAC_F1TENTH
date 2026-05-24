// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/car_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_CarState_ekf_twist
{
public:
  explicit Init_CarState_ekf_twist(::f110_msgs::msg::CarState & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::CarState ekf_twist(::f110_msgs::msg::CarState::_ekf_twist_type arg)
  {
    msg_.ekf_twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::CarState msg_;
};

class Init_CarState_slam_pose
{
public:
  Init_CarState_slam_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarState_ekf_twist slam_pose(::f110_msgs::msg::CarState::_slam_pose_type arg)
  {
    msg_.slam_pose = std::move(arg);
    return Init_CarState_ekf_twist(msg_);
  }

private:
  ::f110_msgs::msg::CarState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::CarState>()
{
  return f110_msgs::msg::builder::Init_CarState_slam_pose();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE__BUILDER_HPP_
