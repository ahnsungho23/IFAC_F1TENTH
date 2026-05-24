// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/proj_opp_traj__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_ProjOppTraj_detections
{
public:
  explicit Init_ProjOppTraj_detections(::f110_msgs::msg::ProjOppTraj & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::ProjOppTraj detections(::f110_msgs::msg::ProjOppTraj::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppTraj msg_;
};

class Init_ProjOppTraj_opp_is_on_trajectory
{
public:
  explicit Init_ProjOppTraj_opp_is_on_trajectory(::f110_msgs::msg::ProjOppTraj & msg)
  : msg_(msg)
  {}
  Init_ProjOppTraj_detections opp_is_on_trajectory(::f110_msgs::msg::ProjOppTraj::_opp_is_on_trajectory_type arg)
  {
    msg_.opp_is_on_trajectory = std::move(arg);
    return Init_ProjOppTraj_detections(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppTraj msg_;
};

class Init_ProjOppTraj_nrofpoints
{
public:
  explicit Init_ProjOppTraj_nrofpoints(::f110_msgs::msg::ProjOppTraj & msg)
  : msg_(msg)
  {}
  Init_ProjOppTraj_opp_is_on_trajectory nrofpoints(::f110_msgs::msg::ProjOppTraj::_nrofpoints_type arg)
  {
    msg_.nrofpoints = std::move(arg);
    return Init_ProjOppTraj_opp_is_on_trajectory(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppTraj msg_;
};

class Init_ProjOppTraj_lapcount
{
public:
  Init_ProjOppTraj_lapcount()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ProjOppTraj_nrofpoints lapcount(::f110_msgs::msg::ProjOppTraj::_lapcount_type arg)
  {
    msg_.lapcount = std::move(arg);
    return Init_ProjOppTraj_nrofpoints(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::ProjOppTraj>()
{
  return f110_msgs::msg::builder::Init_ProjOppTraj_lapcount();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__BUILDER_HPP_
