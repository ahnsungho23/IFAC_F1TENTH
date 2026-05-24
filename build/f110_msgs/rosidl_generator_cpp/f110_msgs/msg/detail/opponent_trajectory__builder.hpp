// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/opponent_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_OpponentTrajectory_oppwpnts
{
public:
  explicit Init_OpponentTrajectory_oppwpnts(::f110_msgs::msg::OpponentTrajectory & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::OpponentTrajectory oppwpnts(::f110_msgs::msg::OpponentTrajectory::_oppwpnts_type arg)
  {
    msg_.oppwpnts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectory msg_;
};

class Init_OpponentTrajectory_opp_is_on_trajectory
{
public:
  explicit Init_OpponentTrajectory_opp_is_on_trajectory(::f110_msgs::msg::OpponentTrajectory & msg)
  : msg_(msg)
  {}
  Init_OpponentTrajectory_oppwpnts opp_is_on_trajectory(::f110_msgs::msg::OpponentTrajectory::_opp_is_on_trajectory_type arg)
  {
    msg_.opp_is_on_trajectory = std::move(arg);
    return Init_OpponentTrajectory_oppwpnts(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectory msg_;
};

class Init_OpponentTrajectory_lap_count
{
public:
  explicit Init_OpponentTrajectory_lap_count(::f110_msgs::msg::OpponentTrajectory & msg)
  : msg_(msg)
  {}
  Init_OpponentTrajectory_opp_is_on_trajectory lap_count(::f110_msgs::msg::OpponentTrajectory::_lap_count_type arg)
  {
    msg_.lap_count = std::move(arg);
    return Init_OpponentTrajectory_opp_is_on_trajectory(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectory msg_;
};

class Init_OpponentTrajectory_id
{
public:
  explicit Init_OpponentTrajectory_id(::f110_msgs::msg::OpponentTrajectory & msg)
  : msg_(msg)
  {}
  Init_OpponentTrajectory_lap_count id(::f110_msgs::msg::OpponentTrajectory::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_OpponentTrajectory_lap_count(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectory msg_;
};

class Init_OpponentTrajectory_header
{
public:
  Init_OpponentTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpponentTrajectory_id header(::f110_msgs::msg::OpponentTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpponentTrajectory_id(msg_);
  }

private:
  ::f110_msgs::msg::OpponentTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::OpponentTrajectory>()
{
  return f110_msgs::msg::builder::Init_OpponentTrajectory_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__BUILDER_HPP_
