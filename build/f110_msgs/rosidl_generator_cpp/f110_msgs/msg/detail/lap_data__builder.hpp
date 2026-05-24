// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__LAP_DATA__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__LAP_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/lap_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_LapData_max_lateral_error_to_global_waypoints
{
public:
  explicit Init_LapData_max_lateral_error_to_global_waypoints(::f110_msgs::msg::LapData & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::LapData max_lateral_error_to_global_waypoints(::f110_msgs::msg::LapData::_max_lateral_error_to_global_waypoints_type arg)
  {
    msg_.max_lateral_error_to_global_waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::LapData msg_;
};

class Init_LapData_average_lateral_error_to_global_waypoints
{
public:
  explicit Init_LapData_average_lateral_error_to_global_waypoints(::f110_msgs::msg::LapData & msg)
  : msg_(msg)
  {}
  Init_LapData_max_lateral_error_to_global_waypoints average_lateral_error_to_global_waypoints(::f110_msgs::msg::LapData::_average_lateral_error_to_global_waypoints_type arg)
  {
    msg_.average_lateral_error_to_global_waypoints = std::move(arg);
    return Init_LapData_max_lateral_error_to_global_waypoints(msg_);
  }

private:
  ::f110_msgs::msg::LapData msg_;
};

class Init_LapData_lap_time
{
public:
  explicit Init_LapData_lap_time(::f110_msgs::msg::LapData & msg)
  : msg_(msg)
  {}
  Init_LapData_average_lateral_error_to_global_waypoints lap_time(::f110_msgs::msg::LapData::_lap_time_type arg)
  {
    msg_.lap_time = std::move(arg);
    return Init_LapData_average_lateral_error_to_global_waypoints(msg_);
  }

private:
  ::f110_msgs::msg::LapData msg_;
};

class Init_LapData_lap_count
{
public:
  explicit Init_LapData_lap_count(::f110_msgs::msg::LapData & msg)
  : msg_(msg)
  {}
  Init_LapData_lap_time lap_count(::f110_msgs::msg::LapData::_lap_count_type arg)
  {
    msg_.lap_count = std::move(arg);
    return Init_LapData_lap_time(msg_);
  }

private:
  ::f110_msgs::msg::LapData msg_;
};

class Init_LapData_header
{
public:
  Init_LapData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LapData_lap_count header(::f110_msgs::msg::LapData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LapData_lap_count(msg_);
  }

private:
  ::f110_msgs::msg::LapData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::LapData>()
{
  return f110_msgs::msg::builder::Init_LapData_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__LAP_DATA__BUILDER_HPP_
