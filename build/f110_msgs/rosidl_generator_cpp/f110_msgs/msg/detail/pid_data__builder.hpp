// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/PidData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PID_DATA__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__PID_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/pid_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_PidData_input
{
public:
  explicit Init_PidData_input(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::PidData input(::f110_msgs::msg::PidData::_input_type arg)
  {
    msg_.input = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_i_value
{
public:
  explicit Init_PidData_i_value(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  Init_PidData_input i_value(::f110_msgs::msg::PidData::_i_value_type arg)
  {
    msg_.i_value = std::move(arg);
    return Init_PidData_input(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_d_value
{
public:
  explicit Init_PidData_d_value(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  Init_PidData_i_value d_value(::f110_msgs::msg::PidData::_d_value_type arg)
  {
    msg_.d_value = std::move(arg);
    return Init_PidData_i_value(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_error
{
public:
  explicit Init_PidData_error(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  Init_PidData_d_value error(::f110_msgs::msg::PidData::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_PidData_d_value(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_actual
{
public:
  explicit Init_PidData_actual(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  Init_PidData_error actual(::f110_msgs::msg::PidData::_actual_type arg)
  {
    msg_.actual = std::move(arg);
    return Init_PidData_error(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_should
{
public:
  explicit Init_PidData_should(::f110_msgs::msg::PidData & msg)
  : msg_(msg)
  {}
  Init_PidData_actual should(::f110_msgs::msg::PidData::_should_type arg)
  {
    msg_.should = std::move(arg);
    return Init_PidData_actual(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

class Init_PidData_header
{
public:
  Init_PidData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PidData_should header(::f110_msgs::msg::PidData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PidData_should(msg_);
  }

private:
  ::f110_msgs::msg::PidData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::PidData>()
{
  return f110_msgs::msg::builder::Init_PidData_header();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PID_DATA__BUILDER_HPP_
