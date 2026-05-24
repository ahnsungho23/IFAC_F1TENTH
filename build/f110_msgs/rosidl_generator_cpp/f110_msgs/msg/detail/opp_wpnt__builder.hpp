// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPP_WPNT__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__OPP_WPNT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/opp_wpnt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_OppWpnt_vs_var
{
public:
  explicit Init_OppWpnt_vs_var(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::OppWpnt vs_var(::f110_msgs::msg::OppWpnt::_vs_var_type arg)
  {
    msg_.vs_var = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_d_var
{
public:
  explicit Init_OppWpnt_d_var(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_vs_var d_var(::f110_msgs::msg::OppWpnt::_d_var_type arg)
  {
    msg_.d_var = std::move(arg);
    return Init_OppWpnt_vs_var(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_vd_mps
{
public:
  explicit Init_OppWpnt_vd_mps(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_d_var vd_mps(::f110_msgs::msg::OppWpnt::_vd_mps_type arg)
  {
    msg_.vd_mps = std::move(arg);
    return Init_OppWpnt_d_var(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_proj_vs_mps
{
public:
  explicit Init_OppWpnt_proj_vs_mps(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_vd_mps proj_vs_mps(::f110_msgs::msg::OppWpnt::_proj_vs_mps_type arg)
  {
    msg_.proj_vs_mps = std::move(arg);
    return Init_OppWpnt_vd_mps(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_y_m
{
public:
  explicit Init_OppWpnt_y_m(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_proj_vs_mps y_m(::f110_msgs::msg::OppWpnt::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_OppWpnt_proj_vs_mps(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_x_m
{
public:
  explicit Init_OppWpnt_x_m(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_y_m x_m(::f110_msgs::msg::OppWpnt::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_OppWpnt_y_m(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_d_m
{
public:
  explicit Init_OppWpnt_d_m(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_x_m d_m(::f110_msgs::msg::OppWpnt::_d_m_type arg)
  {
    msg_.d_m = std::move(arg);
    return Init_OppWpnt_x_m(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_s_m
{
public:
  explicit Init_OppWpnt_s_m(::f110_msgs::msg::OppWpnt & msg)
  : msg_(msg)
  {}
  Init_OppWpnt_d_m s_m(::f110_msgs::msg::OppWpnt::_s_m_type arg)
  {
    msg_.s_m = std::move(arg);
    return Init_OppWpnt_d_m(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

class Init_OppWpnt_id
{
public:
  Init_OppWpnt_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OppWpnt_s_m id(::f110_msgs::msg::OppWpnt::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_OppWpnt_s_m(msg_);
  }

private:
  ::f110_msgs::msg::OppWpnt msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::OppWpnt>()
{
  return f110_msgs::msg::builder::Init_OppWpnt_id();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OPP_WPNT__BUILDER_HPP_
