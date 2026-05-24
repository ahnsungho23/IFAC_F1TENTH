// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/ProjOppPoint.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/proj_opp_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_ProjOppPoint_vd_var
{
public:
  explicit Init_ProjOppPoint_vd_var(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::ProjOppPoint vd_var(::f110_msgs::msg::ProjOppPoint::_vd_var_type arg)
  {
    msg_.vd_var = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_vs_var
{
public:
  explicit Init_ProjOppPoint_vs_var(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_vd_var vs_var(::f110_msgs::msg::ProjOppPoint::_vs_var_type arg)
  {
    msg_.vs_var = std::move(arg);
    return Init_ProjOppPoint_vd_var(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_d_var
{
public:
  explicit Init_ProjOppPoint_d_var(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_vs_var d_var(::f110_msgs::msg::ProjOppPoint::_d_var_type arg)
  {
    msg_.d_var = std::move(arg);
    return Init_ProjOppPoint_vs_var(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_s_var
{
public:
  explicit Init_ProjOppPoint_s_var(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_d_var s_var(::f110_msgs::msg::ProjOppPoint::_s_var_type arg)
  {
    msg_.s_var = std::move(arg);
    return Init_ProjOppPoint_d_var(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_time
{
public:
  explicit Init_ProjOppPoint_time(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_s_var time(::f110_msgs::msg::ProjOppPoint::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_ProjOppPoint_s_var(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_is_visible
{
public:
  explicit Init_ProjOppPoint_is_visible(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_time is_visible(::f110_msgs::msg::ProjOppPoint::_is_visible_type arg)
  {
    msg_.is_visible = std::move(arg);
    return Init_ProjOppPoint_time(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_is_static
{
public:
  explicit Init_ProjOppPoint_is_static(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_is_visible is_static(::f110_msgs::msg::ProjOppPoint::_is_static_type arg)
  {
    msg_.is_static = std::move(arg);
    return Init_ProjOppPoint_is_visible(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_vd
{
public:
  explicit Init_ProjOppPoint_vd(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_is_static vd(::f110_msgs::msg::ProjOppPoint::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_ProjOppPoint_is_static(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_vs
{
public:
  explicit Init_ProjOppPoint_vs(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_vd vs(::f110_msgs::msg::ProjOppPoint::_vs_type arg)
  {
    msg_.vs = std::move(arg);
    return Init_ProjOppPoint_vd(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_d
{
public:
  explicit Init_ProjOppPoint_d(::f110_msgs::msg::ProjOppPoint & msg)
  : msg_(msg)
  {}
  Init_ProjOppPoint_vs d(::f110_msgs::msg::ProjOppPoint::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_ProjOppPoint_vs(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

class Init_ProjOppPoint_s
{
public:
  Init_ProjOppPoint_s()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ProjOppPoint_d s(::f110_msgs::msg::ProjOppPoint::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_ProjOppPoint_d(msg_);
  }

private:
  ::f110_msgs::msg::ProjOppPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::ProjOppPoint>()
{
  return f110_msgs::msg::builder::Init_ProjOppPoint_s();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__BUILDER_HPP_
