// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/Wpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__WPNT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/wpnt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_Wpnt_ax_mps2
{
public:
  explicit Init_Wpnt_ax_mps2(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::Wpnt ax_mps2(::f110_msgs::msg::Wpnt::_ax_mps2_type arg)
  {
    msg_.ax_mps2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_vx_mps
{
public:
  explicit Init_Wpnt_vx_mps(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_ax_mps2 vx_mps(::f110_msgs::msg::Wpnt::_vx_mps_type arg)
  {
    msg_.vx_mps = std::move(arg);
    return Init_Wpnt_ax_mps2(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_kappa_radpm
{
public:
  explicit Init_Wpnt_kappa_radpm(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_vx_mps kappa_radpm(::f110_msgs::msg::Wpnt::_kappa_radpm_type arg)
  {
    msg_.kappa_radpm = std::move(arg);
    return Init_Wpnt_vx_mps(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_psi_rad
{
public:
  explicit Init_Wpnt_psi_rad(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_kappa_radpm psi_rad(::f110_msgs::msg::Wpnt::_psi_rad_type arg)
  {
    msg_.psi_rad = std::move(arg);
    return Init_Wpnt_kappa_radpm(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_d_left
{
public:
  explicit Init_Wpnt_d_left(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_psi_rad d_left(::f110_msgs::msg::Wpnt::_d_left_type arg)
  {
    msg_.d_left = std::move(arg);
    return Init_Wpnt_psi_rad(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_d_right
{
public:
  explicit Init_Wpnt_d_right(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_d_left d_right(::f110_msgs::msg::Wpnt::_d_right_type arg)
  {
    msg_.d_right = std::move(arg);
    return Init_Wpnt_d_left(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_y_m
{
public:
  explicit Init_Wpnt_y_m(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_d_right y_m(::f110_msgs::msg::Wpnt::_y_m_type arg)
  {
    msg_.y_m = std::move(arg);
    return Init_Wpnt_d_right(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_x_m
{
public:
  explicit Init_Wpnt_x_m(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_y_m x_m(::f110_msgs::msg::Wpnt::_x_m_type arg)
  {
    msg_.x_m = std::move(arg);
    return Init_Wpnt_y_m(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_d_m
{
public:
  explicit Init_Wpnt_d_m(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_x_m d_m(::f110_msgs::msg::Wpnt::_d_m_type arg)
  {
    msg_.d_m = std::move(arg);
    return Init_Wpnt_x_m(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_s_m
{
public:
  explicit Init_Wpnt_s_m(::f110_msgs::msg::Wpnt & msg)
  : msg_(msg)
  {}
  Init_Wpnt_d_m s_m(::f110_msgs::msg::Wpnt::_s_m_type arg)
  {
    msg_.s_m = std::move(arg);
    return Init_Wpnt_d_m(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

class Init_Wpnt_id
{
public:
  Init_Wpnt_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wpnt_s_m id(::f110_msgs::msg::Wpnt::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Wpnt_s_m(msg_);
  }

private:
  ::f110_msgs::msg::Wpnt msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::Wpnt>()
{
  return f110_msgs::msg::builder::Init_Wpnt_id();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__WPNT__BUILDER_HPP_
