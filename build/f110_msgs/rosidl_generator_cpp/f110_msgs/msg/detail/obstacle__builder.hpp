// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
#define F110_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "f110_msgs/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace f110_msgs
{

namespace msg
{

namespace builder
{

class Init_Obstacle_is_visible
{
public:
  explicit Init_Obstacle_is_visible(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  ::f110_msgs::msg::Obstacle is_visible(::f110_msgs::msg::Obstacle::_is_visible_type arg)
  {
    msg_.is_visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_is_static
{
public:
  explicit Init_Obstacle_is_static(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_is_visible is_static(::f110_msgs::msg::Obstacle::_is_static_type arg)
  {
    msg_.is_static = std::move(arg);
    return Init_Obstacle_is_visible(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_vd_var
{
public:
  explicit Init_Obstacle_vd_var(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_is_static vd_var(::f110_msgs::msg::Obstacle::_vd_var_type arg)
  {
    msg_.vd_var = std::move(arg);
    return Init_Obstacle_is_static(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_vs_var
{
public:
  explicit Init_Obstacle_vs_var(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_vd_var vs_var(::f110_msgs::msg::Obstacle::_vs_var_type arg)
  {
    msg_.vs_var = std::move(arg);
    return Init_Obstacle_vd_var(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_d_var
{
public:
  explicit Init_Obstacle_d_var(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_vs_var d_var(::f110_msgs::msg::Obstacle::_d_var_type arg)
  {
    msg_.d_var = std::move(arg);
    return Init_Obstacle_vs_var(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_s_var
{
public:
  explicit Init_Obstacle_s_var(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_d_var s_var(::f110_msgs::msg::Obstacle::_s_var_type arg)
  {
    msg_.s_var = std::move(arg);
    return Init_Obstacle_d_var(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_vd
{
public:
  explicit Init_Obstacle_vd(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_s_var vd(::f110_msgs::msg::Obstacle::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_Obstacle_s_var(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_vs
{
public:
  explicit Init_Obstacle_vs(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_vd vs(::f110_msgs::msg::Obstacle::_vs_type arg)
  {
    msg_.vs = std::move(arg);
    return Init_Obstacle_vd(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_size
{
public:
  explicit Init_Obstacle_size(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_vs size(::f110_msgs::msg::Obstacle::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_Obstacle_vs(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_d_center
{
public:
  explicit Init_Obstacle_d_center(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_size d_center(::f110_msgs::msg::Obstacle::_d_center_type arg)
  {
    msg_.d_center = std::move(arg);
    return Init_Obstacle_size(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_s_center
{
public:
  explicit Init_Obstacle_s_center(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_d_center s_center(::f110_msgs::msg::Obstacle::_s_center_type arg)
  {
    msg_.s_center = std::move(arg);
    return Init_Obstacle_d_center(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_is_actually_a_gap
{
public:
  explicit Init_Obstacle_is_actually_a_gap(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_s_center is_actually_a_gap(::f110_msgs::msg::Obstacle::_is_actually_a_gap_type arg)
  {
    msg_.is_actually_a_gap = std::move(arg);
    return Init_Obstacle_s_center(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_d_left
{
public:
  explicit Init_Obstacle_d_left(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_is_actually_a_gap d_left(::f110_msgs::msg::Obstacle::_d_left_type arg)
  {
    msg_.d_left = std::move(arg);
    return Init_Obstacle_is_actually_a_gap(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_d_right
{
public:
  explicit Init_Obstacle_d_right(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_d_left d_right(::f110_msgs::msg::Obstacle::_d_right_type arg)
  {
    msg_.d_right = std::move(arg);
    return Init_Obstacle_d_left(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_s_end
{
public:
  explicit Init_Obstacle_s_end(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_d_right s_end(::f110_msgs::msg::Obstacle::_s_end_type arg)
  {
    msg_.s_end = std::move(arg);
    return Init_Obstacle_d_right(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_s_start
{
public:
  explicit Init_Obstacle_s_start(::f110_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_s_end s_start(::f110_msgs::msg::Obstacle::_s_start_type arg)
  {
    msg_.s_start = std::move(arg);
    return Init_Obstacle_s_end(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_id
{
public:
  Init_Obstacle_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacle_s_start id(::f110_msgs::msg::Obstacle::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Obstacle_s_start(msg_);
  }

private:
  ::f110_msgs::msg::Obstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::f110_msgs::msg::Obstacle>()
{
  return f110_msgs::msg::builder::Init_Obstacle_id();
}

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
