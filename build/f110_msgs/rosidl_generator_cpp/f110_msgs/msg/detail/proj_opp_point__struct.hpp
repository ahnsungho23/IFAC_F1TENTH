// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/ProjOppPoint.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__ProjOppPoint __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__ProjOppPoint __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ProjOppPoint_
{
  using Type = ProjOppPoint_<ContainerAllocator>;

  explicit ProjOppPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->s = 0.0;
      this->d = 0.0;
      this->vs = 0.0;
      this->vd = 0.0;
      this->is_static = false;
      this->is_visible = false;
      this->time = 0.0;
      this->s_var = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
      this->vd_var = 0.0;
    }
  }

  explicit ProjOppPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->s = 0.0;
      this->d = 0.0;
      this->vs = 0.0;
      this->vd = 0.0;
      this->is_static = false;
      this->is_visible = false;
      this->time = 0.0;
      this->s_var = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
      this->vd_var = 0.0;
    }
  }

  // field types and members
  using _s_type =
    double;
  _s_type s;
  using _d_type =
    double;
  _d_type d;
  using _vs_type =
    double;
  _vs_type vs;
  using _vd_type =
    double;
  _vd_type vd;
  using _is_static_type =
    bool;
  _is_static_type is_static;
  using _is_visible_type =
    bool;
  _is_visible_type is_visible;
  using _time_type =
    double;
  _time_type time;
  using _s_var_type =
    double;
  _s_var_type s_var;
  using _d_var_type =
    double;
  _d_var_type d_var;
  using _vs_var_type =
    double;
  _vs_var_type vs_var;
  using _vd_var_type =
    double;
  _vd_var_type vd_var;

  // setters for named parameter idiom
  Type & set__s(
    const double & _arg)
  {
    this->s = _arg;
    return *this;
  }
  Type & set__d(
    const double & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__vs(
    const double & _arg)
  {
    this->vs = _arg;
    return *this;
  }
  Type & set__vd(
    const double & _arg)
  {
    this->vd = _arg;
    return *this;
  }
  Type & set__is_static(
    const bool & _arg)
  {
    this->is_static = _arg;
    return *this;
  }
  Type & set__is_visible(
    const bool & _arg)
  {
    this->is_visible = _arg;
    return *this;
  }
  Type & set__time(
    const double & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__s_var(
    const double & _arg)
  {
    this->s_var = _arg;
    return *this;
  }
  Type & set__d_var(
    const double & _arg)
  {
    this->d_var = _arg;
    return *this;
  }
  Type & set__vs_var(
    const double & _arg)
  {
    this->vs_var = _arg;
    return *this;
  }
  Type & set__vd_var(
    const double & _arg)
  {
    this->vd_var = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::ProjOppPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::ProjOppPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__ProjOppPoint
    std::shared_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__ProjOppPoint
    std::shared_ptr<f110_msgs::msg::ProjOppPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ProjOppPoint_ & other) const
  {
    if (this->s != other.s) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->vs != other.vs) {
      return false;
    }
    if (this->vd != other.vd) {
      return false;
    }
    if (this->is_static != other.is_static) {
      return false;
    }
    if (this->is_visible != other.is_visible) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->s_var != other.s_var) {
      return false;
    }
    if (this->d_var != other.d_var) {
      return false;
    }
    if (this->vs_var != other.vs_var) {
      return false;
    }
    if (this->vd_var != other.vd_var) {
      return false;
    }
    return true;
  }
  bool operator!=(const ProjOppPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ProjOppPoint_

// alias to use template instance with default allocator
using ProjOppPoint =
  f110_msgs::msg::ProjOppPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_HPP_
