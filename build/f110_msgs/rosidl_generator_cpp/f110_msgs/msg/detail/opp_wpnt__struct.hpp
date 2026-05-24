// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__OppWpnt __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__OppWpnt __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OppWpnt_
{
  using Type = OppWpnt_<ContainerAllocator>;

  explicit OppWpnt_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->s_m = 0.0;
      this->d_m = 0.0;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->proj_vs_mps = 0.0;
      this->vd_mps = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
    }
  }

  explicit OppWpnt_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->s_m = 0.0;
      this->d_m = 0.0;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->proj_vs_mps = 0.0;
      this->vd_mps = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _s_m_type =
    double;
  _s_m_type s_m;
  using _d_m_type =
    double;
  _d_m_type d_m;
  using _x_m_type =
    double;
  _x_m_type x_m;
  using _y_m_type =
    double;
  _y_m_type y_m;
  using _proj_vs_mps_type =
    double;
  _proj_vs_mps_type proj_vs_mps;
  using _vd_mps_type =
    double;
  _vd_mps_type vd_mps;
  using _d_var_type =
    double;
  _d_var_type d_var;
  using _vs_var_type =
    double;
  _vs_var_type vs_var;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__s_m(
    const double & _arg)
  {
    this->s_m = _arg;
    return *this;
  }
  Type & set__d_m(
    const double & _arg)
  {
    this->d_m = _arg;
    return *this;
  }
  Type & set__x_m(
    const double & _arg)
  {
    this->x_m = _arg;
    return *this;
  }
  Type & set__y_m(
    const double & _arg)
  {
    this->y_m = _arg;
    return *this;
  }
  Type & set__proj_vs_mps(
    const double & _arg)
  {
    this->proj_vs_mps = _arg;
    return *this;
  }
  Type & set__vd_mps(
    const double & _arg)
  {
    this->vd_mps = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::OppWpnt_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::OppWpnt_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OppWpnt_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OppWpnt_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__OppWpnt
    std::shared_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__OppWpnt
    std::shared_ptr<f110_msgs::msg::OppWpnt_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OppWpnt_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->s_m != other.s_m) {
      return false;
    }
    if (this->d_m != other.d_m) {
      return false;
    }
    if (this->x_m != other.x_m) {
      return false;
    }
    if (this->y_m != other.y_m) {
      return false;
    }
    if (this->proj_vs_mps != other.proj_vs_mps) {
      return false;
    }
    if (this->vd_mps != other.vd_mps) {
      return false;
    }
    if (this->d_var != other.d_var) {
      return false;
    }
    if (this->vs_var != other.vs_var) {
      return false;
    }
    return true;
  }
  bool operator!=(const OppWpnt_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OppWpnt_

// alias to use template instance with default allocator
using OppWpnt =
  f110_msgs::msg::OppWpnt_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_HPP_
