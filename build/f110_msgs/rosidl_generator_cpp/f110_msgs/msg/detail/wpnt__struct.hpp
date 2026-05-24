// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/Wpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__WPNT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__Wpnt __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__Wpnt __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Wpnt_
{
  using Type = Wpnt_<ContainerAllocator>;

  explicit Wpnt_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->s_m = 0.0;
      this->d_m = 0.0;
      this->x_m = 0.0;
      this->y_m = 0.0;
      this->d_right = 0.0;
      this->d_left = 0.0;
      this->psi_rad = 0.0;
      this->kappa_radpm = 0.0;
      this->vx_mps = 0.0;
      this->ax_mps2 = 0.0;
    }
  }

  explicit Wpnt_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->d_right = 0.0;
      this->d_left = 0.0;
      this->psi_rad = 0.0;
      this->kappa_radpm = 0.0;
      this->vx_mps = 0.0;
      this->ax_mps2 = 0.0;
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
  using _d_right_type =
    double;
  _d_right_type d_right;
  using _d_left_type =
    double;
  _d_left_type d_left;
  using _psi_rad_type =
    double;
  _psi_rad_type psi_rad;
  using _kappa_radpm_type =
    double;
  _kappa_radpm_type kappa_radpm;
  using _vx_mps_type =
    double;
  _vx_mps_type vx_mps;
  using _ax_mps2_type =
    double;
  _ax_mps2_type ax_mps2;

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
  Type & set__d_right(
    const double & _arg)
  {
    this->d_right = _arg;
    return *this;
  }
  Type & set__d_left(
    const double & _arg)
  {
    this->d_left = _arg;
    return *this;
  }
  Type & set__psi_rad(
    const double & _arg)
  {
    this->psi_rad = _arg;
    return *this;
  }
  Type & set__kappa_radpm(
    const double & _arg)
  {
    this->kappa_radpm = _arg;
    return *this;
  }
  Type & set__vx_mps(
    const double & _arg)
  {
    this->vx_mps = _arg;
    return *this;
  }
  Type & set__ax_mps2(
    const double & _arg)
  {
    this->ax_mps2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::Wpnt_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::Wpnt_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::Wpnt_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::Wpnt_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__Wpnt
    std::shared_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__Wpnt
    std::shared_ptr<f110_msgs::msg::Wpnt_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Wpnt_ & other) const
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
    if (this->d_right != other.d_right) {
      return false;
    }
    if (this->d_left != other.d_left) {
      return false;
    }
    if (this->psi_rad != other.psi_rad) {
      return false;
    }
    if (this->kappa_radpm != other.kappa_radpm) {
      return false;
    }
    if (this->vx_mps != other.vx_mps) {
      return false;
    }
    if (this->ax_mps2 != other.ax_mps2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Wpnt_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Wpnt_

// alias to use template instance with default allocator
using Wpnt =
  f110_msgs::msg::Wpnt_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__WPNT__STRUCT_HPP_
