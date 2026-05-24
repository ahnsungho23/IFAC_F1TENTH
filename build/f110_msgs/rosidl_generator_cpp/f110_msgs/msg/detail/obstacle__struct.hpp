// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__Obstacle __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__Obstacle __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Obstacle_
{
  using Type = Obstacle_<ContainerAllocator>;

  explicit Obstacle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->s_start = 0.0;
      this->s_end = 0.0;
      this->d_right = 0.0;
      this->d_left = 0.0;
      this->is_actually_a_gap = false;
      this->s_center = 0.0;
      this->d_center = 0.0;
      this->size = 0.0;
      this->vs = 0.0;
      this->vd = 0.0;
      this->s_var = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
      this->vd_var = 0.0;
      this->is_static = false;
      this->is_visible = false;
    }
  }

  explicit Obstacle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->s_start = 0.0;
      this->s_end = 0.0;
      this->d_right = 0.0;
      this->d_left = 0.0;
      this->is_actually_a_gap = false;
      this->s_center = 0.0;
      this->d_center = 0.0;
      this->size = 0.0;
      this->vs = 0.0;
      this->vd = 0.0;
      this->s_var = 0.0;
      this->d_var = 0.0;
      this->vs_var = 0.0;
      this->vd_var = 0.0;
      this->is_static = false;
      this->is_visible = false;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _s_start_type =
    double;
  _s_start_type s_start;
  using _s_end_type =
    double;
  _s_end_type s_end;
  using _d_right_type =
    double;
  _d_right_type d_right;
  using _d_left_type =
    double;
  _d_left_type d_left;
  using _is_actually_a_gap_type =
    bool;
  _is_actually_a_gap_type is_actually_a_gap;
  using _s_center_type =
    double;
  _s_center_type s_center;
  using _d_center_type =
    double;
  _d_center_type d_center;
  using _size_type =
    double;
  _size_type size;
  using _vs_type =
    double;
  _vs_type vs;
  using _vd_type =
    double;
  _vd_type vd;
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
  using _is_static_type =
    bool;
  _is_static_type is_static;
  using _is_visible_type =
    bool;
  _is_visible_type is_visible;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__s_start(
    const double & _arg)
  {
    this->s_start = _arg;
    return *this;
  }
  Type & set__s_end(
    const double & _arg)
  {
    this->s_end = _arg;
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
  Type & set__is_actually_a_gap(
    const bool & _arg)
  {
    this->is_actually_a_gap = _arg;
    return *this;
  }
  Type & set__s_center(
    const double & _arg)
  {
    this->s_center = _arg;
    return *this;
  }
  Type & set__d_center(
    const double & _arg)
  {
    this->d_center = _arg;
    return *this;
  }
  Type & set__size(
    const double & _arg)
  {
    this->size = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::Obstacle_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::Obstacle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::Obstacle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::Obstacle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__Obstacle
    std::shared_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__Obstacle
    std::shared_ptr<f110_msgs::msg::Obstacle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Obstacle_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->s_start != other.s_start) {
      return false;
    }
    if (this->s_end != other.s_end) {
      return false;
    }
    if (this->d_right != other.d_right) {
      return false;
    }
    if (this->d_left != other.d_left) {
      return false;
    }
    if (this->is_actually_a_gap != other.is_actually_a_gap) {
      return false;
    }
    if (this->s_center != other.s_center) {
      return false;
    }
    if (this->d_center != other.d_center) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->vs != other.vs) {
      return false;
    }
    if (this->vd != other.vd) {
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
    if (this->is_static != other.is_static) {
      return false;
    }
    if (this->is_visible != other.is_visible) {
      return false;
    }
    return true;
  }
  bool operator!=(const Obstacle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Obstacle_

// alias to use template instance with default allocator
using Obstacle =
  f110_msgs::msg::Obstacle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_HPP_
