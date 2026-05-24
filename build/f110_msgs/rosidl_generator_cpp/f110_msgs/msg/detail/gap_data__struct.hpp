// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/GapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__GAP_DATA__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__GAP_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__GapData __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__GapData __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GapData_
{
  using Type = GapData_<ContainerAllocator>;

  explicit GapData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gap_diff = 0.0;
      this->vs_diff = 0.0;
      this->gap_int = 0.0;
    }
  }

  explicit GapData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gap_diff = 0.0;
      this->vs_diff = 0.0;
      this->gap_int = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _gap_diff_type =
    double;
  _gap_diff_type gap_diff;
  using _vs_diff_type =
    double;
  _vs_diff_type vs_diff;
  using _gap_int_type =
    double;
  _gap_int_type gap_int;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__gap_diff(
    const double & _arg)
  {
    this->gap_diff = _arg;
    return *this;
  }
  Type & set__vs_diff(
    const double & _arg)
  {
    this->vs_diff = _arg;
    return *this;
  }
  Type & set__gap_int(
    const double & _arg)
  {
    this->gap_int = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::GapData_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::GapData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::GapData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::GapData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::GapData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::GapData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::GapData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::GapData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::GapData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::GapData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__GapData
    std::shared_ptr<f110_msgs::msg::GapData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__GapData
    std::shared_ptr<f110_msgs::msg::GapData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GapData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->gap_diff != other.gap_diff) {
      return false;
    }
    if (this->vs_diff != other.vs_diff) {
      return false;
    }
    if (this->gap_int != other.gap_int) {
      return false;
    }
    return true;
  }
  bool operator!=(const GapData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GapData_

// alias to use template instance with default allocator
using GapData =
  f110_msgs::msg::GapData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__GAP_DATA__STRUCT_HPP_
