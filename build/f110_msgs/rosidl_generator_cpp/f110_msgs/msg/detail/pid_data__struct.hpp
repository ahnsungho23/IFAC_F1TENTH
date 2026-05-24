// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/PidData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_HPP_

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
# define DEPRECATED__f110_msgs__msg__PidData __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__PidData __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PidData_
{
  using Type = PidData_<ContainerAllocator>;

  explicit PidData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->should = 0.0;
      this->actual = 0.0;
      this->error = 0.0;
      this->d_value = 0.0;
      this->i_value = 0.0;
      this->input = 0.0;
    }
  }

  explicit PidData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->should = 0.0;
      this->actual = 0.0;
      this->error = 0.0;
      this->d_value = 0.0;
      this->i_value = 0.0;
      this->input = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _should_type =
    double;
  _should_type should;
  using _actual_type =
    double;
  _actual_type actual;
  using _error_type =
    double;
  _error_type error;
  using _d_value_type =
    double;
  _d_value_type d_value;
  using _i_value_type =
    double;
  _i_value_type i_value;
  using _input_type =
    double;
  _input_type input;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__should(
    const double & _arg)
  {
    this->should = _arg;
    return *this;
  }
  Type & set__actual(
    const double & _arg)
  {
    this->actual = _arg;
    return *this;
  }
  Type & set__error(
    const double & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__d_value(
    const double & _arg)
  {
    this->d_value = _arg;
    return *this;
  }
  Type & set__i_value(
    const double & _arg)
  {
    this->i_value = _arg;
    return *this;
  }
  Type & set__input(
    const double & _arg)
  {
    this->input = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::PidData_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::PidData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::PidData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::PidData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::PidData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::PidData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::PidData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::PidData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::PidData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::PidData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__PidData
    std::shared_ptr<f110_msgs::msg::PidData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__PidData
    std::shared_ptr<f110_msgs::msg::PidData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PidData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->should != other.should) {
      return false;
    }
    if (this->actual != other.actual) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->d_value != other.d_value) {
      return false;
    }
    if (this->i_value != other.i_value) {
      return false;
    }
    if (this->input != other.input) {
      return false;
    }
    return true;
  }
  bool operator!=(const PidData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PidData_

// alias to use template instance with default allocator
using PidData =
  f110_msgs::msg::PidData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_HPP_
