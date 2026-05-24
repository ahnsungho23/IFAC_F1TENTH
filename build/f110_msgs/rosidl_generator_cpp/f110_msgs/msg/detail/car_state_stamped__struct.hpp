// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/CarStateStamped.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_HPP_

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
// Member 'state'
#include "f110_msgs/msg/detail/car_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__CarStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__CarStateStamped __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarStateStamped_
{
  using Type = CarStateStamped_<ContainerAllocator>;

  explicit CarStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state(_init)
  {
    (void)_init;
  }

  explicit CarStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    f110_msgs::msg::CarState_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const f110_msgs::msg::CarState_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::CarStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::CarStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::CarStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::CarStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__CarStateStamped
    std::shared_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__CarStateStamped
    std::shared_ptr<f110_msgs::msg::CarStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarStateStamped_

// alias to use template instance with default allocator
using CarStateStamped =
  f110_msgs::msg::CarStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_HPP_
