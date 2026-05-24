// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'slam_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'ekf_twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__CarState __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__CarState __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarState_
{
  using Type = CarState_<ContainerAllocator>;

  explicit CarState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slam_pose(_init),
    ekf_twist(_init)
  {
    (void)_init;
  }

  explicit CarState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : slam_pose(_alloc, _init),
    ekf_twist(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _slam_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _slam_pose_type slam_pose;
  using _ekf_twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _ekf_twist_type ekf_twist;

  // setters for named parameter idiom
  Type & set__slam_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->slam_pose = _arg;
    return *this;
  }
  Type & set__ekf_twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->ekf_twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::CarState_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::CarState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::CarState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::CarState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::CarState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::CarState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::CarState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::CarState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::CarState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::CarState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__CarState
    std::shared_ptr<f110_msgs::msg::CarState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__CarState
    std::shared_ptr<f110_msgs::msg::CarState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarState_ & other) const
  {
    if (this->slam_pose != other.slam_pose) {
      return false;
    }
    if (this->ekf_twist != other.ekf_twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarState_

// alias to use template instance with default allocator
using CarState =
  f110_msgs::msg::CarState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_HPP_
