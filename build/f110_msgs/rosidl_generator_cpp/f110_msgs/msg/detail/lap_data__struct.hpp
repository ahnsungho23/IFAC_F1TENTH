// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_HPP_

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
# define DEPRECATED__f110_msgs__msg__LapData __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__LapData __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LapData_
{
  using Type = LapData_<ContainerAllocator>;

  explicit LapData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_count = 0l;
      this->lap_time = 0.0;
      this->average_lateral_error_to_global_waypoints = 0.0;
      this->max_lateral_error_to_global_waypoints = 0.0;
    }
  }

  explicit LapData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_count = 0l;
      this->lap_time = 0.0;
      this->average_lateral_error_to_global_waypoints = 0.0;
      this->max_lateral_error_to_global_waypoints = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _lap_count_type =
    int32_t;
  _lap_count_type lap_count;
  using _lap_time_type =
    double;
  _lap_time_type lap_time;
  using _average_lateral_error_to_global_waypoints_type =
    double;
  _average_lateral_error_to_global_waypoints_type average_lateral_error_to_global_waypoints;
  using _max_lateral_error_to_global_waypoints_type =
    double;
  _max_lateral_error_to_global_waypoints_type max_lateral_error_to_global_waypoints;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__lap_count(
    const int32_t & _arg)
  {
    this->lap_count = _arg;
    return *this;
  }
  Type & set__lap_time(
    const double & _arg)
  {
    this->lap_time = _arg;
    return *this;
  }
  Type & set__average_lateral_error_to_global_waypoints(
    const double & _arg)
  {
    this->average_lateral_error_to_global_waypoints = _arg;
    return *this;
  }
  Type & set__max_lateral_error_to_global_waypoints(
    const double & _arg)
  {
    this->max_lateral_error_to_global_waypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::LapData_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::LapData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::LapData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::LapData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::LapData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::LapData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::LapData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::LapData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::LapData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::LapData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__LapData
    std::shared_ptr<f110_msgs::msg::LapData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__LapData
    std::shared_ptr<f110_msgs::msg::LapData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LapData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->lap_count != other.lap_count) {
      return false;
    }
    if (this->lap_time != other.lap_time) {
      return false;
    }
    if (this->average_lateral_error_to_global_waypoints != other.average_lateral_error_to_global_waypoints) {
      return false;
    }
    if (this->max_lateral_error_to_global_waypoints != other.max_lateral_error_to_global_waypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const LapData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LapData_

// alias to use template instance with default allocator
using LapData =
  f110_msgs::msg::LapData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_HPP_
