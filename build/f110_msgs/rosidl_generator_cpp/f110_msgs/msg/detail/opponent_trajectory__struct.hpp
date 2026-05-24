// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_HPP_

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
// Member 'oppwpnts'
#include "f110_msgs/msg/detail/opp_wpnt__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__OpponentTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__OpponentTrajectory __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OpponentTrajectory_
{
  using Type = OpponentTrajectory_<ContainerAllocator>;

  explicit OpponentTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->lap_count = 0.0;
      this->opp_is_on_trajectory = false;
    }
  }

  explicit OpponentTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->lap_count = 0.0;
      this->opp_is_on_trajectory = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    int32_t;
  _id_type id;
  using _lap_count_type =
    double;
  _lap_count_type lap_count;
  using _opp_is_on_trajectory_type =
    bool;
  _opp_is_on_trajectory_type opp_is_on_trajectory;
  using _oppwpnts_type =
    std::vector<f110_msgs::msg::OppWpnt_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::OppWpnt_<ContainerAllocator>>>;
  _oppwpnts_type oppwpnts;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__lap_count(
    const double & _arg)
  {
    this->lap_count = _arg;
    return *this;
  }
  Type & set__opp_is_on_trajectory(
    const bool & _arg)
  {
    this->opp_is_on_trajectory = _arg;
    return *this;
  }
  Type & set__oppwpnts(
    const std::vector<f110_msgs::msg::OppWpnt_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::OppWpnt_<ContainerAllocator>>> & _arg)
  {
    this->oppwpnts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__OpponentTrajectory
    std::shared_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__OpponentTrajectory
    std::shared_ptr<f110_msgs::msg::OpponentTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OpponentTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->lap_count != other.lap_count) {
      return false;
    }
    if (this->opp_is_on_trajectory != other.opp_is_on_trajectory) {
      return false;
    }
    if (this->oppwpnts != other.oppwpnts) {
      return false;
    }
    return true;
  }
  bool operator!=(const OpponentTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OpponentTrajectory_

// alias to use template instance with default allocator
using OpponentTrajectory =
  f110_msgs::msg::OpponentTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_HPP_
