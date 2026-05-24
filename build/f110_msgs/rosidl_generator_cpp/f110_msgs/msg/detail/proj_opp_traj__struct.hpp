// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'detections'
#include "f110_msgs/msg/detail/proj_opp_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__ProjOppTraj __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__ProjOppTraj __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ProjOppTraj_
{
  using Type = ProjOppTraj_<ContainerAllocator>;

  explicit ProjOppTraj_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lapcount = 0.0;
      this->nrofpoints = 0.0;
      this->opp_is_on_trajectory = false;
    }
  }

  explicit ProjOppTraj_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lapcount = 0.0;
      this->nrofpoints = 0.0;
      this->opp_is_on_trajectory = false;
    }
  }

  // field types and members
  using _lapcount_type =
    double;
  _lapcount_type lapcount;
  using _nrofpoints_type =
    double;
  _nrofpoints_type nrofpoints;
  using _opp_is_on_trajectory_type =
    bool;
  _opp_is_on_trajectory_type opp_is_on_trajectory;
  using _detections_type =
    std::vector<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__lapcount(
    const double & _arg)
  {
    this->lapcount = _arg;
    return *this;
  }
  Type & set__nrofpoints(
    const double & _arg)
  {
    this->nrofpoints = _arg;
    return *this;
  }
  Type & set__opp_is_on_trajectory(
    const bool & _arg)
  {
    this->opp_is_on_trajectory = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::ProjOppPoint_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::ProjOppTraj_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::ProjOppTraj_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::ProjOppTraj_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::ProjOppTraj_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__ProjOppTraj
    std::shared_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__ProjOppTraj
    std::shared_ptr<f110_msgs::msg::ProjOppTraj_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ProjOppTraj_ & other) const
  {
    if (this->lapcount != other.lapcount) {
      return false;
    }
    if (this->nrofpoints != other.nrofpoints) {
      return false;
    }
    if (this->opp_is_on_trajectory != other.opp_is_on_trajectory) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const ProjOppTraj_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ProjOppTraj_

// alias to use template instance with default allocator
using ProjOppTraj =
  f110_msgs::msg::ProjOppTraj_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_HPP_
