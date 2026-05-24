// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/OTWpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_HPP_

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
// Member 'last_switch_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'wpnts'
#include "f110_msgs/msg/detail/wpnt__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__OTWpntArray __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__OTWpntArray __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OTWpntArray_
{
  using Type = OTWpntArray_<ContainerAllocator>;

  explicit OTWpntArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    last_switch_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side_switch = false;
      this->ot_side = "";
      this->ot_line = "";
    }
  }

  explicit OTWpntArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    last_switch_time(_alloc, _init),
    ot_side(_alloc),
    ot_line(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side_switch = false;
      this->ot_side = "";
      this->ot_line = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _last_switch_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_switch_time_type last_switch_time;
  using _side_switch_type =
    bool;
  _side_switch_type side_switch;
  using _ot_side_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ot_side_type ot_side;
  using _ot_line_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ot_line_type ot_line;
  using _wpnts_type =
    std::vector<f110_msgs::msg::Wpnt_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::Wpnt_<ContainerAllocator>>>;
  _wpnts_type wpnts;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__last_switch_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_switch_time = _arg;
    return *this;
  }
  Type & set__side_switch(
    const bool & _arg)
  {
    this->side_switch = _arg;
    return *this;
  }
  Type & set__ot_side(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ot_side = _arg;
    return *this;
  }
  Type & set__ot_line(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ot_line = _arg;
    return *this;
  }
  Type & set__wpnts(
    const std::vector<f110_msgs::msg::Wpnt_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::Wpnt_<ContainerAllocator>>> & _arg)
  {
    this->wpnts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::OTWpntArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::OTWpntArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OTWpntArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::OTWpntArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__OTWpntArray
    std::shared_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__OTWpntArray
    std::shared_ptr<f110_msgs::msg::OTWpntArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OTWpntArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->last_switch_time != other.last_switch_time) {
      return false;
    }
    if (this->side_switch != other.side_switch) {
      return false;
    }
    if (this->ot_side != other.ot_side) {
      return false;
    }
    if (this->ot_line != other.ot_line) {
      return false;
    }
    if (this->wpnts != other.wpnts) {
      return false;
    }
    return true;
  }
  bool operator!=(const OTWpntArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OTWpntArray_

// alias to use template instance with default allocator
using OTWpntArray =
  f110_msgs::msg::OTWpntArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_HPP_
