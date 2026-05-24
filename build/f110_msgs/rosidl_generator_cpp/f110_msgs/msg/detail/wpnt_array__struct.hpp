// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from f110_msgs:msg/WpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT_ARRAY__STRUCT_HPP_
#define F110_MSGS__MSG__DETAIL__WPNT_ARRAY__STRUCT_HPP_

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
// Member 'wpnts'
#include "f110_msgs/msg/detail/wpnt__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__f110_msgs__msg__WpntArray __attribute__((deprecated))
#else
# define DEPRECATED__f110_msgs__msg__WpntArray __declspec(deprecated)
#endif

namespace f110_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WpntArray_
{
  using Type = WpntArray_<ContainerAllocator>;

  explicit WpntArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit WpntArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
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
  Type & set__wpnts(
    const std::vector<f110_msgs::msg::Wpnt_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<f110_msgs::msg::Wpnt_<ContainerAllocator>>> & _arg)
  {
    this->wpnts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    f110_msgs::msg::WpntArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const f110_msgs::msg::WpntArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::WpntArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      f110_msgs::msg::WpntArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__f110_msgs__msg__WpntArray
    std::shared_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__f110_msgs__msg__WpntArray
    std::shared_ptr<f110_msgs::msg::WpntArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WpntArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->wpnts != other.wpnts) {
      return false;
    }
    return true;
  }
  bool operator!=(const WpntArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WpntArray_

// alias to use template instance with default allocator
using WpntArray =
  f110_msgs::msg::WpntArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace f110_msgs

#endif  // F110_MSGS__MSG__DETAIL__WPNT_ARRAY__STRUCT_HPP_
