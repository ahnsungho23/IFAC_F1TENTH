// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from f110_msgs:msg/OTWpntArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "f110_msgs/msg/detail/ot_wpnt_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace f110_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OTWpntArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) f110_msgs::msg::OTWpntArray(_init);
}

void OTWpntArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<f110_msgs::msg::OTWpntArray *>(message_memory);
  typed_message->~OTWpntArray();
}

size_t size_function__OTWpntArray__wpnts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<f110_msgs::msg::Wpnt> *>(untyped_member);
  return member->size();
}

const void * get_const_function__OTWpntArray__wpnts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<f110_msgs::msg::Wpnt> *>(untyped_member);
  return &member[index];
}

void * get_function__OTWpntArray__wpnts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<f110_msgs::msg::Wpnt> *>(untyped_member);
  return &member[index];
}

void fetch_function__OTWpntArray__wpnts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const f110_msgs::msg::Wpnt *>(
    get_const_function__OTWpntArray__wpnts(untyped_member, index));
  auto & value = *reinterpret_cast<f110_msgs::msg::Wpnt *>(untyped_value);
  value = item;
}

void assign_function__OTWpntArray__wpnts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<f110_msgs::msg::Wpnt *>(
    get_function__OTWpntArray__wpnts(untyped_member, index));
  const auto & value = *reinterpret_cast<const f110_msgs::msg::Wpnt *>(untyped_value);
  item = value;
}

void resize_function__OTWpntArray__wpnts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<f110_msgs::msg::Wpnt> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OTWpntArray_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "last_switch_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, last_switch_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "side_switch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, side_switch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ot_side",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, ot_side),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ot_line",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, ot_line),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wpnts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<f110_msgs::msg::Wpnt>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OTWpntArray, wpnts),  // bytes offset in struct
    nullptr,  // default value
    size_function__OTWpntArray__wpnts,  // size() function pointer
    get_const_function__OTWpntArray__wpnts,  // get_const(index) function pointer
    get_function__OTWpntArray__wpnts,  // get(index) function pointer
    fetch_function__OTWpntArray__wpnts,  // fetch(index, &value) function pointer
    assign_function__OTWpntArray__wpnts,  // assign(index, value) function pointer
    resize_function__OTWpntArray__wpnts  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OTWpntArray_message_members = {
  "f110_msgs::msg",  // message namespace
  "OTWpntArray",  // message name
  6,  // number of fields
  sizeof(f110_msgs::msg::OTWpntArray),
  OTWpntArray_message_member_array,  // message members
  OTWpntArray_init_function,  // function to initialize message memory (memory has to be allocated)
  OTWpntArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OTWpntArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OTWpntArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace f110_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<f110_msgs::msg::OTWpntArray>()
{
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::OTWpntArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, f110_msgs, msg, OTWpntArray)() {
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::OTWpntArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
