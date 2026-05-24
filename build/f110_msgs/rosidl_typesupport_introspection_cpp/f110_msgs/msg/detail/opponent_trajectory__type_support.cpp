// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "f110_msgs/msg/detail/opponent_trajectory__struct.hpp"
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

void OpponentTrajectory_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) f110_msgs::msg::OpponentTrajectory(_init);
}

void OpponentTrajectory_fini_function(void * message_memory)
{
  auto typed_message = static_cast<f110_msgs::msg::OpponentTrajectory *>(message_memory);
  typed_message->~OpponentTrajectory();
}

size_t size_function__OpponentTrajectory__oppwpnts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<f110_msgs::msg::OppWpnt> *>(untyped_member);
  return member->size();
}

const void * get_const_function__OpponentTrajectory__oppwpnts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<f110_msgs::msg::OppWpnt> *>(untyped_member);
  return &member[index];
}

void * get_function__OpponentTrajectory__oppwpnts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<f110_msgs::msg::OppWpnt> *>(untyped_member);
  return &member[index];
}

void fetch_function__OpponentTrajectory__oppwpnts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const f110_msgs::msg::OppWpnt *>(
    get_const_function__OpponentTrajectory__oppwpnts(untyped_member, index));
  auto & value = *reinterpret_cast<f110_msgs::msg::OppWpnt *>(untyped_value);
  value = item;
}

void assign_function__OpponentTrajectory__oppwpnts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<f110_msgs::msg::OppWpnt *>(
    get_function__OpponentTrajectory__oppwpnts(untyped_member, index));
  const auto & value = *reinterpret_cast<const f110_msgs::msg::OppWpnt *>(untyped_value);
  item = value;
}

void resize_function__OpponentTrajectory__oppwpnts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<f110_msgs::msg::OppWpnt> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OpponentTrajectory_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OpponentTrajectory, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OpponentTrajectory, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lap_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OpponentTrajectory, lap_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "opp_is_on_trajectory",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OpponentTrajectory, opp_is_on_trajectory),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "oppwpnts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<f110_msgs::msg::OppWpnt>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::OpponentTrajectory, oppwpnts),  // bytes offset in struct
    nullptr,  // default value
    size_function__OpponentTrajectory__oppwpnts,  // size() function pointer
    get_const_function__OpponentTrajectory__oppwpnts,  // get_const(index) function pointer
    get_function__OpponentTrajectory__oppwpnts,  // get(index) function pointer
    fetch_function__OpponentTrajectory__oppwpnts,  // fetch(index, &value) function pointer
    assign_function__OpponentTrajectory__oppwpnts,  // assign(index, value) function pointer
    resize_function__OpponentTrajectory__oppwpnts  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OpponentTrajectory_message_members = {
  "f110_msgs::msg",  // message namespace
  "OpponentTrajectory",  // message name
  5,  // number of fields
  sizeof(f110_msgs::msg::OpponentTrajectory),
  OpponentTrajectory_message_member_array,  // message members
  OpponentTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  OpponentTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OpponentTrajectory_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OpponentTrajectory_message_members,
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
get_message_type_support_handle<f110_msgs::msg::OpponentTrajectory>()
{
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::OpponentTrajectory_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, f110_msgs, msg, OpponentTrajectory)() {
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::OpponentTrajectory_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
