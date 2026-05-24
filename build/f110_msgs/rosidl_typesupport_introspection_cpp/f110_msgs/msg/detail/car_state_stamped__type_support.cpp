// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from f110_msgs:msg/CarStateStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "f110_msgs/msg/detail/car_state_stamped__struct.hpp"
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

void CarStateStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) f110_msgs::msg::CarStateStamped(_init);
}

void CarStateStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<f110_msgs::msg::CarStateStamped *>(message_memory);
  typed_message->~CarStateStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CarStateStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::CarStateStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<f110_msgs::msg::CarState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs::msg::CarStateStamped, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CarStateStamped_message_members = {
  "f110_msgs::msg",  // message namespace
  "CarStateStamped",  // message name
  2,  // number of fields
  sizeof(f110_msgs::msg::CarStateStamped),
  CarStateStamped_message_member_array,  // message members
  CarStateStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  CarStateStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CarStateStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CarStateStamped_message_members,
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
get_message_type_support_handle<f110_msgs::msg::CarStateStamped>()
{
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::CarStateStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, f110_msgs, msg, CarStateStamped)() {
  return &::f110_msgs::msg::rosidl_typesupport_introspection_cpp::CarStateStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
