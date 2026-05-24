// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "f110_msgs/msg/detail/proj_opp_traj__rosidl_typesupport_introspection_c.h"
#include "f110_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "f110_msgs/msg/detail/proj_opp_traj__functions.h"
#include "f110_msgs/msg/detail/proj_opp_traj__struct.h"


// Include directives for member types
// Member `detections`
#include "f110_msgs/msg/proj_opp_point.h"
// Member `detections`
#include "f110_msgs/msg/detail/proj_opp_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  f110_msgs__msg__ProjOppTraj__init(message_memory);
}

void f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_fini_function(void * message_memory)
{
  f110_msgs__msg__ProjOppTraj__fini(message_memory);
}

size_t f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__size_function__ProjOppTraj__detections(
  const void * untyped_member)
{
  const f110_msgs__msg__ProjOppPoint__Sequence * member =
    (const f110_msgs__msg__ProjOppPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_const_function__ProjOppTraj__detections(
  const void * untyped_member, size_t index)
{
  const f110_msgs__msg__ProjOppPoint__Sequence * member =
    (const f110_msgs__msg__ProjOppPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_function__ProjOppTraj__detections(
  void * untyped_member, size_t index)
{
  f110_msgs__msg__ProjOppPoint__Sequence * member =
    (f110_msgs__msg__ProjOppPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__fetch_function__ProjOppTraj__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const f110_msgs__msg__ProjOppPoint * item =
    ((const f110_msgs__msg__ProjOppPoint *)
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_const_function__ProjOppTraj__detections(untyped_member, index));
  f110_msgs__msg__ProjOppPoint * value =
    (f110_msgs__msg__ProjOppPoint *)(untyped_value);
  *value = *item;
}

void f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__assign_function__ProjOppTraj__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  f110_msgs__msg__ProjOppPoint * item =
    ((f110_msgs__msg__ProjOppPoint *)
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_function__ProjOppTraj__detections(untyped_member, index));
  const f110_msgs__msg__ProjOppPoint * value =
    (const f110_msgs__msg__ProjOppPoint *)(untyped_value);
  *item = *value;
}

bool f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__resize_function__ProjOppTraj__detections(
  void * untyped_member, size_t size)
{
  f110_msgs__msg__ProjOppPoint__Sequence * member =
    (f110_msgs__msg__ProjOppPoint__Sequence *)(untyped_member);
  f110_msgs__msg__ProjOppPoint__Sequence__fini(member);
  return f110_msgs__msg__ProjOppPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_member_array[4] = {
  {
    "lapcount",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs__msg__ProjOppTraj, lapcount),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nrofpoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs__msg__ProjOppTraj, nrofpoints),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "opp_is_on_trajectory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs__msg__ProjOppTraj, opp_is_on_trajectory),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(f110_msgs__msg__ProjOppTraj, detections),  // bytes offset in struct
    NULL,  // default value
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__size_function__ProjOppTraj__detections,  // size() function pointer
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_const_function__ProjOppTraj__detections,  // get_const(index) function pointer
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__get_function__ProjOppTraj__detections,  // get(index) function pointer
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__fetch_function__ProjOppTraj__detections,  // fetch(index, &value) function pointer
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__assign_function__ProjOppTraj__detections,  // assign(index, value) function pointer
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__resize_function__ProjOppTraj__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_members = {
  "f110_msgs__msg",  // message namespace
  "ProjOppTraj",  // message name
  4,  // number of fields
  sizeof(f110_msgs__msg__ProjOppTraj),
  f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_member_array,  // message members
  f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_type_support_handle = {
  0,
  &f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_f110_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, f110_msgs, msg, ProjOppTraj)() {
  f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, f110_msgs, msg, ProjOppPoint)();
  if (!f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_type_support_handle.typesupport_identifier) {
    f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &f110_msgs__msg__ProjOppTraj__rosidl_typesupport_introspection_c__ProjOppTraj_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
