// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/opp_wpnt__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "f110_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "f110_msgs/msg/detail/opp_wpnt__struct.h"
#include "f110_msgs/msg/detail/opp_wpnt__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _OppWpnt__ros_msg_type = f110_msgs__msg__OppWpnt;

static bool _OppWpnt__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _OppWpnt__ros_msg_type * ros_message = static_cast<const _OppWpnt__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: s_m
  {
    cdr << ros_message->s_m;
  }

  // Field name: d_m
  {
    cdr << ros_message->d_m;
  }

  // Field name: x_m
  {
    cdr << ros_message->x_m;
  }

  // Field name: y_m
  {
    cdr << ros_message->y_m;
  }

  // Field name: proj_vs_mps
  {
    cdr << ros_message->proj_vs_mps;
  }

  // Field name: vd_mps
  {
    cdr << ros_message->vd_mps;
  }

  // Field name: d_var
  {
    cdr << ros_message->d_var;
  }

  // Field name: vs_var
  {
    cdr << ros_message->vs_var;
  }

  return true;
}

static bool _OppWpnt__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _OppWpnt__ros_msg_type * ros_message = static_cast<_OppWpnt__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: s_m
  {
    cdr >> ros_message->s_m;
  }

  // Field name: d_m
  {
    cdr >> ros_message->d_m;
  }

  // Field name: x_m
  {
    cdr >> ros_message->x_m;
  }

  // Field name: y_m
  {
    cdr >> ros_message->y_m;
  }

  // Field name: proj_vs_mps
  {
    cdr >> ros_message->proj_vs_mps;
  }

  // Field name: vd_mps
  {
    cdr >> ros_message->vd_mps;
  }

  // Field name: d_var
  {
    cdr >> ros_message->d_var;
  }

  // Field name: vs_var
  {
    cdr >> ros_message->vs_var;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_f110_msgs
size_t get_serialized_size_f110_msgs__msg__OppWpnt(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _OppWpnt__ros_msg_type * ros_message = static_cast<const _OppWpnt__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_m
  {
    size_t item_size = sizeof(ros_message->s_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d_m
  {
    size_t item_size = sizeof(ros_message->d_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x_m
  {
    size_t item_size = sizeof(ros_message->x_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_m
  {
    size_t item_size = sizeof(ros_message->y_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name proj_vs_mps
  {
    size_t item_size = sizeof(ros_message->proj_vs_mps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vd_mps
  {
    size_t item_size = sizeof(ros_message->vd_mps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d_var
  {
    size_t item_size = sizeof(ros_message->d_var);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vs_var
  {
    size_t item_size = sizeof(ros_message->vs_var);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _OppWpnt__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_f110_msgs__msg__OppWpnt(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_f110_msgs
size_t max_serialized_size_f110_msgs__msg__OppWpnt(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: s_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: x_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: y_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: proj_vs_mps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vd_mps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d_var
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vs_var
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = f110_msgs__msg__OppWpnt;
    is_plain =
      (
      offsetof(DataType, vs_var) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _OppWpnt__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_f110_msgs__msg__OppWpnt(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_OppWpnt = {
  "f110_msgs::msg",
  "OppWpnt",
  _OppWpnt__cdr_serialize,
  _OppWpnt__cdr_deserialize,
  _OppWpnt__get_serialized_size,
  _OppWpnt__max_serialized_size
};

static rosidl_message_type_support_t _OppWpnt__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_OppWpnt,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, f110_msgs, msg, OppWpnt)() {
  return &_OppWpnt__type_support;
}

#if defined(__cplusplus)
}
#endif
