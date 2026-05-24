// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/obstacle__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "f110_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "f110_msgs/msg/detail/obstacle__struct.h"
#include "f110_msgs/msg/detail/obstacle__functions.h"
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


using _Obstacle__ros_msg_type = f110_msgs__msg__Obstacle;

static bool _Obstacle__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Obstacle__ros_msg_type * ros_message = static_cast<const _Obstacle__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: s_start
  {
    cdr << ros_message->s_start;
  }

  // Field name: s_end
  {
    cdr << ros_message->s_end;
  }

  // Field name: d_right
  {
    cdr << ros_message->d_right;
  }

  // Field name: d_left
  {
    cdr << ros_message->d_left;
  }

  // Field name: is_actually_a_gap
  {
    cdr << (ros_message->is_actually_a_gap ? true : false);
  }

  // Field name: s_center
  {
    cdr << ros_message->s_center;
  }

  // Field name: d_center
  {
    cdr << ros_message->d_center;
  }

  // Field name: size
  {
    cdr << ros_message->size;
  }

  // Field name: vs
  {
    cdr << ros_message->vs;
  }

  // Field name: vd
  {
    cdr << ros_message->vd;
  }

  // Field name: s_var
  {
    cdr << ros_message->s_var;
  }

  // Field name: d_var
  {
    cdr << ros_message->d_var;
  }

  // Field name: vs_var
  {
    cdr << ros_message->vs_var;
  }

  // Field name: vd_var
  {
    cdr << ros_message->vd_var;
  }

  // Field name: is_static
  {
    cdr << (ros_message->is_static ? true : false);
  }

  // Field name: is_visible
  {
    cdr << (ros_message->is_visible ? true : false);
  }

  return true;
}

static bool _Obstacle__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Obstacle__ros_msg_type * ros_message = static_cast<_Obstacle__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: s_start
  {
    cdr >> ros_message->s_start;
  }

  // Field name: s_end
  {
    cdr >> ros_message->s_end;
  }

  // Field name: d_right
  {
    cdr >> ros_message->d_right;
  }

  // Field name: d_left
  {
    cdr >> ros_message->d_left;
  }

  // Field name: is_actually_a_gap
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_actually_a_gap = tmp ? true : false;
  }

  // Field name: s_center
  {
    cdr >> ros_message->s_center;
  }

  // Field name: d_center
  {
    cdr >> ros_message->d_center;
  }

  // Field name: size
  {
    cdr >> ros_message->size;
  }

  // Field name: vs
  {
    cdr >> ros_message->vs;
  }

  // Field name: vd
  {
    cdr >> ros_message->vd;
  }

  // Field name: s_var
  {
    cdr >> ros_message->s_var;
  }

  // Field name: d_var
  {
    cdr >> ros_message->d_var;
  }

  // Field name: vs_var
  {
    cdr >> ros_message->vs_var;
  }

  // Field name: vd_var
  {
    cdr >> ros_message->vd_var;
  }

  // Field name: is_static
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_static = tmp ? true : false;
  }

  // Field name: is_visible
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_visible = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_f110_msgs
size_t get_serialized_size_f110_msgs__msg__Obstacle(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Obstacle__ros_msg_type * ros_message = static_cast<const _Obstacle__ros_msg_type *>(untyped_ros_message);
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
  // field.name s_start
  {
    size_t item_size = sizeof(ros_message->s_start);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_end
  {
    size_t item_size = sizeof(ros_message->s_end);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d_right
  {
    size_t item_size = sizeof(ros_message->d_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d_left
  {
    size_t item_size = sizeof(ros_message->d_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_actually_a_gap
  {
    size_t item_size = sizeof(ros_message->is_actually_a_gap);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_center
  {
    size_t item_size = sizeof(ros_message->s_center);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d_center
  {
    size_t item_size = sizeof(ros_message->d_center);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name size
  {
    size_t item_size = sizeof(ros_message->size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vs
  {
    size_t item_size = sizeof(ros_message->vs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vd
  {
    size_t item_size = sizeof(ros_message->vd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_var
  {
    size_t item_size = sizeof(ros_message->s_var);
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
  // field.name vd_var
  {
    size_t item_size = sizeof(ros_message->vd_var);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_static
  {
    size_t item_size = sizeof(ros_message->is_static);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_visible
  {
    size_t item_size = sizeof(ros_message->is_visible);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Obstacle__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_f110_msgs__msg__Obstacle(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_f110_msgs
size_t max_serialized_size_f110_msgs__msg__Obstacle(
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
  // member: s_start
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: s_end
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: is_actually_a_gap
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: s_center
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d_center
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: size
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: s_var
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
  // member: vd_var
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: is_static
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_visible
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = f110_msgs__msg__Obstacle;
    is_plain =
      (
      offsetof(DataType, is_visible) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Obstacle__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_f110_msgs__msg__Obstacle(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Obstacle = {
  "f110_msgs::msg",
  "Obstacle",
  _Obstacle__cdr_serialize,
  _Obstacle__cdr_deserialize,
  _Obstacle__get_serialized_size,
  _Obstacle__max_serialized_size
};

static rosidl_message_type_support_t _Obstacle__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Obstacle,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, f110_msgs, msg, Obstacle)() {
  return &_Obstacle__type_support;
}

#if defined(__cplusplus)
}
#endif
