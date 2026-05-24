// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/opp_wpnt__rosidl_typesupport_fastrtps_cpp.hpp"
#include "f110_msgs/msg/detail/opp_wpnt__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace f110_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_f110_msgs
cdr_serialize(
  const f110_msgs::msg::OppWpnt & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: id
  cdr << ros_message.id;
  // Member: s_m
  cdr << ros_message.s_m;
  // Member: d_m
  cdr << ros_message.d_m;
  // Member: x_m
  cdr << ros_message.x_m;
  // Member: y_m
  cdr << ros_message.y_m;
  // Member: proj_vs_mps
  cdr << ros_message.proj_vs_mps;
  // Member: vd_mps
  cdr << ros_message.vd_mps;
  // Member: d_var
  cdr << ros_message.d_var;
  // Member: vs_var
  cdr << ros_message.vs_var;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_f110_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  f110_msgs::msg::OppWpnt & ros_message)
{
  // Member: id
  cdr >> ros_message.id;

  // Member: s_m
  cdr >> ros_message.s_m;

  // Member: d_m
  cdr >> ros_message.d_m;

  // Member: x_m
  cdr >> ros_message.x_m;

  // Member: y_m
  cdr >> ros_message.y_m;

  // Member: proj_vs_mps
  cdr >> ros_message.proj_vs_mps;

  // Member: vd_mps
  cdr >> ros_message.vd_mps;

  // Member: d_var
  cdr >> ros_message.d_var;

  // Member: vs_var
  cdr >> ros_message.vs_var;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_f110_msgs
get_serialized_size(
  const f110_msgs::msg::OppWpnt & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: s_m
  {
    size_t item_size = sizeof(ros_message.s_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: d_m
  {
    size_t item_size = sizeof(ros_message.d_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: x_m
  {
    size_t item_size = sizeof(ros_message.x_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_m
  {
    size_t item_size = sizeof(ros_message.y_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: proj_vs_mps
  {
    size_t item_size = sizeof(ros_message.proj_vs_mps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vd_mps
  {
    size_t item_size = sizeof(ros_message.vd_mps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: d_var
  {
    size_t item_size = sizeof(ros_message.d_var);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vs_var
  {
    size_t item_size = sizeof(ros_message.vs_var);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_f110_msgs
max_serialized_size_OppWpnt(
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


  // Member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: s_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: d_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: x_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: y_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: proj_vs_mps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: vd_mps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: d_var
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: vs_var
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
    using DataType = f110_msgs::msg::OppWpnt;
    is_plain =
      (
      offsetof(DataType, vs_var) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _OppWpnt__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const f110_msgs::msg::OppWpnt *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _OppWpnt__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<f110_msgs::msg::OppWpnt *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _OppWpnt__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const f110_msgs::msg::OppWpnt *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _OppWpnt__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_OppWpnt(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _OppWpnt__callbacks = {
  "f110_msgs::msg",
  "OppWpnt",
  _OppWpnt__cdr_serialize,
  _OppWpnt__cdr_deserialize,
  _OppWpnt__get_serialized_size,
  _OppWpnt__max_serialized_size
};

static rosidl_message_type_support_t _OppWpnt__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_OppWpnt__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace f110_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_f110_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<f110_msgs::msg::OppWpnt>()
{
  return &f110_msgs::msg::typesupport_fastrtps_cpp::_OppWpnt__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, f110_msgs, msg, OppWpnt)() {
  return &f110_msgs::msg::typesupport_fastrtps_cpp::_OppWpnt__handle;
}

#ifdef __cplusplus
}
#endif
