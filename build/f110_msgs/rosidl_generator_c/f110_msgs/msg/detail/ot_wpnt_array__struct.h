// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/OTWpntArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'last_switch_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'ot_side'
// Member 'ot_line'
#include "rosidl_runtime_c/string.h"
// Member 'wpnts'
#include "f110_msgs/msg/detail/wpnt__struct.h"

/// Struct defined in msg/OTWpntArray in the package f110_msgs.
typedef struct f110_msgs__msg__OTWpntArray
{
  std_msgs__msg__Header header;
  builtin_interfaces__msg__Time last_switch_time;
  bool side_switch;
  rosidl_runtime_c__String ot_side;
  rosidl_runtime_c__String ot_line;
  f110_msgs__msg__Wpnt__Sequence wpnts;
} f110_msgs__msg__OTWpntArray;

// Struct for a sequence of f110_msgs__msg__OTWpntArray.
typedef struct f110_msgs__msg__OTWpntArray__Sequence
{
  f110_msgs__msg__OTWpntArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__OTWpntArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OT_WPNT_ARRAY__STRUCT_H_
