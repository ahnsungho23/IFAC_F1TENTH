// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/PidData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_H_

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

/// Struct defined in msg/PidData in the package f110_msgs.
typedef struct f110_msgs__msg__PidData
{
  std_msgs__msg__Header header;
  double should;
  double actual;
  double error;
  double d_value;
  double i_value;
  double input;
} f110_msgs__msg__PidData;

// Struct for a sequence of f110_msgs__msg__PidData.
typedef struct f110_msgs__msg__PidData__Sequence
{
  f110_msgs__msg__PidData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__PidData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__PID_DATA__STRUCT_H_
