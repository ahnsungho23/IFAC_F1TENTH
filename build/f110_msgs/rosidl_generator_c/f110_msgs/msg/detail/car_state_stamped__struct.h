// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/CarStateStamped.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_H_

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
// Member 'state'
#include "f110_msgs/msg/detail/car_state__struct.h"

/// Struct defined in msg/CarStateStamped in the package f110_msgs.
typedef struct f110_msgs__msg__CarStateStamped
{
  std_msgs__msg__Header header;
  f110_msgs__msg__CarState state;
} f110_msgs__msg__CarStateStamped;

// Struct for a sequence of f110_msgs__msg__CarStateStamped.
typedef struct f110_msgs__msg__CarStateStamped__Sequence
{
  f110_msgs__msg__CarStateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__CarStateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE_STAMPED__STRUCT_H_
