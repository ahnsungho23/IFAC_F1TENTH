// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_H_

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

/// Struct defined in msg/LapData in the package f110_msgs.
typedef struct f110_msgs__msg__LapData
{
  std_msgs__msg__Header header;
  int32_t lap_count;
  double lap_time;
  double average_lateral_error_to_global_waypoints;
  double max_lateral_error_to_global_waypoints;
} f110_msgs__msg__LapData;

// Struct for a sequence of f110_msgs__msg__LapData.
typedef struct f110_msgs__msg__LapData__Sequence
{
  f110_msgs__msg__LapData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__LapData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__LAP_DATA__STRUCT_H_
