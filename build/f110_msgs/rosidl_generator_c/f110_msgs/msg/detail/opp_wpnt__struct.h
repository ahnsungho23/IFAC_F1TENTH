// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/OppWpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OppWpnt in the package f110_msgs.
typedef struct f110_msgs__msg__OppWpnt
{
  int32_t id;
  /// frenet coordinates
  double s_m;
  double d_m;
  /// map coordinates
  double x_m;
  double y_m;
  /// track information
  double proj_vs_mps;
  double vd_mps;
  /// variance
  double d_var;
  double vs_var;
} f110_msgs__msg__OppWpnt;

// Struct for a sequence of f110_msgs__msg__OppWpnt.
typedef struct f110_msgs__msg__OppWpnt__Sequence
{
  f110_msgs__msg__OppWpnt * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__OppWpnt__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OPP_WPNT__STRUCT_H_
