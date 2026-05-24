// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/Wpnt.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__WPNT__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__WPNT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Wpnt in the package f110_msgs.
typedef struct f110_msgs__msg__Wpnt
{
  int32_t id;
  /// frenet coordinates
  double s_m;
  double d_m;
  /// map coordinates
  double x_m;
  double y_m;
  /// track bound distance
  double d_right;
  double d_left;
  /// track information
  double psi_rad;
  double kappa_radpm;
  double vx_mps;
  double ax_mps2;
} f110_msgs__msg__Wpnt;

// Struct for a sequence of f110_msgs__msg__Wpnt.
typedef struct f110_msgs__msg__Wpnt__Sequence
{
  f110_msgs__msg__Wpnt * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__Wpnt__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__WPNT__STRUCT_H_
