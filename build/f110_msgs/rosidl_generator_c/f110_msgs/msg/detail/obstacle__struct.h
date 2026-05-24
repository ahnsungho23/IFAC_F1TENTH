// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Obstacle in the package f110_msgs.
typedef struct f110_msgs__msg__Obstacle
{
  int32_t id;
  double s_start;
  double s_end;
  /// defined as right bound of the obstacle
  double d_right;
  /// defined as left bound of the obstacle
  double d_left;
  /// used by the frenet planner to choos points through which we want to go
  bool is_actually_a_gap;
  double s_center;
  double d_center;
  double size;
  double vs;
  double vd;
  double s_var;
  double d_var;
  double vs_var;
  double vd_var;
  bool is_static;
  bool is_visible;
} f110_msgs__msg__Obstacle;

// Struct for a sequence of f110_msgs__msg__Obstacle.
typedef struct f110_msgs__msg__Obstacle__Sequence
{
  f110_msgs__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
