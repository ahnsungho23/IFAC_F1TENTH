// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/ProjOppPoint.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ProjOppPoint in the package f110_msgs.
/**
  * Velocities are projected onto ego race line
 */
typedef struct f110_msgs__msg__ProjOppPoint
{
  double s;
  double d;
  double vs;
  double vd;
  bool is_static;
  bool is_visible;
  double time;
  double s_var;
  double d_var;
  double vs_var;
  double vd_var;
} f110_msgs__msg__ProjOppPoint;

// Struct for a sequence of f110_msgs__msg__ProjOppPoint.
typedef struct f110_msgs__msg__ProjOppPoint__Sequence
{
  f110_msgs__msg__ProjOppPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__ProjOppPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_POINT__STRUCT_H_
